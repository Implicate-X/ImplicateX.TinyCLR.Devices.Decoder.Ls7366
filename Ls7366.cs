/// <summary>
/// Implements a high-precision 32-bit quadrature decoder using the LS7366R 
/// via SPI in TinyCLR 3 for manual rotary encoders, such as the EC11.
/// Unlike all software-based solutions, no dropouts or skips are to be expected here.
/// (c) 2026 by Robert Dettmann.
/// </summary>

using System;
using System.Diagnostics;
using System.Threading;
using GHIElectronics.TinyCLR.Devices.Gpio;
using GHIElectronics.TinyCLR.Devices.Spi;
using GHIElectronics.TinyCLR.Devices.Spi.Provider;

namespace ImplicateX.Devices.Decoder
{
	public sealed class Ls7366 : IDisposable
	{
		/// <summary>
		/// Indicates the detected direction of encoder movement.
		/// </summary>
		public enum Direction
		{
			/// <summary>
			/// The direction could not be determined.
			/// </summary>
			Unknown = 0,

			/// <summary>
			/// The encoder moved downward or counter-clockwise, depending on wiring and interpretation.
			/// </summary>
			Down = -1,

			/// <summary>
			/// The encoder moved upward or clockwise, depending on wiring and interpretation.
			/// </summary>
			Up = 1
		}

		/// <summary>
		/// Provides data for decoder change notifications.
		/// </summary>
		public sealed class ChangedEventArgs( long count, Direction direction, byte status ) : EventArgs
		{
			/// <summary>
			/// Initializes a new instance of the <see cref="ChangedEventArgs"/> class with raw and detent counts.
			/// </summary>
			/// <param name="count">The detent-aligned count reported to consumers.</param>
			/// <param name="rawCount">The raw decoder count read from the counter.</param>
			/// <param name="direction">The detected direction of movement.</param>
			/// <param name="status">The status register value associated with the reading.</param>
			public ChangedEventArgs( long count, long rawCount, Direction direction, byte status ) : this( count, direction, status )
			{
				this.RawCount = rawCount;
			}

			/// <summary>
			/// Gets the detent-aligned count.
			/// </summary>
			public long Count { get; } = count;

			/// <summary>
			/// Gets the raw counter value from the decoder hardware.
			/// </summary>
			public long RawCount { get; }

			/// <summary>
			/// Gets the detected direction of movement.
			/// </summary>
			public Direction Direction { get; } = direction;

			/// <summary>
			/// Gets the status register value captured at the time of the event.
			/// </summary>
			public byte Status { get; } = status;
		}

		/// <summary>
		/// Contains the LS7366 command opcodes used to read, write, clear, and load device registers.
		/// </summary>
		private sealed class Command
		{
			/// <summary>Writes the MDR0 configuration register.</summary>
			public const byte WriteMdr0 = 0x88;

			/// <summary>Writes the MDR1 configuration register.</summary>
			public const byte WriteMdr1 = 0x90;

			/// <summary>Writes the data transfer register (DTR).</summary>
			public const byte WriteDtr = 0x98;

			/// <summary>Clears the MDR0 configuration register.</summary>
			public const byte ClearMdr0 = 0x08;

			/// <summary>Clears the MDR1 configuration register.</summary>
			public const byte ClearMdr1 = 0x10;

			/// <summary>Clears the counter register.</summary>
			public const byte ClearCntr = 0x20;

			/// <summary>Clears the status register.</summary>
			public const byte ClearStr = 0x30;

			/// <summary>Reads the MDR0 configuration register.</summary>
			public const byte ReadMdr0 = 0x48;

			/// <summary>Reads the MDR1 configuration register.</summary>
			public const byte ReadMdr1 = 0x50;

			/// <summary>Reads the counter register.</summary>
			public const byte ReadCntr = 0x60;

			/// <summary>Reads the output transfer register (OTR).</summary>
			public const byte ReadOtr = 0x68;

			/// <summary>Reads the status register.</summary>
			public const byte ReadStr = 0x70;

			/// <summary>Loads the counter register from the DTR.</summary>
			public const byte LoadCntr = 0xE0;

			/// <summary>Loads the OTR from the counter register.</summary>
			public const byte LoadOtr = 0xE8;
		}

		/// <summary>
		/// Contains LS7366 MDR0/MDR1 flag values used to configure quadrature decoding,
		/// index behavior, filter clock selection, counter width, and counter enable state.
		/// </summary>
		public sealed class Mode
		{
			/// <summary>Configures quadrature decoding for 4x resolution.</summary>
			public const byte QuadratureX4 = 0x03;

			/// <summary>Configures quadrature decoding for 2x resolution.</summary>
			public const byte QuadratureX2 = 0x02;

			/// <summary>Configures quadrature decoding for 1x resolution.</summary>
			public const byte QuadratureX1 = 0x01;

			/// <summary>Disables free-running mode.</summary>
			public const byte FreeRun = 0x00;

			/// <summary>Disables index input handling.</summary>
			public const byte DisableIndex = 0x00;

			/// <summary>Uses asynchronous index handling.</summary>
			public const byte AsynchronousIndex = 0x00;

			/// <summary>Uses synchronous index handling.</summary>
			public const byte SynchronousIndex = 0x80;

			/// <summary>Selects the default filter clock divisor.</summary>
			public const byte FilterClockDiv1 = 0x00;

			/// <summary>Selects a divided filter clock.</summary>
			public const byte FilterClockDiv2 = 0x80;

			/// <summary>Configures the counter for 4-byte mode.</summary>
			public const byte Byte4Mode = 0x00;

			/// <summary>Keeps the counter enabled during normal operation.</summary>
			public const byte CounterEnabled = 0x00;
		}

		/// <summary>
		/// The number of bytes used to read and write the LS7366 counter register.
		/// </summary>
		private const int CounterByteCount = 4;

		/// <summary>
		/// The number of raw decoder counts that make up one reported detent step.
		/// </summary>
		private const int DetentDivider = 4;

		/// <summary>
		/// The status register bit that indicates count direction.
		/// </summary>
		private const int StatusDirectionBit = 0x02;

		/// <summary>
		/// The interval, in milliseconds, between decoder polling operations.
		/// </summary>
		private const int PollIntervalMs = 5;

		/// <summary>
		/// Synchronizes access to the SPI device and polling state.
		/// </summary>
		private readonly object syncRoot = new();

		/// <summary>
		/// Masks raw counter values to the 32-bit unsigned LS7366 range.
		/// </summary>
		private readonly long counterMask = 0xFFFFFFFFL;

		/// <summary>
		/// The SPI device used to communicate with the LS7366.
		/// </summary>
		private SpiDevice spiDevice_;

		/// <summary>
		/// The timer used to poll the decoder at a fixed interval.
		/// </summary>
		private Timer pollTimer_;

		/// <summary>
		/// The most recent raw counter value read from the device.
		/// </summary>
		private long lastCounter_;

		/// <summary>
		/// The last detent-aligned count reported to subscribers.
		/// </summary>
		private long lastReportedCount_ = long.MinValue;

		/// <summary>
		/// Indicates whether the instance has been disposed.
		/// </summary>
		private bool disposed_;

		/// <summary>
		/// Occurs when the decoder count changes on a detent boundary.
		/// </summary>
		public event Action<ChangedEventArgs> CounterChanged;

		/// <summary>
		/// Initializes a new instance of the <see cref="Ls7366"/> class and configures the SPI-backed decoder interface.
		/// </summary>
		/// <param name="gpio">The GPIO controller used to create the SPI software provider and chip-select pin.</param>
		/// <param name="csPin">The chip-select GPIO pin number.</param>
		/// <param name="sclkPin">The SPI clock GPIO pin number.</param>
		/// <param name="mosiPin">The SPI MOSI GPIO pin number.</param>
		/// <param name="misoPin">The SPI MISO GPIO pin number.</param>
		public Ls7366( GpioController gpio, int csPin, int sclkPin, int mosiPin, int misoPin )
		{
			var settings = new SpiConnectionSettings()
			{
				ChipSelectType = SpiChipSelectType.Gpio,
				ChipSelectLine = gpio.OpenPin( csPin ),
				ClockFrequency = 4_000_000,
				Mode = SpiMode.Mode0
			};

			var spiProvider = new SpiControllerSoftwareProvider( gpio, mosiPin, misoPin, sclkPin );
			spiProvider.SetActiveSettings( settings );

			var spiController = SpiController.FromProvider( spiProvider );
			this.spiDevice_ = spiController.GetDevice( settings );

			this.ConfigureForEc11();

			this.lastCounter_ = this.ReadCounter();
		}

		/// <summary>
		/// Starts periodic polling of the decoder state.
		/// </summary>
		public void Start()
		{
			if( this.disposed_ )
			{
				throw new ObjectDisposedException( nameof( Ls7366 ) );
			}

			if( this.pollTimer_ != null )
			{
				return;
			}

			this.pollTimer_ = new Timer( this.PollDecoder, null, PollIntervalMs, PollIntervalMs );
		}

		/// <summary>
		/// Stops periodic polling of the decoder state.
		/// </summary>
		public void Stop()
		{
			Timer timer = this.pollTimer_;
			this.pollTimer_ = null;
			timer?.Dispose();
		}

		/// <summary>
		/// Applies the default decoder configuration.
		/// </summary>
		public void ConfigureForDecoder()
		{
			this.ConfigureForEc11();
		}

		/// <summary>
		/// Configures the LS7366 for EC11-style quadrature decoder operation using the default settings.
		/// </summary>
		public void ConfigureForEc11()
		{
			this.ConfigureForEc11
			(
				Mode.QuadratureX4 |
				Mode.FreeRun |
				Mode.DisableIndex |
				Mode.AsynchronousIndex |
				Mode.FilterClockDiv2,

				Mode.Byte4Mode |
				Mode.CounterEnabled
			);
		}

		/// <summary>
		/// Configures the LS7366 with explicit MDR0 and MDR1 values.
		/// </summary>
		/// <param name="mdr0">The MDR0 register value to write.</param>
		/// <param name="mdr1">The MDR1 register value to write.</param>
		public void ConfigureForEc11( byte mdr0, byte mdr1 )
		{
			lock( this.syncRoot )
			{
				this.SendData( Command.WriteMdr0, mdr0 );
				this.SendData( Command.WriteMdr1, mdr1 );
				this.SendCommand( Command.ClearCntr );
				this.SendCommand( Command.ClearStr );

				this.lastCounter_ = 0;
				this.lastReportedCount_ = long.MinValue;
			}
		}

		/// <summary>
		/// Reads the current counter value from the hardware.
		/// </summary>
		/// <returns>The current counter value.</returns>
		public long ReadCounter()
		{
			lock( this.syncRoot )
			{
				return this.ReceiveCounter();
			}
		}

		/// <summary>
		/// Reads the current status register value from the hardware.
		/// </summary>
		/// <returns>The status register value.</returns>
		public byte ReadStatus()
		{
			lock( this.syncRoot )
			{
				return this.ReceiveStatus();
			}
		}

		/// <summary>
		/// Clears the counter and status registers.
		/// </summary>
		public void ResetCounter()
		{
			lock( this.syncRoot )
			{
				this.SendCommand( Command.ClearCntr );
				this.SendCommand( Command.ClearStr );
				this.lastCounter_ = 0;
				this.lastReportedCount_ = long.MinValue;
			}
		}

		/// <summary>
		/// Loads the specified value into the counter.
		/// </summary>
		/// <param name="value">The value to load into the counter.</param>
		public void LoadCounter( long value )
		{
			lock( this.syncRoot )
			{
				this.SendData( Command.WriteDtr, value );
				this.SendCommand( Command.LoadCntr );
				this.lastCounter_ = value & this.counterMask;
				this.lastReportedCount_ = long.MinValue;
			}
		}

		/// <summary>
		/// Releases all resources used by this instance.
		/// </summary>
		public void Dispose()
		{
			if( this.disposed_ )
			{
				return;
			}

			this.disposed_ = true;
			this.Stop();
			this.spiDevice_?.Dispose();
		}

		/// <summary>
		/// Polls the decoder hardware for changes and raises <see cref="CounterChanged"/> when a new
		/// detent-aligned count is detected.
		/// </summary>
		/// <param name="state">Unused timer callback state.</param>
		/// <remarks>
		/// The method reads the raw counter and status values under lock, filters out duplicate samples,
		/// maps the raw count to a detent count using <c>DetentDivider</c>, and publishes the current
		/// direction and status to subscribers.
		/// </remarks>
		private void PollDecoder( object state )
		{
			if( this.disposed_ )
			{
				return;
			}

			long counter;
			byte status;

			lock( this.syncRoot )
			{
				counter = this.ReceiveCounter();
				status = this.ReceiveStatus();
			}

			if( counter == this.lastCounter_ )
			{
				return;
			}

			Direction direction = ( status & StatusDirectionBit ) != 0 ? Direction.Up : Direction.Down;
			this.lastCounter_ = counter;

			if( counter % DetentDivider != 0 )
			{
				return;
			}

			long detentCount = counter / DetentDivider;
			if( detentCount == this.lastReportedCount_ )
			{
				return;
			}

			this.lastReportedCount_ = detentCount;

			Action<ChangedEventArgs> handler = this.CounterChanged;
			handler?.Invoke( new ChangedEventArgs( detentCount, counter, direction, status ) );
		}

		/// <summary>
		/// Reads the current counter value from the LS7366 output transfer register.
		/// </summary>
		/// <remarks>
		/// The counter is first latched into the OTR so the returned value is stable during the read.
		/// </remarks>
		/// <returns>The current 32-bit counter value.</returns>
		private long ReceiveCounter()
		{
			this.SendCommand( Command.LoadOtr );
			return this.ReceiveData( Command.ReadOtr, CounterByteCount );
		}

		/// <summary>
		/// Reads the LS7366 status register.
		/// </summary>
		/// <returns>The raw status byte returned by the device.</returns>
		private byte ReceiveStatus()
		{
			byte[] writeBuffer = new byte[ 2 ];
			byte[] readBuffer = new byte[ 2 ];
			writeBuffer[ 0 ] = Command.ReadStr;

			this.spiDevice_.TransferFullDuplex( writeBuffer, readBuffer );

			return readBuffer[ 1 ];
		}

		/// <summary>
		/// Reads a single-byte register from the LS7366.
		/// </summary>
		/// <param name="command">The register read command opcode.</param>
		/// <returns>The register value.</returns>
		private byte ReceiveData( byte command )
		{
			return ( byte )this.ReceiveData( command, 1 );
		}

		/// <summary>
		/// Writes a command to an LS7366 register.
		/// </summary>
		/// <param name="command">The register write command opcode.</param>
		private void SendCommand( byte command )
		{
			byte[] writeBuffer = new byte[ 1 ];
			byte[] readBuffer = new byte[ 1 ];
			writeBuffer[ 0 ] = command;

			this.spiDevice_.TransferFullDuplex( writeBuffer, readBuffer );
		}

		/// <summary>
		/// Writes a single-byte value to an LS7366 register.
		/// </summary>
		/// <param name="command">The register write command opcode.</param>
		/// <param name="value">The value to write.</param>
		private void SendData( byte command, byte value )
		{
			byte[] writeBuffer = new byte[ 2 ];
			byte[] readBuffer = new byte[ 2 ];
			writeBuffer[ 0 ] = command;
			writeBuffer[ 1 ] = value;

			this.spiDevice_.TransferFullDuplex( writeBuffer, readBuffer );
		}

		/// <summary>
		/// Writes a multi-byte register value to the LS7366 in big-endian order.
		/// </summary>
		/// <param name="command">The register write command opcode.</param>
		/// <param name="value">The value to write.</param>
		private void SendData( byte command, long value )
		{
			byte[] writeBuffer = new byte[ 1 + CounterByteCount ];
			byte[] readBuffer = new byte[ 1 + CounterByteCount ];
			writeBuffer[ 0 ] = command;

			for( int i = CounterByteCount; i > 0; i-- )
			{
				writeBuffer[ i ] = ( byte )( value & 0xFF );
				value >>= 8;
			}

			this.spiDevice_.TransferFullDuplex( writeBuffer, readBuffer );
		}

		/// <summary>
		/// Reads a multi-byte register value from the LS7366 in big-endian order.
		/// </summary>
		/// <param name="command">The register read command opcode.</param>
		/// <param name="dataByteCount">The number of data bytes to read.</param>
		/// <returns>The unsigned value composed from the returned bytes.</returns>
		private uint ReceiveData( byte command, int dataByteCount )
		{
			byte[] writeBuffer = new byte[ 1 + dataByteCount ];
			byte[] readBuffer = new byte[ 1 + dataByteCount ];
			writeBuffer[ 0 ] = command;

			this.spiDevice_.TransferFullDuplex( writeBuffer, readBuffer );

			uint value = 0;

			for( int i = 1; i <= dataByteCount; i++ )
			{
				value = ( value << 8 ) | readBuffer[ i ];
			}

			return value;
		}

		/// <summary>
		/// Writes the current register values and counter state to the debug output window.
		/// </summary>
		public void DumpRegisters()
		{
			lock( this.syncRoot )
			{
				byte mdr0 = this.ReceiveData( Command.ReadMdr0 );
				byte mdr1 = this.ReceiveData( Command.ReadMdr1 );
				uint cntr = this.ReceiveData( Command.ReadCntr, CounterByteCount );

				this.SendCommand( Command.LoadOtr );
				uint otr = this.ReceiveData( Command.ReadOtr, CounterByteCount );
				byte str = this.ReceiveStatus();

				Debug.WriteLine(
					$"LS7366R MDR0=0x{mdr0:X2}, " +
					$"MDR1=0x{mdr1:X2}, " +
					$"CNTR=0x{cntr:X8}, OTR=0x{otr:X8}, " +
					$"STR=0x{str:X2} [{this.DescribeStatusBits( str )}]" );
			}
		}

		/// <summary>
		/// Converts the LS7366R status byte into a human-readable flag list.
		/// </summary>
		/// <param name="status">The raw status register value returned by the device.</param>
		/// <returns>
		/// A formatted string containing each status bit as <c>0</c> or <c>1</c>
		/// for the <c>CY</c>, <c>BW</c>, <c>CMP</c>, <c>IDX</c>, <c>CEN</c>, <c>PLS</c>,
		/// <c>U/D</c>, and <c>S</c> flags.
		/// </returns>
		private string DescribeStatusBits( byte status )
		{
			return $"CY={( ( status & 0x80 ) != 0 ? 1 : 0 )}, " +
					$"BW={( ( status & 0x40 ) != 0 ? 1 : 0 )}, " +
					$"CMP={( ( status & 0x20 ) != 0 ? 1 : 0 )}, " +
					$"IDX={( ( status & 0x10 ) != 0 ? 1 : 0 )}, " +
					$"CEN={( ( status & 0x08 ) != 0 ? 1 : 0 )}, " +
					$"PLS={( ( status & 0x04 ) != 0 ? 1 : 0 )}, " +
					$"U/D={( ( status & 0x02 ) != 0 ? 1 : 0 )}, " +
					$"S={( ( status & 0x01 ) != 0 ? 1 : 0 )}";
		}
	}
}

# ImplicateX.TinyCLR.Devices.Decoder.Ls7366

A high‑precision 32‑bit quadrature decoder for TinyCLR OS 3.x based on the LS7366R hardware counter.  
Unlike software‑based rotary encoder solutions, the LS7366R provides dropout‑free, skip‑free, fully deterministic counting — ideal for mechanical encoders such as the EC11.

## ✨ Features

- True hardware quadrature decoding (1×, 2×, 4×)
- 32‑bit counter width
- Stable SPI communication via TinyCLR Software‑SPI
- Event‑driven detent reporting (`CounterChanged`)
- Direction detection via LS7366R status register
- Detent alignment using configurable divider (default: 4)
- Full register access (MDR0, MDR1, CNTR, OTR, STR)
- Counter reset and load operations
- Thread‑safe implementation

## 📦 Installation

Install via NuGet:
Install-Package ImplicateX.TinyCLR.Devices.Decoder.Ls7366

Dependencies:

- GHIElectronics.TinyCLR.Core (≥ 3.0.1.6000)
- GHIElectronics.TinyCLR.Devices.Gpio (≥ 3.0.1.6000)
- GHIElectronics.TinyCLR.Native (≥ 3.0.1.6000)

## 🔧 Hardware Requirements

- LS7366R quadrature decoder IC  
- EC11 or compatible mechanical rotary encoder  
- 4 GPIO pins for Software‑SPI (SCLK, MOSI, MISO, CS)  
- TinyCLR‑compatible SITCore board  

## 🚀 Usage Example (Modern C#)

```csharp
// LS7366R Quadrature Decoder/Counter
quadDecoder_ = new(
    gpioController_,
    ControllerPin.DecoderCS,
    ControllerPin.DecoderSCLK,
    ControllerPin.DecoderMOSI,
    ControllerPin.DecoderMISO
);

// Subscribe to detent events
quadDecoder_.CounterChanged += decoderEvent =>
{
    double frequency = decoderEvent.Direction switch
    {
        Direction.Up   => ApplyTuneStep(+0.1, "Tune Up command received."),
        Direction.Down => ApplyTuneStep(-0.1, "Tune Down command received."),
        _              => tuner_.GetFrequencyMHz()
    };
};

// Reset and start polling
quadDecoder_.ResetCounter();
quadDecoder_.Start();

📡 How It Works
The decoder operates on a fixed 5 ms polling interval.
Each poll performs:
Latching the counter into the OTR register
Reading the 32‑bit counter value
Reading the status register
Determining movement direction
Mapping raw counts to detent counts (counter / 4)
Raising CounterChanged when a new detent boundary is reached
All SPI operations are synchronized using an internal lock.

📑 API Overview
Ls7366
Start() / Stop()
ReadCounter()
ReadStatus()
ResetCounter()
LoadCounter(long value)
ConfigureForEc11()
CounterChanged event
ChangedEventArgs
Count — detent‑aligned count
RawCount — raw LS7366R counter value
Direction — Up / Down / Unknown
Status — raw status register byte

Direction
Up
Down
Unknown

🛠 Debugging
csharp
decoder.DumpRegisters();
Outputs:
MDR0
MDR1
CNTR
OTR
STR (with decoded bit flags)
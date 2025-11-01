

# Hardware Design of the MOTIF Hand

## Project Structure

```
00_Reference/                  # Component Data Sheet
01_Function/                   # Function diagram of the System
02_Hardware/
├── Film_Palm_Tactile/         # Schematic diagram and PCB of the Palm Board
└── Film_Tactile/              # Schematic diagram and PCB of the Finger Board
03_Firmware/                   ## Based on STM32H7
├── BMM350/                    # Source code of the Magnetometer
├── DWT/                       # Source code of the DWT Clock
├── EKF/                       # Source code of the EKF
├── FSR/                       # Source code of FSR multi-channel scanning
├── ICM42688P/                 # Source code of the IMU
├── MODBUS-LIB/                # Source code of the Modbus
├── RS485/                     # Source code of the RS485 Communication
└── WS2812/                    # Source code of the LED color control
04_Software/
└── Tools/                     # Tools for Calibration and Communication test
05_Mechanical/
├── Circuits/                  # 3D models of the Circuits
└── Hand_Structure/            # 3D models of the modified Hand Structure
```
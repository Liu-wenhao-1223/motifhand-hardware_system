

# Hardware Design of the MOTIF Hand

[![arXiv](https://img.shields.io/badge/ArXiv-2506.19201-b31b1b.svg?style=plastic)](https://arxiv.org/abs/2506.19201) [![web](https://img.shields.io/badge/Web-Motif_Hand-blue.svg?style=plastic)](https://slurm-lab-usc.github.io/motif-hand/) [![git](https://img.shields.io/badge/Github-Main_Page-orange.svg?style=plastic)](https://github.com/slurm-lab-usc/motif-hand-documents) [![license](https://img.shields.io/badge/LICENSE-Apache--2.0-white.svg?style=plastic)](./LICENSE)

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
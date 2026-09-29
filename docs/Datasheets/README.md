# Datasheets

Each file is named by the component's reference designator on the Wombat
schematic (`../Schematics/`). Part numbers come from the bill of materials
(`../Wombat_BOM_rev4.xlsx`) and the schematic.

| File | Part | Role |
|---|---|---|
| `U37.pdf` | STMicroelectronics STM32F427VIT6 | Microcontroller this firmware runs on |
| `U5.pdf` | InvenSense MPU-9250 | 9-axis IMU (accelerometer, gyroscope, magnetometer), on SPI3 |
| `U61.pdf`, `U64.pdf` | Toshiba TB6612FNG | Dual H-bridge motor drivers |
| `U38.pdf` | Epson TSX-3225, 24.000 MHz | Main crystal; matches `HSE_VALUE` in `CMakeLists.txt` |
| `U44.pdf` | ECS ECS-.327-12.5-34B, 32.768 kHz | Low-speed crystal |
| `U43.pdf` | Maxim DS1340U-33 | Real-time clock |
| `U2.pdf` | Microchip 24LC02B | 2 Kbit I²C EEPROM |
| `U67.pdf` | Microchip AT24CS01 | 1 Kbit I²C EEPROM with serial number |
| `U56.pdf` | TI TXS0104E | 4-bit bidirectional voltage-level translator |
| `U1.pdf` | TI TFP401 | DVI/HDMI receiver |
| `X1.pdf` | Molex 47151-0001 | HDMI connector |
| `U7.pdf` | TI LM3671MF-3.3 | 3.3 V step-down regulator |
| `U72.pdf`, `U73.pdf` | Diodes Inc. AP65502 | Step-down regulators |
| `U4.pdf` | Fairchild FAN5333B | Voltage regulator |
| `U8.pdf` | Sumida CDRH2D14NP-2R2NC | 2.2 µH inductor |
| `Q1.pdf`, `Q2.pdf` | P-channel MOSFETs | The BOM lists ON Semi NTF6P02T3G; the datasheet files are for Infineon BSP170P |
| `U45 (DNP).pdf` | Maxim MAX98357 | I²S audio amplifier; not populated (DNP) |

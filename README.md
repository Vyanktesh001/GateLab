# GateLab

Interactive logic gate, K-Map and memory trainer board: a PyQt6 desktop app plus Arduino Uno firmware.

## Contents
- `logic_trainer.py` - desktop app (Quine-McCluskey minimiser, adder/subtractor, 74138/74153 simulators, shift registers, K-Map, schematic and timing views)
- `firmware/gatelab_trainer/gatelab_trainer.ino` - Arduino Uno firmware (DIP switches, LEDs, optional I2C 16x2 LCD)
- `GateLab_Project_Overview.pdf` - project overview

## Run
```
pip install -r requirements.txt
python logic_trainer.py
```
Serial link: 115200 8-N-1. Close GateLab before uploading firmware (one program per COM port).

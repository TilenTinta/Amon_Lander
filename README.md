# Amon Lander — Lander/Hopper (Frame, Electronics & Software)

**Project Status:** In Progress

**Short description:** An open‑source EDF‑hovering lander/hopper inspired by **Sprite** (BPS.space)

> Part of the Amon Lander project:
>
> - **Amon Board** — flight controller hardware & firmware  
>   https://github.com/TilenTinta/Amon_Board
> - **Amon Link** — PC - drone RF bridge (PCB & firmware)  
>   https://github.com/TilenTinta/Amon_Link
> - **Amon Ground Control** — desktop GCS for telemetry & control  
>   https://github.com/TilenTinta/Amon_GroundControl_Software

![Amon Lander](https://github.com/TilenTinta/Amon_Lander/blob/main/Pictures/lander_unfinished.jpg)

---

## Table of Contents

- [Overview](#overview)
- [System Concept](#system-concept)
- [Hardware](#hardware)
  - [Frame](#frame)
  - [LED Boards](#led-boards)
- [Getting Started](#getting-started)
  - [Print & Materials](#print--materials)
  - [Assembly Notes](#assembly-notes)
- [Calibration & Testing](#calibration--testing)
- [Repository Structure](#repository-structure)
- [Roadmap](#roadmap)
- [Status & Disclaimer](#status--disclaimer)
- [License](#license)
- [Acknowledgments](#acknowledgments)

---

## Overview

Amon Lander is an educational lander/hopper platform designed to demonstrate vertical takeoff/landing and attitude control with thrust‑vectoring fins. The project emphasizes **building everything from scratch** and using mostly **free maker‑friendly tools**.

---

## System Concept

- **Propulsion:** Single **EDF fan** for hover.  
- **Control:** **Four servos** drives fins for stabilization (**TVC**).  
- **Sensing:** Onboard sensors monitor flight parameters.  
- **Flight Computer:** **Custom PCB** handles computation and comms with the ground computer.  
- **Ground Segment:** Desktop app for **monitoring and control**.

> The goal is to learn: electronics, 3D modeling, embedded/desktop software, reading documentation, flight mechanics, and control theory—while having fun. Many things could be done faster or simpler, but this project idea is learning and exploration.

---

## Hardware

### Frame

The frame is inspired by Sprite, with custom ideas and modifications. The entire airframe is provided in **STL** and **STEP** and is printable on a standard **FDM** printer.  
- Early prototypes used **XPETG**; later versions used **LW‑PLA** to reduce weight.  
- Frame on image is **not yet flight‑tested** (at least not autonomous flight).  
- Designed in **Autodesk Fusion 360 (Personal Use license)**.

![3D Model](https://github.com/TilenTinta/Amon_Lander/blob/main/Pictures/3DModel/3D_model2.PNG)

### LED Boards

Compact PCBs with two LEDs (**red** and **white**) indicate system state. They are mounted on the leg ends and top tubes. Although each LED board has a screw hole, **adhesive is recommended** for attachment.

![LED PCB](https://github.com/TilenTinta/Amon_Lander/blob/main/Pictures/PCB/PCB_LED1.PNG)

---

## Getting Started

### Print & Materials

- Slice **STL** parts per your printer’s capabilities.  
- Recommended filaments:
  - **LW‑PLA** (weight savings) or **PETG/XPETG** (durability).  
- Orient parts to maximize strength along load paths (legs, fins, mounts).

### Assembly Notes

- Dry‑fit major components before final fastening/adhesive.  
- Verify servo travel and linkage geometry with power **off** first.  
- Route wiring to avoid EDF ingestion and moving fin linkages.  

---

## Calibration & Testing

- Bench‑test **sensors** and **servo directions** before powering the EDF.  
- Perform staged tests: electronics bring‑up → servo only → tethered hover.  
- Log telemetry and review for oscillations; tune control gains incrementally.  

### Test stand
For each test there is a separate test stand. There are:
- Thrust stand (test of EDF thrust level)
- Servo stand (test of servo rotation)
- One and two DoF stand (test of drone moments and fin deflection effect)

All tests are done with the help of an Arduino Uno board used for collecting data from sensors. At the same time, a connection over UART-USB is established to a drone for direct communication and command transmission. Everything is monitored by Jupyter scripts for easy and quick tests. Results are saved in **.csv** files for easy future analysis.

![TEST STAND](https://github.com/TilenTinta/Amon_Lander/blob/main/Pictures/Test_stand/PXL_20260328_113547344.jpg)

---

## Control algorithm
The initial plan was to use **Matlab** with **Simulink** to create a complete mathematical model of the drone. After further research, I found that many different methods could be used for controlling a drone like this. After designing the first model in **Simulink** and experimenting with the **Simulink Coder** plugin — with the idea of converting it to standard C code for use on my flight controller — I abandoned this approach. The complete model was then rewritten in **Python** using the **NumPy** library, which is used to test, tune, and further refine the drone model. This same model was subsequently rewritten in **CasADi** style, inheriting the **NumPy** parameters. It is still used in Python, but now with the **acados** framework and optimizer to export the complete control code directly to my MCU/flight controller.

Links to libraries:
- **CasADi**: https://github.com/casadi/casadi
- **acados**: https://github.com/acados/acados

---

## Status & Disclaimer

**In active development.** Designs and documents are evolving; expect changes and rough edges. Exercise caution when testing EDF propulsion and keep bystanders at a safe distance.


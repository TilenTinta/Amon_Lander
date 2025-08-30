# Amon Lander

**Project Status:** In Progress

Amon Lander is an open-source lander/hopper inspired by the **Sprite** vehicle created by Joe Barnard from the BPS.space YouTube channel.  
Channel: https://www.youtube.com/channel/UCILl8ozWuxnFYXIe2svjHhg

![Amon Lander (work in progress)](https://github.com/TilenTinta/Amon_Lander/blob/main/Pictures/lander_unfinished.jpg)

Everything has been built from the ground up—frame, electronics, firmware, and software. Most tools and programs used are free and widely available to makers.

---

## Concept

- Uses an **EDF fan** to hover.
- **Four servos + fins** for stabilization (TVC).
- **Sensors** to monitor flight parameters.
- **Custom PCB** handles computation and communications with a ground computer.
- **Ground app** for monitoring and control.

The main goal is to learn as much as possible about **electronics**, **3D modeling**, **embedded and desktop software**, **datasheets and documentation**, **flight mechanics**, and **control theory**—and to have fun along the way. Many things could be done faster and simpler, but the point of this project is learning.

---

## Hardware

### Frame

The frame is inspired by the BPS.space Sprite drone, with custom ideas and modifications. The entire drone is available in **STL** and **STEP** file formats and can be 3D printed on a standard FDM printer.  
- Early prints used **XPETG**; later versions switched to **LW-PLA** to reduce weight.  
- At the time of writing, these frames have **not** yet been flight-tested.  
- Designed in **Autodesk Fusion 360 (Personal Use license)**.

![3D Model](https://github.com/TilenTinta/Amon_Lander/blob/main/Pictures/3DModel/3D_model2.PNG)

### LED Boards

No project is complete without some LEDs. The drone uses small PCBs with two LEDs (**red** and **white**) that indicate system state. They are mounted at the ends of the landing legs and on the top tubes. Although the boards include a mounting hole for a screw, **adhesive is recommended** for attachment.

![LED PCB](https://github.com/TilenTinta/Amon_Lander/blob/main/Pictures/PCB/PCB_LED1.PNG)

---

## Additional Repositories

This project was originally a single repository, but due to complexity it is now split across multiple repos. Click the names below to open each repository:

- **Amon Lander (this repo):** frame, test and control software, data.
- **[Amon Board][amon-board]:** flight controllers and firmware.
- **[Amon Link][amon-link]:** PC–drone communication (PCB and firmware).
- **[Amon Ground Control][amon-gcs]:** PC software for communicating with the drone.

---

[amon-board]: https://github.com/TilenTinta/Amon_Board
[amon-link]:  https://github.com/TilenTinta/Amon_Link
[amon-gcs]:   https://github.com/TilenTinta/Amon_Ground_Control

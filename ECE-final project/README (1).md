# Vehicle Control Unit (VCU) Firmware – Ben-Gurion Racing

## 📌 Project Overview
This repository contains the ECU/VCU firmware developed as part of the **Ben-Gurion Racing (BGRacing) Formula Student** program.  
The project was designed and implemented as a **final-year engineering project** at **Ben-Gurion University of the Negev**.

The ECU code was deployed in the team’s **electric Formula Student race car**, which successfully competed at **Formula Student France 2025**.  
🏆 **BGRacing was awarded the “Best Spirit Team” award at FS France.**

---

## 🚗 Purpose
The firmware provides a **real-time communication and control platform** for a four-motor electric race car.  
It is responsible for:
- Managing the **finite state machine (FSM)** for safe startup and driving operation.  
- Coordinating **four AMK inverters** via dual CAN buses.  
- Processing **pedal, brake, and steering sensors**, along with dashboard signals.  
- Running **torque vectoring, safety checks, and shutdown logic**.  
- Detecting system **faults and handling errors automatically** to ensure safe vehicle operation.  

---

## ⚙️ Technical Highlights
- **Hardware**: STM32F4 microcontroller (single VCU).  
- **Buses**:  
  - Dual **CAN** (1 Mbit/s) for inverter control and sensors.  
  - **UART** used for debug messages.  
- **Software Architecture**:
  - Modular, layered design with **database-driven communication**.  
  - **FSM** implementation handling initialization, R2D (Ready-to-Drive), inverter startup, and driving modes.  
  - **Queue-based message handling** with DMA acceleration for minimal latency.  
  - **Hash-table dispatch** for fast CAN frame parsing.  
- **Safety & Error Management**:
  - Sensor plausibility checks (APPS, BPPS, SWPS, BIOPS).  
  - Brake light and buzzer control.  
  - Watchdog timers and keep-alive monitoring for all nodes.  
  - Automatic **fault detection, logging, and recovery mechanisms** to maintain safe operation.  

---

## 📂 Repository Structure
- `FSM.c / FSM.h` – Vehicle state machine (Stages 1–3).  
- `operators.c / operators.h` – High-level operations (LEDs, buzzer, sensors, safety).  
- `inverters.c / inverters.h` – CAN communication with 4 AMK inverters.  
- `database.c / DbSetFunctions.c` – Centralized system database and setters.  
- `utils.c` – Queue implementation for communication buffers.  
- `callbacks.c` – Protocol callback routing and database integration.  
- `can.c, uart.c, spi.c, tim.c, adc.c` – Low-level drivers.  
- `platform.c` – Platform abstraction for handlers and callbacks.  
- `main.c` – System entry point, initialization, and main loop.  

---

## 🏁 Competition Context
This firmware was field-tested on the BGRacing electric race car during **Formula Student France 2025**.  
- Successfully managed **real-time inverter control** and **sensor integration**.  
- Included robust **error detection and management routines**, ensuring stable operation during competition.  
- Contributed to the team winning the **“Best Spirit Team”** award.  

---

## 👥 Credits
- **Students**: Tav Sheli & Yair Yamin  
- **Supervisor**: Dr. Igal Bilik  
- **Team**: Ben-Gurion Racing, Faculty of Engineering, BGU  
- **Year**: 2025  

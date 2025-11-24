# Embedded RL Air Quality Controller  
### (XIAO ESP32S3 + CCS811 — *CCS811 ONLY, NO OTHER SENSORS*)

A fully embedded Reinforcement Learning (RL) air-quality controller running on the **Seeed XIAO ESP32S3 Sense**, using **only a CCS811** gas sensor.  
The system monitors eCO₂ and TVOC levels and uses an on-device RL agent to choose optimal ventilation actions that maintain target air quality, reduce power usage, and adapt over time — without cloud connectivity or additional sensors.

---

## Table of Contents
- [Project Overview](#1-project-overview)
- [Hardware Requirements](#2-hardware-requirements)
- [Software & Dependencies](#3-software--dependencies)
- [Reinforcement Learning Details](#4-reinforcement-learning-details)
  - [State Vector](#state-vector)
  - [Action Space](#action-space)
  - [Model Integration](#model-integration-run_rl_agent)
- [Setup and Wiring](#5-setup-and-wiring)
- [Operation Flow](#6-operation-flow)

---

## 1. Project Overview

This project demonstrates how a **Reinforcement Learning (RL) agent** can run directly on a **tiny embedded microcontroller**, using only a CCS811 air-quality sensor as input.  
The goal is to build a **self-learning, autonomous ventilation controller** that improves air quality while minimising power consumption.

With **no secondary sensors**, the system relies entirely on:

- CCS811 **eCO₂** readings  
- CCS811 **TVOC** readings  
- Optional time/context derived internally  

Despite lacking temperature/humidity compensation hardware, the RL agent learns relationships between CCS811 output patterns and actuator outcomes over time, enabling smarter behaviour than simple threshold logic.

This provides a minimal, low-component-count, ultra-efficient embedded AI system.

---

## 2. Hardware Requirements

Built around the **Seeed XIAO ESP32S3 Sense** with **CCS811 only**.

| Component | Function | Pins Used | Notes |
|----------|----------|-----------|-------|
| **Seeed XIAO ESP32S3 Sense** | RL inference + actuator control | — | Small, powerful, ideal for TinyML |
| **CCS811 eCO₂/TVOC Sensor** | Gas sensor (PRIMARY SENSOR) | SDA (6), SCL (5) | Only sensor used |
| **Actuator** | Fan / relay / vent controller | Digital/PWM (e.g., 7, 8) | Controlled by RL agent |

No additional temp/humidity sensor is used.

---

## 3. Software & Dependencies

Install in Arduino IDE:

- `Wire.h` — I²C communication  
- `Adafruit_CCS811` — CCS811 hardware driver  
- **TensorFlow Lite for Microcontrollers** (TFLM) — for running RL model inference  

`run_rl_agent()` performs inference or placeholder logic.

---

## 4. Reinforcement Learning Details

### State Vector

Because ONLY the CCS811 is present, the state vector is simplified:

STATE_VECTOR_SIZE = 3

yaml
Copy code

| Index | Parameter | Unit | Purpose |
|-------|-----------|------|---------|
| 0 | eCO₂ | ppm | Primary IAQ metric |
| 1 | TVOC | ppb | Secondary IAQ metric |
| 2 | Time Since Boot | Minutes | Learned context (optional) |

The RL model learns trends even without temperature/humidity compensation, because:

- CCS811 readings drift with environment  
- RL adapts to patterns over time  
- Actions influence airflow and therefore sensor behaviour  

This yields a surprisingly effective control loop in minimal configurations.

---

### Action Space

The RL agent selects from **3 discrete actions**:

ACTION_SPACE_SIZE = 3

yaml
Copy code

| Action ID | Description | Effect |
|-----------|-------------|--------|
| **0** | Do Nothing | Fan/actuator off |
| **1** | Low Ventilation | PWM ~30% |
| **2** | High Ventilation | PWM ~80% |

The agent’s goal is to keep eCO₂/TVOC within acceptable levels using the **least amount of energy**.

---

### Model Integration (`run_rl_agent`)

`run_rl_agent(float* current_state)` performs:

1. Normalising the 3-element state  
2. Running TFLite Micro inference  
3. Returning the chosen action  

Until a model is trained, placeholder logic may be used.

---

## 5. Setup and Wiring

### I²C Connections (CCS811 Only)

| XIAO Pin | Signal | CCS811 Pin |
|----------|--------|-------------|
| 6 | SDA | SDA |
| 5 | SCL | SCL |
| 3.3V | VCC | VCC |
| GND | Ground | GND |

### Actuator Wiring

Typical airflow actuator (fan via MOSFET or relay):

- PWM pin → MOSFET gate  
- Fan → MOSFET drain  
- Flyback diode (if inductive load)  

Control is implemented in `execute_action()`.

---

## 6. Operation Flow

### **Startup**
- Initialise I²C and CCS811  
- Sensor warm-up (~20 seconds mandatory)  
- RL agent enters active mode  

### **Main Loop (10-second cycle)**

1. Read eCO₂ + TVOC from CCS811  
2. Construct 3-element state vector  
3. Call RL inference:  
   ```cpp
   int action = run_rl_agent(state_vector);
Apply action via PWM / GPIO

Log all values through Serial

This structure supports:

On-device RL

Offline RL dataset generation

Live environment control

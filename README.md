# Embedded RL Air Quality Controller  
### (XIAO ESP32S3 + CCS811)

An embedded Reinforcement Learning (RL) system running entirely on the **Seeed XIAO ESP32S3 Sense**, paired with a **CCS811** gas sensor for eCO₂/TVOC monitoring.  
The RL agent selects optimal environmental control actions—such as adjusting ventilation airflow—to proactively maintain indoor air quality in a more efficient, predictive manner than traditional rule-based systems.

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

This project showcases how a **trained Reinforcement Learning agent** can run on a **tiny microcontroller** such as the ESP32-S3 to perform intelligent, adaptive environmental control.  
Whereas most embedded systems rely on simple thresholds (“if eCO₂ > X then turn fan on”), RL learns a **policy** that can account for dynamic trends and subtle relationships, such as:

- gradual changes in air quality  
- temperature & humidity influence on CO₂ readings  
- daily patterns or usage behaviour  
- energy-saving trade-offs  
- actuator wear or inefficiency  
- context from previous states  

The system becomes **predictive**, **adaptive**, and **energy-efficient**, even within the constraints of a low-power embedded platform.

This framework is suitable for smart ventilation, micro-climate control, appliance optimisation, and ultra-low-power intelligent sensing applications.

---

## 2. Hardware Requirements

Built around the **Seeed XIAO ESP32S3 Sense**.

| Component | Function | Pins Used | Notes |
|----------|----------|-----------|-------|
| **Microcontroller** | Seeed XIAO ESP32S3 Sense | — | Small footprint, dual-core, onboard sensors |
| **Air Quality Sensor** | CCS811 (eCO₂/TVOC) | I2C → SDA (6), SCL (5) | Monitors gas levels with compensation support |
| **Environmental Sensor** | BME280 or SHT30 | I2C → SDA (6), SCL (5) | Provides Temp/Humidity for CCS811 compensation |
| **Actuator** | Relay / MOSFET / Fan driver | Digital/PWM 7, 8 | Controls ventilation or airflow |

---

## 3. Software & Dependencies

Install the following Arduino libraries:

- `Wire.h` — I²C communication  
- `Adafruit_CCS811` — CCS811 driver  
- `Adafruit_Sensor` + `BME280` / `SHT30` — optional but recommended  
- **TensorFlow Lite for Microcontrollers** — required for real RL inference  
- `run_rl_agent()` manages inference (a placeholder model is included)

---

## 4. Reinforcement Learning Details

### State Vector

The RL agent operates on a **5-element state vector**:

STATE_VECTOR_SIZE = 5


| Index | Parameter | Unit | Meaning |
|-------|-----------|------|---------|
| 0 | eCO₂ | ppm | Primary indicator of air quality |
| 1 | TVOC | ppb | VOC measurement |
| 2 | Temperature | °C | Compensation + environment context |
| 3 | Humidity | %RH | Compensation + environment context |
| 4 | Time Since Boot | Min | Temporal context for long-term behaviour |

---

### Action Space

The system defines **3 discrete actions**:

ACTION_SPACE_SIZE = 3


| Action ID | Description | Effect |
|-----------|-------------|--------|
| **0** | Do Nothing | Fan off, conserve energy |
| **1** | Fan Low | PWM ≈ 30% |
| **2** | Fan High | PWM ≈ 80% |

---

### Model Integration (`run_rl_agent`)

`run_rl_agent(float* current_state)` performs:

1. **State normalisation**  
2. **TFLite Micro interpreter execution**  
3. **Q-value extraction**  
4. **Best action selection**  

The provided example includes **simple rule-based logic**—replace this with your TensorFlow Lite RL model for full deployment.

---

## 5. Setup and Wiring

### I2C Wiring

Both sensors share the same I²C bus:

| XIAO Pin | Signal | Connect To |
|----------|--------|-------------|
| 6 | SDA | CCS811 SDA |
| 5 | SCL | CCS811 SCL |
| 3.3V | VCC | CCS811 VCC |
| GND | GND | CCS811 GND |

### Actuator Output

Controlled in `execute_action()` using PWM:

| Action | PWM Output | Behaviour |
|--------|------------|-----------|
| 0 | 0/255 | Fan off |
| 1 | 77/255 | Low speed |
| 2 | 200/255 | High speed |

---

## 6. Operation Flow

### Setup Phase
- Initialise I²C  
- Start CCS811 and optional Temp/Humidity sensor  
- Wait ~20s warm-up time  

### Loop (every 10 seconds)
1. Read eCO₂, TVOC, Temp, Humidity  
2. Construct the **5-element state vector**  
3. Evaluate RL model:  
   ```cpp
   int action = run_rl_agent(state_vector);

Apply the action:

execute_action(action);


Output readings + action over Serial for monitoring or dataset collection

This loop is suitable for:

closed-loop control

offline RL training data generation

testing new RL architectures

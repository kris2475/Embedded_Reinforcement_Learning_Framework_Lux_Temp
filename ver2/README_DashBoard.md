# 💡 ESP32 EmbeddedRL Illuminance Control Unit (ICCU) with Q-Learning  
*Illuminance as a Proxy for Printed Heater Control in RVs*

## Project Summary

This project implements a complete **Q-Learning** framework on an **ESP32 microcontroller** to create an Illuminance Control Unit (ICCU). The system maintains a **Target Illuminance (300 Lux)** while minimizing energy usage through learned control actions.

Illuminance is used here as a **clean and responsive proxy** for real environmental control. The exact same Embedded RL architecture applies directly to:

- Printed RV heaters  
- Underfloor heating  
- Any PWM-driven energy system requiring adaptive control  

A fully embedded, single-page dashboard visualizes Q-Table evolution, actions, rewards, and states in real time.

---

## 1. System Overview and Core Concept

The agent solves a classic embedded control problem:

> **Maintain a target (300 Lux) with the minimum possible energy cost.**  
> When applied to RV heating, the same RL logic maintains a temperature target using printed heater PWM control.

### System Components

| Component | Role | Details |
|----------|------|---------|
| **Goal** | Maintain 300 Lux | Illuminance is the proxy target; identical RL architecture works for heaters. |
| **Algorithm** | Q-Learning | Model-free RL; learns optimal actions per state. |
| **Hardware** | ESP32 | Runs RL loop + web dashboard. |
| **Sensor** | Simulated BH1750 | Continuous illuminance readings. |
| **Actuators** | Simulated PWM | Equivalent to heater PWM in RV systems. |

---

## 2. Q-Learning Cycle (Every 5 Seconds)

Each iteration includes sensing, rewarding, updating the Q-Table, selecting the next action, and executing it.

| Step | Description |
|------|-------------|
| **Sense (S)** | Read current Lux and discretize into state index (`S0`–`S8`). |
| **Reward (R)** | Reward = closeness to 300 Lux − energy cost of action. |
| **Learn** | Update Q(S,A) using TD error. |
| **Select (A')** | Epsilon-greedy: exploit the best action or explore. |
| **Execute** | Apply PWM action (`LIGHT+`, `LIGHT-`, `IDLE`). |

### Q-Learning Equation

```
Q(S, A) ← Q(S, A) + α [ R + γ * max(Q(S', A')) – Q(S, A) ]
```

---

## 3. Dashboard & Flash Streaming

The dashboard is a single-page HTML/CSS/JS file stored in **PROGMEM**. Large files typically fail on ESP32 due to memory limits.

### Memory-Safe Streaming Solution

- **`sendLargeHtmlFromProgmem()`** streams HTML directly from Flash.  
- Sends data in **512-byte chunks** to avoid RAM exhaustion.  
- Prevents watchdog resets and ensures stable delivery.

**Expected Serial output:**

```
HTML stream complete. Connection closed.
```

---

## 4. Dashboard Interpretation

### A. Sensor & Status

| Element | Meaning |
|---------|---------|
| **Lux** | Raw illuminance reading (proxy for environment). |
| **Gauge** | Shows deviation from the 300 Lux target. |
| **Reward** | Range: `-1.0` to `+1.0`; higher = closer to target with less energy. |
| **State Index (S0–S8)** | Discretized illuminance state. |
| **Action** | RL selected action: `LIGHT+`, `LIGHT-`, `IDLE`. |

(*In RV heating, these map directly to PWM heater adjustments.*)

---

### B. State Space (S0–S8)

| State | Lux Range | Meaning |
|-------|-----------|---------|
| **S0** | `< 50` | Very dark. |
| **S4** | `250–350` | Target zone. |
| **S8** | `> 600` | Over-bright environment. |

These bins mirror the discretization approach for temperature states when controlling heaters.

---

### C. Q-Table Heatmap

The Q-Table shows what the agent has learned.

- **Rows:** States (`S0–S8`)  
- **Columns:** Actions (`LIGHT+`, `LIGHT-`, `IDLE`)  
- **Cell Value:** Expected cumulative reward  
- **Green = Good**, **Red = Bad**  
- **Blue Star (★):** Greedy (best) action  
- **Highlighted Row:** Current state  

---

### D. RL Log Events

- `EXPLORE:` — random exploratory action  
- `EXPLOIT:` — greedy (highest Q-value) action  
- `LEARN: [CRUCIAL UPDATE]` — large TD error; big Q-value change  
- `LEARN: [POLICY CHANGE: XXX]` — greedy action changed → the agent learned a **new policy**

---

## Notes

- Illuminance is a **training proxy** because it reacts instantly and is easy to simulate.  
- The entire RL pipeline transfers directly to **printed heater control** in RVs.  
- PWM actions, discretization, reward design, and policy learning remain identical.

---

## Future Extensions

- Replace simulated sensors with BH1750 hardware  
- Add heater PWM output for real RV testing  
- Implement persistent Q-Table storage  
- Add dynamic environmental disturbances (sunlight, door events)

---



# 💡 Embedded Reinforcement Learning Framework: **Lux-Temp** 🌡️
**Adaptive On-Device Control for Light & Temperature Optimization**

📦 **Repository:**  
https://github.com/kris2475/Embedded_Reinforcement_Learning_Framework_Lux_Temp

---

## 🌟 What is Embedded Reinforcement Learning (RL)?

Traditional embedded systems rely on static, rule-based logic  
(e.g., “If Temp > 75°F, turn on fan”).

**Reinforcement Learning (RL)** replaces rigid rules with *adaptive behavior*.  
An **agent** (the device’s control logic) interacts with an **environment**  
(the physical world). It takes actions, receives rewards or penalties, and  
learns a **policy**—a strategy that maps states to optimal actions over time.

- **Policy (π):** Mapping from observed states → actions  
- **Value Function:** Predicts long-term reward (Q-values or V-values)

**Embedded RL** brings these capabilities directly to constrained hardware  
(MCUs, SBCs), enabling devices to adapt locally with minimal cloud dependence.

**Lux-Temp** applies Embedded RL to optimize environmental comfort  
(Lux + Temp) while minimizing energy consumption—problems too dynamic for  
traditional rule-based systems.

---

## ⛰️ The Core Challenge: RL on the Edge

| Constraint Category | Impact |
|--------------------|--------|
| ⚡ **Power** | MCU must keep inference ultra-fast to minimize active CPU time. |
| 💾 **Memory** | Only KB–small MB available → tiny policy networks required. |
| ⏱️ **Latency** | Control loops require microsecond-level execution. |
| 📉 **Data Input** | Low-bandwidth sensors demand high sample efficiency. |

---

## 🔄 The RL Loop: Why MCUs Struggle

1. **Observation Processing** — Sensor readings (Lux, Temp) → normalized state vectors.  
2. **Policy Inference** — Most computationally expensive (many MAC operations).  
3. **Experience Storage** — Store (State, Action, Reward, Next State) tuples.  
4. **Policy Update** — Gradient descent & backprop (very compute-heavy).

This motivates **quantization**, **compression**, and **minimalist RL algorithms**.

---

## 🧠 Adapting RL for Embedded Devices

### 1️⃣ Tabular Q-Learning – Minimalist RL

- Stores Q-values in a simple table.  
- Decision = pick action with highest Q-value.  
- Occasional random action via ε-greedy exploration.

**Update Rule:**
```
Q(s,a) ← Q(s,a) + α [ r + γ max_a’ Q(s’,a’) − Q(s,a) ]
```

✓ Ultra-low memory  
✓ Integer math only  
✓ Fastest possible RL on MCUs  

---

### 2️⃣ Neural Network Compression & Pruning

- Shrink network width/depth  
- Remove low-impact weights  
- Reduce MAC operations + RAM usage  

---

### 3️⃣ Quantization – MCU Efficiency Booster

Converts FP32 → INT8.

**Linear quantization:**
```
r ≈ S (q − Z)
```

Enables fast integer-only inference.

Handled via **Quantization-Aware Training (QAT)** to minimize accuracy loss.

---

### 4️⃣ Training Strategies

| Mode | Benefits | Drawbacks |
|------|----------|-----------|
| **Off-Device Training** | Minimal MCU load | No long-term adaptation |
| **On-Device Training** | Lifelong learning | High RAM/power demands |

---

## 💻 Lux-Temp Framework Architecture

### 🧩 Components

#### **1. RL Agent Library**
Tiny DQN / Tabular Q-Learning optimized for embedded devices.

#### **2. Sensor Abstraction Layer**
Reads & normalizes Lux/Temp sensors → quantized state vector.

#### **3. Inference Engine**
Integer-optimized policy execution (TF-Lite Micro or custom).

#### **4. Control Actuator Interface**
Translates actions → PWM, relays, dimming controls, etc.

---

## 🎯 Result

Lux-Temp enables:

- Adaptive, efficient comfort control  
- Energy-optimized decision-making  
- Real-time inference on low-power microcontrollers  
- Local learning without the cloud  

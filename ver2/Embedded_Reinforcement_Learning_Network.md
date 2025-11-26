# 💡 Embedded Reinforcement Learning Framework: **Lux-Temp** 🌡️
**Adaptive On-Device Control for Light & Temperature Optimization**

📦 **Repository:**  
https://github.com/kris2475/Embedded_Reinforcement_Learning_Framework_Lux_Temp

---

## 🧩 A Simple, Layman’s Introduction to RL (Agents, States & Q-Tables)

Before diving into the technical details, here’s a plain-English explanation of the core ideas:

### 👤 **Agent**
Think of the **agent** as the “brain” inside the device.  
It decides what to do based on what it senses.

Example:  
The device reads the room’s temperature → the agent decides whether to turn the fan up, down, or keep it the same.

---

### 🌍 **Environment**
The **environment** is simply the real world the agent interacts with:  
the room, the lights, the temperature, the user’s habits, and so on.

---

### 🧭 **State**
A **state** is what the agent sees **right now**.

Examples of states:
- “Room is bright + temperature is cold”
- “Room is dim + temperature is hot”

You can think of the *state* as a snapshot of the environment.

---

### 🎮 **Action**
An **action** is what the agent *does* in response to the state.

Examples of actions:
- Increase brightness  
- Decrease brightness  
- Increase temperature  
- Do nothing  

---

### 🍬 **Reward**
A **reward** is how we tell the agent whether it made a good decision.

Examples:
- If the environment moves closer to user comfort → give **positive reward**
- If it wastes energy or makes comfort worse → give **negative reward**

Over time, the agent learns to make choices that maximize rewards.

---

### 📊 **Q-Table (Like a Cheat Sheet of Experience)**

A **Q-Table** is the simplest possible “memory” an agent uses to remember what works and what doesn’t.  

Imagine a grid:

- Rows = possible **states**
- Columns = possible **actions**
- Each cell = a score (a **Q-value**) representing how good that action is in that state

The agent looks up the current state’s row and picks the action with the highest score—just like checking a cheat sheet.

As the agent gains experience, it updates the table’s numbers so it gets smarter over time.

This method is incredibly simple and fast, making it ideal for tiny microcontrollers.

---

## 🔍 Detailed, Step-by-Step Walkthrough — **Lux-Temp Use Case**

Below is a concrete, stepwise walkthrough of how an embedded RL system like Lux-Temp operates in the real world. It preserves the earlier layman explanations and then adds technical detail for each stage so you can see exactly what happens on-device, why it matters, and how constraints are managed.

### Scenario
Goal: Keep the room comfortable while using as little energy as possible. The device can:
- measure **Lux** (light level) and **Temp** (temperature),
- dim or brighten lights (PWM),
- adjust heating/cooling (relay or fan control).

Device hardware: small MCU with limited RAM (tens to hundreds of KB), low-power CPU, basic ADC/I2C sensors for Lux & Temp, PWM outputs for lights, GPIO/relay for HVAC control.

---

### 1. Initialization (Boot & Setup)

What happens:
- MCU boots, initializes sensors (e.g., ADC or I²C for light and temperature sensors).  
- Load policy representation from flash: could be a small Q-Table (array) or a quantized tiny DQN (INT8 weights + metadata).  
- Initialize hyperparameters stored in flash: learning rate (α), discount factor (γ), epsilon schedule for exploration, reward weights for comfort vs. energy.

Why it matters:
- All initial data must fit in flash/RAM. Keep metadata tiny and precompute index maps for discrete states.
- If on-device learning is used, initialize an experience buffer with a set capacity fine-tuned to available RAM.

---

### 2. Sensing & Observation Processing (State Construction)

What happens (every control cycle):
- Sample sensors at scheduled intervals (e.g., once per 10s or triggered by events).  
- Raw readings are scaled and discretized into the agent’s state representation. Example discretization:
  - Lux: {Dark, Dim, Normal, Bright} → map ADC range into 4 discrete bins.
  - Temp: {Cold, Cool, Comfortable, Warm, Hot} → map temperature range into 5 bins.
- Construct discrete state index: `state_idx = lux_bin * N_temp_bins + temp_bin` (one integer index into Q-Table).

Why it matters:
- Discretization compresses continuous signals into small state space sizes so a Q-Table remains feasible.
- Precomputing bin thresholds and using integer math avoids floating-point arithmetic every cycle (saves power).

---

### 3. Decision Making — Inference (Choose Action)

Two possible policy types and how inference is done:
- **Tabular Q-Learning (Q-Table):**
  - Read row `Q[state_idx]` of length `A` (actions count).  
  - With probability `ε`, pick a random action (exploration). Otherwise pick `argmax_a Q[state_idx, a]` (exploitation).
  - This is a single memory read + a short max operation — extremely cheap.
- **Quantized Tiny DQN:**
  - Convert discrete or continuous state into a tiny input vector (already quantized).  
  - Run INT8 inference: a few matrix multiplies using integer MACs.  
  - Pick action from output logits (again with ε-greedy exploration).

Why it matters:
- Tabular Q is literally the fastest and lowest-power inference possible: memory lookup + compare.
- Quantized DQN trades a bit more compute for generalization when state space is large, but still runs integer math for efficiency.

---

### 4. Actuation (Apply Action)

What happens:
- Translate action → hardware command:
  - For lights: update PWM duty cycle (0–255 integer) via timer registers.  
  - For HVAC/fans: toggle relay or set fan PWM.  
- Optionally apply smoothing (ramp up/down) to avoid sudden jumps that annoy users or damage actuators.

Why it matters:
- The agent’s chosen action must be converted to safe, real-world commands. Timing and jitter control are important on an MCU.
- Actuator commands are small integer writes and must complete quickly to minimize wake time.

---

### 5. Reward Calculation (Immediate Feedback)

What happens:
- Immediately after actuation (or after a short evaluation window), compute a reward scalar `r` that balances:
  - **Comfort component:** how close Lux & Temp are to user-preferred ranges.
  - **Energy component:** estimated energy usage from action (e.g., higher PWM => higher energy penalty).
  - Combine: `r = w_comfort * comfort_score − w_energy * energy_penalty` (weights chosen offline and stored in flash).

Examples:
- If action reduces temperature error and lowers light power -> positive reward.
- If action increases energy use without improving comfort -> negative reward.

Why it matters:
- Careful reward engineering is crucial; poorly designed rewards create undesired behaviors.
- Keep reward computation integer-friendly (scaled integers) on-device.

---

### 6. Experience Storage (Optional — For Learning)

What happens:
- If on-device learning is enabled, store the transition `(s, a, r, s')` into a circular buffer (experience replay) sized for available RAM.
- For strict memory environments, use very small buffers (e.g., 16–256 transitions) or no buffer at all (online TD updates without replay).

Why it matters:
- Replay buffers stabilize learning but cost RAM. Choose buffer strategy depending on memory/power constraints.

---

### 7. Policy Update (Learning Step - If Enabled)

What happens:
- **Tabular Q-Learning:** apply TD update immediately (online):
  ```
  Q(s,a) += α * ( r + γ * max_a' Q(s', a') - Q(s,a) )
  ```
  - Uses integer arithmetic, constant time, cheap on MCU.
- **Tiny DQN (On-Device):**
  - Periodically sample from the replay buffer and perform small gradient updates on quantized weights (requires integer-friendly optimizers or mixed precision).
  - Use QAT-style simulated quantization during training to keep weights INT8-friendly.

Why it matters:
- Tabular updates are cheap enough to run every cycle. DQN updates are expensive, so schedule them sparsely (e.g., once per hour or when plugged in).
- Learning must be throttled to avoid battery drain and heat issues.

---

### 8. Logging & Checkpointing

What happens:
- Periodically write compact checkpoints to flash:
  - Q-Table entries or quantized model weights.
  - Training metadata (epsilon, step counters).
- Optionally send periodic summaries to the cloud when connectivity and power allow (not required).

Why it matters:
- Flash writes are slow and power-hungry — batch updates and align to wear-leveling constraints.
- Checkpoints allow safe recovery and offline analysis.

---

### 9. Continual Operation & Safety

What happens:
- The loop repeats every control cycle.
- Safety checks ensure actuator commands remain within hardware limits.
- Watchdog timers and failsafe policies (e.g., revert to a safe static rule) protect against runaway behaviors.

Why it matters:
- On-device RL must be predictable and safe; always include fallbacks and bounds checks.

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


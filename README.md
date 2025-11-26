# 💡 Embedded Reinforcement Learning Framework for Lux and Temperature Control

This project presents a comprehensive **Embedded Reinforcement Learning (RL) Framework** designed to create energy-efficient and highly adaptive **Smart Building Automation Systems (SBAS)**. The framework’s core function is to deploy resource-optimized RL algorithms onto **constrained embedded systems** (e.g., low-power microcontrollers) to autonomously manage and optimize settings for key environmental variables: **Illuminance (Lux)** and **Temperature**.

The goal is to move beyond static control systems by enabling the embedded node to learn optimal operational policies directly from its environment, reducing energy waste while maintaining occupant comfort.

---

## 🏗️ Development Progress and Repository Structure

The repository is logically separated into a high-level **training environment** (`simulations`) and multiple, iterative **embedded deployment versions** (`ver1`, `ver2`, `ver3`). This structure clearly maps the progression from conceptual testing to highly optimized hardware implementation.

### 1. ⚙️ `simulations`

This folder represents the **pre-deployment phase** where the Reinforcement Learning algorithms are developed and validated.

* **Purpose:** To provide a controlled, high-level environment for training robust RL agents before porting the resulting policy or inference code to C++.
* **Content Focus:**
    * **Environment Model:** Contains the Python implementation of the building physics, simulating the thermal and lighting dynamics, occupant behavior, and actuator responses.
    * **Agent Training Scripts:** Scripts for various RL algorithms (e.g., Q-Learning, SARSA, or DQN implementations) used to explore and converge on optimal control policies.
    * **Validation Tools:** Includes plotting scripts, logging handlers, and metrics calculation to assess agent performance against baseline control systems (PID, heuristics) in terms of convergence, energy consumption, and comfort compliance.
* **Key Outcome:** A proven, stable RL policy ready for translation and integration into the embedded C++ environment.

---

### 2. ⚡ `ver1` (Initial Embedded Proof-of-Concept)

This marks the **first successful translation** of the framework's core logic into a deployable, resource-constrained environment.

* **Focus:** Establishing the **basic infrastructure** and validating the feasibility of running an RL agent on the target hardware.
* **Content Focus:**
    * **Basic RL Implementation:** Typically houses the C/C++ code for a simple tabular RL algorithm (like **Q-Learning**) with a highly discretized state-action space, prioritizing computational simplicity.
    * **Hardware Abstraction Layer (HAL):** Initial code for interfacing directly with sensors (e.g., I2C lux/temp sensors) and actuators (e.g., PWM dimmers, relay controls).
    * **Minimalist Codebase:** Emphasis on core RL loop functionality with minimal memory optimization or power-saving features.
* **Progress Highlight:** **Successful execution** of the learning loop on the MCU, demonstrating the ability to take measurements, calculate rewards, and update the Q-table/policy in real-time.

---

### 3. 🚀 `ver2` (Optimization and Algorithm Refinement)

Building upon the functional foundation of `ver1`, this version focuses on improving efficiency and utilizing more sophisticated algorithms.

* **Focus:** **Computational and Memory Optimization**. Addressing the constraints identified in `ver1` and exploring more advanced RL techniques that remain viable on low-power hardware.
* **Content Focus:**
    * **Advanced RL Agent:** Implementation of an improved algorithm, potentially featuring **function approximation** (e.g., lightweight neural networks, if using a high-end MCU, or optimized tile coding) or a **more efficient sparse Q-table structure**.
    * **Optimization Techniques:** Includes low-level memory optimizations, efficient data structures, and reduced floating-point operations.
    * **Enhanced Measurement Logic:** Refinement of the sensor reading and data processing pipeline to improve accuracy and speed.
* **Progress Highlight:** Demonstrates **faster convergence** and the ability to handle a **larger state space** than `ver1`, leading to improved control performance and energy savings.

---

### 4. ⭐ `ver3` (Feature Completion and Robust Deployment)

This iteration integrates all necessary components for a robust field deployment, aligning with typical SBAS requirements.

* **Focus:** **System Robustness, Connectivity, and Power Management**. Preparing the framework for real-world, long-term operation.
* **Content Focus:**
    * **Power Management:** Integration of deep sleep modes and scheduling logic. This includes implementing the core functionality where a **PIR sensor wakes the node to take measurements**, but the **wireless transmission is handled by a separate wakeup module** (as per project constraints), ensuring energy efficiency.
    * **Wireless Communication:** Code for communication modules (e.g., Zigbee, Wi-Fi, or LoRa) to report status, receive setpoints, and enable over-the-air policy updates.
    * **Safety & Fallback:** Implementation of safety limits and non-RL fallback control policies to ensure system stability during sensor failure or unexpected environmental events.
* **Progress Highlight:** Represents the **final, highly optimized framework** suitable for industrial prototyping, featuring adaptive RL control combined with essential low-power and networking capabilities.

---

## 📋 Technology & Licensing

* **Core Languages:** C++ (Embedded), Python (Simulation)
* **License:** The project is made available under the **Apache-2.0 License**.

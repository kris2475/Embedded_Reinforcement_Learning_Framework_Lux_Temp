# Embedded Reinforcement Learning Framework for Lux and Temperature Control

This project presents a comprehensive **Embedded Reinforcement Learning (RL) Framework** designed to create energy-efficient and highly adaptive **Smart Building Automation Systems (SBAS)**. The framework’s core function is to deploy resource-optimised RL algorithms onto **constrained embedded systems** (for example, low-power microcontrollers) to autonomously manage and optimise settings for key environmental variables: **Illuminance (Lux)** and **Temperature**.

The aim is to move beyond static control systems by enabling the embedded node to learn optimal operational policies directly from its environment, thereby reducing energy waste while maintaining occupant comfort.

---

## Development Progress and Repository Structure

The repository is logically separated into a high-level **training environment** (`simulations`) and multiple, iterative **embedded deployment versions** (`ver1`, `ver2`, `ver3`). This structure clearly maps the progression from conceptual testing to highly optimised hardware implementation.

### 1. `simulations`

This folder represents the **pre-deployment phase**, where the Reinforcement Learning algorithms are developed and validated.

* **Purpose:** To provide a controlled, high-level environment for training robust RL agents before porting the resultant policy or inference code to C++.  
* **Content Focus:**  
    * **Environment Model:** Contains a Python implementation of the building physics, simulating thermal and lighting dynamics, occupant behaviour, and actuator responses.  
    * **Agent Training Scripts:** Scripts for a variety of RL algorithms (for instance, Q-Learning, SARSA, or DQN implementations) used to explore and converge on optimal control policies.  
    * **Validation Tools:** Includes plotting scripts, logging handlers, and metrics calculations to assess agent performance against baseline control systems (such as PID controllers or heuristic-based methods), in terms of convergence, energy consumption, and comfort compliance.  
* **Key Outcome:** A proven, stable RL policy ready for translation and integration into the embedded C++ environment.

---

### 2. `ver1` (Initial Embedded Proof-of-Concept)

This marks the **first successful translation** of the framework's core logic into a deployable, resource-constrained environment.

* **Focus:** Establishing the **basic infrastructure** and validating the feasibility of running an RL agent on the target hardware.  
* **Content Focus:**  
    * **Basic RL Implementation:** Typically houses the C/C++ code for a simple tabular RL algorithm (such as **Q-Learning**) with a highly discretised state–action space, prioritising computational simplicity.  
    * **Hardware Abstraction Layer (HAL):** Initial code for interfacing directly with sensors (for example, I²C lux/temperature sensors) and actuators (for example, PWM dimmers, relay controls).  
    * **Minimalist Codebase:** Emphasis on core RL loop functionality, with minimal memory optimisation or power-saving features.  
* **Progress Highlight:** Successful execution of the learning loop on the microcontroller, demonstrating the node’s ability to take measurements, compute rewards and update the Q-table or policy in real time.

---

### 3. `ver2` (Optimisation and Algorithm Refinement)

Building upon the functional foundation of `ver1`, this version concentrates on improving efficiency and employing more sophisticated algorithms.

* **Focus:** **Computational and memory optimisation**, addressing the constraints identified in `ver1` and exploring more advanced RL techniques that remain viable on low-power hardware.  
* **Content Focus:**  
    * **Advanced RL Agent:** Implementation of an improved algorithm, potentially featuring **function approximation** (such as lightweight neural networks, if using a more capable MCU, or optimised tile-coding) or a more efficient sparse Q-table structure.  
    * **Optimisation Techniques:** Includes low-level memory optimisations, efficient data structures and reduced floating-point operations.  
    * **Enhanced Measurement Logic:** Refinement of sensor reading and data-processing pipelines to improve both accuracy and execution speed.  
* **Progress Highlight:** Demonstrates **faster convergence** and the ability to manage a **larger state space** than `ver1`, leading to improved control performance and enhanced energy savings.

---

### 4. `ver3` (Feature Completion and Robust Deployment)

This iteration integrates all necessary components for a robust field deployment, aligning with typical SBAS requirements.

* **Focus:** **System robustness, connectivity and power management** — preparing the framework for real-world, long-term operation.  
* **Content Focus:**  
    * **Power Management:** Integration of deep-sleep modes and scheduling logic. This includes implementing functionality where a **PIR sensor wakes the node to take measurements**, while wireless transmission is handled by a separate wake-up module (as per project constraints), ensuring energy efficiency.  
    * **Wireless Communication:** Code for communication modules (for example, Zigbee, Wi-Fi or LoRa) to report status, receive setpoints, and permit over-the-air policy updates.  
    * **Safety & Fallback Mechanisms:** Implementation of safety limits and non-RL fallback control policies to ensure system stability during sensor failure or unexpected environmental events.  
* **Progress Highlight:** Represents the **final, highly optimised framework** suitable for industrial prototyping, combining adaptive RL control with essential low-power and networking capabilities.

---

## Technology & Licensing

* **Core Languages:** C++ (Embedded), Python (Simulation)  
* **Licence:** The project is distributed under the **Apache-2.0 Licence**.




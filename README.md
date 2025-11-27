# Embedded Reinforcement Learning (ERL) Framework Summary

This project shows the development of a Q-Learning agent from a controlled simulation to an adaptive, network-enabled microcontroller application.

---

## **Simulation:** `simulations/ERL_Simulation.py`

**What this script does:**  
This is the essential offline training and testing environment written in Python.

**Purpose:**  
It models the physical system (e.g., how the lights react to commands) and allows the developer to test and tune the core Q-Learning logic, the reward function, and key parameters (alpha, gamma, epsilon) rapidly without needing the hardware. It ensures the maths works before deployment.

---

## **Version 1:** `ver1/Embedded_Reinforcement_Learning_Framework_Lux_Temp.ino`

**What this sketch does:**  
This is the initial, fundamental hardware deployment.

**Purpose:**  
It proves that the complete Q-Learning logic can run on an embedded microcontroller (like the ESP32) using local sensor data (Lux and Temperature) and directly controlling actuators. It operates entirely in isolation with no networking or external communication.

---

## **Version 2:** `ver2/Embedded_Reinforcement_Learning_Framework_Lux_Temp_WiFi.ino`

**What this sketch does:**  
This is the network-enabled agent.

**Purpose:**  
It adds WiFi connectivity to the microcontroller. This allows the device to report its status and sensor data, enabling basic remote monitoring. This step makes initial testing and remote observation possible.

---

## **Version 3:** `ver3/Adaptive_Q_Learning_Agent_RL_Lighting_Control_Trainer_5_states.ino`

**What this sketch does:**  
This is the refined, adaptive agent.

**Purpose:**  
It specialises the agent for a specific task (lighting control) and implements Adaptive Q-Learning. This means the agent's learning parameters (like the learning rate) change dynamically to achieve faster and more robust optimisation, using a refined 5-state system.

---

## **Version 4 (Proposed):** *(Based on `Embedded_RL_Framework_logging.ino`)*

**What this sketch does:**  
This is the Robust Logging and Debugging Agent.

**Purpose:**  
It is designed to make the system ready for long-term field testing. It implements non-blocking data streaming over a dedicated network channel to prevent system crashes (Watchdog Timer resets) that occur when trying to stream large amounts of data (like the Q-table) using standard, blocking network libraries. It guarantees that valuable learning history is reliably recorded.






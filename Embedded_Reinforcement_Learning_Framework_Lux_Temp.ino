/**
 * @file lux_rl_framework.ino
 * @brief Embedded Reinforcement Learning (ERL) Q-Learning Trainer with Crucial Moment Logging.
 * *************************************************************************************************
 * *************************************************************************************************
 * * PROJECT TITLE: REINFORCEMENT LEARNING ADAPTIVE CLIMATE CONTROL UNIT (ACCU) Q-LEARNING TRAINER
 * *************************************************************************************************
 * * DESCRIPTION:
 * * This sketch implements Tabular Q-Learning with advanced logging focused on "Crucial Moments."
 * * The system logs and reports specifically on events that drive or define the learning process:
 * * 1. EXPLORATION: When the agent takes a random action (Epsilon-Greedy strategy).
 * * 2. CRUCIAL UPDATE: When the Temporal Difference (TD) Error magnitude exceeds a threshold (1.0), 
 * * indicating a surprisingly good or bad outcome, which results in a large Q-Table update.
 * * 3. POLICY CHANGE: When the best-known action for a state changes as a result of a Q-update.
 * * * * The core goal is to learn an energy-efficient policy by assigning a **negative reward penalty** * * to all active actuator actions (LIGHT+, TEMP+, etc.) and a **small bonus** to the IDLE action 
 * * when the environment is already in the target comfort zone. This forces the agent to prefer 
 * * stability and energy conservation when comfort is met.
 * * * * @note **Graceful Sensor Failure Policy**: If a sensor fails to initialize or read (e.g., Lux < 0), 
 * * the system logs a warning/error and continues the RL loop, using the `TARGET_LUX` as a neutral 
 * * fallback value for the current iteration to prevent state errors.
 * *********************************************************************************
 * * HARDWARE CONTEXT: TYPICAL ARDUINO/ESP32 EMBEDDED SYSTEM
 * *********************************************************************************
 * * 1. Microcontroller: ESP32/ESP8266/Arduino (Used to run the ERL Framework and Q-Learning).
 * * 2. Sensor 1 (I2C 0x23): BH1750 (GY-30) - Measures **Illuminance (Lux)**.
 * * 3. Sensor 2 (I2C 0x77): BMP180 - Measures **Temperature (°C)**.
 * * 4. Actuator Proxy: Console/Serial Output - Simulates the ACCU's control signals (no physical I/O implemented here).
 * *********************************************************************************
 * * ERL SPACE & Q-LEARNING CONFIGURATION
 * *********************************************************************************
 * * 1. STATE DISCRETIZATION (S_d): 3 (Lux Bins) x 3 (Temp Bins) = **9 Total Discrete States**.
 * * - LUX Bins: LOW (0-100 lx), MEDIUM (101-499 lx), HIGH (>= 500 lx)
 * * - TEMP Bins: COLD (<= 18°C), COMFORT (18.1-23.9°C), HOT (>= 24°C)
 * * * * 2. ACTION SPACE (A): **5 Discrete Actions**.
 * * 3. Q-TABLE SIZE: 9 States x 5 Actions = **45 Q-Values** (stored in memory).
 * * --------------------------------------------------------------------------------------
 * * | ACTION ID | ACTION NAME     | ACCU CONTROL INTERPRETATION                          |
 * * --------------------------------------------------------------------------------------
 * * | 0         | LIGHT+          | Increase Illumination (e.g., Turn on light, increase PWM). |
 * * | 1         | LIGHT-          | Decrease Illumination (e.g., Dim light, close blinds).     |
 * * | 2         | TEMP+           | Activate Heater/Fan to Raise Temperature.                  |
 * * | 3         | TEMP-           | Activate Cooler/Fan to Lower Temperature.                  |
 * * | 4         | IDLE            | Zero Energy Consumption. Optimal when comfort is met.      |
 * * --------------------------------------------------------------------------------------
 * * Q-Learning Hyperparameters:
 * * - LEARNING RATE (Alpha - α): 0.1 (Moderate rate; prioritizes recent experience).
 * * - DISCOUNT FACTOR (Gamma - γ): 0.9 (High; values long-term, future rewards heavily).
 * * - EXPLORATION RATE (Epsilon - ε): 0.3 (High; ensures the agent actively explores suboptimal actions).
 * * - **CRUCIAL UPDATE THRESHOLD**: 1.0 (TD Error magnitude for detailed Serial logging).
 * * * * The Bellman equation drives the learning update:
 * * $$
 * * Q(S, A) \leftarrow Q(S, A) + \alpha \times [R + \gamma \times \max Q(S') - Q(S, A)]
 * * $$
 * * * *********************************************************************************
 */
#include <Wire.h>
#include <math.h> // For round() and fabs()
#include <limits.h> // For FLT_MAX
#include <float.h> // For FLT_MAX

// --------------------------------------------------------
// --- I2C Sensor Configuration (from previous file) ---
// --------------------------------------------------------
#define SDA_PIN 43
#define SCL_PIN 44

// BH1750 / GY-30 Configuration
#define BH1750_ADDR 0x23
#define BH1750_CONT_HIGH_RES_MODE 0x10

// BMP180 Configuration
#define BMP_ADDR 0x77
#define OSS 3 // Oversampling Setting (Highest Resolution)
#define BMP_CALIBRATION_START_REG 0xAA
#define BMP_CALIBRATION_SIZE 22

// Global BMP180 Calibration Variables
int16_t bmp_AC1, bmp_AC2, bmp_AC3;
uint16_t bmp_AC4, bmp_AC5, bmp_AC6;
int16_t bmp_B1, bmp_B2, bmp_MB, bmp_MC, bmp_MD;

// --- Sensor Functions (Integrated below for completeness) ---

bool bh1750Init() {
  Wire.beginTransmission(BH1750_ADDR);
  Wire.write(BH1750_CONT_HIGH_RES_MODE);
  if (Wire.endTransmission() == 0) {
    delay(180);
    return true;
  }
  return false;
}

float bh1750ReadLux() {
  Wire.requestFrom(BH1750_ADDR, 2);
  if (Wire.available() == 2) {
    uint16_t raw = (Wire.read() << 8) | Wire.read();
    return raw / 1.2;
  }
  return -1.0;
}

bool readCalibration() {
  uint8_t buffer[BMP_CALIBRATION_SIZE];
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(BMP_CALIBRATION_START_REG);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(BMP_ADDR, BMP_CALIBRATION_SIZE) != BMP_CALIBRATION_SIZE) return false;
  
  for (int i = 0; i < BMP_CALIBRATION_SIZE; i++) buffer[i] = Wire.read();

  int i = 0;
  bmp_AC1 = (int16_t) (buffer[i++] << 8 | buffer[i++]);
  bmp_AC2 = (int16_t) (buffer[i++] << 8 | buffer[i++]);
  bmp_AC3 = (int16_t) (buffer[i++] << 8 | buffer[i++]);
  bmp_AC4 = (uint16_t)(buffer[i++] << 8 | buffer[i++]);
  bmp_AC5 = (uint16_t)(buffer[i++] << 8 | buffer[i++]);
  bmp_AC6 = (uint16_t)(buffer[i++] << 8 | buffer[i++]);
  bmp_B1  = (int16_t) (buffer[i++] << 8 | buffer[i++]);
  bmp_B2  = (int16_t) (buffer[i++] << 8 | buffer[i++]);
  bmp_MB  = (int16_t) (buffer[i++] << 8 | buffer[i++]);
  bmp_MC  = (int16_t) (buffer[i++] << 8 | buffer[i++]);
  bmp_MD  = (int16_t) (buffer[i++] << 8 | buffer[i++]);

  return true;
}

int32_t readRawTemp() {
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(0xF6);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP_ADDR, 2);
  if (Wire.available() == 2) return (Wire.read() << 8) | Wire.read();
  return 0;
}

// Note: readRawPressure and computePressure were not strictly necessary for temperature
// but are included as they were in the original context provided.
int32_t readRawPressure() {
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS << 6));
  Wire.endTransmission();
  delay(26);
  Wire.beginTransmission(BMP_ADDR);
  Wire.write(0xF6);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP_ADDR, 3);
  if (Wire.available() == 3) {
    int32_t msb = Wire.read();
    int32_t lsb = Wire.read();
    int32_t xlsb = Wire.read();
    return ((msb << 16) + (lsb << 8) + xlsb) >> (8 - OSS);
  }
  return 0;
}

int32_t computeB5(int32_t UT) {
  int32_t X1 = ((UT - bmp_AC6) * bmp_AC5) >> 15;
  int32_t X2 = (bmp_MC << 11) / (X1 + bmp_MD);
  return X1 + X2;
}

int32_t computePressure(int32_t UP, int32_t B5) {
  int32_t B6 = B5 - 4000;
  int32_t X1 = (bmp_B2 * ((B6 * B6) >> 12)) >> 11;
  int32_t X2 = (bmp_AC2 * B6) >> 11;
  int32_t X3 = X1 + X2;
  int32_t B3 = ((((int32_t)bmp_AC1 * 4 + X3) << OSS) + 2) >> 2;

  X1 = (bmp_AC3 * B6) >> 13;
  X2 = (bmp_B1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  uint32_t B4 = (bmp_AC4 * (uint32_t)(X3 + 32768)) >> 15;
  uint32_t B7 = ((uint32_t)UP - B3) * (50000 >> OSS);

  int32_t p;
  if (B7 < 0x80000000) p = (B7 << 1) / B4;
  else p = (B7 / B4) << 1;

  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  p = p + ((X1 + X2 + 3791) >> 4);
  return p;
}


// --------------------------------------------------------
// --- Reinforcement Learning Configuration (Q-Learning)----
// --------------------------------------------------------

// ----------------- STATE SPACE -----------------
// The state S is defined by two factors: LUX and TEMPERATURE, discretized into bins.

// LUX Bins: 3 Bins (LOW, MEDIUM, HIGH)
#define LUX_BIN_LOW_MAX 100.0f // Lux <= 100
#define LUX_BIN_HIGH_MIN 500.0f // Lux >= 500
#define LUX_BINS 3

// TEMP Bins: 3 Bins (COLD, COMFORT, HOT)
#define TEMP_BIN_COLD_MAX 18.0f // Temp <= 18°C
#define TEMP_BIN_HOT_MIN 24.0f // Temp >= 24°C
#define TEMP_BINS 3

// Total State Space Size (S = LUX_BINS * TEMP_BINS = 9 states)
#define STATE_SPACE_SIZE (LUX_BINS * TEMP_BINS) 

// ----------------- ACTION SPACE -----------------
// The action A is defined by controlling two theoretical actuators: Light and Fan/Heater
#define ACTION_SPACE_SIZE 5
enum Action {
  A_LIGHT_UP_FAN_OFF = 0, // Increase Light Output (e.g. by 20%) & No Temp Change
  A_LIGHT_DOWN_FAN_OFF,   // Decrease Light Output (e.g. by 20%) & No Temp Change
  A_TEMP_UP_LIGHT_OFF,    // Activate Heater/Fan to Raise Temp
  A_TEMP_DOWN_LIGHT_OFF,  // Activate Cooler/Fan to Lower Temp
  A_DO_NOTHING            // Idle (Zero energy consumption)
};
const char* actionNames[ACTION_SPACE_SIZE] = {
  "LIGHT+", "LIGHT-", "TEMP+", "TEMP-", "IDLE"
};

// ----------------- RL Parameters -----------------
#define ALPHA 0.1f       // Learning Rate (Weight given to new information)
#define GAMMA 0.9f       // Discount Factor (Importance of future rewards)
#define EPSILON 0.3f     // Exploration Rate (Epsilon-Greedy strategy)
#define CRUCIAL_TD_THRESHOLD 1.0f // Threshold for logging a "Crucial Update"

// Q-Table: Stores the expected reward for taking action 'A' in state 'S'
float Q_table[STATE_SPACE_SIZE][ACTION_SPACE_SIZE];

// Previous state and action for the Q-Learning update
int S_prev = -1;
int A_prev = -1; 

// Target Comfort Zones
#define TARGET_LUX 300.0f // Desired Illumination (e.g., medium bin)
#define TARGET_TEMP 21.0f // Desired Temperature (e.g., comfort bin)
#define REWARD_MAX_DISTANCE 10.0f // Max distance in combined state space for reward scaling


// --------------------------------------------------------
// --- Reinforcement Learning Functions -------------------
// --------------------------------------------------------

/**
 * @brief Maps the continuous sensor readings (Lux, Temp) to a discrete state index (0-8).
 * State index = (Lux_Bin * TEMP_BINS) + Temp_Bin
 * @param lux Current lux reading.
 * @param temp Current temperature reading.
 * @return int The discrete state index (0 to STATE_SPACE_SIZE - 1).
 */
int getState(float lux, float temp) {
  int lux_bin = -1;
  int temp_bin = -1;

  // Determine LUX Bin
  if (lux <= LUX_BIN_LOW_MAX) {
    lux_bin = 0; // LOW
  } else if (lux >= LUX_BIN_HIGH_MIN) {
    lux_bin = 2; // HIGH
  } else {
    lux_bin = 1; // MEDIUM
  }

  // Determine TEMP Bin
  if (temp <= TEMP_BIN_COLD_MAX) {
    temp_bin = 0; // COLD
  } else if (temp >= TEMP_BIN_HOT_MIN) {
    temp_bin = 2; // HOT
  } else {
    temp_bin = 1; // COMFORT
  }

  // Calculate the final state index
  return (lux_bin * TEMP_BINS) + temp_bin;
}

/**
 * @brief Calculates the immediate reward based on deviation from target comfort zone,
 * incorporating a small energy penalty for active actions (A_prev).
 * * The reward function is structured as: R = R_Comfort - R_Energy_Cost.
 * - R_Comfort is 1.0 - (Normalized Distance from Target), max 1.0.
 * - R_Energy_Cost penalizes active actions (-0.05) and rewards IDLE (+0.01) to encourage 
 * energy conservation when comfort is achieved.
 * * @param lux Current lux reading.
 * @param temp Current temperature reading.
 * @return float The immediate reward (R) value.
 */
float calculateReward(float lux, float temp) {
  // Simple L2-norm-based distance to target, scaled and squared for higher penalty further away.
  float lux_error = fabs(lux - TARGET_LUX) / 100.0f; // Scale lux error down
  float temp_error = fabs(temp - TARGET_TEMP); // Temp error in °C

  // Combined weighted error distance
  float distance = sqrt(pow(lux_error, 2) + pow(temp_error, 2));

  // Base reward is inversely proportional to distance, with a max reward of 1.0
  float reward = 1.0f - (distance / REWARD_MAX_DISTANCE);
  
  // --- START ENERGY COST IMPLEMENTATION ---
  
  float energy_cost = 0.0f;
  
  // Apply penalty/bonus based on the action that was just taken (A_prev)
  if (A_prev != -1) { 
      if (A_prev != A_DO_NOTHING) {
          // Penalty for any active control action to discourage unnecessary adjustments
          energy_cost = -0.05f; 
      } else {
          // Slight bonus for staying idle. This makes IDLE the long-term optimal action 
          // when in the comfort state (S4), outweighing the high, short-term rewards 
          // that active actions receive when the environment is already near-perfect.
          energy_cost = 0.01f; 
      }
  }

  reward += energy_cost;
  
  // --- END ENERGY COST IMPLEMENTATION ---

  // Clamp reward to a range (e.g., -1.0 to 1.0)
  if (reward > 1.0f) reward = 1.0f;
  if (reward < -1.0f) reward = -1.0f;

  return reward;
}

/**
 * @brief Uses the Epsilon-Greedy strategy to select the next action (A').
 * If a random number is less than EPSILON (e.g., 0.3), the agent EXPLORES.
 * Otherwise, the agent EXPLOITS, choosing the action with the maximum Q-value.
 * * @param S The current state index.
 * @return int The chosen action index.
 */
int chooseAction(int S) {
  // Determine if we explore or exploit
  if ((float)random(100) / 100.0f < EPSILON) {
    // EXPLORE: Choose a random action
    int A = random(ACTION_SPACE_SIZE);
    Serial.print("[RL-LOG] EXPLORE: Random action ");
    Serial.println(actionNames[A]);
    return A;
  } else {
    // EXPLOIT: Choose the action with the maximum Q-value for the current state
    float max_q = -FLT_MAX; 
    int best_action = 0;
    
    for (int A = 0; A < ACTION_SPACE_SIZE; A++) {
      if (Q_table[S][A] > max_q) {
        max_q = Q_table[S][A];
        best_action = A;
      }
    }
    Serial.print("[RL-LOG] EXPLOIT: Greedy action ");
    Serial.println(actionNames[best_action]);
    return best_action;
  }
}

/**
 * @brief Finds the maximum Q-value for the next state S' (S_new).
 * This max value is used in the Bellman update to estimate the expected future reward.
 * * @param S_new The next state index.
 * @return float The maximum Q-value found for state S_new.
 */
float maxQ(int S_new) {
  float max_q = -FLT_MAX;
  for (int A = 0; A < ACTION_SPACE_SIZE; A++) {
    if (Q_table[S_new][A] > max_q) {
      max_q = Q_table[S_new][A];
    }
  }
  return max_q;
}

/**
 * @brief Executes the chosen action (A) by sending a command to the environment (e.g., actuators).
 * This function serves as a simulation or placeholder for a real embedded system's I/O control.
 * @param A The action index to execute.
 */
void executeAction(int A) {
  // In a real application, this is where you control PWM, GPIO, or send serial commands.
  // This section currently uses Serial Output as an "Actuator Proxy" as defined in the header.
  switch (A) {
    case A_LIGHT_UP_FAN_OFF:
      Serial.println("[ACTUATOR] Light Output Increased.");
      // REAL HARDWARE: digitalWrite(LIGHT_PIN, HIGH); or analogWrite(LIGHT_PWM, 255);
      break;
    case A_LIGHT_DOWN_FAN_OFF:
      Serial.println("[ACTUATOR] Light Output Decreased.");
      // REAL HARDWARE: digitalWrite(LIGHT_PIN, LOW); or analogWrite(LIGHT_PWM, 0);
      break;
    case A_TEMP_UP_LIGHT_OFF:
      Serial.println("[ACTUATOR] Heater/Fan ON (Temp Up).");
      // REAL HARDWARE: digitalWrite(HEATER_PIN, HIGH);
      break;
    case A_TEMP_DOWN_LIGHT_OFF:
      Serial.println("[ACTUATOR] Cooler/Fan ON (Temp Down).");
      // REAL HARDWARE: digitalWrite(COOLER_PIN, HIGH);
      break;
    case A_DO_NOTHING:
      Serial.println("[ACTUATOR] IDLE (No Change).");
      // REAL HARDWARE: Ensure all actuator pins are in the lowest power state.
      break;
  }
}

/**
 * @brief The core Q-Learning update step (Bellman Equation implementation).
 * Calculates the Temporal Difference (TD) error and updates the Q-value for the 
 * (S_prev, A_prev) pair.
 * * Update Rule: Q(S,A) <- Q(S,A) + alpha * [R + gamma * maxQ(S') - Q(S,A)]
 * * @param S_prev Previous state.
 * @param A_prev Previous action.
 * @param R Current reward.
 * @param S_new Current (new) state.
 */
void updateQTable(int S_prev, int A_prev, float R, int S_new) {
  if (S_prev < 0 || A_prev < 0) {
    // Skip update for the very first iteration
    return;
  }
  
  // Find the maximum expected future reward for the new state S_new
  float max_q_new = maxQ(S_new);
  
  // Calculate the Temporal Difference (TD) Error
  float old_q = Q_table[S_prev][A_prev];
  float future_expected_reward = GAMMA * max_q_new;
  float td_error = R + future_expected_reward - old_q;
  
  // Log the detailed update calculation for transparency
  Serial.println("[RL-LEARN] Q-Update Calculation:");
  Serial.print("  Q(S"); Serial.print(S_prev); Serial.print(", "); Serial.print(actionNames[A_prev]); Serial.print(") <- "); 
  Serial.print(old_q, 4); Serial.print(" + "); Serial.print(ALPHA, 2); Serial.print(" * [");
  Serial.print(R, 4); Serial.print(" (R) + "); Serial.print(future_expected_reward, 4); Serial.print(" (γ*maxQ') - ");
  Serial.print(old_q, 4); Serial.print(" (Q_old)]");
  Serial.print("\n  -> TD Error (Target - Old Q): "); Serial.println(td_error, 4);
  
  // Check for a policy change (pre-update best action)
  int old_best_action = -1;
  float old_max_q = -FLT_MAX;
  for(int a=0; a<ACTION_SPACE_SIZE; a++){
      if(Q_table[S_prev][a] > old_max_q){
          old_max_q = Q_table[S_prev][a];
          old_best_action = a;
      }
  }

  // Update the Q-value
  float new_q = old_q + ALPHA * td_error;
  Q_table[S_prev][A_prev] = new_q;
  Serial.print("  -> New Q-Value: "); Serial.println(new_q, 4);
  
  // Check for new best action after update
  int new_best_action = -1;
  float new_max_q = -FLT_MAX;
  for(int a=0; a<ACTION_SPACE_SIZE; a++){
      if(Q_table[S_prev][a] > new_max_q){
          new_max_q = Q_table[S_prev][a];
          new_best_action = a;
      }
  }
  
  // --- Crucial Moment Logging ---
  if (fabs(td_error) > CRUCIAL_TD_THRESHOLD) {
      Serial.print("[CRUCIAL UPDATE] TD Error of ");
      Serial.print(td_error);
      Serial.print(" led to large Q-update in S");
      Serial.print(S_prev);
      Serial.print(", A");
      Serial.println(A_prev);
  }
  
  if (old_best_action != new_best_action) {
      Serial.print("[POLICY CHANGE] State S");
      Serial.print(S_prev);
      Serial.print(": Best action changed from ");
      Serial.print(actionNames[old_best_action]);
      Serial.print(" to ");
      Serial.println(actionNames[new_best_action]);
  }
}

/**
 * @brief Initializes the Q-Table with zeros.
 */
void initQTable() {
  Serial.println("[RL-SETUP] Initializing Q-Table...");
  for (int s = 0; s < STATE_SPACE_SIZE; s++) {
    for (int a = 0; a < ACTION_SPACE_SIZE; a++) {
      // Initialize to zero for a clean start
      Q_table[s][a] = 0.0f;
    }
  }
  Serial.println("[RL-SETUP] Q-Table initialized.");
}

/**
 * @brief Prints the current Q-Table (the learned policy) to the serial monitor for debugging.
 */
void printQTable() {
    Serial.println("\n----------------- Q-TABLE (Policy) -----------------");
    Serial.print("State |");
    for (int a = 0; a < ACTION_SPACE_SIZE; a++) {
        Serial.printf(" %-7s |", actionNames[a]);
    }
    Serial.println();
    Serial.println("---------------------------------------------------------");

    for (int s = 0; s < STATE_SPACE_SIZE; s++) {
        // Find the best action for the current state S
        float max_q = -FLT_MAX; 
        int best_action = -1;
        for (int a = 0; a < ACTION_SPACE_SIZE; a++) {
            if (Q_table[s][a] > max_q) {
                max_q = Q_table[s][a];
                best_action = a;
            }
        }
        
        String row = "S" + String(s) + "  |";
        for (int a = 0; a < ACTION_SPACE_SIZE; a++) {
            char val[10];
            // Format the Q-value string (highlight the best action with asterisks)
            if (a == best_action) {
                sprintf(val, " *%.2f*", Q_table[s][a]);
            } else {
                sprintf(val, "  %.2f ", Q_table[s][a]);
            }
            row += String(val) + " |";
        }
        Serial.println(row);
    }
    Serial.println("---------------------------------------------------------");
    Serial.println("(* indicates the current greedy (best) action for that state)");
    Serial.println("State Map: S0-S2=LOW_LUX, S3-S5=MED_LUX, S6-S8=HIGH_LUX");
    Serial.println("           S(x)0=COLD, S(x)1=COMFORT, S(x)2=HOT");
}


// --------------------------------------------------------
// --- Main Setup & Loop ----------------------------------
// --------------------------------------------------------

void setup() {
  // Initialize Serial communication for debugging and output
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n--- Embedded RL Framework Initialization ---");

  // Initialize the Wire (I2C) library
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // Initialize sensors and check for success
  if (bh1750Init()) {
    Serial.println("✓ BH1750 (Lux) initialized successfully.");
  } else {
    Serial.println("✗ BH1750 (Lux) FAILED to initialize. Using TARGET_LUX as failover.");
  }

  if (readCalibration()) {
    Serial.println("✓ BMP180 (Temp/Press) calibration data read successfully.");
  } else {
    Serial.println("✗ BMP180 (Temp/Press) FAILED to read calibration.");
  }
  
  // Initialize the Q-Table for the RL agent
  initQTable();

  // Initialize random seed for Epsilon-Greedy selection
  randomSeed(analogRead(0)); 

  Serial.println("--------------------------------------------");
  Serial.println("Starting Q-Learning Training...");
  Serial.println("NOTE: Energy cost has been added to the reward function.");
}

void loop() {
  // --- 1. SENSE: Read Current Environment State ---
  
  // Read Lux
  float lux = bh1750ReadLux();
  // Graceful sensor failure policy: Use TARGET_LUX if sensor read failed
  if (lux < 0) lux = TARGET_LUX; 

  // Read Temp 
  int32_t UT = readRawTemp();
  int32_t B5 = computeB5(UT);
  float temp = ((B5 + 8) >> 4) / 10.0;

  // Map sensor data to the discrete state S'
  int S_new = getState(lux, temp);

  // Print current state for tracking
  Serial.print("\n[EPISODE] LUX: "); Serial.print(lux, 2); 
  Serial.print(" lx | TEMP: "); Serial.print(temp, 2); 
  Serial.print(" °C | STATE S': "); Serial.println(S_new);

  // --- 2. REWARD: Calculate Immediate Reward ---
  float R = calculateReward(lux, temp);
  Serial.print("[REWARD] R: "); Serial.println(R, 3);
  
  // --- 3. LEARN: Perform Q-Table Update ---
  // The first iteration skips this as S_prev and A_prev are invalid
  updateQTable(S_prev, A_prev, R, S_new);
  
  // --- 4. ACTION: Select Next Action A' (Exploit/Explore) ---
  int A_new = chooseAction(S_new);
  
  // --- 5. EXECUTE: Apply the Action to the Environment ---
  executeAction(A_new);

  // --- 6. SHIFT: Set up for Next Iteration ---
  S_prev = S_new; // S becomes S_prev
  A_prev = A_new; // A becomes A_prev
  
  // --- 7. MONITOR: Print Q-Table Periodically ---
  if (millis() % 10000 < 1000) { // Print roughly every 10 seconds
    printQTable();
  }

  // Delay before the next episode (e.g., waiting for environment to change)
  delay(5000); // 5 second episode duration
}








/**
 * @file ADAPTIVE_RL_AGENT.CPP
 * @brief Embedded Reinforcement Learning Agent Implementation for Control.
 * @project Embedded Real-Time Adaptive System (ERTAS)
 *
 * This module implements the Reinforcement Learning (RL) agent responsible for
 * real-time, sequential decision-making directly on the embedded target hardware.
 * It acts as the core control loop for systems requiring dynamic policy optimization
 * based on continuous environmental interaction.
 *
 * ---
 *
 * ## 1. WHAT: System Definition
 * The system is an **Adaptive Decision Engine** utilizing a specific, optimized
 * Reinforcement Learning algorithm (e.g., Q-Learning or lightweight DQN) trained
 * to find an optimal policy $\pi^*$ for control actions ($a_t$) given observed
 * environmental states ($s_t$) and maximization of cumulative future reward ($R_t$).
 * The primary goal is long-term performance optimization.
 *
 * ## 2. HOW: Mechanism and Architecture
 * The control loop operates sequentially:
 * 1.  **State Observation ($s_t$):** Gathers processed input from physical sensors (e.g., analog/digital inputs, temperature, system variables) and maps them into a structured state space.
 * 2.  **Action Selection ($a_t$):** Uses the current policy (Q-table lookup or network forward pass) to select a control action (e.g., setting PWM, adjusting a proportional coefficient).
 * 3.  **Reward & Next State ($r_t, s_{t+1}$):** Executes the action in the hardware, measures the immediate reward ($r_t$), and observes the resultant state ($s_{t+1}$).
 * 4.  **Policy Update:** Uses the experience tuple $(s_t, a_t, r_t, s_{t+1})$ to update the internal policy representation (model weights or table entries) to improve future decisions.
 *
 * ## 3. WHY EMBEDDED RL? (Advantages of On-Device Autonomy)
 * The approach is mandatory for mission-critical control due to:
 * - **Ultra-Low Latency:** Decisions are made locally, ensuring reaction times are fast enough for real-time control applications (e.g., $< 1$ ms).
 * - **Offline Operation/Resilience:** Full functionality is maintained independent of network status or cloud availability.
 * - **Continuous Adaptation:** The agent can self-tune its policy to compensate for local environmental changes, component aging (sensor drift, mechanical wear), and unique operational profiles not seen during initial training.
 *
 * ## 4. BENEFITS OVER IF-THEN-ELSE (Rule-Based Logic)
 * | Feature | Rule-Based Logic | Embedded RL Agent |
 * | :--- | :--- | :--- |
 * | **Optimality** | Sub-optimal; reflects human heuristics. | Finds globally optimal policies by systematic exploration. |
 * | **Adaptation** | Manual code updates required for novelty. | Automatically adapts to drift and new scenarios via reward feedback. |
 * | **Complexity** | Becomes brittle and unmanageable with many variables. | Scales efficiently, learning complex relationships autonomously. |
 *
 * ## 5. BENEFITS OVER OTHER ML/DL (Classification/Regression)
 * RL is superior for control tasks because it focuses on **sequential optimization**:
 * - **Long-Term Reward:** Optimizes for the total cumulative reward ($R_t$), prioritizing long-term goals over immediate, instantaneous accuracy.
 * - **Causality & Control:** Explicitly models the causal impact of actions on future states, making it a true controller (answering "What should I *do*?").
 * - **Active Learning:** Built-in mechanism for the **Exploration-Exploitation Tradeoff** (e.g., $\epsilon$-greedy), enabling continuous discovery of better actions.
 */

// --- Agent Implementation Start ---

#include <Wire.h>
#include <math.h> 
#include <float.h> 
#include <WiFi.h>
#include <WebServer.h>
#include <pgmspace.h> 
#include "driver/ledc.h" // For low-level PWM control

// --------------------------------------------------------
// --- WIFI Configuration ---
// --------------------------------------------------------
const char* ssid = ""; 
const char* password = ""; 

WebServer server(80); // Web server on port 80

// --------------------------------------------------------
// --- I2C Sensor Configuration (BH1750 only) ---
// --------------------------------------------------------
#define SDA_PIN 43
#define SCL_PIN 44
// BH1750 / GY-30 Configuration
#define BH1750_ADDR 0x23
#define BH1750_CONT_HIGH_RES_MODE 0x10

// --- BH1750 Sensor Functions ---
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

// --------------------------------------------------------
// --- ACTUATOR: LED PWM Configuration (GPIO 8 / D9) ---
// --------------------------------------------------------
const int ledPin = 8; // D9 corresponds to GPIO 8 (Confirmed)
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE 
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_RESOLUTION LEDC_TIMER_8_BIT // 0-255 range
#define LEDC_FREQUENCY 5000 // 5 kHz frequency

// Current LED PWM value (Duty Cycle: 0-255)
int current_pwm_duty = 0; 
#define PWM_STEP_SIZE 30 // How much the PWM changes per action (0-255 range)

void ledc_setup() {
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_MODE,
    .duty_resolution = LEDC_RESOLUTION,
    .timer_num = LEDC_TIMER,
    .freq_hz = LEDC_FREQUENCY,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {
    .gpio_num = ledPin,
    .speed_mode = LEDC_MODE,
    .channel = LEDC_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER,
    .duty = 0, 
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channel);
  
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void set_led_pwm(int duty) {
    // Clamp duty cycle between 0 and 255
    if (duty > 255) duty = 255;
    if (duty < 0) duty = 0;
    
    current_pwm_duty = duty;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, current_pwm_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}


// --------------------------------------------------------
// --- RL Configuration and Logic (Simplified for Lux Only) ---
// --------------------------------------------------------
#define LUX_BIN_LOW_MAX 100.0f 
#define LUX_BIN_HIGH_MIN 500.0f
#define LUX_BINS 3 // Low (0), Medium (1), High (2)
#define STATE_SPACE_SIZE LUX_BINS 

// C++ definitions for human-readable state names (for Serial Debugging)
const char* stateNames[LUX_BINS] = {
    "Low (<100)", "Medium (100-500)", "High (>500)"
};

#define ACTION_SPACE_SIZE 3 // LIGHT+, LIGHT-, IDLE
enum Action {
    A_LIGHT_UP = 0,     // Increase LED brightness
    A_LIGHT_DOWN,       // Decrease LED brightness
    A_DO_NOTHING        // Idle
};
const char* actionNames[ACTION_SPACE_SIZE] = {
    "LIGHT+", "LIGHT-", "IDLE"
};

#define ALPHA 0.1f      // Learning Rate
#define GAMMA 0.9f      // Discount Factor
#define EPSILON 0.3f    // Exploration Rate
#define CRUCIAL_TD_THRESHOLD 1.0f 

float Q_table[STATE_SPACE_SIZE][ACTION_SPACE_SIZE];
int S_prev = -1;
int A_prev = -1; 
float current_lux = 0.0f;
int S_current = 0;
int A_current = A_DO_NOTHING; // Default to IDLE
float R_current = 0.0f;
String rl_log_message = "System Initializing...";

#define TARGET_LUX 300.0f 
#define REWARD_MAX_DISTANCE 10.0f // Normalization factor for reward

/**
 * @brief Maps current Lux reading to a state index (0, 1, or 2).
 * @param lux Current illuminance value.
 * @return State index.
 */
int getState(float lux) {
    return (lux <= LUX_BIN_LOW_MAX) ? 0 : (lux >= LUX_BIN_HIGH_MIN) ? 2 : 1;
}

/**
 * @brief Calculates reward based on distance from TARGET_LUX and energy cost.
 * @param lux Current illuminance value.
 * @return Calculated reward (-1.0 to 1.0).
 */
float calculateReward(float lux) {
    // Lux error normalized by 100.0 (arbitrary scale factor for reward curve)
    float lux_error = fabs(lux - TARGET_LUX) / 100.0f; 
    float distance = lux_error;

    // Base reward: Closer to target (distance=0) gives max reward (1.0)
    float reward = 1.0f - (distance / REWARD_MAX_DISTANCE);
    
    // Energy Cost (Penalty for acting, small reward for idling)
    float energy_cost = 0.0f;
    if (A_prev != -1) { 
        if (A_prev != A_DO_NOTHING) {
            energy_cost = -0.05f; // Penalize for consuming energy (LIGHT+ or LIGHT-)
        } else {
            energy_cost = 0.01f; // Small reward for saving energy (IDLE)
        }
    }

    reward += energy_cost;

    // Clamp reward between -1.0 and 1.0
    if (reward > 1.0f) reward = 1.0f;
    if (reward < -1.0f) reward = -1.0f;

    return reward;
}

int maxQAction(int S_new) {
    float max_q = -FLT_MAX;
    int best_action = 0;
    for (int A = 0; A < ACTION_SPACE_SIZE; A++) {
        if (Q_table[S_new][A] > max_q) {
            max_q = Q_table[S_new][A];
            best_action = A;
        }
    }
    return best_action;
}

float maxQ(int S_new) {
    return Q_table[S_new][maxQAction(S_new)];
}

int chooseAction(int S) {
    if ((float)random(100) / 100.0f < EPSILON) {
        int A = random(ACTION_SPACE_SIZE);
        rl_log_message = "EXPLORE: Random action " + String(actionNames[A]);
        return A;
    } else {
        int best_action = maxQAction(S);
        rl_log_message = "EXPLOIT: Greedy action " + String(actionNames[best_action]);
        return best_action;
    }
}

/**
 * @brief Executes the chosen action by adjusting the LED PWM duty cycle.
 * @param A The action to execute.
 */
void executeAction(int A) {
    int new_duty = current_pwm_duty;
    
    switch (A) {
        case A_LIGHT_UP:
            new_duty += PWM_STEP_SIZE;
            break;
        case A_LIGHT_DOWN:
            new_duty -= PWM_STEP_SIZE;
            break;
        case A_DO_NOTHING:
            // Duty remains the same
            break;
    }

    set_led_pwm(new_duty); // Update the physical LED and global state

    rl_log_message += " -> ACTUATOR: " + String(actionNames[A]) + " (PWM=" + String(current_pwm_duty) + ") Executed.";
}


void updateQTable(int S_prev, int A_prev, float R, int S_new) {
    if (S_prev < 0 || A_prev < 0) {
        return;
    }
    
    float max_q_new = maxQ(S_new);
    float old_q = Q_table[S_prev][A_prev];
    float future_expected_reward = GAMMA * max_q_new;
    float td_error = R + future_expected_reward - old_q;
    
    int old_best_action = maxQAction(S_prev);
    
    float new_q = old_q + ALPHA * td_error;
    Q_table[S_prev][A_prev] = new_q;
    
    int new_best_action = maxQAction(S_prev);

    String update_log = "LEARN: S" + String(S_prev) + ", A=" + String(actionNames[A_prev]) + 
                        ", R=" + String(R, 3) + ", TD=" + String(td_error, 3);
                        
    if (fabs(td_error) > CRUCIAL_TD_THRESHOLD) {
        update_log += " [CRUCIAL UPDATE]";
    }
    
    if (old_best_action != new_best_action) {
        update_log += " [POLICY CHANGE: " + String(actionNames[new_best_action]) + "]";
    }
    rl_log_message = update_log;
}

void initQTable() {
    for (int s = 0; s < STATE_SPACE_SIZE; s++) {
        for (int a = 0; a < ACTION_SPACE_SIZE; a++) {
            Q_table[s][a] = 0.0f;
        }
    }
}

// --------------------------------------------------------
// --- Embedded HTML/CSS/JavaScript Dashboard Client ---
// --------------------------------------------------------
// IMPORTANT: Temperature related elements removed.
const char HTML_CONTENT[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>RL Adaptive Lighting Dashboard</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;600;800&display=swap');
        body { font-family: 'Inter', sans-serif; background-color: #f8fafc; }
        .card { 
            background: #ffffff; 
            border-radius: 1rem; 
            box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.1), 0 4px 6px -4px rgba(0, 0, 0, 0.1); 
            transition: transform 0.3s ease;
        }
        .card:hover {
            transform: translateY(-2px);
            box-shadow: 0 20px 25px -5px rgba(0, 0, 0, 0.1), 0 8px 10px -6px rgba(0, 0, 0, 0.1);
        }
        .state-active { 
            box-shadow: 0 0 0 4px rgba(59, 130, 246, 0.7); /* Tailwind blue-500 equivalent */
            animation: pulse 1.5s infinite;
        }
        @keyframes pulse {
            0%, 100% { box-shadow: 0 0 0 0 rgba(59, 130, 246, 0.6); }
            50% { box-shadow: 0 0 0 8px rgba(59, 130, 246, 0); }
        }

        /* Gauge styling for sensor visualization */
        .gauge-container { position: relative; width: 100%; height: 10px; border-radius: 5px; overflow: hidden; }
        .lux-gauge { background: linear-gradient(to right, #fcd34d 16.66%, #10b981 50%, #fcd34d 83.33%); }
        .gauge-pointer { 
            position: absolute; 
            top: -5px; 
            width: 3px; 
            height: 20px; 
            background-color: #0f172a; 
            border-radius: 1.5px;
            transform: translateX(-50%);
            transition: left 0.5s ease-out;
            box-shadow: 0 0 5px rgba(0,0,0,0.5);
        }
        .gauge-target-line {
            position: absolute;
            top: 0;
            width: 2px;
            height: 100%;
            background-color: #0f172a;
            transform: translateX(-50%);
        }
    </style>
</head>
<body class="p-4 sm:p-8">
    <div class="max-w-4xl mx-auto">
        <h1 class="text-3xl font-extrabold text-gray-800 mb-6 pb-2 flex items-center">
            <svg class="w-8 h-8 mr-2 text-yellow-600" fill="currentColor" viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg"><path d="M11.049 2.152A.75.75 0 0112 2.25v.75a.75.75 0 01-1.5 0V2.25c0-.414.336-.75.75-.75zM12 4.5a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0v-.75A.75.75 0 0112 4.5zM12 6.75a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0v-.75a.75.75 0 01.75-.75zM11.049 9.152a.75.75 0 01.951.098l.6.6a.75.75 0 01-.998 1.12l-.6-.6a.75.75 0 01.098-.951zM11.75 11.25a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0v-.75a.75.75 0 01.75-.75zM12 13.5a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0v-.75a.75.75 0 01.75-.75zM11.049 15.848a.75.75 0 01.098.951l.6.6a.75.75 0 01-1.12.998l-.6-.6a.75.75 0 01-.951-.098zM9.75 17.25a.75.75 0 01-.75.75h-.75a.75.75 0 010-1.5h.75a.75.75 0 01.75.75zM7.5 17.25a.75.75 0 01-.75.75h-.75a.75.75 0 010-1.5h.75a.75.75 0 01.75.75zM5.25 17.25a.75.75 0 01-.75.75h-.75a.75.75 0 010-1.5h.75a.75.75 0 01.75.75zM3.951 15.848a.75.75 0 01-.951.098l-.6.6a.75.75 0 011.12.998l.6-.6a.75.75 0 01-.098-.951zM2.25 13.5a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0v-.75a.75.75 0 01.75-.75zM2.25 11.25a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0v-.75a.75.75 0 01.75-.75zM3.951 9.152a.75.75 0 01.951.098l.6.6a.75.75 0 01-.998 1.12l-.6-.6a.75.75 0 01.098-.951zM6.75 6.75a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0v-.75a.75.75 0 01.75-.75zM6.75 4.5a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0v-.75a.75.75 0 01.75-.75zM6.75 2.25a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0V2.25a.75.75 0 01.75-.75zM12 8.25a3.75 3.75 0 100 7.5 3.75 3.75 0 000-7.5zM12 9.75a2.25 2.25 0 110 4.5 2.25 2.25 0 010-4.5z"></path></svg>
            RALU RL Trainer Dashboard
        </h1>

        <!-- Sensor Data & Status -->
        <div class="grid grid-cols-1 md:grid-cols-3 gap-6 mb-8">
            
            <!-- LUX Card -->
            <div class="card p-6 border-l-4 border-yellow-500">
                <p class="text-sm font-semibold text-gray-500 uppercase">Illuminance (Lux)</p>
                <p class="text-5xl font-extrabold mt-1 text-yellow-600 flex items-baseline" id="lux-value">--.-</p>
                <div class="gauge-container lux-gauge mt-2 mb-4">
                    <div class="gauge-target-line" style="left:50%;"></div>
                    <div class="gauge-pointer" id="lux-pointer"></div>
                </div>
                <div class="text-xs text-gray-500 font-medium flex justify-between">
                    <span>Low (&lt;100)</span>
                    <span class="font-bold text-green-600">Target (300)</span>
                    <span>High (&gt;500)</span>
                </div>
            </div>
            
            <!-- LED Output Card (NEW) -->
            <div class="card p-6 border-l-4 border-amber-500">
                <p class="text-sm font-semibold text-gray-500 uppercase">LED PWM Output</p>
                <p class="text-5xl font-extrabold mt-1 text-amber-600 flex items-baseline" id="pwm-value">--</p>
                <div class="mt-2 text-sm text-gray-500 font-medium">
                    <p>Duty Cycle (0-255)</p>
                    <div class="w-full h-2 bg-gray-200 rounded-full mt-2 overflow-hidden">
                        <div id="pwm-bar" class="h-full bg-amber-400 transition-all duration-500" style="width: 0%;"></div>
                    </div>
                </div>
            </div>


            <!-- RL Status Card -->
            <div class="card p-6 border-l-4 border-blue-500">
                <p class="text-sm font-semibold text-gray-500 uppercase">RL Status</p>
                <p class="text-4xl font-extrabold mt-1 text-blue-600" id="reward-value">-.---</p>
                <div class="text-sm text-gray-600 mt-2">
                    <p>State Index: <span id="state-index" class="font-bold">--</span></p>
                    <p>Action Taken: <span id="action-icon" class="text-xl mr-1 align-middle"></span><span id="action-taken" class="font-bold">IDLE</span></p>
                </div>
            </div>
        </div>

        <!-- RL Log -->
        <div class="card p-4 mb-8 bg-blue-50 border-blue-200 border">
            <p class="text-xs font-mono text-gray-700" id="rl-log">System Initializing...</p>
        </div>

        <!-- Q-Table Policy Heatmap -->
        <div class="card p-6">
            <h2 class="text-xl font-semibold mb-4 text-gray-700">Q-Table Policy Heatmap (Expected Rewards)</h2>
            <p class="text-sm text-gray-500 mb-4">
                <span class="text-green-500">Green = High Reward (Good Action)</span> | 
                <span class="text-red-500">Red = Low/Negative Reward (Bad Action)</span> | 
                <span class="font-extrabold text-blue-600 text-lg">★</span> = **Current Best Policy** for that State
            </p>
            
            <div class="overflow-x-auto">
                <table class="w-full text-center whitespace-nowrap border-collapse">
                    <thead>
                        <tr class="bg-gray-100 text-sm font-semibold text-gray-600">
                            <th class="p-3 border">State (Lux Bin)</th>
                            <th class="p-3 border">LIGHT+</th>
                            <th class="p-3 border">LIGHT-</th>
                            <th class="p-3 border">IDLE</th>
                        </tr>
                    </thead>
                    <tbody id="q-table-body">
                        <!-- Rows S0 to S2 will be inserted here by JavaScript -->
                    </tbody>
                </table>
            </div>
            
            <div class="mt-4 text-xs text-gray-600 font-medium">
                <p>State Bins: S0 (Low Lux &lt;100) | S1 (Medium Lux 100-500) | S2 (High Lux &gt;500)</p>
            </div>
        </div>
    </div>

    <script>
        const LUX_BINS = ["Low", "Medium/Target", "High"];
        const ACTION_NAMES = ["LIGHT+", "LIGHT-", "IDLE"];

        // Maps action names to simple icons (using unicode/emojis for simplicity on ESP32)
        const ACTION_ICONS = {
            "LIGHT+": "💡⬆️",
            "LIGHT-": "💡⬇️",
            "IDLE": "⏸️"
        };
        
        // Function to map Q-value to Tailwind color class for Heatmap
        function getHeatmapColor(qValue) {
            const absoluteQ = Math.min(Math.abs(qValue), 1.0);
            const intensity = Math.min(Math.ceil(absoluteQ * 8) * 100, 800);

            if (qValue > 0.05) {
                return `bg-green-${intensity} text-green-900`;
            } else if (qValue < -0.05) {
                return `bg-red-${intensity} text-red-900`;
            } else {
                return 'bg-gray-200 text-gray-600';
            }
        }
        
        // Function to render the Q-Table heatmap
        function renderQTable(qTable, currentState) {
            const tbody = document.getElementById('q-table-body');
            tbody.innerHTML = ''; 

            // Only 3 states (0, 1, 2)
            for (let s = 0; s < 3; s++) {
                
                const tr = document.createElement('tr');
                tr.className = `border-b hover:bg-gray-50 ${s === currentState ? 'state-active bg-blue-50' : ''}`;
                
                // State Label Cell
                const stateCell = document.createElement('td');
                stateCell.className = `p-3 border font-extrabold text-gray-700`;
                stateCell.innerHTML = `S${s} <div class="text-xs font-normal text-gray-500">(${LUX_BINS[s]})</div>`;
                tr.appendChild(stateCell);

                // Find the best action index
                let maxQ = -Infinity;
                let bestActionIndex = -1;
                // Only 3 actions
                qTable[s].slice(0, 3).forEach((q, index) => {
                    if (q > maxQ) {
                        maxQ = q;
                        bestActionIndex = index;
                    }
                });

                // Action Cells
                // Only 3 actions (0, 1, 2)
                qTable[s].slice(0, 3).forEach((q, a) => {
                    const actionCell = document.createElement('td');
                    // Apply color based on Q-value
                    actionCell.className = `p-3 border heatmap-cell ${getHeatmapColor(q)} text-xs font-mono`;
                    actionCell.textContent = q.toFixed(4);
                    
                    // Highlight the greedy action for ALL states with a star
                    if (a === bestActionIndex) {
                        actionCell.innerHTML = `<span class="text-blue-600 text-lg font-extrabold">★</span><br>${q.toFixed(4)}`;
                        actionCell.classList.add('font-extrabold', 'shadow-inner', 'shadow-white/20');
                    }
                    
                    tr.appendChild(actionCell);
                });
                
                tbody.appendChild(tr);
            }
        }

        // Function to map Lux value to gauge position (0-100% visualized range: 0 to 600)
        function getGaugePosition(lux_value) {
            const MIN_LUX = 0;
            const MAX_LUX_VISUAL = 600; // Use 600 to visualize the full range including the target of 300
            
            const clampedValue = Math.min(Math.max(lux_value, MIN_LUX), MAX_LUX_VISUAL);
            const normalized = (clampedValue - MIN_LUX) / (MAX_LUX_VISUAL - MIN_LUX);
            
            return Math.min(Math.max(normalized * 100, 0), 100);
        }
        
        // Main fetch loop
        async function fetchRLData() {
            try {
                const response = await fetch('/data');
                const data = await response.json();

                // 1. Update Lux Value & Gauge
                document.getElementById('lux-value').textContent = data.lux.toFixed(2);
                const luxPos = getGaugePosition(data.lux); 
                document.getElementById('lux-pointer').style.left = `${luxPos}%`;
                
                // 2. Update PWM Output
                document.getElementById('pwm-value').textContent = data.pwm;
                document.getElementById('pwm-bar').style.width = `${(data.pwm / 255) * 100}%`;

                // 3. Update RL Status
                document.getElementById('reward-value').textContent = data.reward.toFixed(3);
                document.getElementById('state-index').textContent = data.state;
                document.getElementById('action-taken').textContent = data.action;
                document.getElementById('action-icon').textContent = ACTION_ICONS[data.action] || '❓';


                // 4. Update Log
                document.getElementById('rl-log').textContent = data.log;

                // 5. Render Heatmap
                renderQTable(data.qTable, data.state);
                
            } catch (error) {
                console.error("Error fetching RL data:", error);
                document.getElementById('rl-log').textContent = "ERROR: Could not connect to ESP32 server. Check WiFi connection and IP address.";
            }
            
            setTimeout(fetchRLData, 5000); 
        }

        fetchRLData();
    </script>
</body>
</html>
)rawliteral";


// --------------------------------------------------------
// --- Web Server Handlers ---
// --------------------------------------------------------

/**
 * @brief Low-level function to stream HTML content from PROGMEM (Flash) 
 * to the client in small chunks.
 */
void sendLargeHtmlFromProgmem(const char *html_content) {
  size_t html_size = strlen_P(html_content);
  
  server.sendHeader("Connection", "close");
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.setContentLength(html_size);
  server.send(200, "text/html", ""); 
  
  WiFiClient client = server.client();
  size_t bytes_sent = 0;
  const size_t chunk_size = 512; 
  
  Serial.printf("Attempting to stream HTML (Size: %u bytes)...\n", html_size);

  while (bytes_sent < html_size) {
    size_t remaining = html_size - bytes_sent;
    size_t current_chunk_size = (remaining < chunk_size) ? remaining : chunk_size;
    
    char buffer[chunk_size];
    
    memcpy_P(buffer, html_content + bytes_sent, current_chunk_size);

    client.write((const uint8_t*)buffer, current_chunk_size);

    bytes_sent += current_chunk_size;
    
    yield(); 
    
    if (client.getWriteError()) {
      Serial.println("Client write error occurred. Aborting stream.");
      break;
    }
  }
  
  client.stop(); 
  Serial.println("HTML stream complete. Connection closed.");
}


/**
 * @brief Handles the root request by streaming the dashboard HTML.
 */
void handleRoot() {
  sendLargeHtmlFromProgmem(HTML_CONTENT);
}


/**
 * @brief Serves the real-time data as JSON.
 * @note Added PWM duty cycle to the JSON payload.
 */
void handleData() {
  String json = "{";
  json += "\"lux\":" + String(current_lux, 2) + ",";
  json += "\"pwm\":" + String(current_pwm_duty) + ","; // NEW: LED PWM output
  json += "\"state\":" + String(S_current) + ",";
  json += "\"action\":\"" + String(actionNames[A_current]) + "\",";
  json += "\"reward\":" + String(R_current, 3) + ",";
  json += "\"log\":\"" + rl_log_message + "\",";
  json += "\"qTable\":[";

  for (int s = 0; s < STATE_SPACE_SIZE; s++) {
    json += "[";
    for (int a = 0; a < ACTION_SPACE_SIZE; a++) {
      json += String(Q_table[s][a], 4);
      if (a < ACTION_SPACE_SIZE - 1) json += ",";
    }
    json += "]";
    if (s < STATE_SPACE_SIZE - 1) json += ",";
  }
  json += "]";
  json += "}";

  server.send(200, "application/json", json);
}

// --------------------------------------------------------
// --- Main Setup & Loop ---
// --------------------------------------------------------

void setup() {
    Serial.begin(115200);
    Serial.println("--- Starting ERTAS RL Agent ---");
    Serial.println("INFO: Set Serial Monitor Baud Rate to 115200 for debug output.");
    
    Wire.begin(SDA_PIN, SCL_PIN);
    
    // 1. Initialize RL, Sensors, and Actuator (PWM)
    initQTable();
    randomSeed(analogRead(0));
    
    // Check and report BH1750 initialization status
    if (bh1750Init()) {
        Serial.println("BH1750 (0x23) sensor initialized successfully.");
    } else {
        Serial.println("ERROR: BH1750 sensor initialization failed. Check I2C wiring (SDA/SCL pins and address).");
    }

    ledc_setup(); // Initialize LEDC PWM
    
    // 2. Connect to WiFi
    Serial.print("\nConnecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected.");
    Serial.print("Access Dashboard at IP: ");
    Serial.println(WiFi.localIP());

    // 3. Setup Web Server Routes
    server.on("/", HTTP_GET, handleRoot);
    server.on("/data", HTTP_GET, handleData);
    server.begin();
    Serial.println("Web server started.");
}

void loop() {
    // Handle web server client requests
    server.handleClient();
    
    // --- RL Episode Logic (Runs every 5 seconds) ---
    static unsigned long lastEpisodeTime = 0;
    if (millis() - lastEpisodeTime >= 5000) { 
        lastEpisodeTime = millis();
        
        // 1. SENSE: Read Current Environment State
        current_lux = bh1750ReadLux();
        if (current_lux < 0) current_lux = TARGET_LUX; // Fallback
        
        int S_new = getState(current_lux);
        S_current = S_new;
    
        // 2. REWARD: Calculate Immediate Reward
        R_current = calculateReward(current_lux);
        
        // 3. LEARN: Perform Q-Table Update
        updateQTable(S_prev, A_prev, R_current, S_new);
        
        // 4. ACTION: Select Next Action A' (Exploit/Explore)
        int A_new = chooseAction(S_new);
        A_current = A_new;
        
        // 5. EXECUTE: Apply the Action to the Environment
        executeAction(A_new);

        // 6. SHIFT: Set up for Next Iteration
        S_prev = S_new; 
        A_prev = A_new; 
        
        // --- DEBUG SERIAL OUTPUT (NEW) ---
        Serial.println("--------------------------------------------------");
        Serial.printf("RL Episode Complete. Time: %lu ms\n", lastEpisodeTime);
        Serial.printf("LUX: %.2f | State: S%d (%s)\n", current_lux, S_current, stateNames[S_current]);
        Serial.printf("Reward: %.3f | Action: %s | PWM: %d\n", R_current, actionNames[A_current], current_pwm_duty);
        Serial.println("--------------------------------------------------");
        // ------------------------------------
    }
}









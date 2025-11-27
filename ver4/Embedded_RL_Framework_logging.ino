/**
 * @file ADAPTIVE_RL_AGENT_WEB_CSV_LOGGING.ino
 * @brief Embedded Reinforcement Learning Agent with Web-based CSV Data Logging.
 *
 * This version implements **non-blocking** network streaming to fix "Network error" 
 * and "File incomplete" issues caused by ESP32 Watchdog Timer (WDT) and TCP stack timeouts.
 * * 🌟 CRITICAL FIX: The /log handler is now served directly by a dedicated TCP connection, 
 * bypassing the standard WebServer library's blocking implementation for large files.
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
const char* ssid = "SKYYRMR7";
const char* password = "K2xWvDFZkuCh"; 

// 🌟 CRITICAL FIX: Use WiFiServer for direct, non-blocking /log handling 🌟
// The standard WebServer is still used for the dashboard (/ and /data), 
// but /log uses this dedicated server for robust streaming.
WiFiServer logServer(8080); // Log server on a different port (8080) to simplify routing.
WebServer server(80); // Web server on port 80 for JSON/HTML

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
const int ledPin = 8;
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE 
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_RESOLUTION LEDC_TIMER_8_BIT // 0-255 range
#define LEDC_FREQUENCY 5000 // 5 kHz frequency

// Current LED PWM value (Duty Cycle: 0-255)
int current_pwm_duty = 0;
#define PWM_STEP_SIZE 10 // How much the PWM changes per action (0-255 range)

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
// --- RL Configuration and Logic (Self-Tuning Implemented) ---
// --------------------------------------------------------
// --- NEW 5-BIN STATE SPACE FOR TIGHTER CONTROL (250-350 Lux Target) ---
#define LUX_BIN_V_LOW_MAX   100.0f // S0 Max
#define LUX_BIN_LOW_MAX     250.0f // S1 Max
#define LUX_BIN_HIGH_MIN    350.0f // S3 Min
#define LUX_BIN_V_HIGH_MIN  500.0f // S4 Min

#define LUX_BINS 5 
#define STATE_SPACE_SIZE LUX_BINS 

const char* stateNames[LUX_BINS] = {
    "Very Low (<100)", 
    "Low (100-250)", 
    "Target (250-350)", 
    "High (350-500)", 
    "Very High (>500)"
};
#define ACTION_SPACE_SIZE 3 // LIGHT+, LIGHT-, IDLE
enum Action {
    A_LIGHT_UP = 0, 
    A_LIGHT_DOWN, 
    A_DO_NOTHING 
};
const char* actionNames[ACTION_SPACE_SIZE] = {
    "LIGHT+", "LIGHT-", "IDLE"
};
// --- SELF-TUNING PARAMETERS ---
#define GAMMA 0.9f          // Discount Factor (Fixed)
#define ALPHA_START 1.0f    // Dynamic Learning Rate starts high
#define CRUCIAL_TD_THRESHOLD 1.0f 

#define EPSILON_START 1.0f  // Start with 100% exploration
#define EPSILON_END 0.05f   // End with 5% minimum exploration
#define EPSILON_DECAY 0.999f // Decay factor (0.999 is slow, 0.99 is fast)

float Q_table[STATE_SPACE_SIZE][ACTION_SPACE_SIZE];
long N_table[STATE_SPACE_SIZE][ACTION_SPACE_SIZE]; 
float currentEpsilon = EPSILON_START;

// --- RL State Variables ---
int S_prev = -1;
int A_prev = -1;
float current_lux = 0.0f;
int S_current = 0;
int A_current = A_DO_NOTHING; 
float R_current = 0.0f;
String rl_log_message = "System Initializing...";

// --- EPISODE TRACKING FOR GRAPHING ---
long episodeCount = 0;
#define TARGET_LUX 300.0f 
// Increased sensitivity for tighter control
#define REWARD_MAX_DISTANCE 1.0f 

/**
 * @brief Maps current Lux reading to a state index (0 to 4 for the 5-bin space).
 */
int getState(float lux) {
    if (lux <= LUX_BIN_V_LOW_MAX) {
        return 0; // S0: Very Low (<100)
    } else if (lux <= LUX_BIN_LOW_MAX) {
        return 1; // S1: Low (100-250)
    } else if (lux < LUX_BIN_HIGH_MIN) {
        return 2; // S2: Target (250-350)
    } else if (lux < LUX_BIN_V_HIGH_MIN) {
        return 3; // S3: High (350-500)
    } else {
        return 4; // S4: Very High (>500)
    }
}

/**
 * @brief Calculates reward based on distance from TARGET_LUX and energy cost.
 */
float calculateReward(float lux) {
    float lux_error = fabs(lux - TARGET_LUX) / 100.0f; 
    float distance = lux_error;
    float reward = 1.0f - (distance / REWARD_MAX_DISTANCE);
    
    // Energy Cost (Penalty for acting, small reward for idling)
    float energy_cost = 0.0f;
    if (A_prev != -1) { 
        if (A_prev != A_DO_NOTHING) {
            energy_cost = -0.05f;
        } else {
            energy_cost = 0.01f;
        }
    }

    reward += energy_cost;
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

/**
 * @brief Selects action using the self-tuning Epsilon-Greedy policy.
 */
int chooseAction(int S) {
    if ((float)random(100) / 100.0f < currentEpsilon) { 
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
            break;
    }

    set_led_pwm(new_duty);
    rl_log_message += " -> ACTUATOR: " + String(actionNames[A]) + " (PWM=" + String(current_pwm_duty) + ") Executed.";
}


/**
 * @brief Performs the Q-Table update using Dynamic Alpha and applies Epsilon Decay.
 */
void updateQTable(int S_prev, int A_prev, float R, int S_new) {
    if (S_prev < 0 || A_prev < 0) {
        return;
    }
    
    // --- 1. Update Visitation Count & Dynamic Alpha ---
    N_table[S_prev][A_prev]++;
    float dynamic_alpha = ALPHA_START / (ALPHA_START + N_table[S_prev][A_prev]);
    
    // --- 2. Q-Learning Update ---
    float max_q_new = maxQ(S_new);
    float old_q = Q_table[S_prev][A_prev];
    float future_expected_reward = GAMMA * max_q_new;
    float td_error = R + future_expected_reward - old_q;
    int old_best_action = maxQAction(S_prev);
    float new_q = old_q + dynamic_alpha * td_error;
    Q_table[S_prev][A_prev] = new_q;
    
    int new_best_action = maxQAction(S_prev);
    // --- 3. Epsilon Decay (Self-Tuning Exploration) ---
    currentEpsilon = max(EPSILON_END, currentEpsilon * EPSILON_DECAY);
    // --- 4. Logging ---
    String update_log = "LEARN: S" + String(S_prev) + ", A=" + String(actionNames[A_prev]) + 
                        ", R=" + String(R, 3) + ", alpha=" + String(dynamic_alpha, 3) + 
                        ", TD=" + String(td_error, 3);
    if (fabs(td_error) > CRUCIAL_TD_THRESHOLD) {
        update_log += " [CRUCIAL UPDATE]";
    }
    
    if (old_best_action != new_best_action) {
        update_log += " [POLICY CHANGE: " + String(actionNames[new_best_action]) + "]";
    }
    rl_log_message = update_log;
}


// --------------------------------------------------------
// --- IN-MEMORY LOGGING BUFFER (MAX 5000 Episodes) ---
// --------------------------------------------------------

#define LOG_HISTORY_SIZE 5000 
float log_lux[LOG_HISTORY_SIZE];
int log_state[LOG_HISTORY_SIZE];
int log_pwm[LOG_HISTORY_SIZE];
float log_reward[LOG_HISTORY_SIZE];
float log_epsilon[LOG_HISTORY_SIZE];
float log_alpha[LOG_HISTORY_SIZE]; 
long log_index = 0; 
bool buffer_full = false;

/**
 * @brief Initializes the Q-Table, N-Table, Epsilon, and Log Buffer.
 */
void initQTable() {
    for (int s = 0; s < STATE_SPACE_SIZE; s++) {
        for (int a = 0; a < ACTION_SPACE_SIZE; a++) {
            Q_table[s][a] = 0.0f;
            N_table[s][a] = 0; 
        }
    }
    currentEpsilon = EPSILON_START;
    episodeCount = 0;
    
    // Reset Log Buffer
    log_index = 0;
    buffer_full = false;
}

/**
 * @brief Saves the current episode's key metrics to the in-memory log buffer.
 */
void save_to_log_buffer() {
    // Only start logging after the first state-action-reward-new_state loop is complete.
    if (episodeCount <= 1) {
        return; 
    }
    
    // Calculate the last applied dynamic alpha
    float last_alpha = ALPHA_START / (ALPHA_START + N_table[S_prev][A_prev]);
    
    int idx = log_index % LOG_HISTORY_SIZE;
    
    // Save data points
    log_lux[idx] = current_lux;
    log_state[idx] = S_current;
    log_pwm[idx] = current_pwm_duty;
    log_reward[idx] = R_current;
    log_epsilon[idx] = currentEpsilon;
    log_alpha[idx] = last_alpha;

    log_index++;
    if (log_index >= LOG_HISTORY_SIZE) {
        buffer_full = true;
    }
}


// --------------------------------------------------------
// --- Embedded HTML/CSS/JavaScript Dashboard Client (FULL CONTENT) ---
// --------------------------------------------------------
const char HTML_CONTENT[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>RL Adaptive Lighting Dashboard</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js"></script>
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
            box-shadow: 0 0 0 4px rgba(59, 130, 246, 0.7);
            animation: pulse 1.5s infinite;
        }
        @keyframes pulse {
            0%, 100% { box-shadow: 0 0 0 0 rgba(59, 130, 246, 0.6); }
            50% { box-shadow: 0 0 0 8px rgba(59, 130, 246, 0); }
        }

        .gauge-container { position: relative; width: 100%; height: 10px; border-radius: 5px; overflow: hidden; }
        .lux-gauge { 
            background: linear-gradient(to right, 
                #f87171 0%, 
                #fcd34d 16.6%, 
                #10b981 41.6%, 
                #fcd34d 58.3%, 
                #f87171 83.3% 
            );
        }
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
<body>
    <div class="max-w-4xl mx-auto p-4 sm:p-8">
        <h1 class="text-3xl font-extrabold text-gray-800 mb-6 pb-2 flex items-center">
            <svg class="w-8 h-8 mr-2 text-yellow-600" fill="currentColor" viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg"><path d="M11.049 2.152A.75.75 0 0112 2.25v.75a.75.75 0 01-1.5 0V2.25c0-.414.336-.75.75-.75zM12 4.5a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0v-.75A.75.75 0 0112 4.5zM12 6.75a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0v-.75a.75.75 0 01.75-.75zM11.049 9.152a.75.75 0 01.951.098l.6.6a.75.75 0 01-.998 1.12l-.6-.6a.75.75 0 01.098-.951zM11.75 11.25a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0v-.75a.75.75 0 01.75-.75zM12 13.5a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0v-.75a.75.75 0 01.75-.75zM11.049 15.848a.75.75 0 01.098.951l.6.6a.75.75 0 01-1.12.998l-.6-.6a.75.75 0 01-.951-.098zM9.75 17.25a.75.75 0 01-.75.75h-.75a.75.75 0 010-1.5h.75a.75.75 0 01.75.75zM7.5 17.25a.75.75 0 01-.75.75h-.75a.75.75 0 010-1.5h.75a.75.75 0 01.75.75zM5.25 17.25a.75.75 0 01-.75.75h-.75a.75.75 0 010-1.5h.75a.75.75 0 01.75.75zM3.951 15.848a.75.75 0 01-.951.098l-.6.6a.75.75 0 011.12.998l.6-.6a.75.75 0 01-.098-.951zM2.25 13.5a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0v-.75a.75.75 0 01.75-.75zM2.25 11.25a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0v-.75a.75.75 0 01.75-.75zM3.951 9.152a.75.75 0 01.951.098l.6.6a.75.75 0 01-.998 1.12l-.6-.6a.75.75 0 01.098-.951zM6.75 6.75a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0v-.75a.75.75 0 01.75-.75zM6.75 4.5a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0v-.75a.75.75 0 01.75-.75zM6.75 2.25a.75.75 0 01.75.75v.75a.75.75 0 01-1.5 0V2.25a.75.75 0 01.75-.75zM12 8.25a3.75 3.75 0 100 7.5 3.75 3.75 0 000-7.5zM12 9.75a2.25 2.25 0 110 4.5 2.25 2.25 0 010-4.5z"></path></svg>
             RALU RL Trainer Dashboard
        </h1>

        <div class="grid grid-cols-1 md:grid-cols-4 gap-6 mb-8">
            
            <div class="card p-6 border-l-4 border-yellow-500">
                <p class="text-sm font-semibold text-gray-500 uppercase">Illuminance (Lux)</p>
                <p class="text-5xl font-extrabold mt-1 text-yellow-600 flex items-baseline" id="lux-value">--.-</p>
                <div class="gauge-container lux-gauge mt-2 mb-4">
                    <div class="gauge-target-line" style="left:50%;"></div>
                    <div class="gauge-pointer" id="lux-pointer"></div>
                </div>
                <div class="text-xs text-gray-500 font-medium flex justify-between">
                    <span>V.Low (&lt;100)</span>
                    <span class="font-bold text-green-600">Target (300)</span>
                    <span>V.High (&gt;500)</span>
                </div>
            </div>
            
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

            <div class="card p-6 border-l-4 border-blue-500">
                <p class="text-sm font-semibold text-gray-500 uppercase">RL Status</p>
                <p class="text-4xl font-extrabold mt-1 text-blue-600" id="reward-value">-.---</p>
                <div class="text-sm text-gray-600 mt-2">
                    <p>Episode: <span id="episode-count" class="font-bold">--</span></p>
                    <p>State Index: <span id="state-index" class="font-bold">--</span></p>
                </div>
            </div>
            
            <div class="card p-6 border-l-4 border-purple-500">
                <p class="text-sm font-semibold text-gray-500 uppercase">Self-Tuning</p>
                <p class="text-2xl font-extrabold mt-1 text-purple-600">
                    &epsilon;: <span id="epsilon-value">1.000</span>
                </p>
                <div class="text-sm text-gray-600 mt-2">
                    <p>Exploration: <span id="explore-pct" class="font-bold">100%</span></p>
                    <p>Learning Rate (&alpha;): <span id="alpha-value" class="font-bold">~1.000</span></p>
                </div>
            </div>
        </div>

        <div class="card p-6 mb-8 border-l-4 border-green-500">
            <h2 class="text-xl font-semibold mb-4 text-gray-700">Learning Progress: Moving Average Reward</h2>
            <div style="height: 250px;">
                <canvas id="rewardChart"></canvas>
            </div>
            <p class="mt-3 text-xs text-gray-500">
                Reward closer to 1.0 indicates stable, optimal control.
            </p>
            <p class="mt-3 text-sm font-semibold">
                <a id="log-link" href="http://192.168.0.15:8080/log" class="text-blue-600 hover:text-blue-800 underline">Download full data log as CSV (for analysis)</a>
            </p>
        </div>
        <div class="card p-4 mb-8 bg-blue-50 border-blue-200 border">
            <p class="text-xs font-mono text-gray-700" id="rl-log">System Initializing...</p>
        </div>

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
                        </tbody>
                </table>
            </div>
            
            <div class="mt-4 text-xs text-gray-600 font-medium">
                <p>State Bins: 
                    S0 (V. Low &lt;100) |
                    S1 (Low 100-250) | 
                    S2 (Target 250-350) | 
                    S3 (High 350-500) |
                    S4 (V. High &gt;500)
                </p>
            </div>
        </div>
    </div>

    <script>
        const STATE_SPACE_SIZE = 5;
        const LUX_BINS = ["V. Low", "Low", "Target", "High", "V. High"];
        const HISTORY_SIZE = 100; // Track the last 100 episodes for the graph
        const IP_ADDRESS = "192.168.0.15"; // Base IP for dashboard
        const LOG_PORT = "8080"; // Port for the direct CSV log server

        let rewardChart;
        let rewardHistory = [];
        let episodeLabels = [];
        let movingAverageData = [];
        // --- Chart Initialization ---
        function initChart() {
            const ctx = document.getElementById('rewardChart').getContext('2d');
            rewardChart = new Chart(ctx, {
                type: 'line',
                data: {
                    labels: episodeLabels,
                    datasets: [
                        {
                            label: 'Moving Avg Reward (Last 100)',
                            data: movingAverageData,
                            borderColor: 'rgb(16, 185, 129)', // Tailwind emerald-500
                            backgroundColor: 'rgba(16, 185, 129, 0.1)',
                            borderWidth: 2,
                            tension: 0.2, // Smoother line
                            pointRadius: 0 // Hide points
                        }
                    ]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    scales: {
                        y: {
                            min: -1.0, 
                            max: 1.0, 
                            title: { display: true, text: 'Reward' }
                        },
                        x: {
                            title: { display: true, text: 'Episode Count' },
                            type: 'linear', // Use linear scale for episode number
                            ticks: {
                                callback: function(value, index, values) {
                                    // Only show integer labels
                                    return Number.isInteger(value) ? value : '';
                                }
                            }
                        }
                    },
                    plugins: {
                        legend: { display: true }
                    }
                }
            });
        }
        
        // --- Utility Functions ---

        function calculateMovingAverage(newReward) {
            // 1. Add new reward to history
            rewardHistory.push(newReward);
            // 2. Trim history to HISTORY_SIZE
            if (rewardHistory.length > HISTORY_SIZE) {
                rewardHistory.shift();
            }

            // 3. Calculate sum and average
            const sum = rewardHistory.reduce((a, b) => a + b, 0);
            return sum / rewardHistory.length;
        }

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
        
        function renderQTable(qTable, currentState) {
            const tbody = document.getElementById('q-table-body');
            tbody.innerHTML = ''; 

            for (let s = 0; s < STATE_SPACE_SIZE; s++) {
                
                const tr = document.createElement('tr');
                tr.className = `border-b hover:bg-gray-50 ${s === currentState ? 'state-active bg-blue-50' : ''}`;
                
                const stateCell = document.createElement('td');
                stateCell.className = `p-3 border font-extrabold text-gray-700`;
                stateCell.innerHTML = `S${s} <div class="text-xs font-normal text-gray-500">(${LUX_BINS[s]})</div>`;
                tr.appendChild(stateCell);

                let maxQ = -Infinity;
                let bestActionIndex = -1;
                if (qTable[s]) { 
                    qTable[s].slice(0, 3).forEach((q, index) => {
                        if (q > maxQ) {
                            maxQ = q;
                            bestActionIndex = index;
                        }
                    });
                    qTable[s].slice(0, 3).forEach((q, a) => {
                        const actionCell = document.createElement('td');
                        actionCell.className = `p-3 border heatmap-cell ${getHeatmapColor(q)} text-xs font-mono`;
                        actionCell.textContent = q.toFixed(4);
            
                        if (a === bestActionIndex) {
                            actionCell.innerHTML = `<span class="text-blue-600 text-lg font-extrabold">★</span><br>${q.toFixed(4)}`;
                            actionCell.classList.add('font-extrabold', 'shadow-inner', 'shadow-white/20');
                        }
                        
                        tr.appendChild(actionCell);
                    });
                } else {
                    for(let i=0; i<3; i++) {
                        const actionCell = document.createElement('td');
                        actionCell.className = `p-3 border bg-gray-100 text-gray-400 text-xs font-mono`;
                        actionCell.textContent = "0.0000";
                        tr.appendChild(actionCell);
                    }
                }
                
                tbody.appendChild(tr);
            }
        }

        function getGaugePosition(lux_value) {
            const MIN_LUX = 0;
            const MAX_LUX_VISUAL = 600; 
            
            const clampedValue = Math.min(Math.max(lux_value, MIN_LUX), MAX_LUX_VISUAL);
            const normalized = (clampedValue - MIN_LUX) / (MAX_LUX_VISUAL - MIN_LUX);
            return Math.min(Math.max(normalized * 100, 0), 100);
        }
        
        // Add a timestamp and update the IP/Port for the CSV download link
        function updateLogLink() {
            const link = document.getElementById('log-link');
            if (link) {
                // Appends a unique timestamp to force the browser to request a fresh file
                link.href = `http://${IP_ADDRESS}:${LOG_PORT}/log?` + Date.now(); 
            }
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
                document.getElementById('episode-count').textContent = data.episode;
                
                // 4. Update Self-Tuning Values
                document.getElementById('epsilon-value').textContent = data.epsilon.toFixed(3);
                document.getElementById('explore-pct').textContent = `${(data.epsilon * 100).toFixed(1)}%`;
                document.getElementById('alpha-value').textContent = data.alpha.toFixed(3);

                // 5. Update Log
                document.getElementById('rl-log').textContent = data.log.replace(/\\/g, ''); // Handle escaped characters from ESP String

                // 6. Update Chart Data
                const avgReward = calculateMovingAverage(data.reward);
                // Add new data point for the chart
                if (episodeLabels.length === 0 || data.episode > episodeLabels[episodeLabels.length - 1]) {
                    episodeLabels.push(data.episode);
                    movingAverageData.push(avgReward);

                    // Trim chart data to HISTORY_SIZE
                    if (episodeLabels.length > HISTORY_SIZE) {
                        episodeLabels.shift();
                        movingAverageData.shift();
                    }

                    // Update chart without animation for real-time smoothness
                    rewardChart.update('none');
                }

                // 7. Render Heatmap
                renderQTable(data.qTable, data.state);
            } catch (error) {
                console.error("Error fetching RL data:", error);
                document.getElementById('rl-log').textContent = "ERROR: Could not connect to ESP32 server. Check WiFi connection and IP address.";
            }
            
            updateLogLink(); // Update link every time
            setTimeout(fetchRLData, 500); // Fetch data every 0.5s to match RL episode time
        }

        // Initialize the chart once the DOM is ready
        initChart();
        fetchRLData();
    </script>
</body>
</html>
)rawliteral";


// --------------------------------------------------------
// --- Web Server Handlers (Standard WebServer for dashboard) ---
// --------------------------------------------------------

/**
 * @brief Low-level function to stream HTML content from PROGMEM (Flash).
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
   yield(); // Yield on large streams
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
 */
void handleData() {
 // Calculate the last applied dynamic alpha for display purposes
 float last_alpha = 0.0f;
 if (S_prev >= 0 && A_prev >= 0) {
    last_alpha = ALPHA_START / (ALPHA_START + N_table[S_prev][A_prev]);
 }
    
 String json = "{";
 json += "\"lux\":" + String(current_lux, 2) + ",";
 json += "\"pwm\":" + String(current_pwm_duty) + ",";
 json += "\"state\":" + String(S_current) + ",";
 json += "\"action\":\"" + String(actionNames[A_current]) + "\",";
 json += "\"reward\":" + String(R_current, 3) + ",";
 json += "\"episode\":" + String(episodeCount) + ",";
 // Add self-tuning parameters
 json += "\"epsilon\":" + String(currentEpsilon, 3) + ",";
 json += "\"alpha\":" + String(last_alpha, 3) + ",";
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
// --- Custom Handler for CSV Download (Non-Blocking Fix) ---
// --------------------------------------------------------

/**
 * @brief Dedicated handler for streaming large CSV log data directly via WiFiClient.
 * This bypasses WebServer.h limitations for robust, non-blocking transfer.
 */
void handleCsvDownload(WiFiClient client) {
    long num_entries = buffer_full ? LOG_HISTORY_SIZE : log_index;
    
    if (num_entries == 0) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/plain");
        client.println("Connection: close");
        client.println();
        client.println("No data logged yet. Run the agent for a few episodes.");
        return;
    }
    
    // 1. Send HTTP Headers for File Download
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/csv");
    client.println("Connection: close");
    // 🌟 Crucial for streaming: Use Chunked Transfer Encoding by omitting Content-Length
    client.println("Transfer-Encoding: chunked");
    client.println("Content-Disposition: attachment; filename=rl_log_data.csv");
    client.println("Cache-Control: no-cache, no-store, must-revalidate");
    client.println(); // End of headers
    
    // 2. Send CSV Header (Chunked encoding requires chunk size prefix)
    const char* csv_header = "EPISODE,LUX,STATE_IDX,PWM,REWARD,EPSILON,ALPHA_LAST,Q_S2_LIGHT_UP,Q_S2_LIGHT_DOWN\n";
    size_t header_len = strlen(csv_header);
    
    // Send header chunk (length in hex + data)
    client.printf("%X\r\n", header_len); 
    client.print(csv_header);
    client.print("\r\n");

    // 3. Stream Data Entries in a non-blocking loop
    long start_idx = buffer_full ? log_index % LOG_HISTORY_SIZE : 0;
    long current_episode_num = buffer_full ? (episodeCount - LOG_HISTORY_SIZE + 1) : 1; 
    
    char csv_line_buffer[256]; 
    const size_t MAX_LINE_SIZE = sizeof(csv_line_buffer);

    for (long i = 0; i < num_entries; i++) {
        long idx = (start_idx + i) % LOG_HISTORY_SIZE;
        
        if (!client.connected()) {
            Serial.println("Client disconnected during CSV stream. Aborting.");
            break;
        }

        // Use current Q-table values for the TARGET state (S2)
        float Q_S2_UP = Q_table[2][A_LIGHT_UP];
        float Q_S2_DOWN = Q_table[2][A_LIGHT_DOWN];

        // Format data into buffer
        int len = snprintf(csv_line_buffer, MAX_LINE_SIZE,
                      "%lu,%.2f,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                      current_episode_num, 
                      log_lux[idx], 
                      log_state[idx], 
                      log_pwm[idx], 
                      log_reward[idx], 
                      log_epsilon[idx], 
                      log_alpha[idx],
                      Q_S2_UP,
                      Q_S2_DOWN);
                      
        // Send data chunk (length in hex + data)
        if (len > 0 && len < MAX_LINE_SIZE) {
            client.printf("%X\r\n", len); 
            client.write((const uint8_t*)csv_line_buffer, len);
            client.print("\r\n");
        }

        current_episode_num++;
        
        // 🌟 AGGRESSIVE YIELD: Critical for preventing WDT/Network timeout
        yield(); 
    }
    
    // 4. Send final zero chunk to signal end of stream
    client.print("0\r\n\r\n"); 
    
    client.stop();
    Serial.printf("CSV Stream complete. %lu episodes sent.\n", num_entries);
}

/**
 * @brief Checks for incoming connections on the dedicated log server port (8080).
 */
void checkLogServer() {
    WiFiClient client = logServer.available();
    if (client) {
        // Wait for request line (with timeout)
        unsigned long startTime = millis();
        while (client.connected() && !client.available() && (millis() - startTime < 1000)) {
            delay(1); 
            yield();
        }
        
        if (client.available()) {
            // Read the first line of the request (e.g., "GET /log HTTP/1.1")
            String requestLine = client.readStringUntil('\n');
            requestLine.trim();

            // Only respond to GET /log requests
            if (requestLine.startsWith("GET /log")) {
                Serial.println("Received GET /log request on port 8080. Starting custom stream...");
                // Process and stream the file
                handleCsvDownload(client);
            } else {
                // Send a simple 404/not found for other requests on this port
                client.println("HTTP/1.1 404 Not Found");
                client.println("Connection: close");
                client.println();
            }
        }
        // Ensure the connection is closed after handling
        client.stop(); 
    }
}


// --------------------------------------------------------
// --- Main Setup & Loop ---
// --------------------------------------------------------

void setup() {
    Serial.begin(115200);
    Serial.println("--- Starting ERTAS RL Agent (Web CSV Logging) ---");
    
    Wire.begin(SDA_PIN, SCL_PIN);
 
    initQTable();
    randomSeed(analogRead(0));
    bh1750Init();

    ledc_setup();
    
    // 2. Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected.");
    
    // 3. Setup Web Servers
    // Standard WebServer for Dashboard/JSON API (Port 80)
    server.on("/", HTTP_GET, handleRoot);
    server.on("/data", HTTP_GET, handleData);
    server.begin();

    // Dedicated WiFiServer for robust CSV streaming (Port 8080)
    logServer.begin();

    Serial.println("Web servers started.");
    Serial.println("Dashboard at: http://" + WiFi.localIP().toString() + ":80");
    // NOTE: The /log endpoint is now on port 8080
    Serial.println("CSV Data at: http://" + WiFi.localIP().toString() + ":8080/log");
}

void loop() {
    // 1. Handle Web Server client requests (Standard dashboard traffic)
    server.handleClient();
    
    // 2. Handle dedicated Log Server client requests (Crucial for streaming)
    checkLogServer();

    // 3. RL Episode Logic (Runs every 500 ms)
    static unsigned long lastEpisodeTime = 0;
    if (millis() - lastEpisodeTime >= 500) { 
        lastEpisodeTime = millis();
        
        // 1. SENSE
        current_lux = bh1750ReadLux();
        if (current_lux < 0) current_lux = TARGET_LUX;
        int S_new = getState(current_lux);
        S_current = S_new;
    
        // 2. REWARD
        R_current = calculateReward(current_lux);
        
        // 3. LEARN (Update Q-Table)
        updateQTable(S_prev, A_prev, R_current, S_new);
        
        // 4. ACTION
        int A_new = chooseAction(S_new);
        A_current = A_new;
        
        // 5. EXECUTE
        executeAction(A_new);
        
        // 6. SHIFT & TRACK
        S_prev = S_new;
        A_prev = A_new; 
        episodeCount++; 
        
        // 7. SAVE DATA TO BUFFER
        save_to_log_buffer();
        
        // Optional debug print to Serial
        float log_alpha = (S_prev >= 0 && A_prev >= 0) ?
        ALPHA_START / (ALPHA_START + N_table[S_prev][A_prev]) : 0.0f;
        Serial.printf("Episode %lu: Lux=%.2f, Reward=%.3f, PWM=%d, Epsilon=%.3f\n", 
                       episodeCount, current_lux, R_current, current_pwm_duty, currentEpsilon);
    }
}








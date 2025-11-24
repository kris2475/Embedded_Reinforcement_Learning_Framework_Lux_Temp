/**
 * @file Embedded_Reinforcement_Learning_Framework_Graphical_Dashboard.ino
 * @brief Embedded Reinforcement Learning (ERL) Q-Learning Trainer with Real-Time Graphical Web Visualization.
 * *************************************************************************************************
 * *************************************************************************************************
 * * PROJECT TITLE: REINFORCEMENT LEARNING ADAPTIVE CLIMATE CONTROL UNIT (ACCU) GRAPHICAL TRAINER
 * *************************************************************************************************
 * * DESCRIPTION:
 * * This sketch implements Q-Learning and hosts a modern, graphical web dashboard on the ESP32.
 * * @note **CRUCIAL FIX**: Using low-level, PROGMEM-safe streaming to ensure the large HTML file 
 * * is delivered to the browser without running out of RAM or timing out.
 * *********************************************************************************
 */
#include <Wire.h>
#include <math.h> 
#include <float.h> 
#include <WiFi.h>
#include <WebServer.h>
#include <pgmspace.h> // Include for PROGMEM access functions

// --------------------------------------------------------
// --- WIFI Configuration ---
// --------------------------------------------------------
const char* ssid = "SKYYRMR7";    // <<< UPDATED
const char* password = "K2xWvDFZkuCh"; // <<< UPDATED

WebServer server(80); // Web server on port 80

// --------------------------------------------------------
// --- I2C Sensor Configuration ---
// --------------------------------------------------------
#define SDA_PIN 43
#define SCL_PIN 44
// BH1750 / GY-30 Configuration
#define BH1750_ADDR 0x23
#define BH1750_CONT_HIGH_RES_MODE 0x10

// BMP180 Configuration
#define BMP_ADDR 0x77
#define OSS 3 
#define BMP_CALIBRATION_START_REG 0xAA
#define BMP_CALIBRATION_SIZE 22

// Global BMP180 Calibration Variables
int16_t bmp_AC1, bmp_AC2, bmp_AC3;
uint16_t bmp_AC4, bmp_AC5, bmp_AC6;
int16_t bmp_B1, bmp_B2, bmp_MB, bmp_MC, bmp_MD;

// --- Sensor Functions (omitted for brevity, assume correct) ---
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

int32_t computeB5(int32_t UT) {
  int32_t X1 = ((UT - bmp_AC6) * bmp_AC5) >> 15;
  int32_t X2 = (bmp_MC << 11) / (X1 + bmp_MD);
  return X1 + X2;
}
// --------------------------------------------------------
// --- RL Configuration and Logic (omitted for brevity, assume correct) ---
// --------------------------------------------------------
#define LUX_BIN_LOW_MAX 100.0f 
#define LUX_BIN_HIGH_MIN 500.0f
#define LUX_BINS 3
#define TEMP_BIN_COLD_MAX 18.0f
#define TEMP_BIN_HOT_MIN 24.0f
#define TEMP_BINS 3
#define STATE_SPACE_SIZE (LUX_BINS * TEMP_BINS) 

#define ACTION_SPACE_SIZE 5
enum Action {
  A_LIGHT_UP_FAN_OFF = 0, 
  A_LIGHT_DOWN_FAN_OFF,   
  A_TEMP_UP_LIGHT_OFF,    
  A_TEMP_DOWN_LIGHT_OFF,  
  A_DO_NOTHING            
};
const char* actionNames[ACTION_SPACE_SIZE] = {
  "LIGHT+", "LIGHT-", "TEMP+", "TEMP-", "IDLE"
};

#define ALPHA 0.1f       
#define GAMMA 0.9f       
#define EPSILON 0.3f     
#define CRUCIAL_TD_THRESHOLD 1.0f 

float Q_table[STATE_SPACE_SIZE][ACTION_SPACE_SIZE];
int S_prev = -1;
int A_prev = -1; 
float current_lux = 0.0f;
float current_temp = 0.0f;
int S_current = 0;
int A_current = 4; // Default to IDLE
float R_current = 0.0f;
String rl_log_message = "System Initializing...";

#define TARGET_LUX 300.0f 
#define TARGET_TEMP 21.0f 
#define REWARD_MAX_DISTANCE 10.0f 

int getState(float lux, float temp) {
  int lux_bin = (lux <= LUX_BIN_LOW_MAX) ? 0 : (lux >= LUX_BIN_HIGH_MIN) ? 2 : 1;
  int temp_bin = (temp <= TEMP_BIN_COLD_MAX) ? 0 : (temp >= TEMP_BIN_HOT_MIN) ? 2 : 1;
  return (lux_bin * TEMP_BINS) + temp_bin;
}

float calculateReward(float lux, float temp) {
  float lux_error = fabs(lux - TARGET_LUX) / 100.0f;
  float temp_error = fabs(temp - TARGET_TEMP);
  float distance = sqrt(pow(lux_error, 2) + pow(temp_error, 2));
  float reward = 1.0f - (distance / REWARD_MAX_DISTANCE);
  
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

void executeAction(int A) {
  rl_log_message += " -> ACTUATOR: " + String(actionNames[A]) + " Executed.";
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
const char HTML_CONTENT[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>RL Climate Control Dashboard</title>
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
        .temp-gauge { background: linear-gradient(to right, #60a5fa 16.66%, #10b981 50%, #f87171 83.33%); }
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
    <div class="max-w-7xl mx-auto">
        <h1 class="text-3xl font-extrabold text-gray-800 mb-6 pb-2 flex items-center">
            <svg class="w-8 h-8 mr-2 text-blue-600" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M13 10V3L4 14h7v7l9-11h-7z"></path></svg>
            ACCU RL Trainer Dashboard
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
            
            <!-- TEMP Card -->
            <div class="card p-6 border-l-4 border-red-500">
                <p class="text-sm font-semibold text-gray-500 uppercase">Temperature (°C)</p>
                <p class="text-5xl font-extrabold mt-1 text-red-600 flex items-baseline" id="temp-value">--.-</p>
                <div class="gauge-container temp-gauge mt-2 mb-4">
                    <div class="gauge-target-line" style="left:50%;"></div>
                    <div class="gauge-pointer" id="temp-pointer"></div>
                </div>
                <div class="text-xs text-gray-500 font-medium flex justify-between">
                    <span>Cold (&lt;18)</span>
                    <span class="font-bold text-green-600">Target (21)</span>
                    <span>Hot (&gt;24)</span>
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
                            <th class="p-3 border">State (Lux/Temp)</th>
                            <th class="p-3 border">LIGHT+</th>
                            <th class="p-3 border">LIGHT-</th>
                            <th class="p-3 border">TEMP+</th>
                            <th class="p-3 border">TEMP-</th>
                            <th class="p-3 border">IDLE</th>
                        </tr>
                    </thead>
                    <tbody id="q-table-body">
                        <!-- Rows S0 to S8 will be inserted here by JavaScript -->
                    </tbody>
                </table>
            </div>
            
            <div class="mt-4 text-xs text-gray-600 font-medium">
                <p>Lux Bins (Rows): S0-S2 (Low Lux) | S3-S5 (Medium/Target Lux) | S6-S8 (High Lux)</p>
                <p>Temp Bins (Cols): S(x)0 (Cold) | S(x)1 (Comfort/Target Temp) | S(x)2 (Hot)</p>
            </div>
        </div>
    </div>

    <script>
        const LUX_BINS = ["Low", "Med/Target", "High"];
        const TEMP_BINS = ["Cold", "Comfort/Target", "Hot"];
        const ACTION_NAMES = ["LIGHT+", "LIGHT-", "TEMP+", "TEMP-", "IDLE"];

        // Maps action names to simple icons (using unicode/emojis for simplicity on ESP32)
        const ACTION_ICONS = {
            "LIGHT+": "💡⬆️",
            "LIGHT-": "💡⬇️",
            "TEMP+": "🔥⬆️",
            "TEMP-": "❄️⬇️",
            "IDLE": "⏸️"
        };
        
        // Function to map Q-value to Tailwind color class for Heatmap
        function getHeatmapColor(qValue) {
            const maxQ = 1.0;
            const minQ = -1.0;
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

            for (let s = 0; s < 9; s++) {
                const lux_index = Math.floor(s / 3);
                const temp_index = s % 3;
                
                const tr = document.createElement('tr');
                tr.className = `border-b hover:bg-gray-50 ${s === currentState ? 'state-active bg-blue-50' : ''}`;
                
                // State Label Cell
                const stateCell = document.createElement('td');
                stateCell.className = `p-3 border font-extrabold text-gray-700`;
                stateCell.innerHTML = `S${s} <div class="text-xs font-normal text-gray-500">(${LUX_BINS[lux_index]} | ${TEMP_BINS[temp_index]})</div>`;
                tr.appendChild(stateCell);

                // Find the best action index
                let maxQ = -Infinity;
                let bestActionIndex = -1;
                qTable[s].forEach((q, index) => {
                    if (q > maxQ) {
                        maxQ = q;
                        bestActionIndex = index;
                    }
                });

                // Action Cells
                qTable[s].forEach((q, a) => {
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

        // Function to map sensor value to gauge position (0-100%)
        function getGaugePosition(value, min_val, max_val, target_val) {
            const range = max_val - min_val;
            const clampedValue = Math.min(Math.max(value, min_val), max_val);
            const normalized = (clampedValue - min_val) / range;
            
            if (min_val === 0 && max_val === 1000) {
                 return Math.min(Math.max(normalized * 100, 0), 100);
            }
            
            return Math.min(Math.max(normalized * 100, 0), 100);
        }
        
        // Main fetch loop
        async function fetchRLData() {
            try {
                const response = await fetch('/data');
                const data = await response.json();

                // 1. Update Sensor Values & Gauges
                document.getElementById('lux-value').textContent = data.lux.toFixed(2);
                document.getElementById('temp-value').textContent = data.temp.toFixed(2);

                const luxPos = getGaugePosition(data.lux, 0, 1000, 300); 
                document.getElementById('lux-pointer').style.left = `${luxPos}%`;

                const tempPos = getGaugePosition(data.temp, 15, 27, 21); 
                document.getElementById('temp-pointer').style.left = `${tempPos}%`;

                // 2. Update RL Status
                document.getElementById('reward-value').textContent = data.reward.toFixed(3);
                document.getElementById('state-index').textContent = data.state;
                document.getElementById('action-taken').textContent = data.action;
                document.getElementById('action-icon').textContent = ACTION_ICONS[data.action] || '❓';


                // 3. Update Log
                document.getElementById('rl-log').textContent = data.log;

                // 4. Render Heatmap
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
 * to the client in small chunks. This avoids crashing the heap memory.
 */
void sendLargeHtmlFromProgmem(const char *html_content) {
  // Get the size of the PROGMEM string
  size_t html_size = strlen_P(html_content);
  
  // 1. Send the HTTP Header
  // We use CONTENT_LENGTH to tell the browser exactly how big the file is, 
  // which is safer than chunked transfer (CONTENT_LENGTH_UNKNOWN) in this context.
  server.sendHeader("Connection", "close");
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.setContentLength(html_size);
  server.send(200, "text/html", ""); // Send the header
  
  // 2. Stream the content in chunks
  WiFiClient client = server.client();
  size_t bytes_sent = 0;
  const size_t chunk_size = 512; // Send 512 bytes at a time
  
  Serial.print("Attempting to stream HTML (Size: ");
  Serial.print(html_size);
  Serial.println(" bytes)...");

  while (bytes_sent < html_size) {
    size_t remaining = html_size - bytes_sent;
    size_t current_chunk_size = (remaining < chunk_size) ? remaining : chunk_size;
    
    // Create a temporary buffer on the stack (small)
    char buffer[chunk_size];
    
    // Read the chunk from PROGMEM into the buffer
    memcpy_P(buffer, html_content + bytes_sent, current_chunk_size);

    // Send the buffer content
    client.write((const uint8_t*)buffer, current_chunk_size);

    bytes_sent += current_chunk_size;
    
    // Yield to the ESP32 scheduler to prevent watchdog timeout
    yield(); 
    
    if (client.getWriteError()) {
      Serial.println("Client write error occurred. Aborting stream.");
      break;
    }
  }
  
  // 3. Close the connection
  client.stop(); 
  Serial.println("HTML stream complete. Connection closed.");
}


/**
 * @brief Handles the root request by streaming the dashboard HTML.
 */
void handleRoot() {
  sendLargeHtmlFromProgmem(HTML_CONTENT);
}


// Serves the real-time data as JSON
void handleData() {
  String json = "{";
  json += "\"lux\":" + String(current_lux, 2) + ",";
  json += "\"temp\":" + String(current_temp, 2) + ",";
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
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // 1. Initialize RL and Sensors
  initQTable();
  randomSeed(analogRead(0));
  bh1750Init();
  readCalibration();
  
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
    if (current_lux < 0) current_lux = TARGET_LUX; 
    
    int32_t UT = readRawTemp();
    int32_t B5 = computeB5(UT);
    current_temp = ((B5 + 8) >> 4) / 10.0;
    
    int S_new = getState(current_lux, current_temp);
    S_current = S_new;
  
    // 2. REWARD: Calculate Immediate Reward
    R_current = calculateReward(current_lux, current_temp);
    
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
  }
}








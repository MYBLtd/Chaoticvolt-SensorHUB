#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include "onewire_manager.h"
#include "webserver.h"
#include "mqtt_manager.h"
#include "config.h"
#include <SPIFFS.h>
#include <ArduinoJson.h>

// Initialize webServer on port 80

TempWebServer::TempWebServer(uint16_t port) 
    : AsyncWebServer(port), events("/events"), mqttManager(nullptr) {
    
    networkMutex = xSemaphoreCreateMutex();
    if (networkMutex == NULL) {
        Serial.println("Failed to create network mutex");
        return;
    }

    // Setup routes
    on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", DASHBOARD_HTML);
    });
    
    on("/style.css", HTTP_GET, [this](AsyncWebServerRequest *request) {
        request->send(200, "text/css", DASHBOARD_CSS);
    });
    
    // Serve static files from SPIFFS
    serveStatic("/", SPIFFS, "/");
    
    // Add default headers
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    
    // Add event handler
    addHandler(&events);
}

// CSS stored in PROGMEM
const char TempWebServer::DASHBOARD_CSS[] PROGMEM = R"(
body {
  font-family: sans-serif;
  margin: 0;
  display: flex;
  flex-direction: column;
  min-height: 100vh;
  padding: 20px;
  box-sizing: border-box;
  background-color: #f0f0f0;
}

.page-container {
  display: flex;
  flex-direction: column;
  width: 100%;
  max-width: 1200px;
  margin: 0 auto;
  gap: 20px;
}

/* Top row - Single column, full width */
.header-panel {
  background-color: white;
  padding: 20px;
  border-radius: 10px;
  box-shadow: 0 2px 4px rgba(0,0,0,0.1);
  text-align: center;
}

.page-title {
  margin: 0;
  color: #333;
}

/* Second row - Two columns with 10px gap */
.dashboard {
  display: flex;
  gap: 10px;
}

.control-panel {
  background: white;
  padding: 20px;
  border-radius: 10px;
  box-shadow: 0 2px 4px rgba(0,0,0,0.1);
  width: 30%; /* Smaller width for left column */
}

.graph-panel {
  background: white;
  padding: 20px;
  border-radius: 10px;
  box-shadow: 0 2px 4px rgba(0,0,0,0.1);
  width: 70%; /* Larger width for right column */
}

/* Third row - Two equal-width columns with 10px gap */
.bottom-row {
  display: flex;
  gap: 10px;
}

.sensor-panel {
  background: white;
  padding: 20px;
  border-radius: 10px;
  box-shadow: 0 2px 4px rgba(0,0,0,0.1);
  width: 50%;
}

.logo-panel {
  background: white;
  padding: 20px;
  border-radius: 10px;
  box-shadow: 0 2px 4px rgba(0,0,0,0.1);
  width: 50%;
  display: flex;
  justify-content: center;
  align-items: center;
}

.logo-container {
  display: flex;
  justify-content: center;
  align-items: center;
  width: 100%;
  height: 100%;
}

.corner-logo {
  max-width: 100%;
  max-height: 100%;
  object-fit: contain;
}

/* Additional styling for specific elements */
.temperature-display {
  font-size: 2em;
  text-align: center;
  margin: 20px 0;
}

.controls {
  display: flex;
  justify-content: space-around;
}

.control-group {
  text-align: center;
}

.btn {
  margin: 5px;
  padding: 10px 15px;
  background-color: #4CAF50;
  color: white;
  border: none;
  border-radius: 5px;
  cursor: pointer;
}

.btn:hover {
  background-color: #45a049;
}

.graph-svg {
  width: 100%;
  height: 300px;
}

.grid-line {
  stroke: #e0e0e0;
  stroke-width: 1;
}

.axis-label {
  font-size: 12px;
  fill: #666;
  text-anchor: middle;
}

.sensor-list {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(150px, 1fr));
  gap: 10px;
}

.sensor-card {
  background-color: #f9f9f9;
  padding: 10px;
  border-radius: 5px;
  text-align: center;
}

.sensor-id {
  font-weight: bold;
  margin-bottom: 5px;
}

.sensor-temp {
  color: #666;
}

#status {
  text-align: center;
  margin-top: 15px;
  font-weight: bold;
}

.graph-container {
    background: white;
    padding: 20px;
    border-radius: 8px;
    box-shadow: 0 2px 4px rgba(0,0,0,0.1);
    margin-top: 20px;
}

.graph-svg {
    width: 100%;
    height: 300px;
    background: #fff;
}

.grid-line {
    stroke: #eee;
    stroke-width: 1;
}

.temp-line {
    fill: none;
    stroke: #2196F3;
    stroke-width: 2;
}

.time-label, .temp-label {
    font-size: 12px;
    fill: #666;
}
)";

// HTML template stored in PROGMEM
const char TempWebServer::DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Chaoticvolt SensorHUB01 Dashboard</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="stylesheet" href="/style.css">
</head>
<body>
    <div class="page-container">
        <div class="header-panel">
            <h1 class="page-title">Chaoticvolt SensorHUB01 Dashboard</h1>
        </div>
        <div class="dashboard">
            <div class="control-panel">
                <h2>Buffervat Temperatuur</h2>
                <div class="temperature-display">
                    <span id="temp">--</span>°C
                </div>
                <div class="controls">
                    <div class="control-group">
                        <h3>Pomp CV</h3>
                        <button onclick="setRelay(1, 'ON')" class="btn">ON</button>
                        <button onclick="setRelay(1, 'OFF')" class="btn">OFF</button>
                    </div>
                    <div class="control-group">
                        <h3>Pomp Solar</h3>
                        <button onclick="setRelay(2, 'ON')" class="btn">ON</button>
                        <button onclick="setRelay(2, 'OFF')" class="btn">OFF</button>
                    </div>
                </div>
                <div id="status"></div>
            </div>
            <div class="graph-panel">
                <h2>Temperature History</h2>
                <svg class="graph-svg" width="1000" height="300" viewBox="0 0 1000 300">
                    <g class="grid"></g>
                    <g class="labels"></g>
                    <path class="temp-line" d=""/>
                </svg>
            </div>
        </div>
        <div class="bottom-row">
            <div class="sensor-panel">
                <h2>Sensors</h2>
                <div id="sensorList" class="sensor-list"></div>
            </div>
            <div class="logo-panel">
                <div class="logo-container">
                    <img src="/logo.png" alt="Logo" class="corner-logo">
                </div>
            </div>
        </div>
    </div>

    <script>
        const eventSource = new EventSource('/events');
        const tempDisplay = document.getElementById('temp');
        console.log('EventSource initialized');

        eventSource.onopen = () => {
            console.log('EventSource connected');
        };

        eventSource.onerror = (err) => {
            console.error('EventSource error:', err);
        };

        eventSource.addEventListener('temperature', (e) => {
            console.log('Temperature event received:', e.data);
            const temp = parseFloat(e.data);
            if (!isNaN(temp)) {
                tempDisplay.textContent = temp.toFixed(1);
                updateGraph(temp);
            }
        });

        eventSource.addEventListener('sensors', (e) => {
            console.log('Received sensor data:', e.data);
            try {
                const sensors = JSON.parse(e.data);
                if (sensors && Array.isArray(sensors)) {
                    updateSensorList(sensors);
                    console.log('Updated sensor list with:', sensors);
                }
            } catch (err) {
                console.error('Error parsing sensor data:', err);
            }
        });

        function updateSensorList(sensors) {
            const container = document.getElementById('sensorList');
            if (!container) return;
            
            container.innerHTML = sensors.map(sensor => `
                <div class="sensor-card">
                    <div class="sensor-id">${sensor.address}</div>
                    <div class="sensor-temp">${sensor.temp.toFixed(1)}°C</div>
                </div>
            `).join('');
        }

        const statusDisplay = document.getElementById('status');
        const svg = document.querySelector('.graph-svg');
        const tempLine = svg.querySelector('.temp-line');
        const tempHistory = [];
        const MAX_POINTS = 360; // 6 hours
        
        // Add cache for sensor data
        let cachedSensorData = [];
        
        function updateSensorList(sensors) {
            // Update cache
            cachedSensorData = sensors;
            const sensorList = document.getElementById('sensor-list');
            sensorList.innerHTML = cachedSensorData.map(sensor => `
                <div class="sensor-row">
                    <span class="sensor-address">${sensor.address}</span>
                    <span class="sensor-value">${sensor.value.toFixed(1)}°C</span>
                </div>
            `).join('');
        }

        // Display cached data on page load
        updateSensorList(cachedSensorData);

        eventSource.addEventListener('sensors', (e) => {
            const sensors = JSON.parse(e.data);
            if (sensors && sensors.length > 0) {
                const container = document.getElementById('sensorList');
                if (!container) return;
                
                container.innerHTML = sensors.map(sensor => `
                    <div class="sensor-row">
                        <span class="sensor-address">${sensor.address}</span>
                        <span class="sensor-value">${sensor.temp.toFixed(1)}°C</span>
                    </div>
                `).join('');
            }
        });

        eventSource.addEventListener('sensors', function(e) {
            const sensors = JSON.parse(e.data);
            cachedSensorData = sensors; // Update cache
            updateSensorList(sensors);
        });

        function updateSensorList(sensors) {
            const container = document.getElementById('sensorList');
            if (!container) return;
            
            container.innerHTML = sensors.map(sensor => `
                <div class="sensor-card">
                    <div class="sensor-id">${sensor.address}</div>
                    <div class="sensor-temp">${sensor.temp.toFixed(1)}°C</div>
                </div>
            `).join('');
        }

        eventSource.onopen = () => {
            statusDisplay.textContent = 'Live updates actief';
            statusDisplay.style.color = '#4CAF50';
        };

        eventSource.onerror = () => {
            statusDisplay.textContent = 'Geen live updates';
            statusDisplay.style.color = '#f44336';
            // Keep displaying cached data during connection issues
            if (cachedSensorData.length > 0) {
                updateSensorList(cachedSensorData);
            }
        };

        function initGrid() {
            const grid = svg.querySelector('.grid');
            const labels = svg.querySelector('.labels');
            
            // Horizontal grid lines and temperature labels
            for(let i = 0; i <= 5; i++) {
                const y = 50 + i * 40;
                grid.innerHTML += `<line x1="50" y1="${y}" x2="950" y2="${y}" class="grid-line"/>`;
                labels.innerHTML += `<text x="30" y="${y + 5}" class="axis-label">${40 - i * 10}°C</text>`;
            }
            
            // Time labels
            for(let i = 0; i <= 6; i++) {
                const x = 50 + i * 150;
                grid.innerHTML += `<line x1="${x}" y1="50" x2="${x}" y2="250" class="grid-line"/>`;
                labels.innerHTML += `<text x="${x}" y="270" class="axis-label">${6 - i}h ago</text>`;
            }
        }
        
        function updateGraph(temp) {
            const now = Date.now();
            tempHistory.push({temp: temp, time: now});
            
            // Keep only last 6 hours of data (360 points at 1 per minute)
            while (tempHistory.length > MAX_POINTS) {
                tempHistory.shift();
            }
            
            // Generate SVG path with proper time scaling
            const points = tempHistory.map((data, i) => {
                const x = 950 - ((tempHistory.length - 1 - i) * (900 / MAX_POINTS));
                const y = 250 - ((data.temp + 30) / 70 * 200); // Scale temp to graph height
                return `${x},${y}`;
            });
            
            // Update path
            if (points.length > 0) {
                tempLine.setAttribute('d', `M${points.join(' L')}`);
            }
            
            // Update grid labels
            updateGridLabels();
        }

        function updateGridLabels() {
            const labels = document.querySelectorAll('.time-label');
            const now = new Date();
            
            labels.forEach((label, i) => {
                const hoursAgo = 6 - i;
                const time = new Date(now - hoursAgo * 3600000);
                label.textContent = time.toLocaleTimeString([], {hour: '2-digit', minute:'2-digit'});
            });
        }

        // Initialize grid with time labels
        initGrid();

        // Update every minute
        setInterval(() => updateGridLabels(), 60000);
        
        eventSource.addEventListener('temperature', (e) => {
            const temp = parseFloat(e.data);
            tempDisplay.textContent = temp.toFixed(1);
            updateGraph(temp);
        });

        function setRelay(relay, state) {
            fetch(`/relay${relay}/set`, {
                method: 'POST',
                headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                body: `state=${state}`
            })
            .then(response => response.text())
            .then(result => console.log('Relay result:', result))
            .catch(error => console.error('Error:', error));
        }

    </script>
</body>
</html>
)rawliteral";


void TempWebServer::begin() {
    // Mount SPIFFS first
    if(!SPIFFS.begin(true)) {
        Serial.println("SPIFFS Mount Failed");
        return;
    }
    Serial.println("SPIFFS mounted successfully");

    // List files in SPIFFS
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while(file) {
        Serial.printf("Found file: %s, size: %d\n", file.name(), file.size());
        file = root.openNextFile();
    }

    // Setup event source
    events.onConnect([](AsyncEventSourceClient *client) {
        client->send("hello!", NULL, millis(), 1000);
    });
    
    // Add handlers
    addHandler(&events);

    // Start web server
    AsyncWebServer::begin();
    Serial.println("Web server started");
}

void TempWebServer::sendTemperature(float temp) {
    if (temp != DEVICE_DISCONNECTED_C) {
        String tempStr = String(temp, 1);
        events.send(tempStr.c_str(), "temperature", millis());
        Serial.printf("[WebServer] Temperature event sent: %s°C\n", tempStr.c_str());
    }
}

void TempWebServer::sendSensorData(const std::vector<std::pair<String, float>>& sensors) {
    if (sensors.empty()) return;

    DynamicJsonDocument doc(1024);
    JsonArray array = doc.to<JsonArray>();
    
    for(const auto& sensor : sensors) {
        JsonObject obj = array.createNestedObject();
        obj["address"] = sensor.first;
        obj["temp"] = sensor.second;
    }

    String jsonString;
    serializeJson(doc, jsonString);
    
    events.send(jsonString.c_str(), "sensors", millis());
    Serial.printf("Sending sensor data: %s\n", jsonString.c_str());
}

void TempWebServer::setMQTTManager(MQTTManager* manager) {
    mqttManager = manager;
    // Optional: Log successful connection
    Serial.println("[WebServer] MQTT manager initialized");
}

void sendSensorData(const std::vector<std::pair<String, float>>& sensorData) {
    // Implementation for sending sensor data
}

// Global instance (matches extern declaration)
TempWebServer webServer(80);

bool MQTTManager::publishTopic(const char* topic, const char* payload) {
    if (!mqttClient.connected()) {
        return false;
    }
    return mqttClient.publish(topic, payload, true);  // Retained message
}




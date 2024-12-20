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

.page-container {å
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
    <title>Temperature Dashboard</title>
    <link rel="stylesheet" href="/style.css">
    <style>
        /* Add vertical spacing between rows */
        .header-panel {
            margin-bottom: 10px;
        }
        .dashboard {
            margin-bottom: 10px;
        }
        
        /* Style for button groups */
        .relay-button-group {
            display: flex;
            gap: 5px;
            margin-top: 5px;
            justify-content: center;
        }

        /* Center temperature content */
        .control-panel {
            text-align: center;
        }

        .temp-display {
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            margin-bottom: 20px;
        }

        .temp-display h2 {
            margin-bottom: 10px;
        }

        #tempDisplay {
            font-size: 2.5em;
            font-weight: bold;
            margin: 10px 0;
        }

        .relay-controls {
            text-align: center;
        }

        .relay {
            margin: 15px 0;
        }

        .sensor-select {
            margin-top: 20px;
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 10px;
        }

        #sensorPicker {
            width: 80%;
            padding: 5px;
            text-align: center;
        }
    </style>
</head>
<body>
    <div class="page-container">
        <!-- First row - Single column with title -->
        <div class="header-panel">
            <h1 class="page-title">Temperature Monitoring System</h1>
        </div>
        
        <!-- Second row - Two columns (1/3 and 2/3) -->
        <div class="dashboard">
            <div class="control-panel">
                <div class="temp-display">
                    <h2>Current Temperature</h2>
                    <div id="tempDisplay">--.-</div>
                    <span>°C</span>
                </div>
                
                <div class="relay-controls">
                    <h2>Relay Controls</h2>
                    <div class="relay">
                        <label>Relay 1:</label>
                        <div class="relay-button-group">
                            <button onclick="setRelay(1, true)" class="btn">On</button>
                            <button onclick="setRelay(1, false)" class="btn">Off</button>
                        </div>
                    </div>
                    <div class="relay">
                        <label>Relay 2:</label>
                        <div class="relay-button-group">
                            <button onclick="setRelay(2, true)" class="btn">On</button>
                            <button onclick="setRelay(2, false)" class="btn">Off</button>
                        </div>
                    </div>
                    <div class="sensor-select">
                        <label for="sensorPicker">Monitor Sensor:</label>
                        <select id="sensorPicker" onchange="onSensorSelected(this.value)">
                            <option value="">Select Sensor...</option>
                        </select>
                    </div>
                </div>
            </div>
            
            <div class="graph-panel">
                <h2>Temperature History</h2>
                <svg id="tempGraph" class="graph-svg" preserveAspectRatio="none">
                    <path id="tempLine" stroke="blue" fill="none" stroke-width="2"/>
                    <g id="gridLines"></g>
                    <g id="axisLabels"></g>
                </svg>
            </div>
        </div>
        
        <!-- Third row - Two equal columns -->
        <div class="bottom-row">
            <div class="sensor-panel">
                <h2>All Sensors</h2>
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
        let selectedSensor = localStorage.getItem('selectedSensor') || '';
        let tempData = [];
        const maxDataPoints = 50;

        function setRelay(relayNumber, state) {
            fetch(`/relay${relayNumber}/set`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/x-www-form-urlencoded',
                },
                body: `state=${state ? 'on' : 'off'}`
            })
            .then(response => response.text())
            .then(status => console.log(`Relay ${relayNumber} set to ${state ? 'ON' : 'OFF'}`))
            .catch(error => console.error('Error:', error));
        }

        function onSensorSelected(sensorAddress) {
            selectedSensor = sensorAddress;
            localStorage.setItem('selectedSensor', sensorAddress);
            updateTempDisplay();
            updateGraph();
        }

        function updateSensorPicker(sensors) {
            const picker = document.getElementById('sensorPicker');
            picker.innerHTML = '<option value="">Select Sensor...</option>';
            sensors.forEach(sensor => {
                const option = document.createElement('option');
                option.value = sensor.address;
                option.textContent = sensor.address;
                picker.appendChild(option);
            });
            picker.value = selectedSensor;
        }

        function updateTempDisplay() {
            const tempDisplay = document.getElementById('tempDisplay');
            const currentSensor = tempData.find(sensor => sensor.address === selectedSensor);
            if (currentSensor) {
                tempDisplay.textContent = currentSensor.temp.toFixed(1);
            } else {
                tempDisplay.textContent = '--.-';
            }
        }

        function updateGraph() {
            const svg = document.getElementById('tempGraph');
            const path = document.getElementById('tempLine');
            const width = svg.width.baseVal.value;
            const height = svg.height.baseVal.value;
            const padding = 40;

            const currentSensorData = tempData.filter(data => data.address === selectedSensor);
            const dataPoints = currentSensorData.slice(-maxDataPoints);

            if (dataPoints.length < 2) return;

            const xScale = (width - 2 * padding) / (dataPoints.length - 1);
            const yMin = Math.min(...dataPoints.map(d => d.temp));
            const yMax = Math.max(...dataPoints.map(d => d.temp));
            const yScale = (height - 2 * padding) / (yMax - yMin);

            let pathD = `M ${padding} ${height - padding - (dataPoints[0].temp - yMin) * yScale}`;
            for (let i = 1; i < dataPoints.length; i++) {
                pathD += ` L ${padding + i * xScale} ${height - padding - (dataPoints[i].temp - yMin) * yScale}`;
            }
            path.setAttribute('d', pathD);

            updateGridAndLabels(svg, width, height, padding, yMin, yMax);
        }

        function updateGridAndLabels(svg, width, height, padding, yMin, yMax) {
            const gridLines = document.getElementById('gridLines');
            const axisLabels = document.getElementById('axisLabels');
            gridLines.innerHTML = '';
            axisLabels.innerHTML = '';

            for (let i = 0; i <= 4; i++) {
                const x = padding + i * (width - 2 * padding) / 4;
                gridLines.innerHTML += `<line x1="${x}" y1="${padding}" x2="${x}" y2="${height - padding}" class="grid-line" />`;
                axisLabels.innerHTML += `<text x="${x}" y="${height - padding + 20}" class="axis-label">${-50 + i * 25}m</text>`;
            }

            for (let i = 0; i <= 4; i++) {
                const y = height - padding - i * (height - 2 * padding) / 4;
                gridLines.innerHTML += `<line x1="${padding}" y1="${y}" x2="${width - padding}" y2="${y}" class="grid-line" />`;
                const temp = yMin + (i / 4) * (yMax - yMin);
                axisLabels.innerHTML += `<text x="${padding - 10}" y="${y}" class="axis-label" text-anchor="end">${temp.toFixed(1)}°C</text>`;
            }
        }

        function updateSensorList(sensors) {
            const sensorList = document.getElementById('sensorList');
            sensorList.innerHTML = '';
            sensors.forEach(sensor => {
                const sensorDiv = document.createElement('div');
                sensorDiv.className = 'sensor-card';
                sensorDiv.innerHTML = `
                    <div class="sensor-id">${sensor.address}</div>
                    <div class="sensor-temp">${sensor.temp.toFixed(1)}°C</div>
                `;
                sensorList.appendChild(sensorDiv);
            });
        }

        const eventSource = new EventSource('/events');

        eventSource.addEventListener('sensors', function(e) {
            const sensors = JSON.parse(e.data);
            tempData = sensors;
            updateSensorPicker(sensors);
            updateSensorList(sensors);
            updateTempDisplay();
            updateGraph();
        });

        document.getElementById('sensorPicker').value = selectedSensor;
        
        const tempHistory = [];
        const MAX_POINTS = 360; // 6 hours at 1 reading/minute
        
        function updateGraph() {
            if (!selectedSensor || !tempData) return;
            
            const svg = document.getElementById('tempGraph');
            const path = document.getElementById('tempLine');
            const width = svg.clientWidth;
            const height = svg.clientHeight;
            const padding = 40;
            
            // Find sensor data
            const sensorData = tempData.find(s => s.address === selectedSensor);
            if (!sensorData) return;
            
            // Update history
            tempHistory.push({
                temp: sensorData.temp,
                time: new Date()
            });
            
            if (tempHistory.length > MAX_POINTS) {
                tempHistory.shift();
            }
            
            // Scale data points
            const xScale = (width - 2 * padding) / (tempHistory.length - 1);
            const yMin = Math.min(...tempHistory.map(d => d.temp));
            const yMax = Math.max(...tempHistory.map(d => d.temp));
            const yScale = (height - 2 * padding) / (yMax - yMin || 1);
            
            // Create path
            let pathD = '';
            tempHistory.forEach((point, i) => {
                const x = padding + (i * xScale);
                const y = height - padding - ((point.temp - yMin) * yScale);
                pathD += `${i === 0 ? 'M' : 'L'} ${x} ${y}`;
            });
            
            path.setAttribute('d', pathD);
            
            // Update grid and labels
            updateGridAndLabels(svg, width, height, padding, yMin, yMax);
        }
        
        function updateGridAndLabels(svg, width, height, padding, yMin, yMax) {
            const grid = document.getElementById('gridLines');
            const labels = document.getElementById('axisLabels');
            grid.innerHTML = '';
            labels.innerHTML = '';
            
            // Add horizontal grid lines
            for (let i = 0; i <= 4; i++) {
                const y = padding + i * (height - 2 * padding) / 4;
                const temp = yMax - (i * (yMax - yMin) / 4);
                grid.innerHTML += `<line x1="${padding}" y1="${y}" x2="${width-padding}" y2="${y}" class="grid-line"/>`;
                labels.innerHTML += `<text x="${padding-5}" y="${y}" class="temp-label" text-anchor="end">${temp.toFixed(1)}°C</text>`;
            }
            
            // Add time labels
            for (let i = 0; i <= 6; i++) {
                const x = padding + i * (width - 2 * padding) / 6;
                const hours = 6 - i;
                grid.innerHTML += `<line x1="${x}" y1="${padding}" x2="${x}" y2="${height-padding}" class="grid-line"/>`;
                labels.innerHTML += `<text x="${x}" y="${height-padding+20}" class="time-label" text-anchor="middle">${hours}h</text>`;
            }
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

    // Add CORS headers
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    
    // Serve static files
    on("/", HTTP_GET, [this](AsyncWebServerRequest *request) {
        request->send(200, "text/html", DASHBOARD_HTML);
    });

    on("/style.css", HTTP_GET, [this](AsyncWebServerRequest *request) {
        request->send(200, "text/css", DASHBOARD_CSS);
    });

    on("/logo.png", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/logo.png", "image/png");
    });
        // Add relay control endpoints
    on("/relay1/set", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (request->hasParam("state", true)) {
            String state = request->getParam("state", true)->value();
            bool newState = (state == "on" || state == "1" || state == "true");
            
            // Update global state
            updateRelayState(SYSTEM_RELAY1_PIN, newState);
            
            // Publish via MQTT
            if (mqttManager) {
                String topic = String(SYSTEM_NAME) + "/" + 
                              String(MQTT_CLIENT_ID) + "/" + 
                              SYSTEM_RELAY1_TOPIC;
                
                if (mqttManager->publishTopic(topic.c_str(), newState ? "on" : "off")) {
                    request->send(200, "text/plain", "OK");
                } else {
                    request->send(500, "text/plain", "Failed to publish");
                }
            } else {
                request->send(500, "text/plain", "MQTT not initialized");
            }
        }
    });

    on("/relay2/set", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (request->hasParam("state", true)) {
            String state = request->getParam("state", true)->value();
            bool newState = (state == "on" || state == "1" || state == "true");
            
            // Update global state
            updateRelayState(SYSTEM_RELAY2_PIN, newState);
            
            // Publish via MQTT
            if (mqttManager) {
                String topic = String(SYSTEM_NAME) + "/" + 
                              String(MQTT_CLIENT_ID) + "/" + 
                              SYSTEM_RELAY2_TOPIC;
                
                if (mqttManager->publishTopic(topic.c_str(), newState ? "on" : "off")) {
                    request->send(200, "text/plain", "OK");
                } else {
                    request->send(500, "text/plain", "Failed to publish");
                }
            } else {
                request->send(500, "text/plain", "MQTT not initialized");
            }
        }
    });

    on("/relay1", HTTP_POST, [this](AsyncWebServerRequest *request) {
        // Get state parameter from request
        if (!request->hasParam("state", true)) {
            request->send(400, "text/plain", "Missing state parameter");
            return;
        }
        
        String stateParam = request->getParam("state", true)->value();
        bool newState = (stateParam == "on" || stateParam == "1" || stateParam == "true");
        
        this->updateRelayState(SYSTEM_RELAY1_PIN, newState);
        request->send(200, "text/plain", newState ? "ON" : "OFF");
    });
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

void TempWebServer::updateRelayState(uint8_t pin, bool state) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, state ? HIGH : LOW);
    
    // Optional: Log relay state change
    Serial.printf("[WebServer] Relay pin %d set to %s\n", 
                 pin, 
                 state ? "ON" : "OFF");
}


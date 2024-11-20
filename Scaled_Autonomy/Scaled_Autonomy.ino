#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <DHT11.h>

Adafruit_MPU6050 mpu;
DHT11 dht11(4);
#define irPin 34

// Replace with your network credentials
const char *ssid = "ALVA4eBn6fh";
const char *password = "25av5nuh9k17";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Motor Config
const int motor1Pin1 = 32, motor1Pin2 = 33, motor2Pin1 = 25, motor2Pin2 = 26;
const int freq = 5000, resolution = 8;
const int pwmChannelA1 = 0, pwmChannelA2 = 1, pwmChannelB1 = 2, pwmChannelB2 = 3;
int motorAspeed = 0, motorBspeed = 0;
bool motorAdirForward = true, motorBdirForward = true;

//================= Sensor variables =================
JSONVar readings;
//================= Sensor variables =================

void handleMotors() {
  // Control Motor A
  if (motorAspeed > 0) {
    if (motorAdirForward) {
      ledcWrite(pwmChannelA1, motorAspeed);
      ledcWrite(pwmChannelA2, 0);
    } else {
      ledcWrite(pwmChannelA1, 0);
      ledcWrite(pwmChannelA2, motorAspeed);
    }
  } else {
    ledcWrite(pwmChannelA1, 0);
    ledcWrite(pwmChannelA2, 0);
  }

  // Control Motor B
  if (motorBspeed > 0) {
    if (motorBdirForward) {
      ledcWrite(pwmChannelB1, motorBspeed);
      ledcWrite(pwmChannelB2, 0);
    } else {
      ledcWrite(pwmChannelB1, 0);
      ledcWrite(pwmChannelB2, motorBspeed);
    }
  } else {
    ledcWrite(pwmChannelB1, 0);
    ledcWrite(pwmChannelB2, 0);
  }
}

String getGyroReadings() {
  sensors_event_t a, g, temp;
  float gyroX = 0, gyroY = 0, gyroZ = 0;
  float gyroXerror = 0.07, gyroYerror = 0.03, gyroZerror = 0.01;

  mpu.getEvent(&a, &g, &temp);

  float gyroX_temp = g.gyro.x;
  if (abs(gyroX_temp) > gyroXerror) {
    gyroX += gyroX_temp / 50.00;
  }

  float gyroY_temp = g.gyro.y;
  if (abs(gyroY_temp) > gyroYerror) {
    gyroY += gyroY_temp / 70.00;
  }

  float gyroZ_temp = g.gyro.z;
  if (abs(gyroZ_temp) > gyroZerror) {
    gyroZ += gyroZ_temp / 90.00;
  }

  readings["gyroX"] = String(gyroX);
  readings["gyroY"] = String(gyroY);
  readings["gyroZ"] = String(gyroZ);

  String jsonString = JSON.stringify(readings);
  return jsonString;
}

String getAccReadings() {
  sensors_event_t a, g, temp;
  float accX = 0, accY = 0, accZ = 0;

  mpu.getEvent(&a, &g, &temp);
  // Get current acceleration values
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  readings["accX"] = String(accX);
  readings["accY"] = String(accY);
  readings["accZ"] = String(accZ);
  String accString = JSON.stringify(readings);
  return accString;
}

String getTemperature() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  readings["mpu_temperature"] = temp.temperature;
  return String(temp.temperature);
}

String getDht() {
  int dhtTemperature = 0, dhtHumidity = 0;
  // Attempt to read the temperature and humidity values from the DHT11 sensor.
  int result = dht11.readTemperatureHumidity(dhtTemperature, dhtHumidity);

  if (result == 0) {
    readings["dhtTemperature"] = String(dhtTemperature);
    readings["dhtHumidity"] = String(dhtHumidity);
    String accString = JSON.stringify(readings);
    return accString;
  } else {
    readings["dhtTemperature"] = "ERROR";
    readings["dhtHumidity"] = "ERROR";
    String accString = JSON.stringify(readings);
    return accString;
  }
}

String getIr() {
  int IRread = digitalRead(irPin);
  readings["ir"] = IRread;
  return String(IRread);
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("WebSocket client connected");
    client->text("Welcome to Scaled Autonomy");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("WebSocket client disconnected");
  } else if (type == WS_EVT_DATA) {
    String message = String((char *)data).substring(0, len);
    Serial.println("WebSocket message received: " + message);

    JSONVar receivedData = JSON.parse(message);
    if (JSON.typeof(receivedData) == "undefined") {
      Serial.println("Invalid JSON received.");
      return;
    }

    String response = "Received Parameters:\n";

    // Motor A
    if (receivedData.hasOwnProperty("motorAspeed")) {
      motorAspeed = int(receivedData["motorAspeed"]);
      motorAspeed = constrain(motorAspeed, 0, 255);
      response += "Motor A Speed: " + String(motorAspeed) + "\n";
    } else {
      response += "Motor A Speed: (unchanged) " + String(motorAspeed) + "\n";
    }

    if (receivedData.hasOwnProperty("motorAdir")) {
      int motorAdir = int(receivedData["motorAdir"]);
      motorAdirForward = (motorAdir == 1);
      response += "Motor A Direction: " + String(motorAdirForward) + "\n";
    } else {
      response += "Motor A Direction: (unchanged) " + String(motorAdirForward) + "\n";
    }

    // Motor B
    if (receivedData.hasOwnProperty("motorBspeed")) {
      motorBspeed = int(receivedData["motorBspeed"]);
      motorBspeed = constrain(motorBspeed, 0, 255);
      response += "Motor B Speed: " + String(motorBspeed) + "\n";
    } else {
      response += "Motor B Speed: (unchanged) " + String(motorBspeed) + "\n";
    }

    if (receivedData.hasOwnProperty("motorBdir")) {
      int motorBdir = int(receivedData["motorBdir"]);
      motorBdirForward = (motorBdir == 1);
      response += "Motor B Direction: " + String(motorBdirForward) + "\n";
    } else {
      response += "Motor B Direction: (unchanged) " + String(motorBdirForward) + "\n";
    }

    handleMotors();

    client->text(response);
  }
}

//================= FreeRTOS tasks =================
// FreeRTOS task for WiFi setup
void WiFiTask(void *parameter) {
  WiFi.begin(ssid, password);
  Serial.println("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Start WebSocket server
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Start the HTTP server
  server.begin();
  Serial.println("HTTP server started.");

  vTaskDelete(NULL);  // End the task after Wi-Fi is connected
}

// FreeRTOS task for motor control
void MotorControlTask(void *parameter) {
  while (true) {
    handleMotors();
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay for motor control task (adjust as needed)
  }
}

// FreeRTOS task for WebSocket server
void WebSocketTask(void *parameter) {
  while (true) {
    ws.cleanupClients();                   // Cleanup disconnected clients
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay for WebSocket cleanup (adjust as needed)
  }
}

// FreeRTOS task for gyro readings
void GyroReadingTask(void *parameter) {
  while (true) {
    getGyroReadings();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// FreeRTOS task for acceleration readings
void AccReadingsTask(void *parameter) {
  while (true) {
    getAccReadings();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void MpuTempReadingsTask(void *parameter) {
  while (true) {
    getTemperature();
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void DhtReadingsTask(void *parameter) {
  while (true) {
    getDht();
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void IrReadingsTask(void *parameter) {
  while (true) {
    getIr();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void SendSensorDataTask(void *parameter) {
  while (true) {
    String combinedData = JSON.stringify(readings);
    ws.textAll(combinedData);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

//================= FreeRTOS tasks =================

void setup() {
  Serial.begin(115200);

  //================= Motor Setup =================
  // Motor pin setup
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Attach PWM channels to motor control pins
  ledcSetup(pwmChannelA1, freq, resolution);
  ledcAttachPin(motor1Pin1, pwmChannelA1);
  ledcSetup(pwmChannelA2, freq, resolution);
  ledcAttachPin(motor1Pin2, pwmChannelA2);

  ledcSetup(pwmChannelB1, freq, resolution);
  ledcAttachPin(motor2Pin1, pwmChannelB1);
  ledcSetup(pwmChannelB2, freq, resolution);
  ledcAttachPin(motor2Pin2, pwmChannelB2);

  // Initial motor state (off)
  ledcWrite(pwmChannelA1, 0);
  ledcWrite(pwmChannelA2, 0);
  ledcWrite(pwmChannelB1, 0);
  ledcWrite(pwmChannelB2, 0);
  //================= Motor Setup =================

  //================= MPU Setup =================
  // MPU Setup Code
  if (mpu.begin()) {
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.println("Accelerometer range set to: 8G");

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.println("Gyro range set to: 500 DEG");

    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    Serial.println("Filter bandwidth set to: 5HZ");

    Serial.println("");
    delay(100);
  } else {
    Serial.println("Failed to find MPU6050 chip");
  }
  //================= MPU Setup =================

  // Create FreeRTOS tasks
  // xTaskCreate(task, taskName, stack size, params, priority, handle);
  // Create FreeRTOS tasks
  xTaskCreate(WiFiTask, "WiFiTask", 4096, NULL, 3, NULL);                        // Highest priority
  xTaskCreate(SendSensorDataTask, "SendSensorDataTask", 8192, NULL, 3, NULL);    // High priority
  xTaskCreate(MotorControlTask, "MotorControlTask", 10000, NULL, 2, NULL);       // Medium priority
  xTaskCreate(MpuTempReadingsTask, "MpuTempReadingsTask", 8192, NULL, 2, NULL);  // Medium priority
  xTaskCreate(GyroReadingTask, "GyroReadingTask", 8192, NULL, 1, NULL);          // Low priority
  xTaskCreate(AccReadingsTask, "AccReadingsTask", 8192, NULL, 1, NULL);          // Low priority
  xTaskCreate(DhtReadingsTask, "DhtReadingsTask", 8192, NULL, 1, NULL);          // Low priority
  xTaskCreate(IrReadingsTask, "IrReadingsTask", 8192, NULL, 1, NULL);            // Low priority
  xTaskCreate(WebSocketTask, "WebSocketTask", 4096, NULL, 1, NULL);              // Low priority
}

void loop() {}

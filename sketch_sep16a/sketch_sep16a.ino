// Load Wi-Fi library
#include <WiFi.h>

// Network credentials Here
const char* ssid = "ALVA4eBn6fh";
const char* password = "25av5nuh9k17";

// Motor Config Here
int motor1Pin1 = 16;
int motor1Pin2 = 17;

int motor2Pin1 = 18;
int motor2Pin2 = 19;

// Motor state
// 1 = motor ON
// 0 = motor OFF
int motorAState = 0;
int motorBState = 0;

// Motor rotation
// 1 = Forward movement
// 0 = Backward movement
int motorAdirection = 1;
int motorBdirection = 0;

// Motor speed
int motorAspeed = 190;
int motorBspeed = 190;

// PWM Properties
const int freq = 30000;     // Frequency of PWM
const int pwmChannel1 = 0;  // PWM channel (0-15 available)
const int pwmChannel2 = 1;
const int resolution = 8;  // PWM resolution (8 bits = 0-255)
int dutyCycle = 185;       // Initial duty cycle (0-255)

// WiFi client setup
WiFiServer server(80);

void setup() {
  Serial.begin(115200);

  // Motor Config here
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Set up PWM on motor1Pin1 with the specified frequency and resolution
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);

  // Attach the motor1Pin1 and motor2Pin1 pins to the PWM channels
  ledcAttachPin(motor1Pin1, pwmChannel1);
  ledcAttachPin(motor2Pin1, pwmChannel2);

  // Start with the motors off
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin2, LOW);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
}

// Function to extract the value of a parameter from the GET request
String getValue(String data, String key) {
  int startIndex = data.indexOf(key);
  if (startIndex >= 0) {
    startIndex += key.length();
    int endIndex = data.indexOf("&", startIndex);
    if (endIndex == -1) {
      endIndex = data.indexOf(" ", startIndex);  // End of query parameters
    }
    return data.substring(startIndex, endIndex);
  }
  return "";
}

void motorAControl() {
  if (motorAState == 1) {
    if (motorAdirection == 1) {
      // Forward direction
      digitalWrite(motor1Pin2, LOW);     // Pin2 LOW for forward
      ledcWrite(pwmChannel1, motorAspeed);  // PWM speed control
    } else if (motorAdirection == 0) {
      // Backward direction
      digitalWrite(motor1Pin2, HIGH);    // Pin2 HIGH for backward
      int invertedSpeed = 255 - motorAspeed;  // Invert the speed
      ledcWrite(pwmChannel1, invertedSpeed);  // PWM speed control
    }
  } else {
    // Motor is OFF
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel1, LOW);  // Set PWM duty cycle to 0
  }
}

void motorBControl() {
  if (motorBState == 1) {
    if (motorBdirection == 1) {
      // Forward direction
      digitalWrite(motor2Pin2, LOW);     // Pin2 LOW for forward
      ledcWrite(pwmChannel2, motorBspeed);  // PWM speed control
    } else if (motorBdirection == 0) {
      // Backward direction
      digitalWrite(motor2Pin2, HIGH);    // Pin2 HIGH for backward
      int invertedSpeed = 255 - motorBspeed;  // Invert the speed
      ledcWrite(pwmChannel2, invertedSpeed);  // PWM speed control
    }
  } else {
    // Motor is OFF
    digitalWrite(motor2Pin2, LOW);
    ledcWrite(pwmChannel2, LOW);  // Set PWM duty cycle to 0
  }
}

void loop() {
  WiFiClient client = server.available();

  if (client) {  // If a new client connects,
    Serial.println("New Client.");
    String header = "";  // Reset header
    while (client.connected()) {
      if (client.available()) {  // If there's bytes to read from the client,
        char c = client.read();
        header += c;  // Store HTTP request in header

        // Check for end of request (blank line)
        if (c == '\n' && header.endsWith("\r\n\r\n")) {

          // Motor state control
          // (state) param required
          // (direction) param optional
          // (speed) param optional
          // http://192.168.1.225/configMotorA/?state=1&speed=180&direction=1
          if (header.indexOf("GET /configMotorA/") >= 0) {
            Serial.println("Setting motor A config");

            // Extract motorA state (optional param)
            String reqMotorAStateStr = getValue(header, "state=");
            bool stateProvided = reqMotorAStateStr != "";

            // Extract motorA direction (optional param)
            String reqMotorADirectionStr = getValue(header, "direction=");
            bool directionProvided = reqMotorADirectionStr != "";

            // Extract motorA speed (optional param)
            String reqMotorASpeedStr = getValue(header, "speed=");
            bool speedProvided = reqMotorASpeedStr != "";

            if (stateProvided) {
              int reqMotorAState = reqMotorAStateStr.toInt();
              if (reqMotorAState == 1 || reqMotorAState == 0) {
                motorAState = reqMotorAState;
                Serial.println("Motor A state: " + String(motorAState));
              }
            }

            if (directionProvided) {
              int reqMotorAdirection = reqMotorADirectionStr.toInt();
              if (reqMotorAdirection == 1 || reqMotorAdirection == 0) {
                motorAdirection = reqMotorAdirection;
                Serial.println("Motor A direction: " + String(motorAdirection));
              }
            }

            if (speedProvided) {
              int reqMotorAspeed = reqMotorASpeedStr.toInt();
              if (reqMotorAspeed >= 0) {
                motorAspeed = reqMotorAspeed;
                Serial.println("Motor A speed: " + String(motorAspeed));
              }
            }
          }

          // http://192.168.1.225/configMotorA/?state=1&speed=180&direction=1
          if (header.indexOf("GET /configMotorB/") >= 0) {
            Serial.println("Setting motor B config");

            // Extract motorA state (required param)
            String reqMotorBStateStr = getValue(header, "state=");
            bool stateProvided = reqMotorBStateStr != "";

            // Extract motorA direction (optional param)
            String reqMotorBDirectionStr = getValue(header, "direction=");
            bool directionProvided = reqMotorBDirectionStr != "";

            // Extract motorA speed (optional param)
            String reqMotorBSpeedStr = getValue(header, "speed=");
            bool speedProvided = reqMotorBSpeedStr != "";

            if (stateProvided) {
              int reqMotorBState = reqMotorBStateStr.toInt();
              if (reqMotorBState == 1 || reqMotorBState == 0) {
                motorBState = reqMotorBState;
                Serial.println("Motor B state: " + String(motorBState));
              }
            }

            if (directionProvided) {
              int reqMotorBdirection = reqMotorBDirectionStr.toInt();
              if (reqMotorBdirection == 1 || reqMotorBdirection == 0) {
                motorBdirection = reqMotorBdirection;
                Serial.println("Motor B direction: " + String(motorBdirection));
              }
            }

            if (speedProvided) {
              int reqMotorBspeed = reqMotorBSpeedStr.toInt();
              if (reqMotorBspeed >= 0) {
                motorBspeed = reqMotorBspeed;
                Serial.println("Motor B speed: " + String(motorBspeed));
              }
            }
          }

          // Call motor control functions
          motorAControl();
          motorBControl();

           // Now send a response back to the client with motor statuses
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:application/json");
          client.println("Connection: close");
          client.println();  // End of headers

           // Now send a response back to the client with motor statuses
          // Construct the JSON response
          String jsonResponse = "{";
          jsonResponse += "\"motorAState\": \"" + String(motorAState) + "\",";
          jsonResponse += "\"motorAdirection\": \"" + String(motorAdirection) + "\",";
          jsonResponse += "\"motorAspeed\": \"" + String(motorAspeed) + "\",";
          jsonResponse += "\"motorBState\": \"" + String(motorBState) + "\",";
          jsonResponse += "\"motorBdirection\": \"" + String(motorBdirection) + "\",";
          jsonResponse += "\"motorBspeed\": " + String(motorBspeed);
          jsonResponse += "}";

          client.println(jsonResponse);

          // Close the connection
          client.stop();
          Serial.println("Client disconnected.");
        }
      }
    }
  }
}

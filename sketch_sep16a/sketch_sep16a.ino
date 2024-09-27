// Load Wi-Fi library
#include <WiFi.h>

// Network credentials Here
const char* ssid = "ALVA4eBn6fh";
const char* password = "25av5nuh9k17";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

//variables to store the current LED states
String stateA = "off";
String stateB = "off";

// Motor Config Here
int motor1Pin1 = 16;
int motor1Pin2 = 17;

int motor2Pin1 = 18;
int motor2Pin2 = 19;

// PWM Properties
const int freq = 30000;     // Frequency of PWM
const int pwmChannel1 = 0;  // PWM channel (0-15 available)
const int pwmChannel2 = 1;
const int resolution = 8;  // PWM resolution (8 bits = 0-255)
int dutyCycle = 185;       // Initial duty cycle (0-255)

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds
const long timeoutTime = 2000;

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

  // Attach the motor1Pin1 pin to the PWM channel
  ledcAttachPin(motor1Pin1, pwmChannel1);
  ledcAttachPin(motor2Pin1, pwmChannel2);

  // Start with the motors off
  digitalWrite(motor1Pin2, LOW);

  digitalWrite(motor2Pin2, LOW);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Print IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
}

void loop() {
  WiFiClient client = server.available();

  if (client) {  // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");  // print a message out in the serial port
    String currentLine = "";        // make a String to hold incoming data from the client

    while (client.connected() && currentTime - previousTime <= timeoutTime) {
      // loop while the client's connected
      currentTime = millis();
      if (client.available()) {  // if there's bytes to read from the client,
        char c = client.read();  // read a byte, then
        Serial.write(c);         // print it out the serial monitor
        header += c;
        if (c == '\n') {  // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // turns the GPIOs on and off
            if (header.indexOf("GET /A/on") >= 0) {
              stateA = "on";
              // Motor A on here
              digitalWrite(motor1Pin2, LOW);
              ledcWrite(pwmChannel1, 200);
            } else if (header.indexOf("GET /A/off") >= 0) {
              stateA = "off";
              // Motor A off here
              digitalWrite(motor1Pin2, LOW);
              ledcWrite(pwmChannel1, LOW);
            }

            if (header.indexOf("GET /B/on") >= 0) {
              stateB = "on";
              // Motor B on here
              digitalWrite(motor2Pin2, LOW);
              ledcWrite(pwmChannel2, 200);
            } else if (header.indexOf("GET /B/off") >= 0) {
              stateB = "off";
              // Motor B off here
              digitalWrite(motor2Pin2, LOW);
              ledcWrite(pwmChannel2, LOW);
            }

            // Motor speed control
            if (header.indexOf("GET /setSpeedA?speed=") >= 0) {
              int speedStart = header.indexOf("speed=") + 6;
              String speedValue = header.substring(speedStart);
              int motorSpeedA = speedValue.toInt();  // Convert the string to integer
              motorSpeedA = constrain(motorSpeedA, 0, 255);  // Limit speed between 0 and 255
              ledcWrite(pwmChannel1, motorSpeedA);  // Set the PWM duty cycle for Motor A
            }

            if (header.indexOf("GET /setSpeedB?speed=") >= 0) {
              int speedStart = header.indexOf("speed=") + 6;
              String speedValue = header.substring(speedStart);
              int motorSpeedB = speedValue.toInt();  // Convert the string to integer
              motorSpeedB = constrain(motorSpeedB, 0, 255);  // Limit speed between 0 and 255
              ledcWrite(pwmChannel2, motorSpeedB);  // Set the PWM duty cycle for Motor B
            }

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons
            client.println("<style>html { font-family: monospace; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: yellowgreen; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 32px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: gray;}</style></head>");

            client.println("<body><h1>ESP32 Web Server</h1>");
            client.println("<p>Control Motor State</p>");

            if (stateA == "off") {
              client.println("<p><a href=\"/A/on\"><button class=\"button button2\">OFF</button></a></p>");
            } else {
              client.println("<p><a href=\"/A/off\"><button class=\"button\">ON</button></a></p>");
            }

            if (stateB == "off") {
              client.println("<p><a href=\"/B/on\"><button class=\"button button2\">OFF</button></a></p>");
            } else {
              client.println("<p><a href=\"/B/off\"><button class=\"button\">ON</button></a></p>");
            }

            client.println("<input type=\"text\" id=\"speedA\" name=\"speedA\" placeholder=\"180-255\">");
            client.println("<button onclick=\"setSpeedA()\">Set Speed A</button>");
            client.println("<input type=\"text\" id=\"speedB\" name=\"speedB\" placeholder=\"180-255\">");
            client.println("<button onclick=\"setSpeedB()\">Set Speed B</button>");
            client.println("<script> function setSpeedA() { var speed = document.getElementById(\"speedA\").value; window.location.href = \"/setSpeedA?speed=\" + speed; } function setSpeedB() { var speed = document.getElementById(\"speedB\").value; window.location.href = \"/setSpeedB?speed=\" + speed; } </script>");

            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else {  // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }


}
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <ArduinoJson.h>


// Create instances for the servo and display
Servo myServo;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// --- Pin Definitions ---
#define SW1_PIN 32
#define SW2_PIN 33
#define SERVO_PIN 15
#define LDR_PIN 34
#define RED_PIN 14
#define GREEN_PIN 12
#define BLUE_PIN 27

// --- Global State Variables ---
int currentState = 0; // Start in a neutral state (0)
int lastState = -1;
bool promptPrinted = false;
int currentAngle = 90; // Start servo at a known middle position

const char* ssid = "Airtel_OTPL_WiFi4g";
const char* password = "Technology5g";

// Secure MQTT Broker Info (no certificates used)
/*const char* mqtt_server = "otplai.com"; // e.g. broker.emqx.io
const int mqtt_port = 8883; // secure MQTT port
const char* mqtt_topic = "Tutorial_MQTT/sensors/data";*/

// Secure MQTT Broker Info (no certificates used)
const char* mqtt_server = "otplai.com"; // e.g. broker.emqx.io
const int mqtt_port = 8883; // secure MQTT port
const char* mqtt_topic = "Tutorial_MQTT/sensors/data(json)";
const char* mqtt_topic1 = "Tutorial_MQTT/sensors/data";
const char* mqtt_topic_rgb_control = "Tutorial_MQTT/control/rgb";
const char* mqtt_topic_servo_control = "Tutorial_MQTT/control/servo";



// MQTT Authentication
const char* mqtt_user = "oyt";
const char* mqtt_password = "123456789";

WiFiClientSecure secureClient;
PubSubClient client(secureClient);




void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32ClientSecure", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      client.subscribe(mqtt_topic_rgb_control);
      client.subscribe(mqtt_topic_servo_control);
      Serial.println("Subscribed to control topics");

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5s");
      delay(5000);
    }
  }
}


//---------------------------------------------------------------------

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.println("] " + msg);

  // --- RGB control topic ---
  if (String(topic) == mqtt_topic_rgb_control) {
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, msg);
    if (!err) {
      int r = doc["r"];
      int g = doc["g"];
      int b = doc["b"];
      setColor(r, g, b);
      Serial.printf("RGB set to: R=%d G=%d B=%d\n", r, g, b);

      // ✅ Add OLED feedback
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 10);
      display.println("MQTT RGB Control");

      display.setTextSize(2);
      display.setCursor(0, 30);
      display.printf("R:%d G:%d B:%d", r, g, b);
      display.display();
    } else {
      Serial.println("Invalid RGB JSON received.");
    }
  }

  // --- Servo control topic ---
  else if (String(topic) == mqtt_topic_servo_control) {
    StaticJsonDocument<64> doc;
    DeserializationError err = deserializeJson(doc, msg);
    if (!err) {
      int angle = doc["angle"];
      if (angle >= 0 && angle <= 180) {
        currentAngle = angle;
        myServo.write(currentAngle);
        Serial.printf("Servo angle set to %d\n", currentAngle);

        // ✅ Add OLED feedback
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 10);
        display.println("MQTT Servo Control");

        display.setTextSize(2);
        display.setCursor(20, 30);
        display.printf("Angle: %d%c", angle, 248);  // 248 is the degree symbol
        display.display();
      }
    } else {
      Serial.println("Invalid Servo JSON received.");
    }
  }
}


//---------------------------------------------------------------------


void setup() {
  Serial.begin(115200);

  // Initialize Pins
  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);
  pinMode(LDR_PIN, INPUT);

  // Initialize Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Infinite loop on failure
  }

  // Initial display message
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(10, 25);
  display.println("Ready!");
  display.display();
  delay(1000);

      
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
  secureClient.setInsecure();  // <------ ALLOW INSECURE CONNECTION TO TLS SERVER

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);


}

void loop() {


  if (!client.connected()) {
    reconnect();
  }
  client.loop();

//---------------------------------------------------

  // --- 1. Read Inputs and Determine Current State ---
  bool sw1 = digitalRead(SW1_PIN);
  bool sw2 = digitalRead(SW2_PIN);

  if (!sw1 && sw2) {
    currentState = 1; // LDR State
  } else if (sw1 && !sw2) {
    currentState = 2; // Servo State
  } else if (sw1 && sw2) {
    currentState = 3; // RGB LED State
  } else {
    currentState = 0; // Default/idle state
  }

  // --- 2. Handle State Transitions (if state has changed) ---
  if (currentState != lastState) {
    // A. Clean up the PREVIOUS state
    switch (lastState) {
      case 2: // Leaving Servo State
        myServo.detach();
        Serial.println("Servo detached");
        break;
      case 3: // Leaving RGB LED State
        setColor(0, 0, 0); // Turn off LED
        // Detach PWM pins to free up resources
        ledcDetach(RED_PIN);
        ledcDetach(GREEN_PIN);
        ledcDetach(BLUE_PIN);
        promptPrinted = false;
        Serial.println("RGB LED turned off and detached");
        break;
    }

    // B. Set up the NEW state
    display.clearDisplay(); // Clear display on any state change
    switch (currentState) {
      case 1: // Entering LDR State

        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 20);
        display.println("State 1: LDR");
        Serial.println("Entered LDR State");
        break;

      case 2: // Entering Servo State

        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 20);
        myServo.attach(SERVO_PIN);
        display.println("State 2: Servo");
        Serial.println("Entered Servo State & Servo Attached");
        
        break;
      case 3: // Entering RGB LED State
        // Attach pins to PWM channels

        display.clearDisplay();

        display.setTextSize(1);
        display.setCursor(0,10);
        display.println("State 3:RGB LED");

        display.setTextSize(1);
        display.setCursor(0,20);
        display.println("Enter colour:");

        ledcAttach(RED_PIN, 5000, 8);
        ledcAttach(GREEN_PIN, 5000, 8);
        ledcAttach(BLUE_PIN, 5000, 8);
        //display.println("State 3: RGB");
        Serial.println("Entered RGB State");
        break;
      case 0:
        display.println("Idle State");
        break;
    }
    display.display();
    delay(50); // Small delay to prevent flickering
  }

  // --- 3. Run the Logic for the Current State ---
  switch (currentState) {
    case 1:
      runLdrState();
      break;
    case 2:
      runServoState();
      break;
    case 3:
      runRgbState();
      break;
  }

  // --- 4. Update lastState for the next loop ---
  lastState = currentState;

  delay(100); // Main loop delay


  // Build JSON manually to match your required format
  /*String payload = "{";
  payload += "\"d1\":\"" + String(ldrValue) + "\",";
  payload += "\"d2\":\"" + String(thermistorValue) + "\",";
  payload += "\"d3\":\"" + String(mq6Value) + "\",";
  payload += "\"header\":{";
  payload += "\"h1\":\"ldr\",";
  payload += "\"h2\":\"thermistor\",";
  payload += "\"h3\":\"mq6\"";
  payload += "}}";

  client.publish(mqtt_topic, payload.c_str());*/

//------------------------------------------------------------------------------------------------------------------------------


  
}


//================================================================
//                 STATE-SPECIFIC FUNCTIONS
//================================================================

/**
 * @brief Reads LDR value and displays it.
 */
void runLdrState() {
  int ldrValue = analogRead(LDR_PIN);

  Serial.print("LDR: ");
  Serial.println(ldrValue);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.println("State 1: LDR Sensor");

  display.setTextSize(2);
  display.setCursor(0, 20);
  display.print("Value:");
  display.print(ldrValue);

  display.display();


  String payload = "{";
  payload += "\"d1\":\"" + String(ldrValue) + "\",";
  payload += "\"header\":{";
  payload += "\"h1\":\"LDR\"";
  payload += "}}";

  client.publish(mqtt_topic, payload.c_str());

  String payload1 = "\Data =" + String(ldrValue); + "\",";
  client.publish(mqtt_topic1, payload1.c_str());

  delay(500); // Slow down LDR reading display

}

/**
 * @brief Controls the servo motor.
 */
void runServoState() {
  // *** THE MAIN FIX IS HERE ***
  // Always send the write command to the servo on every loop.
  // This tells the servo to actively hold its position.
  myServo.write(currentAngle);

  // Check for new commands from the Serial Monitor
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    int angle = input.toInt();

    if (angle >= 0 && angle <= 180) {
      currentAngle = angle; // Update the angle

      String payload_servo = "{";
      payload_servo += "\"angle\": " + String(currentAngle);
      payload_servo += "}";
      client.publish(mqtt_topic_servo_control, payload_servo.c_str());

      // The myServo.write() at the top of this function will handle the move
      Serial.print("Angle set to: ");
      Serial.println(currentAngle);
    } else {
      Serial.println("Invalid input. Enter angle 0-180.");
    }
  }

  // Update the OLED display
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.println("State 2:Servo Control");
  display.setCursor(0, 20);
  display.println("Current Angle:");
  display.setTextSize(2);
  display.setCursor(30, 40);
  display.print(currentAngle);
  display.print((char)247); // Degree symbol
  display.display();

  String payload = "{";
  payload += "\"d1\":\"" + String(currentAngle) + "\",";
  payload += "\"header\":{";
  payload += "\"h1\":\"SERVO\"";
  payload += "}}";

  client.publish(mqtt_topic, payload.c_str());

  String payload1 = "\Data =" + String(currentAngle); + "\",";
  client.publish(mqtt_topic1, payload1.c_str());

}


/**
 * @brief Controls the RGB LED based on serial input.
 */
void runRgbState() {
  if (!promptPrinted) {
    Serial.println("Enter color: red, green, blue, yellow, cyan, pink");
    promptPrinted = true;
  }

  if (Serial.available()) {
    String color = Serial.readStringUntil('\n');
    color.trim();
    color.toLowerCase();

    display.clearDisplay();

    display.setTextSize(1);
    display.setCursor(0,10);
    display.println("State 3:RGB LED");

    display.setTextSize(1);
    display.setCursor(0,20);
    display.println("Enter colour:");

    display.setTextSize(2);
    display.setCursor(0, 30);

    if (color == "red") {
      setColor(255, 0, 0);
      display.print("Red");

      String payload = "{";
      payload += "\"d1\":\"" + String(color) + "\",";
      payload += "\"header\":{";
      payload += "\"h1\":\"RGB\"";
      payload += "}}";

      client.publish(mqtt_topic, payload.c_str());

      String payload1 = "\Data =" + String(color); + "\",";
      client.publish(mqtt_topic1, payload1.c_str());

    } else if (color == "green") {
      setColor(0, 255, 0);
      display.print("Green");

      String payload = "{";
      payload += "\"d1\":\"" + String(color) + "\",";
      payload += "\"header\":{";
      payload += "\"h1\":\"RGB\"";
      payload += "}}";

      client.publish(mqtt_topic, payload.c_str());

      String payload1 = "\Data =" + String(color); + "\",";
      client.publish(mqtt_topic1, payload1.c_str());

    } else if (color == "blue") {
      setColor(0, 0, 255);
      display.print("Blue");

      String payload = "{";
      payload += "\"d1\":\"" + String(color) + "\",";
      payload += "\"header\":{";
      payload += "\"h1\":\"RGB\"";
      payload += "}}";

      client.publish(mqtt_topic, payload.c_str());

      String payload1 = "\Data =" + String(color); + "\",";
      client.publish(mqtt_topic1, payload1.c_str());

    } else if (color == "yellow") {
      setColor(255, 255, 0);
      display.print("Yellow");

      String payload = "{";
      payload += "\"d1\":\"" + String(color) + "\",";
      payload += "\"header\":{";
      payload += "\"h1\":\"RGB\"";
      payload += "}}";

      client.publish(mqtt_topic, payload.c_str());

      String payload1 = "\Data =" + String(color); + "\",";
      client.publish(mqtt_topic1, payload1.c_str());

    } else if (color == "cyan") {
      setColor(0, 255, 255);
      display.print("Cyan");

      String payload = "{";
      payload += "\"d1\":\"" + String(color) + "\",";
      payload += "\"header\":{";
      payload += "\"h1\":\"RGB\"";
      payload += "}}";

      client.publish(mqtt_topic, payload.c_str());

      String payload1 = "\Data =" + String(color); + "\",";
      client.publish(mqtt_topic1, payload1.c_str());

    } else if (color == "pink") {
      setColor(255, 0, 127);
      display.print("Pink");

      String payload = "{";
      payload += "\"d1\":\"" + String(color) + "\",";
      payload += "\"header\":{";
      payload += "\"h1\":\"RGB\"";
      payload += "}}";

      client.publish(mqtt_topic, payload.c_str());

      String payload1 = "\Data =" + String(color); + "\",";
      client.publish(mqtt_topic1, payload1.c_str());

    } else {
      display.setTextSize(1);
      display.setCursor(0,20);
      display.println("Unknown color!");
    }
    display.display();
  }
  
}

/**
 * @brief Sets the color of the RGB LED.
 * @param r Red value (0-255)
 * @param g Green value (0-255)
 * @param b Blue value (0-255)
 */
void setColor(int r, int g, int b) {
  ledcWrite(RED_PIN, r);
  ledcWrite(GREEN_PIN, g);
  ledcWrite(BLUE_PIN, b);
}
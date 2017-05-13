/*
  This a simple example of the aREST Library for the ESP8266 WiFi chip.
  See the README file for more details.

  Written in 2015 by Marco Schwartz under a GPL license.
*/

// Import required libraries
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <aREST.h>
#include <SimpleDHT.h>
#include <Servo.h>

Servo ESC;

// Create aREST instance
aREST rest = aREST();

// DHT11 sensor
SimpleDHT11 dhtSensor;

// WiFi parameters
const char* ssid = "MyScooter";
const char* password = "ThisJustRocks!";

// The port to listen for incoming TCP connections
#define LISTEN_PORT           80

#define CODE  7151
#define ACCEL_RATE 100
#define DHT_RATE 5000
#define SPEEDTHROTTLE_MIN 0
#define SPEEDTHROTTLE_MAX 370
#define NOISE_REDUCTION 2
#define ESC_MIN 30
#define ESC_MAX 160
#define ESC_DECREASE_STEP 6
#define ESC_INCREASE_STEP 3
#define EEPROM_MAX_SPEED 1
#define EEPROM_LIGHTS 2

// Pins
#define LED_RED D6
#define LED_GREEN D7
#define LED_ESP8266 D4
#define ESC_MOTOR D0
#define DHT11 D3
#define FRONTLIGHT D5
#define BACKLIGHT D8
#define THROTTLE A0

// Create an instance of the server
WiFiServer server(LISTEN_PORT);

long previousMillisDHT = 0;
long previousMillisESC = 0;

// Variables to be exposed to the API
int temperatureDHT = 0;
int humidityDHT = 0;
int enabled = 0;
int maxSpeed = ESC_MAX;
int ESCValue = ESC_MIN;
int ESCValueOld = ESCValue;

// Declare functions to be exposed to the API
int lock(String code);
int lights(String state);
int speedLimit(String speed);

void setup(void)
{
  // Start Serial
  Serial.begin(115200);
  EEPROM.begin(16); //init ESP8266 EEPROM 16 bytes
  maxSpeed = EEPROM.read(EEPROM_MAX_SPEED) * 4;
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_ESP8266, OUTPUT);
  pinMode(FRONTLIGHT, OUTPUT);
  pinMode(BACKLIGHT, OUTPUT);

  digitalWrite(LED_RED, HIGH); // Locked state
  digitalWrite(FRONTLIGHT, EEPROM.read(EEPROM_LIGHTS));
  digitalWrite(BACKLIGHT, EEPROM.read(EEPROM_LIGHTS));

  // Init variables and expose them to REST API
  rest.variable("temperature", &temperatureDHT);
  rest.variable("humidity", &humidityDHT);
  rest.variable("enabled", &enabled);
  rest.variable("speed", &maxSpeed);

  // Function to be exposed
  rest.function("lock", lock);
  rest.function("lights", lights);
  rest.function("speedlimit", speedLimit);

  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id("1");
  rest.set_name("E-Scooter");

  // Setup WiFi network
  WiFi.softAP(ssid, password);
  Serial.println("");
  Serial.println("WiFi created");

  // Start the server
  server.begin();
  Serial.println("Server started");

  // Print the IP address
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
}

void loop() {
  // Handle REST calls
  updateESC();
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
  while (!client.available()) {
    delay(1);
  }
  readDHT();
  rest.handle(client);
}

int readDHT() {
  if (millis() - previousMillisDHT > DHT_RATE) { // sampling rate
    previousMillisDHT = millis();
    byte temp;
    byte humidi;
    if (dhtSensor.read(DHT11, &temp, &humidi, NULL)) {
      Serial.println("[ERROR] DHT11 reading failed!");
      return 0;
    }
    temperatureDHT = (int)temp;
    humidityDHT = (int)humidi;
    return 1;
  }
}

int updateESC() {
  // Accelerate to the right speed
  if (millis() - previousMillisESC > ACCEL_RATE) {
    previousMillisESC = millis();

    digitalWrite(LED_ESP8266, LOW); // Turn LED on when we update the ESC (active low)

    int ESCValue = analogRead(THROTTLE);
    ESCValue = constrain(map(ESCValue, SPEEDTHROTTLE_MIN, SPEEDTHROTTLE_MAX, ESC_MIN, maxSpeed), ESC_MIN, maxSpeed); //Limit speed to maximum speed which can be set by calling /speedlimit?params=MAXSPEED

    if ((ESCValue > ESCValueOld + NOISE_REDUCTION) && enabled ) { // Accelerate (only when enabled)
      ESCValueOld = ESCValueOld + ESC_INCREASE_STEP;
      Serial.println("+");
      if (ESCValue < ESCValueOld) { // Overflow encounter
        ESCValueOld = ESCValue;
      }
    }
    else if (ESCValue < ESCValueOld - NOISE_REDUCTION) { //Deaccelerate
      ESCValueOld = ESCValueOld - ESC_DECREASE_STEP;
      Serial.println("-");
      if (ESCValue > ESCValueOld) { // Underflow encounter
        ESCValueOld = ESCValue;
      }
    }
    else if (!enabled && ESCValue < ESCValueOld) { // Slow down when scooter is disabled
      ESCValueOld = ESCValueOld - ESC_DECREASE_STEP;
    }

    ESC.write(ESCValueOld); //Write to ESC
    
    return 1;
  }
  digitalWrite(LED_ESP8266, HIGH); // Turn LED off on next cycle (active low)
  
  return 0;
}

// Custom function accessible by the API
int lock(String code) {

  if (code.toInt() == CODE) {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, LOW);
    enabled = armESC(1);
    return 1;
  }
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, HIGH);
  enabled = armESC(0);
  
  return 0;
}

int lights(String state) {
  if (!enabled) { //Only when scooter is enabled
    return 0;
  }

  if (state.toInt() == 0) {
    digitalWrite(FRONTLIGHT, LOW);
    digitalWrite(BACKLIGHT, LOW);
    EEPROM.write(EEPROM_LIGHTS, 0);
    return 0;
  }
  digitalWrite(FRONTLIGHT, HIGH);
  digitalWrite(BACKLIGHT, HIGH);

  // Save state
  EEPROM.write(EEPROM_LIGHTS, 1);
  EEPROM.commit();
  
  return 1;
}

int speedLimit(String speed) {
  if (!enabled) { //Only when scooter is enabled
    return 0;
  }
  
  maxSpeed = constrain(speed.toInt(), ESC_MIN, ESC_MAX);

  // Save state
  EEPROM.write(EEPROM_MAX_SPEED, maxSpeed / 4);
  EEPROM.commit();
  
  return 1;
}

int armESC(int state) {
  if (state == 1) {
    ESC.attach(ESC_MOTOR);
    ESC.write(ESC_MIN);
    delay(1000);
    ESC.write(ESC_MAX);
    delay(1000);
    ESC.write(ESC_MIN);
    return 1;
  }
  else {
    //Disarm
    ESC.detach();
    return 0;
  }
}



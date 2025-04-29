#include "Automata.h"
#include "ArduinoJson.h"
#include <Adafruit_AHTX0.h>
#include <Wire.h>

#define FAN 4
#define FAN1 2
#define FAN2 9
#define LED 10
#define HEAT_PIN 6
#define SDA 1
#define SCL 0

// const char *HOST = "192.168.29.67";
// int PORT = 8080;

const char *HOST = "raspberry.local";
int PORT = 8010;

Automata automata("Fan", HOST, PORT);
Preferences preferences;
Adafruit_AHTX0 aht;

volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
unsigned long lastTime = 0;
float fanSpeed = 0;
float fanSpeed2 = 0;

int speed = 40;
int pwm1 = 0;

unsigned long previousMillis = 0;

long start = millis();
JsonDocument doc;

void action(const Action action)
{
  analogWrite(LED, 20);
  if (action.data.containsKey("pwm1"))
    pwm1 = action.data["pwm1"];

  if (action.data.containsKey("speed"))
  {
    int sp = action.data["speed"];
    preferences.putInt("speed", sp);
    speed = sp;
    analogWrite(FAN, sp);
  }

  String jsonString;
  serializeJson(action.data, jsonString);
  Serial.println(jsonString);
  analogWrite(LED, 0);
}

void sendData()
{
  automata.sendData(doc);
}

void countPulse2()
{
  pulseCount2++;
}
void countPulse1()
{
  pulseCount1++;
}

void setup()
{
  pinMode(FAN, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(FAN1, INPUT_PULLUP); // Tachometer signal on pin 2
  Wire.begin(SDA, SCL);
  analogWrite(FAN, speed);
  analogWrite(HEAT_PIN, pwm1);
  attachInterrupt(digitalPinToInterrupt(FAN1), countPulse1, RISING);

  pinMode(FAN2, INPUT_PULLUP); // Tachometer signal on pin 2
  attachInterrupt(digitalPinToInterrupt(FAN2), countPulse2, RISING);
  JsonDocument doc;
  doc["max"] = 255;
  doc["min"] = 0;
  automata.begin();
  automata.addAttribute("fan1", "Fan 1", "RPM", "DATA|MAIN");
  automata.addAttribute("fan2", "Fan 2", "RPM", "DATA|MAIN");
  automata.addAttribute("temp", "Temp", "°C", "DATA|MAIN");
  automata.addAttribute("humid", "Humidity", "%", "DATA|MAIN");
  automata.addAttribute("pwm1", "Heat", "", "ACTION|SLIDER", doc);
  automata.addAttribute("speed", "Speed", "", "ACTION|SLIDER", doc);

  automata.registerDevice();
  automata.onActionReceived(action);
  automata.delayedUpdate(sendData);

  preferences.begin("fan", false);
  speed = preferences.getInt("speed", 0);
  analogWrite(FAN, speed);
  analogWrite(LED, 20);

  if (aht.begin()) {
    Serial.println("Found AHT20");
  } else {
    Serial.println("Didn't find AHT20");
  }
}

void loop()
{
  automata.loop();
  analogWrite(FAN, speed);
  analogWrite(HEAT_PIN, pwm1);

  sensors_event_t humidity, temp;
  
  aht.getEvent(&humidity, &temp);

  doc["temp"] = String(temp.temperature, 2);
  doc["humid"] = String(humidity.relative_humidity, 2);
  doc["speed"] = speed;
  doc["pwm1"] = pwm1;
  doc["fan1"] = fanSpeed;
  doc["fan2"] = fanSpeed2;
  if ((millis() - start) > 1000)
  {
    // analogWrite(LED, 20);
    automata.sendLive(doc);
    start = millis();
  }

  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 2000)
  {                                        // Update every second
    fanSpeed = (pulseCount1 / 2.0) * 60.0; // Convert to RPM

    pulseCount1 = 0; // Reset pulse count

    fanSpeed2 = (pulseCount2 / 2.0) * 60.0; // Convert to RPM
    
    pulseCount2 = 0; // Reset pulse count
    lastTime = currentTime;
  }

  delay(100);
  analogWrite(LED, 0);
  // put your main code here, to run repeatedly:
}

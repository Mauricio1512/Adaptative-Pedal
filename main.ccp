#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <ESP32Servo.h>

// WiFi
const char *ssid = "Elkokas";
const char *password = "siks12345";
char clientId[50];

// MQTT
const char *mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char *action_topic = "adaptative-pedal/actions";
const char *settings_topic = "adaptative-pedal/toggle-piano";

// Pines de salida
const int pinCelestino = 27;
const int pinSostenuto = 26;
const int pinSustain = 25;

// Estados actuales
bool estadoCelestino = false;
bool estadoSostenuto = false;
bool estadoSustain = false;
bool isDigital = true;

// Servos
#define ON_SERVO_ANGLE 55
#define OFF_SERVO_ANGLE 0
const int freq = 50;
const int resolution = 8; // 8 bits: valores de 0 a 255
Servo servo_celestina;
Servo servo_sostenuto;
Servo servo_sustain;

// Sensor para sustain
const int sensorPin = 33;
const int umbral = 400;  // Umbral
const int muestras = 10; // Para media móvil

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi()
{
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
}

void toggleCelestino()
{
  estadoCelestino = !estadoCelestino;
  if (isDigital) {
    digitalWrite(pinCelestino, estadoCelestino ? HIGH : LOW);
  } else {
    servo_celestina.write(estadoCelestino ? ON_SERVO_ANGLE : OFF_SERVO_ANGLE);
  }
  client.publish("adaptative-pedal/celestina", estadoCelestino ? "true" : "false");
}

void toggleSostenuto()
{
  estadoSostenuto = !estadoSostenuto;
  if (isDigital){
    digitalWrite(pinSostenuto, estadoSostenuto ? HIGH : LOW);
  } else {
    servo_sostenuto.write(estadoSostenuto ? ON_SERVO_ANGLE : OFF_SERVO_ANGLE);
  }
  client.publish("adaptative-pedal/sostenuto", estadoSostenuto ? "true" : "false");
}

void setSustain() 
{
  if (isDigital) {
    digitalWrite(pinSustain, estadoSustain ? HIGH : LOW);
  }
  else {
    servo_sustain.write(estadoSustain ? ON_SERVO_ANGLE : OFF_SERVO_ANGLE);
  }
  client.publish("adaptative-pedal/sustain", estadoSustain ? "true" : "false");
}

void handle_sustain_detection() {
  int suma = 0;

  for (int i = 0; i < muestras; i++)
  {
    suma += analogRead(sensorPin);
  }
  int valorMedio = suma / muestras;

  // Serial.print("Valor sensor: ");
  // Serial.println(valorMedio);

  if (valorMedio < umbral)
  {
    Serial.println("Threshold exceeded");
    estadoSustain = true;
  }
  else
  {
    estadoSustain = false;
  }

  setSustain();
  delay(5);
}

void callback(char *topic, byte *payload, unsigned int length)
{
  String message = "";
  for (unsigned int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }
  Serial.println(message);
  if (message == "TOGGLE_CELESTINO")
  {
    toggleCelestino();
  }
  else if (message == "TOGGLE_SOSTENUTO")
  {
    toggleSostenuto();
  }
  else if (message == "TOGGLE_DIGITAL") {
    isDigital = !isDigital;
    Serial.print("Cambiando tipo de piano a "); 
    Serial.println(isDigital ? "Digital" : "Acústico");

    if (isDigital) {
      // Quitar la salida PWM
      servo_celestina.write(0);
      servo_sostenuto.write(0);
      servo_sustain.write(0);
      delay(500);
      servo_celestina.detach();
      servo_sostenuto.detach();
      servo_sustain.detach();
      // Apagar la salida digital
      digitalWrite(pinCelestino, LOW);
      digitalWrite(pinSostenuto, LOW);
      digitalWrite(pinSustain, LOW);

      delay(300);
    } else {
      // Configura cada canal PWM
      servo_celestina.attach(pinCelestino);
      servo_sostenuto.attach(pinSostenuto);
      servo_sustain.attach(pinSustain);
      // Poner en 0 los servos
      servo_celestina.write(0);
      servo_sostenuto.write(0);
      servo_sustain.write(0);

      delay(300);
    }
  }
}

void reconnect()
{
  while (!client.connected())
  {
    Serial.println("Attempting MQTT connection...");
    long r = random(1000);
    sprintf(clientId, "clientId-%ld", r);
    if (client.connect(clientId))
    {
      client.subscribe(action_topic);
      client.subscribe(settings_topic);
      Serial.print(clientId);
      Serial.println(" connected");
    }
    else
    {
      delay(2000);
    }
  }
}

void setup()
{
  pinMode(pinCelestino, OUTPUT);
  pinMode(pinSostenuto, OUTPUT);
  pinMode(pinSustain, OUTPUT);
  digitalWrite(pinCelestino, LOW);
  digitalWrite(pinSostenuto, LOW);
  digitalWrite(pinSustain, LOW);

  Serial.begin(9600);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }

  handle_sustain_detection();
  client.loop();
}

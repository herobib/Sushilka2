#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>


#define LED_PIN 5
#define Heater_PIN 6
#define Termister_PIN A0

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
int targetTemp=27;
int currTemp;
unsigned long workTime = millis();
unsigned long lastWorkTime=millis();
bool heaterOn=false;
float airTemp=0.0;
int targetAirTemp=30;

unsigned long pwmStart=millis();
int pwmPeriod=1000;
float iTerm=0.0;

const int Kp = 1;
const int Ki = 0;
const int Kd = 0;
Adafruit_BMP280 bmp;

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("WebSocket client connected");
  } else if (type == WS_EVT_DATA) {
    Serial.printf("WS data: %.*s\n", len, data);
  }
  String msg;
  msg.reserve(len + 1);
  for (size_t i = 0; i < len; ++i) msg += (char)data[i];
  Serial.println(msg);
  
  if(msg.length()>=45){
    size_t start_command = msg.indexOf("value\":") + 7;
    String a= msg.substring(start_command,msg.length()-1);
    targetTemp=a.toInt();
    Serial.println(targetTemp);    
  } 
  else if(msg=="{\"type\":\"cmd\",\"action\":\"heater_on\"}"){    
    heaterOn=true;
    pwmStart=millis();
    workTime=millis();
    Serial.println("Heater on");
  }
  else if(msg=="{\"type\":\"cmd\",\"action\":\"heater_off\"}"){    
    lastWorkTime=millis();
    heaterOn=false;
    Serial.println("Heater off");
  }
}


const float Vcc = 3.3;
const float R_FIXED = 82500.0; 
const float R0 = 85000.0;
const float T0 = 300.15;
const float BETA = 3950.0;

int readAdcAvg() {
  const int SAMPLES = 32;
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogRead(Termister_PIN);
    delay(2);
  }
  return sum / SAMPLES;
}

float getFilteredTemp(float newTemp) {
  static float filtered = newTemp; // память между вызовами
  filtered = filtered * 0.65 + newTemp * 0.35;
  return filtered;
}

float AdcToTemp(int adc) {
  if (adc <= 0 || adc >= 4095) return -100;
  float v = adc * Vcc / 4095.0;
  float Rt = R_FIXED * v / (Vcc - v);

  float invT = (1.0 / T0) + (1.0 / BETA) * log(Rt / R0);
  return 1.0 / invT - 273.15;
}
float getTemp(){
  int adc = readAdcAvg();
  float tempRaw = AdcToTemp(adc);
  return getFilteredTemp(tempRaw);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(Heater_PIN, OUTPUT);
  pinMode(Termister_PIN, INPUT);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
    return;
  }

  WiFi.softAP("Dryer", "12345678");
  Serial.println("AP ready: Dryer / 12345678");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.begin();
  Serial.println("Server started");

  Wire.begin(21,20);
  Wire.setClock(50000);

  
  unsigned status;
  status = bmp.begin();
  if (!status) {
    Serial.print("Ne bom-bom"); Serial.println(bmp.sensorID(),16);
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, /* Operating Mode. */
  Adafruit_BMP280::SAMPLING_X2, /* Temp. oversampling */
  Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
  Adafruit_BMP280::FILTER_X16, /* Filtering. */
  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

unsigned long lastRead = 0;
void loop() {
  ws.cleanupClients();
  unsigned long now = millis();
  if (now - lastRead > 1000) {
    lastRead = now;

    JsonDocument doc;
    doc["type"] = "status";
    doc["realTemp"] = currTemp;
    doc["heaterOn"]=heaterOn;
    if(heaterOn)
      doc["runSeconds"]=int((millis()-workTime)/1000)%36000;
    else
      doc["runSeconds"]=int((lastWorkTime-workTime)/1000)%36000;
    doc["targetTemp"] = targetTemp;
    doc["airTemp"]=airTemp;
    doc["targetAirTemp"]=targetAirTemp;
    String msg;
    serializeJson(doc, msg);
    ws.textAll(msg);
    currTemp = getTemp();
    if(heaterOn){      
      float error = targetTemp - currTemp;
      iTerm += Ki * error;
      iTerm = constrain(iTerm, 0, 100);
      float dError = currTemp - targetTemp;
      float output = Kp * error + iTerm - Kd * dError;

      output = constrain(output, 0, 100);
      if(now - pwmStart >= pwmPeriod){
       pwmStart = now;
      }
      if(output > (now - pwmStart) * 100 / pwmPeriod){
        digitalWrite(Heater_PIN, HIGH);
        digitalWrite(LED_PIN, HIGH);
      }else{
        digitalWrite(Heater_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
      }
    }
    Serial.print("Curr Temp: ");
    Serial.println(currTemp);
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
  }
}
//y = 0.0086x2 - 0.6275x + 22.984 - скорее всего ошибка
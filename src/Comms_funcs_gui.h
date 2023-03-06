#include <Arduino.h>
#include <Wifi.h>
#include <WiFiserver.h>


WiFiServer server(80);

#define log Serial.print
#define logln Serial.println

#define WIFI_TIMEOUT_MS 20000

const char* ssid = "Quedusalzerr";
const char* password = "667mmsovg";

int throttleCmd = 1000;
float newKp, newKi, newKd = 0;

void connectToWifi(){
  log("Connecting to wifi");
  WiFi.mode(WIFI_STA);             //THE ESP CONNECTS TO AN EXISTING WIFI NETWORK (STA = STATION)(AP = ACCESS POINT)
  WiFi.begin(ssid,password);

  unsigned long startConnectionTime = millis();

  while(WiFi.status() != WL_CONNECTED){
    log(".");
    delay(100);
  }

  if(WiFi.status() != WL_CONNECTED){
    logln("Connection timed out.");
  } else {
    logln("Connected successfully!");
    logln(WiFi.localIP());
  }
}


void handleClientRequest(WiFiClient client){
  Serial.println("New client");
  uint16_t th;
  float kp, ki, kd;
  char buf[14];
  
  // Receive the packed values from the client
  client.read((uint8_t*)buf, 14);

  // Unpack the values
  th = *(uint16_t*)(&buf[0]);
  kp = *(float*)(&buf[2]);
  ki = *(float*)(&buf[6]);
  kd = *(float*)(&buf[10]);

  // Print the values
  /* Serial.print("th = ");
  Serial.println(th);
  Serial.print("kp = ");
  Serial.println(kp);
  Serial.print("ki = ");
  Serial.println(ki);
  Serial.print("kd = ");
  Serial.println(kd); */
  if (th < 1000) throttleCmd = 1000; 
  else throttleCmd = th;
  newKp = kp;
  newKi = ki;
  newKd = kd;
  
  // Send a response back to the client
  client.write((uint8_t*)&th, 2);
  client.write((uint8_t*)&kp, 4);
  client.write((uint8_t*)&ki, 4);
  client.write((uint8_t*)&kd, 4);
}


/* void setup() {
  Serial.begin(9600);
  connectToWifi();
  server.begin();
}

void loop() {
 WiFiClient client = server.available();
  if (client) {
    handleClientRequest(client);
  }
} */
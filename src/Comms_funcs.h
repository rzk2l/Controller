#ifndef _COMMS_FUNCS_H
#define _COMMS_FUNCS_H
#include <Arduino.h>
#include <Wifi.h>
#include <WiFiserver.h>


WiFiServer server(80);

#define log Serial.print
#define logln Serial.println



const char* ssid = "Quedusalzerr";
const char* password = "667mmsovg";

int throttleCmd = 1000;

void connectToWifi(){
  log("Connecting to wifi");
  WiFi.mode(WIFI_STA);             //THE ESP CONNECTS TO AN EXISTING WIFI NETWORK (STA = STATION)(AP = ACCESS POINT)
  WiFi.begin(ssid,password);

  //unsigned long startConnectionTime = millis();

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

/* void setThrottleFromRequest(String request) {
  // Extract the throttle value from the request URL
  int throttleIndex = request.indexOf("value=");
  if (throttleIndex != -1) {
    String throttleValueStr = request.substring(throttleIndex + 6);
    int newThrottle = throttleValueStr.toInt();
    throttle = newThrottle;
    Serial.print("New throttle value: ");
    Serial.println(throttle);
  }
} */

/* void sendThrottleSetResponse(WiFiClient client) {
  // Send the value of throttle as bytes directly
  client.write((uint8_t *)&throttle, sizeof(throttle));
} */

void sendThrottleSetResponse(WiFiClient client) {
  // Send a response to the client
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("");
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  client.println("<body>");
  client.println("<h1>Throttle set!</h1>");
  client.println("</body>");
  client.println("</html>");
}

void sendNotFoundResponse(WiFiClient client) {
  // Send a response to the client
  client.println("HTTP/1.1 404 Not Found");
  client.println("Content-Type: text/html");
  client.println("");
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  client.println("<body>");
  client.println("<h1>Page not found</h1>");
  client.println("</body>");
  client.println("</html>");
}

void handleRoot(WiFiClient client) {
  // Send a response to the client
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("");
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  client.println("<h1>Throttle Control</h1>");
  client.println("<form action=\"/set-throttle\">");
  client.print("Throttle value: ");
  client.print(throttleCmd);
  client.println("<br>");
  client.println("<input type=\"range\" name=\"value\" min=\"0\" max=\"255\">");
  client.println("<br>");
  client.println("<input type=\"submit\" value=\"Set throttle\">");
  client.println("</form>");
  client.println("</html>");
}

/* void handleClientRequest(WiFiClient client){
  Serial.println("New client");
  while (!client.available()) {
    delay(1);
    //logln("g3nda?");
  }
  log("Atchou chwya");
  // Read the first line of the request
  String request = client.readStringUntil('\r');
  Serial.println(request);
  client.flush();

  // Check if the request is for setting the throttle value
  if (request.startsWith("Throttle")) {
    setThrottleFromRequest(request);
    sendThrottleSetResponse(client);
  } else {
    handleRoot(client);
  }

  //Serial.println("Client disconnected");
} */
void handleClientRequest(WiFiClient client) {
  Serial.println("New client");

  // Read the throttle value from the client
  uint16_t value;
  client.read((uint8_t*)&value, sizeof(value));
  throttleCmd = value;

  Serial.print("New throttle value: ");
  Serial.println(throttleCmd);

  // Send the throttle value back to the client
  client.write((uint8_t*)&throttleCmd, sizeof(throttleCmd));
}
#endif


#include <WiFi.h>
#include <WebSocketsServer.h>

const char* ssid = "<SSID_OF_NETWORD>";
const char* password = "<PASSWORD_OF_NETWORK>";

WebSocketsServer webSocket = WebSocketsServer(81);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) 
{
  switch (type) 
  {
    case WStype_TEXT:
      Serial.printf("[%u] Received: %s\n", num, payload);
      
      webSocket.sendTXT(num, payload);
      
      break;
  }
}

void setup() 
{
  Serial.begin(115200);
  delay(1000);
  
  // Connect to Wi-Fi
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.println("Connecting...");
  }
  
  Serial.println("Connected to Wi-Fi");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() 
{
  webSocket.loop();
}

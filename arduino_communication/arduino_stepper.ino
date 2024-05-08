
#include <WiFi.h>
#include <WebSocketsServer.h>

#define STEPPER_PIN_1 26
#define STEPPER_PIN_2 27
#define STEPPER_PIN_3 14
#define STEPPER_PIN_4 12

const char* ssid = "ALAN_NETWORK";
const char* password = "Zim77har";

int step_number = 0;
bool turning_right = false;
bool turning_left = false;

WebSocketsServer webSocket = WebSocketsServer(81);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) 
{
  switch (type) 
  {
    case WStype_TEXT:
      Serial.printf("[%u] Received: %s\n", num, payload);

      if (strcmp((char *)payload, "right") == 0) 
      {
        turning_left = false;
        turning_right = true;
        
        webSocket.sendTXT(num, "Turning Right"); // Send confirmation message
      } 
      else if (strcmp((char *)payload, "left") == 0) 
      {
        turning_right = false;
        turning_left = true;
        
        webSocket.sendTXT(num, "Turning Left"); // Send confirmation message
      }
      else if (strcmp((char *)payload, "stop") == 0) 
      {
        turning_left = false;
        turning_right = false;
        
        webSocket.sendTXT(num, "Stopping"); // Send confirmation message
      }
      
      break;
  }
}

void setup() 
{
  pinMode(STEPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_PIN_3, OUTPUT);
  pinMode(STEPPER_PIN_4, OUTPUT);
  
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
  
  if (turning_right == true )
  {
    OneStep(true);
  } 
  else if (turning_left == true)
  {
    OneStep(false);
  }
  
  delay(2);
}

void OneStep(bool dir)
{
  if(dir)
  {
    switch(step_number)
    {
      case 0:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
     } 
  }
  else
  {
    switch(step_number)
    {
      case 0:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
    } 
  }
    
  step_number++;
  if(step_number > 3)
  {
    step_number = 0;
  }
}

#define NO_POTENTIOMETER 1

#include <WiFi.h>
#include <WebSocketsServer.h>

#define STEPPER_PIN_1 26
#define STEPPER_PIN_2 27
#define STEPPER_PIN_3 14
#define STEPPER_PIN_4 12

#define STEPS_PER_REVOLUTION 2048

const char* ssid = "<NETWORK_SSID>";
const char* password = "<NETWORK_PASSWORD>";

#ifdef NO_POTENTIOMETER
int potentiometer_reading = 1;  // Used for temporary getAngle function
#endif

int step_number = 0;
double current_angle = 0.0;

double fault_tolerance = .1;

WebSocketsServer webSocket = WebSocketsServer(81);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) 
{
  switch (type) 
  {
    case WStype_TEXT:
      Serial.printf("[%u] Received: %s\n", num, payload);

      turnToAngle(atof((char *)payload));
      
      webSocket.sendTXT(num, "1"); // Send confirmation message
      
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
}

/*
 * Temporary function until a means to get the angle of the servo is obtained.
 * Returns values that are progressivly closer to desired angle each time it is called
*/
#ifdef NO_POTENTIOMETER
double getFakeAngle(double angle)
{
  int i;
  double inv = angle-current_angle;
  
  for (i=0; i<potentiometer_reading; i++)
  {
     inv *= .5;
  }

  potentiometer_reading++;

  return angle-inv;
}
#endif

double getAngle()
{
    return 0;
}

void turnToAngle(double angle)
{
  double d_angle;
  int steps;
  
  do
  {
    d_angle = angle - current_angle;
    steps = abs ((int) ((d_angle / 360.0) * STEPS_PER_REVOLUTION));
  
    if (d_angle >= 0)
    {
      turnSteps(true, steps);
    }
    else 
    {
      turnSteps(false, steps);
    }

    #ifdef NO_POTENTIOMETER
    current_angle = getFakeAngle(angle);
    #else
    current_angle = getAngle();  // Yet to be implemented
    #endif

    delay(100);
    
  } while (abs(angle-current_angle) > fault_tolerance);

  #ifdef NO_POTENTIOMETER
  potentiometer_reading = 1;  // Used for temporary returnAngle function
  #endif
}

void turnSteps(bool dir, int steps)
{
  for (int i = 0; i < steps; i++) 
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

    delay(2);
  }
}

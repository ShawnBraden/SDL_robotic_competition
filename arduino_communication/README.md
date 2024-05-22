# Wireless motor control through Arduino ESP32 module

Each of these files contains a different implementation of Arduino-Computer interommunication using the tcp ip protocol through websocket

We chose tcp ip for its speed and simplicity, and using websocket made the implementation simpler still.

# Instructions

## Setting Up Arduino IDE

To program our Arduino ESP32 board, we used the arduino IDE version 1.8.19. Instructions for installing this can be found online

After installing this version of the IDE, ensure you have the ESP32 board installed. Once again, guides to do this can be found online.

Finally, open the Library Manager in the IDE and install the WebSockets libary.

## Uploading code to the Arduino

Each of the 3 demo folders contain 2 files, one being the computer-side .py file, and the other being the arduino-side .ino file. We reccomend beginning with the communication_demo, as it is the simplest and does not require anything to be connected to the arduino board.

Open the esp_control.ino file in the Arduino IDE, and replace the strings in lines 4 and 5 with your networks SSID and PASSWORD

```ino 
const char* ssid = "<SSID_OF_NETWORD>";
const char* password = "<PASSWORD_OF_NETWORK>";
```

You are now ready to upload your code to the ESP32 board.

## Establishing communication between computer and board

When the ESP32 connects to your network, it will be assigned a local IP, which your computer will need to know in order to communicate. 

As soon as the arduino sketch has finished uploading, open the serial monitor, which will on; confirm a wifi connection, and two; give you the IP address of your ESP32

Once you have the IP, open the esp_com.py file, and edit line 5, replacing <IP_OF_ESP32> with the IP given to the ESP32

```python
ws_server_address = "ws://<IP_OF_ESP32>:81"
```

Finally, ensure your computer is connected to the same network as the ESP32, run the python code in a terminal, and the program should now work!
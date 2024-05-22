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
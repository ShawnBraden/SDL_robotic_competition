import websocket
import time

# Define the WebSocket server address
ws_server_address = "ws://<IP_OF_ESP32>:81"

# Connect to the WebSocket server
ws = websocket.WebSocket()
ws.connect(ws_server_address)

angle = input('angle: ')

ws.send(str(angle))

# Receive a message from the server (blocking call)
message = ws.recv()

# Process the received message
print("Received message:", message)

# Close WebSocket connection
ws.close() 

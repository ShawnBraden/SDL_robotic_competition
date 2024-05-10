import websocket
import time

# Define the WebSocket server address
ws_server_address = "ws://<IP_OF_ESP32>:81"

# Connect to the WebSocket server
ws = websocket.WebSocket()
ws.connect(ws_server_address)



message_sent = input('Message to send: ')

ws.send(message_sent)

message_received = ws.recv()

print("Received message:", message_received)

# Close WebSocket connection
ws.close()

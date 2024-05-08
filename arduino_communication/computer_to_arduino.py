
import websocket
import time

# Define the WebSocket server address
ws_server_address = "ws://10.0.0.159:81"

# Function to send command to ESP32
def send_command(command):
    ws.send(command)

# WebSocket event handler
def on_message(ws, message):
    print("Received message:", message)

# Connect to the WebSocket server
ws = websocket.WebSocket()
ws.connect(ws_server_address)

# Example usage
#send_command('100')

direction = input('direction? ')

send_command(str(direction))

# Receive a message from the server (blocking call)
message = ws.recv()

# Process the received message
print("Received message:", message)

# Close WebSocket connection
ws.close()

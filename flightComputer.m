import socket
import serial

# Configure your serial port and baud rate
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

# Set up local TCP server socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(("127.0.0.1", 5000))
server_socket.listen(1)

print(
    f"Bridge running: Listening for MATLAB on localhost:5000 (reading"
    f" {SERIAL_PORT})..."
)

try:
  ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
  while True:
    client_socket, addr = server_socket.accept()
    print(f"MATLAB connected from {addr}")
    while True:
      try:
        line = ser.readline().decode("utf-8")
        if line:
          client_socket.sendall(line.encode("utf-8"))
      except (socket.error, IOError):
        print("MATLAB disconnected. Waiting for reconnection...")
        client_socket.close()
        break
except KeyboardInterrupt:
  print("Bridge closed.")
finally:
  server_socket.close()

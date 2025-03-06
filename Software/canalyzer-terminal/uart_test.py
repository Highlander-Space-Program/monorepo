import serial
import serial.tools.list_ports
import time

# List all available serial ports
ports = list(serial.tools.list_ports.comports())
if not ports:
    print("No serial ports found.")
    exit()
print("Available serial ports:")
for index, port in enumerate(ports):
    print(f"{index}: {port.device} - {port.description}")

# Ask the user to select a port by its index
try:
    port_index = int(input("Enter the index of the port you want to use: "))
    selected_port = ports[port_index].device
except (ValueError, IndexError):
    print("Invalid selection. Exiting.")
    exit()

# Ask the user to input a baud rate or use default
try:
    baud_input = input("Enter the baud rate (default: 115200): ")
    baud_rate = int(baud_input) if baud_input else 115200
except ValueError:
    print("Invalid baud rate. Using default: 115200")
    baud_rate = 115200

# Create and open the serial connection
try:
    ser = serial.Serial(port=selected_port, baudrate=baud_rate, timeout=1)
    print(f"Connected to {selected_port} at {baud_rate} baud.")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

print("Reading and sending data continuously. Press Ctrl+C to exit.")
last_sent_time = time.time()
counter = 0

try:
    while True:
        # Check for incoming data and print it
        if ser.in_waiting:
            data = ser.readline().decode('utf-8', errors='replace').rstrip()
            print(f"Received: {data}")
        
        # Send a 10-byte message every 3 seconds
        current_time = time.time()
        if current_time - last_sent_time >= 1:
            # Generate a compact time message that's exactly 10 bytes
            # Format: T:HH:MM:S\n (where T is a marker, and HH:MM:S is the time with single-digit seconds)
            # This format is exactly 10 bytes including the newline
            hour = time.localtime(current_time).tm_hour
            minute = time.localtime(current_time).tm_min
            second = time.localtime(current_time).tm_sec
            
            # Create a message that's exactly 10 bytes (including newline)
            message = f"T:{hour:02d}:{minute:02d}:{second%10}\n"
            
            # Verify the message is exactly 10 bytes
            message_bytes = message.encode('utf-8')
            if len(message_bytes) != 10:
                # Alternative fixed-size format if needed
                counter = (counter + 1) % 100
                message = f"CNT:{counter:03d}!\n"
                message_bytes = message.encode('utf-8')
            
            # Send the message
            ser.write(message_bytes)
            print(f"Sent: {message.strip()} ({len(message_bytes)} bytes)")
            last_sent_time = current_time
        
        # Sleep briefly to prevent high CPU usage
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    print("Serial port closed")
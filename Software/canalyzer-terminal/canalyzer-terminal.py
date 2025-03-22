import serial
import serial.tools.list_ports
import threading
import queue
import time

"""
Filename: canalyzer-terminal.py
Author(s): Alexander Dobmeier (modified)
Date: 12-12-2024
Version: 0.5
Description: This code creates a Command Line Interface that a user can use to
  send USB messages to a CAN test board that then get converted to CAN and sent
  to boards to test. It uses separate threads for receiving and sending messages
  to ensure reliable communication. Extended CAN IDs (29-bit) are now supported.
  Enhanced to support servo, thermocouple, and heater components on servo boards.
"""

# Note about Extended CAN IDs:
# - The extended CAN ID uses 29 bits (compared to standard 11-bit IDs)
# - The format is: [8 bits sender][8 bits board ID][8 bits component type][5 bits instance]
#
# - For arbitration (which message gets priority on the bus), lower CAN ID values have higher priority
# - This means senders with lower IDs will always have priority over senders with higher IDs
#
#  CAN_ID is acquired based on a scheme. The first 11 bits are 0. the
#  Sender field (8 bits - first part of ID, highest priority impact):
#  - 0: Break wire system (highest priority)
#  - 1: Pad controller
#  - 2: Servo board
#  - 3: Sensor board
#  - 4: Tester board (this code and controller)
#  - 254: Tester board (hardware)
#  - 255: PC/Terminal (lowest priority)
#  - 5-253: Reserved for future devices

#  Board ID field (8 bits - identifies specific physical boards):
#  - 0-255: Unique identifier for each physical board in the system
#  - This allows up to 256 individual boards on the network

#  Component type field (8 bits - defines what type of component is being addressed):
#  - 0: System/board control
#  - 1: Servo
#  - 2: Thermocouple
#  - 3: Pressure transducer
#  - 4: Heater
#  - 5: LED
#  - 6: Flash Status
#  - 8-255: Reserved for future component types

#  Instance field (5 bits - identifies specific component instance):
#  - 0-31: Allows up to 32 instances of each component type per board
#  - For example, a board could have up to 32 servos, 32 thermocouples, etc.

#  Example of a full extended ID:
#  0x01050401 (hex) = 00000001 00000101 00000100 00001 000 (binary)
#                      |        |        |        |     |
#                      |        |        |        |		+-- Padding
# 					   |		|		 |		  +-------- Instance 1
#                      |        |        +----------------- Component Type 4 (Heater)
#                      |        +-------------------------- Board ID 5
#                      +----------------------------------- Sender 1 (Pad Controller)

#  This example ID represents: "Pad Controller (1) sending a message to Board 5,
#  addressing Heater (4), instance 1"

# Sender type definitions:
SENDER_BREAK_WIRE = 0
SENDER_PAD_CONTROLLER = 1
SENDER_SERVO_BOARD = 2
SENDER_SENSOR_BOARD = 3
SENDER_TESTER_BOARD = 4
SENDER_HW_TESTER = 254
SENDER_PC = 255

# Component type definitions:
MSG_TYPE_SYSTEM = 0
MSG_TYPE_SERVO = 1
MSG_TYPE_THERMOCOUPLE = 2
MSG_TYPE_PRESSURE = 3
MSG_TYPE_HEATER = 4
MSG_TYPE_LED = 5
MSG_TYPE_FLASH_SIGNAL = 6

# From servo_state_machine.h
SERVO_CMD_OPEN = 0    # OPEN_SERVO
SERVO_CMD_CLOSE = 1   # CLOSE_SERVO 

# From heater_state_machine.h
HEATER_CMD_OFF = 0    # H_OFF
HEATER_CMD_ON = 1     # H_ON
HEATER_CMD_AUTO = 2   # H_AUTO

# From thermocouple_state_machine.h
THERMO_CMD_GET_TEMP = 0       # FORCE_GET_TEMP
THERMO_CMD_RESET_TIMER = 1    # FORCE_RESET_TIMER

# Global variables
DEFAULT_BAUDRATE = 115200
PACKET_SIZE = 12  # Updated from 10 to 12 bytes to accommodate 4-byte extended ID
running = True
received_messages = {}
verbose_mode = True  # Default to verbose output

def create_extended_id(sender, board_id, component_type, instance):
    """Creates a 29-bit extended CAN ID with the specified fields
    
    Args:
        sender (int): 8-bit sender ID (0-255)
        board_id (int): 8-bit board ID (0-255)
        component_type (int): 8-bit component type (0-255)
        instance (int): 5-bit instance number (0-31)
        
    Returns:
        int: The 29-bit extended CAN ID
    """
    return (
        ((sender & 0xFF) << 21) |
        ((board_id & 0xFF) << 13) |
        ((component_type & 0xFF) << 5) |
        (instance & 0x1F)
    )

def parse_extended_id(ext_id):
    """Parses a 29-bit extended CAN ID into its component fields
    
    Args:
        ext_id (int): The 29-bit extended CAN ID
        
    Returns:
        tuple: (sender, board_id, component_type, instance)
    """
    sender = (ext_id >> 21) & 0xFF
    board_id = (ext_id >> 13) & 0xFF
    component_type = (ext_id >> 5) & 0xFF
    instance = ext_id & 0x1F
    return (sender, board_id, component_type, instance)

def format_extended_id(ext_id):
    """Formats a 29-bit extended CAN ID into a human-readable string
    
    Args:
        ext_id (int): The 29-bit extended CAN ID
        
    Returns:
        str: A human-readable representation
    """
    sender, board_id, msg_type, instance = parse_extended_id(ext_id)
    
    # Translate component types
    msg_type_str = "Unknown"
    if msg_type == MSG_TYPE_SERVO:
        msg_type_str = "Servo"
    elif msg_type == MSG_TYPE_THERMOCOUPLE:
        msg_type_str = "Thermocouple"
    elif msg_type == MSG_TYPE_PRESSURE:
        msg_type_str = "Pressure"
    elif msg_type == MSG_TYPE_HEATER:
        msg_type_str = "Heater"
    elif msg_type == MSG_TYPE_LED:
        msg_type_str = "LED"
    
    # Translate sender
    sender_str = "Unknown"
    if sender == SENDER_BREAK_WIRE:
        sender_str = "Break Wire"
    elif sender == SENDER_PAD_CONTROLLER:
        sender_str = "Pad Controller"
    elif sender == SENDER_SERVO_BOARD:
        sender_str = "Servo Board"
    elif sender == SENDER_SENSOR_BOARD:
        sender_str = "Sensor Board"
    elif sender == SENDER_TESTER_BOARD:
        sender_str = "Tester Board"
    elif sender == SENDER_HW_TESTER:
        sender_str = "HW Tester"
    elif sender == SENDER_PC:
        sender_str = "PC/Terminal"
    
    return f"Sender: {sender_str}({sender}), Board: {board_id}, Component: {msg_type_str}({msg_type}), Instance: {instance}"

def receive_thread(ser, rx_queue):
    """Thread function to continuously receive data from serial port"""
    global running, received_messages
    
    print(f"RX thread started on {ser.port}")
    
    while running:
        try:
            if ser.in_waiting >= PACKET_SIZE:  # Ensure we have a full packet
                recvd = bytearray(PACKET_SIZE)
                ser.readinto(recvd)
                
                # Extract CAN ID (now 29-bit extended ID)
                recvd_id = (recvd[0] << 24) | (recvd[1] << 16) | (recvd[2] << 8) | recvd[3]
                
                recvd_packet = [None] * 8  # Now we have 8 data bytes
                for i in range(8):
                    recvd_packet[i] = recvd[i + 4]
                
                # Store message with ID as key
                received_messages[recvd_id] = recvd_packet
                
                # Print received data for monitoring
                if verbose_mode:
                    print(f"RX: ID=0x{recvd_id:08X} ({format_extended_id(recvd_id)})")
                    print(f"    Data={[hex(b) for b in recvd_packet if b is not None]}")
                
            time.sleep(0.01)  # Short delay to prevent high CPU usage
        except Exception as e:
            print(f"RX Thread Error: {e}")
            time.sleep(1)  # Wait before trying again
    
    print("RX thread terminated")

def transmit_thread(ser, tx_queue):
    """Thread function to transmit data from queue to serial port"""
    global running
    
    print(f"TX thread started on {ser.port}")
    
    while running:
        try:
            # Get message from queue with timeout
            msg = tx_queue.get(timeout=0.5)
            
            if msg is not None:
                if isinstance(msg, int):
                    # This is a query for a specific CAN ID
                    if verbose_mode:
                        print(f"Querying message with ID=0x{msg:08X} ({format_extended_id(msg)})")
                    
                    # Check if message exists in our stored messages
                    if msg in received_messages:
                        print(f"Found: {[hex(b) for b in received_messages[msg] if b is not None]}")
                    else:
                        print(f"Message with ID=0x{msg:08X} not found")
                        
                elif isinstance(msg, bytearray):
                    # This is a message to transmit
                    ser.write(msg)
                    
                    # Extract and display ID for user feedback
                    id_val = (msg[0] << 24) | (msg[1] << 16) | (msg[2] << 8) | msg[3]
                    
                    if verbose_mode:
                        print(f"TX: ID=0x{id_val:08X} ({format_extended_id(id_val)})")
                        print(f"    Data={[hex(b) for b in msg[4:PACKET_SIZE]]}")
                    
            tx_queue.task_done()
            
        except queue.Empty:
            # No message in queue, just continue
            pass
        except Exception as e:
            print(f"TX Thread Error: {e}")
            time.sleep(1)  # Wait before trying again
    
    print("TX thread terminated")

def list_ports():
    """Lists and returns available serial ports"""
    ports = list(serial.tools.list_ports.comports())
    
    if not ports:
        print("No serial ports found.")
        return []
    
    print("\nAvailable serial ports:")
    for index, port in enumerate(ports):
        print(f"{index}: {port.device} - {port.description}")
    
    return ports

def configure_serial():
    """Allows user to select and configure a serial port"""
    ports = list_ports()
    
    if not ports:
        return None
    
    # Get port selection from user
    try:
        port_index = int(input("\nEnter the index of the port you want to use (default: 0): ") or "0")
        selected_port = ports[port_index].device
    except (ValueError, IndexError):
        print("Invalid selection. Using first available port.")
        selected_port = ports[0].device
    
    # Get baud rate from user or use default
    try:
        baud_input = input(f"Enter the baud rate (default: {DEFAULT_BAUDRATE}): ")
        baud_rate = int(baud_input) if baud_input else DEFAULT_BAUDRATE
    except ValueError:
        print(f"Invalid baud rate. Using default: {DEFAULT_BAUDRATE}")
        baud_rate = DEFAULT_BAUDRATE
    
    # Try to open the port
    try:
        ser = serial.Serial()
        ser.port = selected_port
        ser.baudrate = baud_rate
        ser.timeout = 0.1
        ser.open()
        print(f"Connected to {selected_port} at {baud_rate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None

def create_toggle_tester_led(board_id=254, instance=0):
    """Creates a packet to toggle the LED on the tester board"""
    packet = bytearray(PACKET_SIZE)
    
    # Create extended ID for tester board LED: sender=PC, board=254 (tester), comp=LED (6), instance=0
    id_val = create_extended_id(SENDER_PC, board_id, MSG_TYPE_LED, instance)
    
    packet[0] = (id_val >> 24) & 0xFF
    packet[1] = (id_val >> 16) & 0xFF
    packet[2] = (id_val >> 8) & 0xFF
    packet[3] = id_val & 0xFF
    
    # Command to toggle LED
    packet[11] = 0x01
    
    return packet

def toggle_tester_led(tx_queue, board_id=254, instance=0):
    """Sends command to tester board to toggle the LED on it"""
    print(f"Sending toggle LED message to board {board_id}, instance {instance}")
    packet = create_toggle_tester_led(board_id, instance)
    tx_queue.put(packet)

def create_signal_all_leds(tx_queue, board_id=254):
    """Creates a packet to toggle the LED on the tester board"""
    packet = bytearray(PACKET_SIZE)
    
    # Create extended ID for tester board LED: sender=PC, board=254 (tester), comp=LED (6), instance=0
    id_val = create_extended_id(SENDER_PC, board_id, MSG_TYPE_FLASH_SIGNAL, 0)
    
    packet[0] = (id_val >> 24) & 0xFF
    packet[1] = (id_val >> 16) & 0xFF
    packet[2] = (id_val >> 8) & 0xFF
    packet[3] = id_val & 0xFF
    
    # Command to toggle LED
    packet[11] = 0x01
    
    tx_queue.put(packet)

def config_messages(tx_queue):
    """Menu for config messages"""
    print("\nSelect the config message you'd like to send\n")
    print("t - Toggle LED on tester board (ID 254)")
    print("l - Toggle LED on specific board")
    print("s - Signal all LEDs on board")
    print("v - Toggle verbose mode")
    
    config_user_in = input("Select your option: ")
    if config_user_in.lower() == "t":
        toggle_tester_led(tx_queue)
    elif config_user_in.lower() == "l":
        try:
            board_id = int(input("Enter board ID (0-255): "), 10)
            instance = int(input("Enter LED instance (0-31, default 0): ") or "0", 10)
            toggle_tester_led(tx_queue, board_id, instance)
        except ValueError as e:
            print(f"Error: {e}")
            print("Please use valid values")
    elif config_user_in.lower() == "s":
        try:
            board_id = int(input("Enter board ID (0-255): "), 10)
            create_signal_all_leds(tx_queue, board_id)
        except ValueError as e:
            print(f"Error: {e}")
            print("Please use valid values")
    elif config_user_in.lower() == "v":
        toggle_verbose_mode()

def toggle_verbose_mode():
    """Toggles verbose output mode"""
    global verbose_mode
    verbose_mode = not verbose_mode
    print(f"Verbose mode {'enabled' if verbose_mode else 'disabled'}")

def raw_message(tx_queue):
    """Prompts user to enter extended ID and bytes for a CAN packet then sends to tester board"""
    try:
        print("\nExtended CAN ID components:")
        print("Sender options:")
        print(f"  {SENDER_BREAK_WIRE}: Break Wire (highest priority)")
        print(f"  {SENDER_PAD_CONTROLLER}: Pad Controller")
        print(f"  {SENDER_SERVO_BOARD}: Servo Board")
        print(f"  {SENDER_SENSOR_BOARD}: Sensor Board")
        print(f"  {SENDER_TESTER_BOARD}: Tester Board")
        print(f"  {SENDER_HW_TESTER}: Hardware Tester")
        print(f"  {SENDER_PC}: PC/Terminal (lowest priority)")
        sender = int(input("Sender (0-255, default 255 for PC): ") or "255", 10)
        
        board_id = int(input("Board ID (0-255): "), 10)
        
        print("\nComponent type options:")
        print(f"  {MSG_TYPE_SYSTEM}: System/Board Control")
        print(f"  {MSG_TYPE_SERVO}: Servo")
        print(f"  {MSG_TYPE_THERMOCOUPLE}: Thermocouple")
        print(f"  {MSG_TYPE_PRESSURE}: Pressure Transducer")
        print(f"  {MSG_TYPE_HEATER}: Heater")
        print(f"  {MSG_TYPE_LED}: LED")
        msg_type = int(input("Component Type (0-255): "), 10)
        
        instance = int(input("Instance (0-31): "), 10)
        
        # Create the extended ID
        ext_id = create_extended_id(sender, board_id, msg_type, instance)
        
        packet = bytearray(PACKET_SIZE)
        packet[0] = (ext_id >> 24) & 0xFF
        packet[1] = (ext_id >> 16) & 0xFF
        packet[2] = (ext_id >> 8) & 0xFF
        packet[3] = ext_id & 0xFF
        
        print("\nEnter 8 payload bytes (0-255 decimal values, press Enter to default to 0):")
        for i in range(8):
            input_value = input(f"Byte {i}: ")
            curr_byte = int(input_value, 10) if input_value else 0
            packet[i + 4] = curr_byte
        
        if verbose_mode:
            print(f"Packet: {[hex(b) for b in packet]}")
        tx_queue.put(packet)
        
    except (ValueError, IndexError) as e:
        print(f"Error creating message: {e}")
        print("Please use valid values")

def create_sensor_board_toggle_led(board_id, instance):
    """Creates a packet to toggle LED on a sensor board"""
    packet = bytearray(PACKET_SIZE)
    
    # Create extended ID - now addressing the LED component type directly
    ext_id = create_extended_id(SENDER_PC, board_id, MSG_TYPE_LED, instance)
    
    packet[0] = (ext_id >> 24) & 0xFF
    packet[1] = (ext_id >> 16) & 0xFF
    packet[2] = (ext_id >> 8) & 0xFF
    packet[3] = ext_id & 0xFF
    
    # Command to toggle LED
    packet[4] = 0x01
    
    return packet

def sensor_board_toggle_led(board_id, instance, tx_queue):
    """Sends a packet to toggle LED on a sensor board"""
    packet = create_sensor_board_toggle_led(board_id, instance)
    tx_queue.put(packet)

def sensor_board_get_value(tx_queue, board_id, instance):
    """Queries the value from a sensor board"""
    # Create the query ID
    ext_id = create_extended_id(SENDER_TESTER_BOARD, board_id, MSG_TYPE_PRESSURE, instance)
    
    tx_queue.put(ext_id)
    
    # Wait a bit for the message to be received
    if verbose_mode:
        print(f"Querying sensor with ID=0x{ext_id:08X} ({format_extended_id(ext_id)})")
    print("Check received message display for response")

def sensor_board_messages(tx_queue):
    """Prompts user to select what kind of sensor board message to send"""
    try:
        board_id = int(input("Enter the board ID (0-255): "), 10)
        instance = int(input("Enter the sensor instance (0-31): "), 10)
        
        print("\nSelect a sensor board message to send:")
        print("t - Toggle status LED")
        print("g - Get sensor board reading")
        
        user_in = input("\nSelect your message: ")
        
        if user_in.lower() == "t":
            sensor_board_toggle_led(board_id, instance, tx_queue)
        elif user_in.lower() == "g":
            sensor_board_get_value(tx_queue, board_id, instance)
        else:
            print("Invalid option")
            
    except (ValueError, IndexError) as e:
        print(f"Error processing sensor board message: {e}")

# Servo Board Functions
def create_servo_board_packet(board_id, msg_type, instance, command_value):
    """Creates a generic packet for sending to a servo board component
    
    Args:
        board_id (int): Board ID (0-255)
        msg_type (int): Component type (1=servo, 2=thermocouple, 4=heater)
        instance (int): Instance number (0-31)
        command_value (int): Command value matching the state machine commands
        
    Returns:
        bytearray: Formatted packet ready to send
    """
    packet = bytearray(PACKET_SIZE)
    
    # Create extended ID
    ext_id = create_extended_id(SENDER_PC, board_id, msg_type, instance)
    
    packet[0] = (ext_id >> 24) & 0xFF
    packet[1] = (ext_id >> 16) & 0xFF
    packet[2] = (ext_id >> 8) & 0xFF
    packet[3] = ext_id & 0xFF
    
    # Command value in first byte - now matches state machine command values
    packet[4] = command_value
    
    return packet

def servo_board_set_servo_position(tx_queue, board_id, instance, is_open):
    """Sends a packet to set servo position based on the servo state machine
    
    Args:
        tx_queue: Queue for sending messages
        board_id (int): Board ID (0-255)
        instance (int): Servo instance (0-31)
        is_open (bool): True to open the servo, False to close it
    """
    # Use the proper command from servo_state_machine.h
    command = SERVO_CMD_OPEN if is_open else SERVO_CMD_CLOSE
    
    packet = create_servo_board_packet(board_id, MSG_TYPE_SERVO, instance, command)
    tx_queue.put(packet)
    
    action = "OPEN" if is_open else "CLOSE"
    print(f"Sent command to {action} servo {instance} on board {board_id}")

def servo_board_query_thermocouple(tx_queue, board_id, instance):
    """Queries the temperature from a thermocouple using state machine commands
    
    Args:
        tx_queue: Queue for sending messages
        board_id (int): Board ID (0-255)
        instance (int): Thermocouple instance (0-31)
    """
    # Use the FORCE_GET_TEMP command from the thermocouple state machine
    packet = create_servo_board_packet(board_id, MSG_TYPE_THERMOCOUPLE, instance, THERMO_CMD_GET_TEMP)
    tx_queue.put(packet)
    
    print(f"Sent command to get temperature from thermocouple {instance} on board {board_id}")
    print("Check received message display for response")

def servo_board_reset_thermocouple_timer(tx_queue, board_id, instance):
    """Resets the timer for a thermocouple
    
    Args:
        tx_queue: Queue for sending messages
        board_id (int): Board ID (0-255)
        instance (int): Thermocouple instance (0-31)
    """
    packet = create_servo_board_packet(board_id, MSG_TYPE_THERMOCOUPLE, instance, THERMO_CMD_RESET_TIMER)
    tx_queue.put(packet)
    
    print(f"Sent command to reset timer for thermocouple {instance} on board {board_id}")

def servo_board_heater_control(tx_queue, board_id, instance, mode):
    """Sends a packet to control a heater with modes from the state machine
    
    Args:
        tx_queue: Queue for sending messages
        board_id (int): Board ID (0-255)
        instance (int): Heater instance (0-31)
        mode (int): 0 for off, 1 for on, 2 for auto
    """
    # Ensure mode matches one of the valid heater state machine commands
    if mode not in [HEATER_CMD_OFF, HEATER_CMD_ON, HEATER_CMD_AUTO]:
        print(f"Invalid heater mode: {mode}. Using OFF mode.")
        mode = HEATER_CMD_OFF
    
    packet = create_servo_board_packet(board_id, MSG_TYPE_HEATER, instance, mode)
    tx_queue.put(packet)
    
    mode_str = "OFF" if mode == HEATER_CMD_OFF else "ON" if mode == HEATER_CMD_ON else "AUTO"
    print(f"Sent command to set heater {instance} on board {board_id} to {mode_str} mode")

def servo_board_messages(tx_queue):
    """Enhanced menu for servo board messages aligned with state machine commands"""
    try:
        # Display information about the servo board components
        print("\n=== Servo Board Control Interface ===")
        print("The servo board can control multiple component types:")
        print(f"  {MSG_TYPE_SERVO}: Servo motors (Instance 0-31)")
        print(f"  {MSG_TYPE_THERMOCOUPLE}: Thermocouples (Instance 0-31)")
        print(f"  {MSG_TYPE_HEATER}: Heaters (Instance 0-31)")
        print("\nBased on config file, board ID 1 is a known servo board.")
        
        # Get board ID
        board_id = int(input("\nEnter the board ID (0-255, default 1 for servo board): ") or "1", 10)
        
        # Select component type
        print("\nSelect component type:")
        print(f"1 - Servo ({MSG_TYPE_SERVO})")
        print(f"2 - Thermocouple ({MSG_TYPE_THERMOCOUPLE})")
        print(f"4 - Heater ({MSG_TYPE_HEATER})")

        msg_choice = input("Enter choice (1/2/4): ")
        
        if msg_choice == "1":  # Servo
            instance = int(input("Enter servo instance (0-31): "), 10)
            
            print("\nServo Control Options:")
            print("o - Open servo")
            print("c - Close servo")
            
            action = input("Select action (o/c): ")
            
            if action.lower() == "o":
                servo_board_set_servo_position(tx_queue, board_id, instance, True)  # Open servo
            elif action.lower() == "c":
                servo_board_set_servo_position(tx_queue, board_id, instance, False)  # Close servo
            else:
                print("Invalid action selected")
                
        elif msg_choice == "2":  # Thermocouple
            instance = int(input("Enter thermocouple instance (0-31): "), 10)
            
            print("\nThermocouple Options:")
            print("r - Read temperature (FORCE_GET_TEMP)")
            print("t - Reset timer (FORCE_RESET_TIMER)")
            
            action = input("Select action (r/t): ")
            
            if action.lower() == "r":
                servo_board_query_thermocouple(tx_queue, board_id, instance)
            elif action.lower() == "t":
                servo_board_reset_thermocouple_timer(tx_queue, board_id, instance)
            else:
                print("Invalid action selected")
                
        elif msg_choice == "4":  # Heater
            instance = int(input("Enter heater instance (0-31): "), 10)
            
            print("\nHeater Control Options:")
            print("0 - Turn heater OFF")
            print("1 - Turn heater ON")
            print("2 - Set heater to AUTO mode (temperature controlled)")
            
            action = input("Select action (0/1/2): ")
            
            if action in ["0", "1", "2"]:
                servo_board_heater_control(tx_queue, board_id, instance, int(action))
            else:
                print("Invalid action selected")
                
        else:
            print("Invalid component type selected")
            
    except (ValueError, IndexError) as e:
        print(f"Error processing servo board message: {e}")
        print("Please use valid numeric values")
        
def main():
    global running
    
    print("=== CAN Analyzer Terminal v0.5 with Enhanced Servo Board Control ===")
    print(f"Default baud rate: {DEFAULT_BAUDRATE}")
    print(f"Packet size: {PACKET_SIZE} bytes (4 ID + 8 data)")
    print(f"Verbose mode: {'On' if verbose_mode else 'Off'}")
    print("Extended ID Format: [8 bits sender][8 bits board ID][8 bits component type][5 bits instance]")
    print("\nComponent Types:")
    print(f"  {MSG_TYPE_SERVO}: Servo")
    print(f"  {MSG_TYPE_THERMOCOUPLE}: Thermocouple")
    print(f"  {MSG_TYPE_PRESSURE}: Pressure Transducer")
    print(f"  {MSG_TYPE_HEATER}: Heater")
    print(f"  {MSG_TYPE_LED}: LED")
    
    # Create message queues
    tx_queue = queue.Queue()
    
    # Initialize serial connection as None
    ser = None
    rx_thread = None
    tx_thread = None
    
    try:
        while True:
            print("\nSelect an option:")
            print("l - List serial ports")
            print("c - Config messages")
            print("r - Make and send raw message")
            print("p - Sensor board messages")
            print("s - Servo board messages (Servo/Thermocouple/Heater)")
            print("t - Configure Serial Port")
            print("v - Toggle verbose mode")
            print("q - Quit")
            
            user_in = input("\nSelect your option: ")
            
            if user_in.lower() == "l":
                list_ports()
                
            elif user_in.lower() == "c":
                if ser and ser.is_open:
                    config_messages(tx_queue)
                else:
                    print("Serial port not configured. Use option 't' first.")
                    
            elif user_in.lower() == "t":
                # Close existing connection and threads if they exist
                if ser and ser.is_open:
                    running = False
                    if rx_thread and rx_thread.is_alive():
                        rx_thread.join(timeout=1.0)
                    if tx_thread and tx_thread.is_alive():
                        tx_thread.join(timeout=1.0)
                    ser.close()
                
                # Configure new connection
                ser = configure_serial()
                
                if ser and ser.is_open:
                    # Reset flags and containers
                    running = True
                    received_messages.clear()
                    
                    # Start threads
                    rx_thread = threading.Thread(target=receive_thread, args=(ser, None))
                    tx_thread = threading.Thread(target=transmit_thread, args=(ser, tx_queue))
                    
                    rx_thread.daemon = True
                    tx_thread.daemon = True
                    
                    rx_thread.start()
                    tx_thread.start()
                
            elif user_in.lower() == "r":
                if ser and ser.is_open:
                    raw_message(tx_queue)
                else:
                    print("Serial port not configured. Use option 't' first.")
                    
            elif user_in.lower() == "p":
                if ser and ser.is_open:
                    sensor_board_messages(tx_queue)
                else:
                    print("Serial port not configured. Use option 't' first.")
                    
            elif user_in.lower() == "s":
                if ser and ser.is_open:
                    servo_board_messages(tx_queue)
                else:
                    print("Serial port not configured. Use option 't' first.")
            
            elif user_in.lower() == "v":
                toggle_verbose_mode()
                    
            elif user_in.lower() == "q":
                print("Exiting...")
                break
                
            else:
                print("Invalid option. Please try again.")
    
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    
    finally:
        # Clean up resources
        running = False
        
        if rx_thread and rx_thread.is_alive():
            rx_thread.join(timeout=1.0)

        if tx_thread and tx_thread.is_alive():
            tx_thread.join(timeout=1.0)
        
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed")

        print("Program terminated")

if __name__ == "__main__":
    main()
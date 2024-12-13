import serial
import serial.tools.list_ports
import multiprocessing

"""
Filename: canalyzer-terminal.py
Author(s): Alexander Dobmeier
Date: 12-12-2024
Version: 0.1
Description: This code creates a Command Line Interface that a user can use to
  send USB messages to a CAN test board that then get converted to CAN and sent
  to boards to test.  After configuring the serial port to talk to the tester
  board, the code creates a process to handle that USB port.  We can send messages
  to that handler to send as well as message IDs to query and display via the tx and
  rx queues.
"""

def Monitor_Serial(tx_queue, rx_queue, port, baud):
    """Function used for the process that handles the USB port
    
    When USB messages from the tester board come in in the form:
    {CAN message ID}{data}
    we store them in a dict with the key being the message ID.  If a new message of
    that ID comes in we overrite it.  We can query those messages by sending a single
    byte to the process via tx_queue.  To send 
    """
    print("monitoring serial")
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.open()
    receivedMessages = {}

    while True:
        if ser.in_waiting:
            recvd = bytearray(10)
            ser.readinto(recvd)
            # print(recvd)
            recvd_id = int((recvd[0] << 8) | recvd[1])
            recvd_packet = [None] * 8
            for i in range(8):
                recvd_packet[i] = recvd[i + 2]
            receivedMessages[recvd_id] = recvd_packet
        
        msg = None
        if not tx_queue.empty():
            msg = tx_queue.get()
            print(msg)
        
        if msg is not None:
            if isinstance(msg, int):
                print(f"Querying {hex(msg)}")
                if msg in receivedMessages.keys():
                    rx_queue.put(receivedMessages[msg])
                else:
                    rx_queue.put(["whoops"])
            elif isinstance(msg, bytearray):
                ser.write(msg)

def List_Ports():
    """Prints list of current ports recognized by pySerial"""

    print("PORTS:")
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        print(f"{port}: {desc} [{hwid}]")

def Create_Toggle_Testser_LED():
    packet = bytearray(10)
    id = 0x0000
    packet[0] = id >> 8
    packet[1] = id & 0x00FF
    packet[9] = 0x01

def Toggle_Tester_LED(tx_queue):
    """Sends command to tester board to toggle the LED on it"""

    print("Sending toggle LED message")
    packet = Create_Toggle_Testser_LED()
    tx_queue.put(packet)

def Configure_Serial(tx_queue, rx_queue):
    """Lists ports then prompts user to configure port and baud rate for serial to tester board"""

    List_Ports()
    port = input("Please enter the port you'd like to use: ")
    baud = int(input("Please enter the baud rate you'd like to use (must be an int): "))
    monitorer = multiprocessing.Process(target=Monitor_Serial, args=(tx_queue, rx_queue, port, baud))
    monitorer.start()
    return monitorer

def Config_Messages(tx_queue):
    """Menu for config messages"""

    print("\nSelect the config message you'd like to send\n")
    print("t - Toggle LED")
    print("p - Print Serial settings")

    config_user_in = input("Select your option: ")
    match config_user_in:
        case "t":
            Toggle_Tester_LED(tx_queue)

        case "p":
            print(ser)

def Raw_Message(tx_queue):
    """Prompts user to enter txID and bytes for a CAN packet then sends to tester board"""

    id = input("Enter tx ID byte: ")
    id_H = id[0] + id[1]
    id_L = id[2] + id[3]
    packet = bytearray(10)
    packet[0] = int(id_H, 16)
    packet[1] = int(id_L, 16)
    for i in range(8):
        currByte = input(f"Enter payload byte {i}: ")
        packet[i + 2] = int(currByte, 16)

    print(packet)
    
    tx_queue.put(packet)

def Create_Sensor_Board_Toggle_LED(sensor_board_id_H, sensor_board_id_L):
    packet = bytearray(10)
    packet[0] = int(sensor_board_id_H, 16)
    packet[1] = int(sensor_board_id_L, 16)
    packet[2] = 0x17
    return packet

def Sensor_Board_Toggle_LED(sensor_board_id_H, sensor_board_id_L, tx_queue):
        packet = Create_Sensor_Board_Toggle_LED(sensor_board_id_H, sensor_board_id_L)
        tx_queue.put(packet)

def Sensor_Board_Get_Value(tx_queue, rx_queue, sense_id):
    tx_queue.put(sense_id)
    packet = rx_queue.get(timeout=1)

    #TODO just a temp val for now
    val = (packet[0] << 24) | (packet[1] << 16) | (packet[2] << 8) | (packet[3])
    print(packet)

def Sensor_Board_Messages(tx_queue, rx_queue):
    """Prompts user to select what kind of sensor board message to send"""
    #TODO make it not just a temporary demo
    sens_id = input("Enter the ID of the sensor board you'd like to talk to: ")
    sens_id_H = sens_id[0] + sens_id[1]
    sens_id_L = sens_id[2] + sens_id[3]
    print("Select a sensor board message to send:\n")
    print("t - Toggle status LED")
    print("g - Get sensor board reading")
    user_in = input("\nSelect your message: ")

    match user_in:
        case "t":
            Sensor_Board_Toggle_LED(sens_id_H, sens_id_L, tx_queue)
        case "g":
            Sensor_Board_Get_Value(tx_queue, rx_queue, int(sens_id, 16))

def Create_Servo_Board_Heater_Off(serv_id_H, serv_id_L):
    packet = bytearray(10)
    packet[0] = int(serv_id_H, 16)
    packet[1] = int(serv_id_L, 16)
    packet[2] = 0x00
    return packet

def Servo_Board_Heater_Off(tx_queue, serv_id_H, serv_id_L):
    packet = Create_Servo_Board_Heater_Off(serv_id_H, serv_id_L)
    tx_queue.put(packet)

def Create_Servo_Board_Heater_On(serv_id_H, serv_id_L):
    packet = bytearray(10)
    packet[0] = int(serv_id_H, 16)
    packet[1] = int(serv_id_L, 16)
    packet[2] = 0x01
    return packet

def Servo_Board_Heater_On(tx_queue, serv_id_H, serv_id_L):
    packet = Create_Servo_Board_Heater_On(serv_id_H, serv_id_L)
    tx_queue.put(packet)
            
def Servo_Board_Messages(tx_queue, rx_queue):
    sens_id = input("Enter the ID of the servo board you'd like to talk to: ")
    serv_id_H = sens_id[0] + sens_id[1]
    serv_id_L = sens_id[2] + sens_id[3]
    print("Select a servo board message to send:\n")
    print("h - Turn Heater on")
    print("j - Turn Heater off")

    user_in = input("\nSelect your message: ")

    match user_in:
        case "h":
            Servo_Board_Heater_On(tx_queue, serv_id_H, serv_id_L)
        case "j":
            Servo_Board_Heater_Off(tx_queue, serv_id_H, serv_id_L)


def main():
    monitorer = None
    tx_queue = multiprocessing.Queue()
    rx_queue = multiprocessing.Queue()

    while True:
        print("\nSelect an option:")
        print("\n")
        print("l - List serial ports")
        print("c - Config messages")
        print("r - Make and send raw message")
        print("p - Sensor board messages")
        print("s - Servo board messages")
        print("q - quit")
        print("t - Configure Serial Port")

        user_in = input("Select your option: ")
        match user_in:
            case "l":
                List_Ports()

            case "c":
                Config_Messages(tx_queue)

            case "t":
                if monitorer is not None:
                    monitorer.terminate()
                monitorer = Configure_Serial(tx_queue, rx_queue)

            case "r":
                Raw_Message(tx_queue)

            case "p":
                Sensor_Board_Messages(tx_queue, rx_queue)

            case "s":
                Servo_Board_Messages(tx_queue, rx_queue)
            
            case "q":
                if monitorer is not None:
                    monitorer.terminate()
                quit()
            
if __name__ == "__main__":
    main()
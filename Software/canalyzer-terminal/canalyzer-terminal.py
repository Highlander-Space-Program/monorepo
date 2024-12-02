import serial
import serial.tools.list_ports

def list_ports():
    print("PORTS:")
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        print(f"{port}: {desc} [{hwid}]")

def config_messages(ser):
    print("\nSelect the config message you'd like to send\n")
    print("t - Toggle LED")
    print("c - Configure Serial")
    print("p - Print Serial settings")

    config_user_in = input("Select your option: ")
    match config_user_in:
        case "t":
            print("Sending toggle LED message")
            packet = bytearray(10)
            id = 0x0000
            packet[0] = id >> 8
            packet[1] = id & 0x00FF
            packet[9] = 0x01
            print(packet)
            ser.write(packet)

        case "c":
            port = input("Please enter the port you'd like to use: ")
            baud = int(input("Please enter the baud rate you'd like to use (must be an int): "))
            ser.port = port
            ser.baudrate = baud
            ser.open()

        case "p":
            print(ser)

def raw_message(ser):
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
    
    ser.write(packet)



def main(ser):

    while True:
        print("\nSelect an option:")
        print("\n")
        print("l - List serial ports")
        print("c - Config messages")
        print("r - Make and send raw message")
        print("q - quit")

        user_in = input("Select your option: ")
        match user_in:
            case "l":
                list_ports()

            case "c":
                config_messages(ser)

            case "r":
                raw_message(ser)
            
            case "q":
                quit()
            
if __name__ == "__main__":
    ser = serial.Serial()
    main(ser)
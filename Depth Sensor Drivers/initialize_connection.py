from brping import Ping1D

def initialize_ping_sensor(serial_port, baudrate):
 
    my_ping = Ping1D()

    my_ping.connect_serial(serial_port, baudrate)


    if not my_ping.initialize():
        print("Failed to initialize the Ping sensor!")
        return None
    else:
        print("Ping sensor initialized successfully.")
        return my_ping

def main():
    
    serial_port = '/dev/ttyUSB0'  
    baudrate = 115200  

    ping_sensor = initialize_ping_sensor(serial_port, baudrate)


if __name__ == "__main__":
    main()

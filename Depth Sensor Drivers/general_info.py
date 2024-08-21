from brping import Ping1D

def initialize_ping_sensor(serial_port, baudrate):
    my_ping = Ping1D()
    my_ping.connect_serial(serial_port, baudrate)
    my_ping.set_ping_interval(29)
    my_ping.set_speed_of_sound(1500)
    my_ping.set_mode_auto(1,verify=True)
    if not my_ping.initialize():
        print("Failed to initialize the Ping sensor!")
        return None
    else:
        print("Ping sensor initialized successfully.")
        return my_ping

def get_general_info(ping_sensor):
    info = ping_sensor.get_general_info()
    print(info) 
#     if info is not None:
#         print("General Information:")
#         print(f"  Firmware Version: {info['firmware_version_major']}.{info['firmware_version_minor']}")
#         print(f"  Device Supply Voltage: {info['voltage_5']} mV")
#         print(f"  Ping Interval: {info['ping_interval']} ms")
#         print(f"  Gain Setting: {info['gain_setting']}")
#         print(f"  Operating Mode: {'Auto' if info['mode_auto'] == 1 else 'Manual'}")
#     else:
#         print("Failed to get general information from the sensor.")

def set_gain(ping_sensor,gain_setting):
    result=ping_sensor.set_gain_setting(gain_setting,verify=True)
    if result:
        print("succesful")
    else:
        print("failed")

def main():
    serial_port = '/dev/ttyUSB0' 
    baudrate = 115200  

    ping_sensor = initialize_ping_sensor(serial_port, baudrate)
    if ping_sensor is not None:
        desired_gain_setting=2
        set_gain(ping_sensor,desired_gain_setting)
        get_general_info(ping_sensor)

if __name__ == "__main__":
    main()


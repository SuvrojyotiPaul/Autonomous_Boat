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

def get_distance_data(ping_sensor):
    data = ping_sensor.get_distance()
    print(data)
    if data:
        print("Distance Data:")
        print(f"  Distance: {data['distance']} mm")
        print(f"  Confidence: {data['confidence']}%")
    else:
        print("Failed to get distance data from the sensor.")
  
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
        desired_gain_setting=6
        set_gain(ping_sensor,desired_gain_setting)
        get_distance_data(ping_sensor)

if __name__ == "__main__":
    main()


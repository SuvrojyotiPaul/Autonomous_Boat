import serial
import struct
import time

class Ping360Driver:
    def __init__(self, serial_port='/dev/ttyUSB0', baudrate=115200):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)

    def send_message(self, message_id, payload=b''):
    
        start1 = 0x42  # ASCII 'B'
        start2 = 0x52  # ASCII 'R'
        payload_length = len(payload)
        checksum = (sum(payload) + payload_length + message_id) & 0xFFFF


        checksum_low_byte = checksum & 0xFF
        checksum_high_byte = (checksum >> 8) & 0xFF

        message = struct.pack('<BBHHBB', start1, start2, payload_length, message_id, checksum_low_byte, checksum_high_byte) + payload
        self.ser.write(message)



    def read_message(self):
        
        header = self.ser.read(7)  
        if len(header) < 7:
            return None  

        _, _, payload_length, message_id, _ = struct.unpack('<BBHHB', header)
        payload = self.ser.read(payload_length)
        checksum = self.ser.read(2)

        return message_id, payload

    def request_device_data(self):
        
        self.send_message(message_id=2300)  

    def close(self):
        
        self.ser.close()

def main():
    driver = Ping360Driver()
    try:
        driver.request_device_data()

       
        time.sleep(1)

        response = driver.read_message()
        if response is not None:
            message_id, payload = response
            print(f"Received message ID: {message_id}, Payload: {payload}")
        else:
            print("No message received or incomplete message.")
    finally:
        driver.close()


if __name__ == "__main__":
    main()

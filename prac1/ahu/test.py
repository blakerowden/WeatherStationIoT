import time
import serial
import select
import re

def test_dongle():
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1)  # open serial port
    print(ser.name)         # check which port was really used
    read_data = ser.read(0xFFF)
    while(1):
        ser.write(b'pb r\n')     # write a string
        read,_,_ = select.select([ser], [], [], 5)
        time.sleep(1)
        read_data = ser.read(0xFFF)
        result = re.search(' { (.*?) } ', str(read_data))
        if result is not None:
            print(f"\u007b{result.group(1)}\u007d")
    ser.close()             # close port

def main():
    test_dongle()
    

if __name__ == "__main__":

    main()

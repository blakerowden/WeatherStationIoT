import paho.mqtt.client as mqtt
import time
import argparse
import serial

def test_dongle():
    ser = serial.Serial('/dev/ttyACM0', 115200)  # open serial port
    print(ser.name)         # check which port was really used
    line = ser.readline()  # read a '\n' terminated line
    print(line)
    ser.close()             # close port

def main():
    test_dongle()
    

if __name__ == "__main__":

    main()

from ctypes import sizeof
import time
import serial
import re
import json
import paho.mqtt.client as mqtt
from threading import Thread
from queue import *

SHORTSLEEP = 1
BAUDRATE = 115200


def publish(client, topic, message):
    client.publish(topic, message)


def on_log(client, userdata, level, buf):
    print("log: " + buf)


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("connected OK")
    else:
        print("Bad connection returned code =", rc)


def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected disconnection.")


def on_message(client, userdata, message):
    print(message.payload)


def get_input(ahu):
    while(ahu.is_open):
        write_data = input('>')
        ahu.write(write_data.encode('utf-8') + bytes([13, 10]))
        ahu.flush()
        time.sleep(2)


def run_terminal(ahu, out_q):
    while(ahu.is_open):
        line = ahu.read(0xFFFF).decode('utf-8')[:-2].strip()
        if line:
            print(f'{line}')
            out_q.put(line)

    ahu.close()


def mqtt_publish(ahu, client, in_q):
    while(client.is_connected):
        while not in_q.empty():
            line = in_q.get()
            strip = re.search('{(.*?)}', line)
            if strip is not None:
                parse_JSON = "{ " + strip.group(1) + " }"
                data = json.loads(parse_JSON)
                for i in data:
                    publish(client, topic=i, message=data[i])


def main():

    # Create a client for MQTT
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.on_log = on_log
    port = input('Port Number:')
    client.connect('localhost', port=int(port))
    print(f"Connecting to Broker 'localhost' on port {port}")
    time.sleep(SHORTSLEEP)

    # Create a serial port object called ser for the serial port connection
    ser = serial.Serial('/dev/ttyACM0', BAUDRATE, timeout=1)
    print(f"Connecting to Serial Port {ser.name}...")
    time.sleep(SHORTSLEEP)

    # Create threads to read/write from the serial port and publish to MQTT
    j_data = Queue()
    thread_input = Thread(target=get_input, args=(ser,))
    thread_output = Thread(target=run_terminal, args=(ser, j_data,))
    thread_publish = Thread(target=mqtt_publish, args=(ser, client, j_data,))

    thread_input.start()
    thread_output.start()
    thread_publish.start()

    Thread.join


if __name__ == "__main__":

    main()

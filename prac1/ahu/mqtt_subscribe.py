import paho.mqtt.client as mqtt
import time
import argparse
import serial

def publish(client, topic, message):
    client.publish(topic, message)

def on_log(client, userdata, level, buf):
    print ("log: " + buf)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print ("connected OK")
    else:
        print ("Bad connection returned code =", rc)

def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected disconnection.")

def on_message(client, userdata, message):
    print (message.payload)

def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.connect("localhost", 1885)
    client.subscribe("1")
    client.loop_forever()

if __name__ == "__main__":
    main()

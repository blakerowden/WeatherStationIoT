import time
import serial
import re
import json
import paho.mqtt.client as mqtt

ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1)

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

def test_dongle(client):
    print(ser.readline())
    print(ser.name)         # check which port was really used
    time.sleep(1)
    while(1):
        result = re.search('{ (.*?) }', str(ser.readline()))
        parse_JSON = ""
        if result is not None:
            parse_JSON = "{ " + result.group(1) + " }"
            print (parse_JSON)
            data = json.loads(parse_JSON)
            for i in data :
                publish(client, topic=i, message=data[i])
    ser.close()         

def main():
      # open serial port
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.on_log = on_log
    print ("connecting to broker", 'localhost')
    client.connect('localhost', 1885)
    #client.subscribe(args.topic)
    #client.loop_forever()
    test_dongle(client)
    

if __name__ == "__main__":
    
    main()

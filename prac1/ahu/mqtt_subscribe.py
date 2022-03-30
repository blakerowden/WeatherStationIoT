import paho.mqtt.client as mqtt


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
    if rc:
        print("Unexpected disconnection.")


def on_message(client, userdata, message):
    print(message.payload)


def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    port = input('Port Number:')
    client.connect('localhost', port=int(port))
    client.subscribe("10")
    client.loop_forever()


if __name__ == "__main__":
    main()

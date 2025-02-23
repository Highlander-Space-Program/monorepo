import paho.mqtt.client as mqtt
import time

"""
The publisher is pretty simple.  It connects to the mqtt server and then publishes either an int or a string to the server.
There is a small catch to this in that it publishes the ints to a topic called 'int' and strings to a topic called 'string'.
This is so that in the subscriber we can print different things based on whether the subscriber receives a message with the
int topic or the string topic.  You can sorta think of it like a map/dict, the topic is the key and the payload is the value
"""

def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")


def main():
    print("connecting to host")
    # Creates Client
    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

    # Sets the on_connect callback function
    # This function is automatically called when we connect to a server
    mqttc.on_connect = on_connect

    # Connects to the mqttc server
    # 127.0.0.1 is localhost since we're just hosting the mosquitto server on our machine
    # 1883 is the default port that mqtt uses
    # 60 sets the amount of seconds we'll wait before pinging the server if we haven't sent/received any messages
    mqttc.connect("127.0.0.1", 1883, 60)

    # starts thread to handle network traffic
    start = time.time()
    mqttc.loop_start()
    for i in range(1000000):
        # publishes the number to the input type topic with quality 2
        msg = mqttc.publish('int', 4, 0, True)
    mqttc.loop_stop()
    end = time.time()
    print(f'Total elapsed time: {end - start}')

if __name__ == '__main__':
    main()
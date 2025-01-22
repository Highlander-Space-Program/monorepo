import  paho.mqtt.client as mqtt

"""
The subscriber first connects to the mosquitto server, then subscribes to the int and string topics.
When a message is received, if the topic received was an int it prints out the int or if the topic was
a string it prints out the string.
"""

def on_connect(client, userdata, flags, reason_code, properties) :
    print(f'Connected with result code {reason_code}')
    client.subscribe('int')
    client.subscribe('string')

def on_message(client, userdata, msg):
    # msg is the message we just received

    if (msg.topic == 'int'):
        print(f'Received int: {int(msg.payload)}')
    if (msg.topic == 'string'):
        rcvd_bytearray = msg.payload
        rcvd_string = rcvd_bytearray.decode('utf-8')
        print(f'Received string: {rcvd_string}')

def main():

    # create client
    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

    # callback functions for connecting and receiving a message
    # called when the client connects or receives a message respectively
    mqttc.on_connect = on_connect
    mqttc.on_message = on_message

    # Connects to the mqttc server
    # 127.0.0.1 is localhost since we're just hosting the mosquitto server on our machine
    # 1883 is the default port that mqtt uses
    # 60 sets the amount of seconds we'll wait before pinging the server if we haven't sent/received any messages
    mqttc.connect("127.0.0.1", 1883, 60)

    # since we don't really need to do anything other than call the callback functions occasionally we can just loop forever
    mqttc.loop_forever()


if __name__ == '__main__':
    main()
This is basically just a little tutorial on how to use mqtt.  First thing you'll want to do is download eclipse mosquitto which is a free server we can run to handle our mqtt messages.  Link to download is below:

https://mosquitto.org/download/
^make sure you add the downloaded directory to your PATH if you're using windows so you can use mosquitto anywhere

I found this pretty good tutorial that goes over MQTT with mosquitto, def worth a look:
https://atadiat.com/en/e-mqtt-101-tutorial-introduction-and-eclipse-mosquitto/

Once you're able to see mosquitto running we'll download the paho-mqtt package which we'll use to talk to the mosquitto server in our Python subscriber and publisher.  Make sure you have pip installed then run:

pip install paho-mqtt

Here's the documentation for paho:
https://pypi.org/project/paho-mqtt/#usage-and-api
https://eclipse.dev/paho/files/paho.mqtt.python/html/client.html#paho.mqtt.client.Client.publish


Now that you have all the setup done run mosquitto first then run the publisher and subscriber and take a look to see what they do.  I left some comments in there that should be pretty helpful.

Love,
Alex
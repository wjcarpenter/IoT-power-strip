# IoT-power-strip
See my write-up of the project here: https://www.hackster.io/wjcarpenter/iot-power-strip-fb6c8b

I wired up the outlets of a power strip with current transformers and burden resisters. 
Then I made a separate little box with an ESP8266. 
The idea is to be able to tell when something is drawing power (or not) and be able to do a notification based on that.

I originally implemented this to use the Cayenne cloud service. 
The sketch for that is IoTCayenne.ino, an Arduino sketch that uploads the data to https://cayenne.mydevices.com
The journey to that implementation is what is described at the Hackster.io URL mentioned above.

In the time since that initial implementation, I have been learning a bit more about home automation tools.
In particular, I now run a Home Assistant (https://www.home-assistant.io/) server,
where I do many integrations using a Mosquitto (https://mosquitto.org/) MQTT server.
It therefore seemed natural to convert this project to use generic MQTT.
You can find that in the Arduino sketch PowerToMqtt.ino.
I use the Adafruit MQTT library (https://github.com/adafruit/Adafruit_MQTT_Library).

In the MQTT version, 
I simplified the calculations done locally to just report the measured RMS voltage.
All other calcuations are done on the server side so that the firmware does not need changing if use it in some other way.

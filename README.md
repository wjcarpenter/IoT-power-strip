# IoT-power-strip
See my write-up of the project here: https://www.hackster.io/wjcarpenter/iot-power-strip-fb6c8b

I wired up the four outlets of a power strip with current transformers and burden resisters. Then I made a separate little box with an ESP32. The idea is to be able to tell when something is drawing power (or not) and be able to do a notification based on that.

The most interesting file, at the moment is IoTCayenne.ino, an Arduino sketch that uploads the data to https://cayenne.mydevices.com

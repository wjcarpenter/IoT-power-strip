/*
 * This is part of a current measuring project. See here for more
 * explanation: https://www.hackster.io/wjcarpenter/iot-power-strip-fb6c8b
 *
 * This particular code removes all links to Cayenne and instead
 * sends data to MQTT in topics compatible with Home Assistant
 * autodiscovery. If you don't use HA or care about HA autodiscovery,
 * no matter. That will just be a few messages that go to MQTT for
 * nobody to listen to.
 * 
 * Iterate over all input channels, average a bunch of reads from each.
 * Send the AC voltage results and friends to MQTT, channel for channel.
 * Print results to the console for local monitoring.
 */

#include "EspMQTTClient.h"

// config start -------------------------------------

// WiFi and MQTT network info.
#if 1

#else
#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASSWORD "wifi password"
#define MQTT_SERVER "MQTT server name or address"
#define MQTT_USER "MQTT user/client ID"
#define MQTT_PASSWORD "MQTT password/token"
#define OTA_USER "for OTA updates; #undef to disable"
//#undef OTA_USER
#define OTA_PASSWORD "for OTA updates"

#endif

#define MQTT_PORT 1883
#define DEVICE_ID "laundry_room"

// one-liner calculated values for each measurement chunk
#define PRINT_MEASUREMENT_SUMMARY 1
// print the tick value for each ADC read
#define PRINT_ALL_SAMPLES 0

// Home Assistant can automatically notice new devices and entities if it
// sees a configuration message sent to a conventionally-named MQTT topic.
// Not much harm done if you send these messages even if you are not
// using Home Assistant, but this can be used to suppress them.
#define SEND_HA_CONFIG 1

// delay (in ms) between iterations of the channel scan
const int ITERATION_DELAY = 60 * 1000; // ms

// delay (in ms) after selecting a channel and first reading it
// I don't know if this "settling time" is necessary
const int PRE_MEASUREMENT_DELAY = 100; // ms

// we're going to take a bunch of samples, but we are really fast
// so sleep between samples.
// Sampling theorem: Need to sample at least 2x the frequency.
// Frequency (North America) is 60 Hz, which is 16.67 ms per cycle.
// 16.67 / 120samples == 0.139 ms/sample, 139 us/sample. Rounding
// down to 100 us/sample to allow for the physical world.
// (The same calculation for 50 Hz gives 167 us/sample, so we're 
// OK there if we aim for 60 Hz.)
const int INTRA_MEASUREMENT_DELAY = 100; // microseconds

// number of readings to take for averaging
// I want a bit more than one cycle; 16.67ms for 60 Hz, 20ms for 50 Hz
// If you go for many cycles, you risk the chance of picking up an outlier
// min or max reading
const int DURATION_OF_MEASUREMENT = 21000; // microseconds

const int READS_PER_MEASUREMENT = DURATION_OF_MEASUREMENT / INTRA_MEASUREMENT_DELAY;

EspMQTTClient mqtt(WIFI_SSID, WIFI_PASSWORD, MQTT_SERVER, MQTT_USER, MQTT_PASSWORD, MQTT_USER, MQTT_PORT);

typedef struct 
{
  float slope; 
  float intercept;
} LINE;  // you know: y = mx + b

// these are AC amps (x axis), AC rms mV (y axis)
// in theory, these should all be the same, but they can be slightly different due to physics
static LINE ac_input_1 {24.1, -1.09};
static LINE ac_input_2 {24.1, -1.09};
static LINE ac_input_3 {24.1, -1.09};
static LINE ac_input_4 {24.1, -1.09};
static LINE ac_input_5 {24.1, -1.09};

// these are AC rms mV (x axis), ticks (y axis)
// in theory, these should all be the same, but they can be slightly different due to physics
static LINE ac_mux_pin_0  {1.25, 6.15};
static LINE ac_mux_pin_4  {1.25, 6.18};
static LINE ac_mux_pin_8  {1.27, 3.79};
static LINE ac_mux_pin_12 {1.26, 3.67};

// these are DC mV (x axis), ticks (y axis)
// in theory, these should all be the same, but they acan be slightly different due to physics
static LINE dc_mux_pin_0  {0.946, 415};
static LINE dc_mux_pin_4  {0.944, 419};
static LINE dc_mux_pin_8  {0.945, 418};
static LINE dc_mux_pin_12 {0.944, 423};

typedef enum {ON=2, OFF=-2, MIDDLE=0} ON_OFF_STATE;

// it's hard to avoid using overloaded terminology
// a "channel" is a path all the way from the AC current source to the ADC pin
typedef struct
{
  int mux_pin;         // the input pin/channel on the multiplexer chip
  char *name;          // we're all human

  LINE ac_mv_vs_amps;  // the AC input geometry
  LINE ticks_vs_ac_mv; // the mux pin AC geometry
  LINE ticks_vs_dc_mv; // the mux pin DC geometry
  int ticks_dc_bias;   // not used in the current code

  int ticks_minimum;   // ADC measured during sampling
  int ticks_average;   // ADC measured during sampling
  int ticks_maximum;   // ADC measured during sampling

  ON_OFF_STATE state;         // is we is, or is we ain't?
  long last_state_change;     // seconds
  float threshold_for_off;    // values below this mean the device is off
  float threshold_for_on;     // values above this mean the device is on
  int seconds_for_off;        // how long must it be off before we believe it?
  int seconds_for_on;         // likewise, for believing on?
} CHANNEL;

// for the current project, there are 4 possible inputs, but I am only using 2
static CHANNEL c0  = {
		      0, "washer",
		      ac_input_4, ac_mux_pin_0,  dc_mux_pin_0, 409,
		      0,0,0,
		      MIDDLE,0,
		      1.0,4.0,600,60
};
static CHANNEL c4  = {
		      4, "channel 4",
		      ac_input_4, ac_mux_pin_4,  dc_mux_pin_4, 409,
		      0,0,0,
		      MIDDLE,0,
		      1.0,4.0,0,0
};
static CHANNEL c8  = {
		      8, "dryer",
		      ac_input_5, ac_mux_pin_8,  dc_mux_pin_8, 409,
		      0,0,0,
		      MIDDLE,0,
		      1.0,4.0,120,60
};
static CHANNEL c12 = {
		      12, "channel 12",
		      ac_input_5, ac_mux_pin_12, dc_mux_pin_12, 409,
		      0,0,0,
		      MIDDLE,0,
		      1.0,4.0,0,0
};

static CHANNEL channels[] = {c0, c4, c8, c12};
const int CHANNEL_COUNT = sizeof(channels)/sizeof(channels[0]);

const int adcPin = A0;        // input pin for the ADC signal
const int ledPin = LED_BUILTIN;  // output pin for the LED
const int LED_ON = LOW;
const int LED_OFF = HIGH;
// the mux uses active high for the address lines
const int ADDRESS_LINE_SELECTED = HIGH;
const int ADDRESS_LINE_DESELECTED = LOW;

// these are the 4 pins used for address lines from the ESP32 to the multiplexer
// they correspond to            s0,s1,s2,s3   on the mux
const int MUX_ADDRESS_LINES[] = {0, 4, 13, 12};
const int MUX_ADDRESS_LINE_COUNT = sizeof(MUX_ADDRESS_LINES) / sizeof(MUX_ADDRESS_LINES[0]);

void
mqtt_setup()
{
  mqtt.enableDebuggingMessages(true);
  mqtt.enableMQTTPersistence();
  mqtt.setKeepAlive((2 * ITERATION_DELAY) / 1000);
  mqtt.setMaxPacketSize(500);
  mqtt.setWifiReconnectionAttemptDelay(5 * 1000);
  mqtt.setMqttReconnectionAttemptDelay(5 * 1000);
#ifdef OTA_USER
  mqtt.enableHTTPWebUpdater(OTA_USER, OTA_PASSWORD);
#endif
}

void
setup() 
{
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LED_OFF);

  Serial.begin(115200);
  mqtt_setup();

  for (int ii=0; ii<MUX_ADDRESS_LINE_COUNT; ++ii)
  {
    int thisPin = MUX_ADDRESS_LINES[ii];
    pinMode(thisPin, OUTPUT);
    digitalWrite(thisPin, LOW); // not really required, just for safety
  }

}


static long previous_now = 0;
void
loop() 
{
  mqtt.loop();

  digitalWrite(ledPin, LED_ON); // just for fun, turn LED on while measuring
  for (int channelDex=0; channelDex<CHANNEL_COUNT; ++channelDex)
  {
#if PRINT_MEASUREMENT_SUMMARY
    if (channelDex == 0) 
    {
      Serial.println();
      long now = get_sorta_time_seconds();
      Serial.print(now); Serial.print(" ");Serial.println(now - previous_now);
      previous_now = now;
    }
#endif
    CHANNEL thisChannel = channels[channelDex];
    measure_one_channel(&thisChannel);
    process_channel_results(&thisChannel);
  }
  digitalWrite(ledPin, LED_OFF);
  int delay_slice = (ITERATION_DELAY + 5) / 10;
  for (int ii=0; ii<10; ++ii)
  {
    delay(delay_slice);
    mqtt.loop();
  }
}

void
measure_one_channel(CHANNEL *thisChannel)
{
  select_a_mux_pin(thisChannel->mux_pin);
  delay(PRE_MEASUREMENT_DELAY);
  int total = 0;
  int min = 1025;
  int max = -1;
#if PRINT_ALL_SAMPLES
  Serial.println();
#endif
  for (int ii=0; ii<READS_PER_MEASUREMENT; ++ii)
  {
    int adcValue = analogRead(adcPin);
#if PRINT_ALL_SAMPLES
    Serial.print(adcValue); Serial.print(" ");
#endif
    if (adcValue < min) min = adcValue;
    if (adcValue > max) max = adcValue;
    total += adcValue;
    if (INTRA_MEASUREMENT_DELAY > 0) delayMicroseconds(INTRA_MEASUREMENT_DELAY);
  }
#if PRINT_ALL_SAMPLES
  Serial.println();
#endif
  thisChannel->ticks_average = ( total + (READS_PER_MEASUREMENT / 2) ) / READS_PER_MEASUREMENT;
  thisChannel->ticks_minimum = min;
  thisChannel->ticks_maximum = max;
}

void
select_a_mux_pin(int mux_pin)
{
  for (int ii=0; ii<MUX_ADDRESS_LINE_COUNT; ++ii)
  {
    if (((mux_pin>>ii) & 0x1) == 0x1)
    {
      digitalWrite(MUX_ADDRESS_LINES[ii], ADDRESS_LINE_SELECTED);
    }
    else
    {
      digitalWrite(MUX_ADDRESS_LINES[ii], ADDRESS_LINE_DESELECTED);
    }
  }
}

static const char* stateTopicFormat = "powerwatcher/" DEVICE_ID "/ch%d/state";

void
process_channel_results(CHANNEL *thisChannel)
{
  float dcVoltage = ticks_to_DC_millivolts(thisChannel);
  float acVoltage = ticks_to_AC_millivolts(thisChannel);
  float acCurrent = AC_millivolts_to_AC_amps(thisChannel, acVoltage);

  ON_OFF_STATE new_state = MIDDLE; // this shouldn't happen except at startup or an unlucky edge measurement
  if (acCurrent <= thisChannel->threshold_for_off)
  {
    new_state = OFF;
  }
  else if (acCurrent >= thisChannel->threshold_for_on)
  {
    new_state = ON;
  }
  ON_OFF_STATE channelState = thisChannel->state;
  if (new_state != thisChannel->state)
  {
    long now = get_sorta_time_seconds();
    long delta = now -thisChannel->last_state_change;
    // The millis() function wraps around every 50 days or so, at which
    // time the delta will be a large negative number. Adjust for that.
    // If we miss a state change, we'll get it on a future next iteration.
    // So, every couple of months, there is a potential for a state change
    // being delayed ... as much as double the configured delay.
    if (delta < 0)
    {
      delta = 0;
      thisChannel->last_state_change = now;
    }
    if ((new_state == ON  &&  delta >= thisChannel->seconds_for_on)
    ||  (new_state == OFF &&  delta >= thisChannel->seconds_for_off))
    {
      thisChannel->state = new_state;
      thisChannel->last_state_change = now;
    }
  }

  char topic[100];
  char payload[100];

  snprintf(topic, sizeof(topic), stateTopicFormat, thisChannel->mux_pin);
  snprintf(payload, sizeof(payload), "{\"mvrms\":\"%.2f\", \"amps\":\"%.2f\", \"onoff\":\"%s\", \"deltaadc\":\"%d\"}",
	   acVoltage, acCurrent, new_state == ON ? "on" : "off", (thisChannel->ticks_maximum - thisChannel->ticks_minimum));
  mqtt.publish(topic, payload);

#if PRINT_MEASUREMENT_SUMMARY
  // if this were time-critical or if I needed the TX/RX pins
  // for something, I would not do the console output
  // handy during development, though
  Serial.print("ch ");
  Serial.print(thisChannel->mux_pin);
  Serial.print(" ");
  Serial.print(thisChannel->name);
  Serial.print(": min=");
  Serial.print(thisChannel->ticks_minimum);
  Serial.print(", ave=");
  Serial.print(thisChannel->ticks_average);
  Serial.print(", max=");
  Serial.print(thisChannel->ticks_maximum);
  Serial.print(", measure DC=");
  Serial.print(dcVoltage);
  Serial.print(" mV, AC=");
  Serial.print(acVoltage);
  Serial.print(" mV, ");
  Serial.print(acCurrent);
  Serial.print(" amps,  ");
  Serial.print(thisChannel->state);
  Serial.println();
#endif  
}

// ticks = (slope * mv) + intercept
// (ticks - intercept) / slope = mv
float
ticks_to_DC_millivolts(CHANNEL *thisChannel)
{
  int ticks       = thisChannel->ticks_average;
  float slope     = thisChannel->ticks_vs_dc_mv.slope;
  float intercept = thisChannel->ticks_vs_dc_mv.intercept;
  float mV = (ticks - intercept) / slope;
  if (mV < 0) mV = 0;
  return mV;
}

// ticks = (slope * mv) + intercept
// (ticks - intercept) / slope = mv
float
ticks_to_AC_millivolts(CHANNEL *thisChannel)
{
  // A more accurate way to do this would be to record all of the sample
  // points and then best-fit a sinusoidal wave regression to those points.
  // Then use the amplitude of that sine wave as the peak. 
  //
  // Simpler to just take the observed maximum as the amplitude (subtracting
  // out the observed average, which ought to be "zero" [the DC bias built
  // into the circuit]). The only problem is if the signal is noisy. Then the
  // observed maximum could be an outlier, and the amplitude would be too high.

  int ticks       = thisChannel->ticks_maximum - thisChannel->ticks_average;
  float slope     = thisChannel->ticks_vs_ac_mv.slope;
  float intercept = thisChannel->ticks_vs_ac_mv.intercept;
  float mV = (ticks - intercept) / slope;
  // convert to rms (Vpeak / sqrt(2))
  mV = mV * 0.7071;
  if (mV < 0) mV = 0;
  return mV;
}

// mv = (slope * amps) + intercept
// (mv - intercept) / slope = amps
float
AC_millivolts_to_AC_amps(CHANNEL *thisChannel, float acVoltage)
{
  float slope     = thisChannel->ac_mv_vs_amps.slope;
  float intercept = thisChannel->ac_mv_vs_amps.intercept;
  float acCurrent = (acVoltage - intercept) / slope;
  if (acCurrent < 0.1) acCurrent = 0; // very low currents are inaccurate
  return acCurrent;
}

// only approximately correct since millis() is only approximately correct
long
get_sorta_time_seconds()
{
  long sorta_millis = millis();
  return (sorta_millis + 500) / 1000;
}

void
ha_sensor_configs()
{
  for (int channelDex=0; channelDex<CHANNEL_COUNT; ++channelDex)
  {
    CHANNEL thisChannel = channels[channelDex];
    ha_sensor_config_one_channel(&thisChannel);
    process_channel_results(&thisChannel);
  }
}

static const char *sensorPayloadFormat =
  "{"
  "  \"unit_of_measurement\":\"%s\","
  "  \"name\":\"ch%d %s\","
  "  \"state_topic\":\"%s\","
  "  \"unique_id\":\"powerwatcher_" DEVICE_ID "_ch%d_%s\","
  "  \"value_template\": \"{{value_json.%s}}\","
  "  \"device\":"
  "  {"
  "    \"name\":\"" DEVICE_ID "_powerwatcher\","
  "    \"identifiers\":\"" DEVICE_ID "_powerwatcher\""
  "  }"
  "}";

static const char *binarySensorPayloadFormat =
  "{"
  "  \"name\":\"ch%d %s\","
  "  \"state_topic\":\"%s\","
  "  \"unique_id\":\"powerwatcher_" DEVICE_ID "_ch%d_%s\","
  "  \"value_template\": \"{{value_json.%s}}\","
  "  \"payload_on\": \"on\", \"payload_off\": \"off\","
  "  \"device\":"
  "  {"
  "    \"name\":\"" DEVICE_ID "_powerwatcher\","
  "    \"identifiers\":\"" DEVICE_ID "_powerwatcher\""
  "  }"
  "}";
     

void
ha_sensor_config_one_channel(CHANNEL *thisChannel)
{
  //mqtt.loop();

  int chnum = thisChannel->mux_pin;
  char ha_topic[100];
  char state_topic[100];
  char payload[500];
  snprintf(state_topic, sizeof(state_topic), stateTopicFormat, thisChannel->mux_pin);
  
  snprintf(ha_topic, sizeof(ha_topic), "homeassistant/sensor/" DEVICE_ID "/ch%d_Delta_ADC/config", chnum);
  snprintf(payload, sizeof(payload), sensorPayloadFormat,
	   "ticks", /* UoM */
	   chnum, "Delta ADC", /* name */
	   state_topic,
	   chnum, "deltaadc", /* uniqueId */
	   "deltaadc"); /* value template */
  Serial.println(payload);
  mqtt.publish(ha_topic, payload, true);

  snprintf(ha_topic, sizeof(ha_topic), "homeassistant/sensor/" DEVICE_ID "/ch%d_ac_mV_rms/config", chnum);
  snprintf(payload, sizeof(payload), sensorPayloadFormat,
	   "mVrms", /* UoM */
	   chnum, "AC mVrms", /* name */
	   state_topic,
	   chnum, "acmvrms", /* uniqueId */
	   "mvrms"); /* value template */
  Serial.println(payload);
  mqtt.publish(ha_topic, payload, true);

  snprintf(ha_topic, sizeof(ha_topic), "homeassistant/sensor/" DEVICE_ID "/ch%d_ac_amps/config", chnum);
  snprintf(payload, sizeof(payload), sensorPayloadFormat,
	   "amps", /* UoM */
	   chnum, "AC amps", /* name */
	   state_topic,
	   chnum, "acamps", /* uniqueId */
	   "amps"); /* value template */
  Serial.println(payload);
  mqtt.publish(ha_topic, payload, true);

  snprintf(ha_topic, sizeof(ha_topic), "homeassistant/binary_sensor/" DEVICE_ID "/ch%d_onoff/config", chnum);
  snprintf(payload, sizeof(payload), binarySensorPayloadFormat,
	   chnum, "On/Off", /* name */
	   state_topic,
	   chnum, "onoff", /* uniqueId */
	   "onoff"); /* value template */
  Serial.println(payload);
  mqtt.publish(ha_topic, payload, true);
}

void
onConnectionEstablished()
{
  Serial.println("MQTT connection established");
  ha_sensor_configs();
}

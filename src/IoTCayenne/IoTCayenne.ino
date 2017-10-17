/*
 * Iterate over all input channels, average a bunch of reads from each.
 * Send the AC voltage results to Cayenne dashboard, channel for channel.
 * Print results to the console for local monitoring.
 */

// Cayenne config start -------------------------------------
//#define CAYENNE_DEBUG
#define CAYENNE_PRINT Serial
#include <CayenneMQTTESP8266.h>

// WiFi network info.
#if 0
#else
char ssid[] = "SSID";
char wifiPassword[] = "wifi password";
#endif

// Cayenne authentication info. This should be obtained from the Cayenne Dashboard.
#if 0
#else
char username[] = "cayenne-device-username";
char password[] = "cayenne-device-password";
char clientID[] = "cayenne-device-clientID";
#endif
// Cayenne config end   -------------------------------------

// the ADC reading with 0v input; in other words, the DC bias voltage (determined experimentally)
const int DC_BIAS_TICKS = 409; // ticks

// the input voltage (in mV) where the ADC tops out at 1024 (determined experimentally)
const int VOLTAGE_CEILING = 670; // millivolts

// number of readings to take for averaging
const int READS_PER_MEASUREMENT = 100;

// delay (in ms) after selecting a channel and first reading it
// I don't know if this "settling time" is necessary
const int PRE_MEASUREMENT_DELAY = 2; // ms

// we're going to take a bunch of samples, but we are really fast
// so sleep between samples.
const int INTRA_MEASUREMENT_DELAY = 1; // ms

// delay (in ms) between iterations of the channel scan
const int ITERATION_DELAY = 5000; // ms

// ---- end of tunables ----

const int signalPin = A0;        // input pin for the signal
const int ledPin = LED_BUILTIN;  // output pin for the LED
const int LED_ON = LOW;
const int LED_OFF = HIGH;

const int CHANNEL_COUNT = 4; // zero-based
int channelNumbers[] = {0, 4, 8, 12};
int channelAverage[CHANNEL_COUNT];
int channelMaximum[CHANNEL_COUNT];
int channelMinimum[CHANNEL_COUNT];

// these are the 4 address lines from the ESP32 to the multiplexer
const int ADDRESS_LINE_COUNT = 4;
const int S[ADDRESS_LINE_COUNT] = {0, 4, 13, 12};

void setup() 
{
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LED_OFF);

  Serial.begin(115200);

  for (int ii=0; ii<ADDRESS_LINE_COUNT; ++ii)
  {
    int thisPin = S[ii];
    pinMode(thisPin, OUTPUT);
    digitalWrite(thisPin, LOW); // not really required, just for safety
  }
  Cayenne.begin(username, password, clientID, ssid, wifiPassword);
}

void loop() 
{
  Cayenne.loop();
  digitalWrite(ledPin, LED_ON); // just for fun, turn LED on while measuring
  for (int ii=0; ii<CHANNEL_COUNT; ++ii)
  {
    int thisChannel = channelNumbers[ii];
    measure_one_channel(ii);
    process_channel_results(ii);
  }
  Serial.println();
  digitalWrite(ledPin, LED_OFF);
  delay(ITERATION_DELAY);
}

void measure_one_channel(int channelPointer)
{
  int channel = channelNumbers[channelPointer];
  select_a_channel(channel);
  delay(PRE_MEASUREMENT_DELAY);
  int total = 0;
  int min = 1025;
  int max = -1;
  for (int ii=0; ii<READS_PER_MEASUREMENT; ++ii)
  {
    int signalValue = analogRead(signalPin);
    //if (channel == 0){Serial.print(signalValue); Serial.print(" ");if (ii%10 == 0) Serial.println();}
    if (signalValue < min) min = signalValue;
    if (signalValue > max) max = signalValue;
    total += signalValue;
    if (INTRA_MEASUREMENT_DELAY > 0) delay(INTRA_MEASUREMENT_DELAY);
  }
  if (channel == 0) Serial.println();
  channelAverage[channelPointer] = ( total + (READS_PER_MEASUREMENT / 2) ) / READS_PER_MEASUREMENT;
  channelMinimum[channelPointer] = min;
  channelMaximum[channelPointer] = max;
}

void select_a_channel(int channel)
{
  for (int ii=0; ii<ADDRESS_LINE_COUNT; ++ii)
  {
    if (((channel>>ii) & 0x1) == 0x1)
    {
      digitalWrite(S[ii], HIGH);
    }
    else
    {
      digitalWrite(S[ii], LOW);
    }
  }
}

void process_channel_results(int channelPointer)
{
  int channel = channelNumbers[channelPointer];
  float dcVoltage = ticks_to_DC_millivolts(channelAverage[channelPointer]);
  float acVoltage = ticks_to_AC_millivolts(channelMinimum[channelPointer], channelMaximum[channelPointer]);

  Cayenne.virtualWrite(channel, acVoltage);    

  Serial.print("ch ");
  Serial.print(channel);
  Serial.print(": min=");
  Serial.print(channelMinimum[channelPointer]);
  Serial.print(", ave=");
  Serial.print(channelAverage[channelPointer]);
  Serial.print(", max=");
  Serial.print(channelMaximum[channelPointer]);
  Serial.print(", measure DC=");
  Serial.print(dcVoltage);
  Serial.print(" mV, AC=");
  Serial.print(acVoltage);
  Serial.print(" mV");
  Serial.println();
}

float ticks_to_DC_millivolts(int ticks)
{
  float voltage = 0.0;
  float fraction = 0.0;
  int unbiased = ticks - DC_BIAS_TICKS;
  if (unbiased > 0)
  {
    fraction = unbiased / 1024.0;
    voltage = fraction * VOLTAGE_CEILING;
  }
  return voltage;
}

float ticks_to_AC_millivolts(int ticksMin, int ticksMax)
{
  float voltage = 0.0;
  float fraction = 0.0;
  int ticks = ticksMax - ticksMin;
  if (ticks > 0)
  {
    fraction = ticks / 1024.0;
    voltage = fraction * VOLTAGE_CEILING; // this is a DC number; not sure if it applies
  }
  return voltage;
}

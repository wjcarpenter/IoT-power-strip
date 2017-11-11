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
char ssid[] = "SSID";
char wifiPassword[] = "wifi password";

// Cayenne authentication info. This should be obtained from the Cayenne Dashboard.
char username[] = "cayenne-device-username";
char password[] = "cayenne-device-password";
char clientID[] = "cayenne-device-clientID";
// Cayenne config end   -------------------------------------

#define CONSOLE_OUTPUT

// number of readings to take for averaging
const int READS_PER_MEASUREMENT = 500;

// delay (in ms) after selecting a channel and first reading it
// I don't know if this "settling time" is necessary
const int PRE_MEASUREMENT_DELAY = 20; // ms

// we're going to take a bunch of samples, but we are really fast
// so sleep between samples.
const int INTRA_MEASUREMENT_DELAY = 1; // ms

// delay (in ms) between iterations of the channel scan
const int ITERATION_DELAY = 60 * 1000; // ms

// ---- end of tunables ----

typedef struct 
{
  float slope; 
  float intercept;
} LINE;  // you know, y= mx + b

// these are AC amps (x axis), ACrms mV (y axis)
// in theory, these should all be the same, but they are slightly different due to physics
static LINE ac_input_nw {37.7, 11.5};
static LINE ac_input_sw {37.7, 11.9};

// these are ACrms mV (x axis), ticks (y axis)
// in theory, these should all be the same, but they are slightly different due to physics
static LINE ac_mux_pin_0  {1.25, 6.15};
static LINE ac_mux_pin_4  {1.25, 6.18};
static LINE ac_mux_pin_8  {1.27, 3.79};
static LINE ac_mux_pin_12 {1.26, 3.67};

// these are DC mV (x axis), ticks (y axis)
// in theory, these should all be the same, but they are slightly different due to physics
static LINE dc_mux_pin_0 {0.946, 415};
static LINE dc_mux_pin_4 {0.944, 419};
static LINE dc_mux_pin_8 {0.945, 418};
static LINE dc_mux_pin_12 {0.944, 423};

// it's hard to avoid using overloaded terminology
// a "channel" is a path all the way from the AC current source to the ADC pin
typedef struct
{
  int mux_pin;         // the input pin/channel on the multiplexer chip
  char *name;          // we're all human
  LINE ac_mv_vs_amps;  // the AC input
  LINE ticks_vs_ac_mv; // the mux pin AC line
  LINE ticks_vs_dc_mv; // the mux pin DC line
  int ticks_dc_bias;   // not used in the current code
  int ticks_minimum;   // ADC measured during sampling
  int ticks_average;   // ADC measured during sampling
  int ticks_maximum;   // ADC measured during sampling
} CHANNEL;

// for the current project, there are 4 possible inputs, but I am only using 2
static CHANNEL c0  = { 0, "000000", ac_input_nw, ac_mux_pin_0,  dc_mux_pin_0,  409, 0,0,0};
// static CHANNEL c4  = { 4, "444444", ac_input_nw, ac_mux_pin_4,  dc_mux_pin_4,  409, 0,0,0};
static CHANNEL c8  = { 8, "888888", ac_input_sw, ac_mux_pin_8,  dc_mux_pin_8,  409, 0,0,0};
// static CHANNEL c12 = {12, "121212", ac_input_sw, ac_mux_pin_12, dc_mux_pin_12, 409, 0,0,0};

static CHANNEL channels[] = {c0, c8};
const int CHANNEL_COUNT = sizeof(channels)/sizeof(channels[0]);

const int adcPin = A0;        // input pin for the ADC signal
const int ledPin = LED_BUILTIN;  // output pin for the LED
const int LED_ON = LOW;
const int LED_OFF = HIGH;
// the mux uses active high for the address lines
const int ADDRESS_LINE_SELECTED = HIGH;
const int ADDRESS_LINE_DESELECTED = LOW;

// these are the 4 pins used for address lines from the ESP32 to the multiplexer
// they correspond to s0,s1,s2,s3 on the mux
const int MUX_ADDRESS_LINES[] = {0, 4, 13, 12};
const int MUX_ADDRESS_LINE_COUNT = sizeof(MUX_ADDRESS_LINES) / sizeof(MUX_ADDRESS_LINES[0]);

void setup() 
{
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LED_OFF);

  Serial.begin(115200);

  for (int ii=0; ii<MUX_ADDRESS_LINE_COUNT; ++ii)
  {
    int thisPin = MUX_ADDRESS_LINES[ii];
    pinMode(thisPin, OUTPUT);
    digitalWrite(thisPin, LOW); // not really required, just for safety
  }

  Cayenne.begin(username, password, clientID, ssid, wifiPassword);
}

void loop() 
{
  Cayenne.loop();
  digitalWrite(ledPin, LED_ON); // just for fun, turn LED on while measuring
  for (int channelDex=0; channelDex<CHANNEL_COUNT; ++channelDex)
  {
#ifdef CONSOLE_OUTPUT    
    if (channelDex == 0) Serial.println();
#endif
    CHANNEL thisChannel = channels[channelDex];
    measure_one_channel(&thisChannel);
    process_channel_results(&thisChannel);
  }
  digitalWrite(ledPin, LED_OFF);
  delay(ITERATION_DELAY);
}

void measure_one_channel(CHANNEL *thisChannel)
{
  select_a_mux_pin(thisChannel->mux_pin);
  delay(PRE_MEASUREMENT_DELAY);
  int total = 0;
  int min = 1025;
  int max = -1;
  for (int ii=0; ii<READS_PER_MEASUREMENT; ++ii)
  {
    int adcValue = analogRead(adcPin);
    if (adcValue < min) min = adcValue;
    if (adcValue > max) max = adcValue;
    total += adcValue;
    if (INTRA_MEASUREMENT_DELAY > 0) delay(INTRA_MEASUREMENT_DELAY);
  }
  thisChannel->ticks_average = ( total + (READS_PER_MEASUREMENT / 2) ) / READS_PER_MEASUREMENT;
  thisChannel->ticks_minimum = min;
  thisChannel->ticks_maximum = max;
}

void select_a_mux_pin(int mux_pin)
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

void process_channel_results(CHANNEL *thisChannel)
{
  float dcVoltage = ticks_to_DC_millivolts(thisChannel);
  float acVoltage = ticks_to_AC_millivolts(thisChannel);
  float acCurrent = AC_millivolts_to_AC_amps(thisChannel, acVoltage);

  Cayenne.virtualWrite(thisChannel->mux_pin, acCurrent);    

#ifdef CONSOLE_OUTPUT
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
  Serial.print(" amps");
  Serial.println();
#endif  
}

// ticks = (slope * mv) + intercept
// (ticks - intercept) / slope = mv
float ticks_to_DC_millivolts(CHANNEL *thisChannel)
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
float ticks_to_AC_millivolts(CHANNEL *thisChannel)
{
  int ticks       = thisChannel->ticks_maximum - thisChannel->ticks_average;
  float slope     = thisChannel->ticks_vs_ac_mv.slope;
  float intercept = thisChannel->ticks_vs_ac_mv.intercept;
  float mV = (ticks - intercept) / slope;
  if (mV < 0) mV = 0;
  return mV;
}

// mv = (slope * amps) + intercept
// (mv - intercept) / slope = amps
float AC_millivolts_to_AC_amps(CHANNEL *thisChannel, float acVoltage)
{
  float slope     = thisChannel->ac_mv_vs_amps.slope;
  float intercept = thisChannel->ac_mv_vs_amps.intercept;
  float acCurrent = (acVoltage - intercept) / slope;
  if (acCurrent < 0) acCurrent = 0;
  return acCurrent;
}

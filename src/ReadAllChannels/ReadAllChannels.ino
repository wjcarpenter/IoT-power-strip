/*
 * Iterate over all input channels, average a bunch of reads from each, print results to the console.
 */

// the ADC reading with 0v input; in other words, the DC bias voltage (determined experimentally)
const int DC_BIAS_TICKS = 409; // ticks
// the input voltage (in mV) where the ADC tops out at 1024 (determined experimentally)
const int VOLTAGE_CEILING = 670; // millivolts
// number of readings to take for averaging
const int READS_PER_MEASUREMENT = 100;
// delay (in ms) after selecting a channel and first reading it
// I don't know if this "settling time" is necessary
const int PRE_MEASUREMENT_DELAY = 2; // ms
// delay (in ms) between iterations of the channel scan
const int ITERATION_DELAY = 5000; // ms

// ---- end of tunable ----

const int signalPin = A0;        // input pin for the signal
const int ledPin = LED_BUILTIN;  // output pin for the LED
const int LED_ON = LOW;
const int LED_OFF = HIGH;

const int CHANNEL_COUNT = 16; // zero-based
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
}

void loop() 
{
  digitalWrite(ledPin, LED_ON); // just for fun, turn LED on while measuring
  for (int ii=0; ii<CHANNEL_COUNT; ++ii)
  {
    measure_one_channel(ii);
  }
  digitalWrite(ledPin, LED_OFF);
  process_channel_results();
  delay(ITERATION_DELAY);
}

void measure_one_channel(int channel)
{
  select_a_channel(channel);
  delay(PRE_MEASUREMENT_DELAY);
  int total = 0;
  int min = 1025;
  int max = -1;
  for (int ii=0; ii<READS_PER_MEASUREMENT; ++ii)
  {
    int signalValue = analogRead(signalPin);
    if (signalValue < min) min = signalValue;
    if (signalValue > max) max = signalValue;
    total += signalValue;
  }
  channelAverage[channel] = ( total + (READS_PER_MEASUREMENT / 2) ) / READS_PER_MEASUREMENT;
  channelMinimum[channel] = min;
  channelMaximum[channel] = max;
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

void process_channel_results()
{
  for (int ii=0; ii<CHANNEL_COUNT; ++ii)
  {
    float dcVoltage = ticks_to_millivolts(channelAverage[ii]);
    float acVoltage = ticks_to_millivolts(channelMaximum[ii]);
    Serial.print("ch ");
    Serial.print(ii);
    Serial.print(": min=");
    Serial.print(channelMinimum[ii]);
    Serial.print(", ave=");
    Serial.print(channelAverage[ii]);
    Serial.print(", max=");
    Serial.print(channelMaximum[ii]);
    Serial.print(", measure DC=");
    Serial.print(dcVoltage);
    Serial.print(" mV, AC=");
    Serial.print(acVoltage);
    Serial.print(" mV");
    Serial.println();
  }
  Serial.println();
}

float ticks_to_millivolts(int ticks)
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


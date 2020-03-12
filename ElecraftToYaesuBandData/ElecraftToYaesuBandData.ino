/*
 * File: main.ino
 * Description:
 *   Main application code.
 * Author: Mooneer Salem <mooneer@gmail.com>
 * License: New BSD License
 */
 
/* Begin user configurable settings. */
#define RADIO_BAUD 38400
#define COMPUTER_BAUD 38400

/* Change for your country. Defaults to US band plan. */
#define BAND_MIN_KHZ_160M 1800
#define BAND_MAX_KHZ_160M 2000

#define BAND_MIN_KHZ_80M 3500
#define BAND_MAX_KHZ_80M 4000

/* FT-817/818 does not define a specific 60m voltage, so you may want
 * to update the frequency ranges below to output e.g. 40M's instead.
 */
//#define BAND_MIN_KHZ_60M 5330
//#define BAND_MAX_KHZ_60M 5407

#define BAND_MIN_KHZ_40M 7000
#define BAND_MAX_KHZ_40M 7300

#define BAND_MIN_KHZ_30M 10000
#define BAND_MAX_KHZ_30M 10150

#define BAND_MIN_KHZ_20M 14000
#define BAND_MAX_KHZ_20M 14350

#define BAND_MIN_KHZ_17M 18068
#define BAND_MAX_KHZ_17M 18168

#define BAND_MIN_KHZ_15M 21000
#define BAND_MAX_KHZ_15M 21450

#define BAND_MIN_KHZ_12M 24930
#define BAND_MAX_KHZ_12M 24990

#define BAND_MIN_KHZ_10M 28000
#define BAND_MAX_KHZ_10M 29700

#define BAND_MIN_KHZ_6M 50000
#define BAND_MAX_KHZ_6M 54000

/* End user configurable settings. */
#define HZ_TO_KHZ 1000
#define COMMAND_TERMINATOR ';'
#define TRANSCEIVER_INFORMATION_RSP "IF"
#define TRANSCIVER_FREQ_INDEX_BEGIN 2
#define TRANSCIVER_FREQ_INDEX_END (TRANSCIVER_FREQ_INDEX_BEGIN + 11)
#define AUTOINFO_CMD "AI1;" /* Tells KX3 etc. to automatically output frequency changes */
#define SERIAL_TIMEOUT_MS 1

#define ANALOG_PIN A0 /* MKR Zero pure analog pin */
#define VOLTAGE_STANDARD 330 /* 3.3V; 5V would be 500 */
#define MAX_INT_FOR_VOLTAGE_STANDARD 4095

#define VOLTAGE_160M  33
#define VOLTAGE_80M   66
//#define VOLTAGE_60M  
#define VOLTAGE_40M  100
#define VOLTAGE_30M  130
#define VOLTAGE_20M  160
#define VOLTAGE_17M  200
#define VOLTAGE_15M  230
#define VOLTAGE_12M  270
#define VOLTAGE_10M  300
#define VOLTAGE_6M   330

// Converts a voltage to a value suitable for analogWrite().
int voltageToInteger(int voltage)
{
  return voltage * MAX_INT_FOR_VOLTAGE_STANDARD / VOLTAGE_STANDARD;
}

int frequencyToVoltage(int frequencyKhz)
{
  if (frequencyKhz >= BAND_MIN_KHZ_160M && frequencyKhz <= BAND_MAX_KHZ_160M)
  {
    return VOLTAGE_160M;
  }
  else if (frequencyKhz >= BAND_MIN_KHZ_80M && frequencyKhz <= BAND_MAX_KHZ_80M)
  {
    return VOLTAGE_80M;
  }
  /*else if (frequencyKhz >= BAND_MIN_KHZ_60M && frequencyKhz <= BAND_MAX_KHZ_60M)
  {
    return VOLTAGE_60M;
  }*/
  else if (frequencyKhz >= BAND_MIN_KHZ_40M && frequencyKhz <= BAND_MAX_KHZ_40M)
  {
    return VOLTAGE_40M;
  }
  else if (frequencyKhz >= BAND_MIN_KHZ_30M && frequencyKhz <= BAND_MAX_KHZ_30M)
  {
    return VOLTAGE_30M;
  }
  else if (frequencyKhz >= BAND_MIN_KHZ_20M && frequencyKhz <= BAND_MAX_KHZ_20M)
  {
    return VOLTAGE_20M;
  }
  else if (frequencyKhz >= BAND_MIN_KHZ_17M && frequencyKhz <= BAND_MAX_KHZ_17M)
  {
    return VOLTAGE_17M;
  }
  else if (frequencyKhz >= BAND_MIN_KHZ_15M && frequencyKhz <= BAND_MAX_KHZ_15M)
  {
    return VOLTAGE_15M;
  }
  else if (frequencyKhz >= BAND_MIN_KHZ_12M && frequencyKhz <= BAND_MAX_KHZ_12M)
  {
    return VOLTAGE_12M;
  }
  else if (frequencyKhz >= BAND_MIN_KHZ_10M && frequencyKhz <= BAND_MAX_KHZ_10M)
  {
    return VOLTAGE_10M;
  }
  else if (frequencyKhz >= BAND_MIN_KHZ_6M && frequencyKhz <= BAND_MAX_KHZ_6M)
  {
    return VOLTAGE_6M;
  }
  else
  {
    return 0; // disable band output
  }
}

void setBand(int frequencyKhz)
{
  analogWriteResolution(12);
  analogWrite(ANALOG_PIN, voltageToInteger(frequencyToVoltage(frequencyKhz)));
}

void parseRadioOutput(String radioBuffer)
{
  if (radioBuffer.startsWith(TRANSCEIVER_INFORMATION_RSP))
  {
    // Grab the 11 bytes after the preamble, which is the frequency in Hz.
    int frequency = radioBuffer.substring(TRANSCIVER_FREQ_INDEX_BEGIN, TRANSCIVER_FREQ_INDEX_END).toInt();
    setBand(frequency / HZ_TO_KHZ);
  }
  else
  {
    // not currently processed
  }
}

void setup() {
  // Set analog output pin to 0V.
  analogWriteResolution(12);
  analogWrite(ANALOG_PIN, 0);

  // Open serial ports
  SerialUSB.begin(COMPUTER_BAUD);
  SerialUSB.setTimeout(SERIAL_TIMEOUT_MS);
  Serial1.begin(RADIO_BAUD);
  Serial1.setTimeout(SERIAL_TIMEOUT_MS);

  // Begin auto-info
  Serial1.print(AUTOINFO_CMD);
}

static String computerBuffer;
static String radioBuffer;
  
void loop() {
  // Read data from both the radio and the computer  
  while (SerialUSB.available() > 0)
  {
    int character = SerialUSB.read();
    if (character == COMMAND_TERMINATOR)
    {
      Serial1.print(computerBuffer);
      Serial1.write(COMMAND_TERMINATOR);
      computerBuffer = "";
    }
    else
    {
      computerBuffer += (char)character;
    }
  }

  while (Serial1.available() > 0)
  {
    int character = Serial1.read();
    if (character == COMMAND_TERMINATOR)
    {
      SerialUSB.print(radioBuffer);
      SerialUSB.write(COMMAND_TERMINATOR);

      // Parse data from the radio to determine if we got a frequency change
      parseRadioOutput(radioBuffer);
      radioBuffer = "";
    }
    else
    {
      radioBuffer += (char)character;
    }    
  }
}

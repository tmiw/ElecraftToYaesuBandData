
/*
 * File: main.ino
 * Description:
 *   Main application code.
 * Author: Mooneer Salem <mooneer@gmail.com>
 * License: New BSD License
 */

#include <assert.h>
#include <FreeRTOS.h>
#include <timers.h>
#include <task.h>
#include <deprecated_definitions.h>
#include <semphr.h>
#include <mpu_wrappers.h>
#include <croutine.h>
#include <portmacro.h>
#include <event_groups.h>
#include <list.h>
#include <portable.h>
#include <FreeRTOSConfig.h>
#include <error_hooks.h>
#include <StackMacros.h>
#include <FreeRTOS_SAMD21.h>
#include <projdefs.h>
#include <queue.h>
 
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
#define RADIO_INFO_CMD "IF" /* Tells KX3 etc. to send frequency/mode */
#define SERIAL_TIMEOUT_MS 1
#define POLLING_PERIOD_MS 500

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

static TaskHandle_t radioSerialTask;
static TaskHandle_t pcSerialTask;
static QueueHandle_t radioInputQueue;
static QueueHandle_t pcOutputQueue;
static TimerHandle_t pollingTimer;

struct RadioIOParameters
{
  char command[64];
  bool receiveOutput;
};

static String computerBuffer;
static String radioBuffer;

void receiveFromRadio(bool sendResponse)
{
  bool doneReceiving = false;
  RadioIOParameters commandToSend;
  
  do
  {
    while (Serial1.available() > 0)
    {
      int character = Serial1.read();
      if (character == COMMAND_TERMINATOR)
      {
        // Parse data from the radio to determine if we got a frequency change
        parseRadioOutput(radioBuffer);

        // If we should be sending a response back to the PC, do it now.
        if (sendResponse)
        {
          strcpy(commandToSend.command, radioBuffer.c_str());
          xQueueSend(pcOutputQueue, &commandToSend, 0);
        }

        radioBuffer = "";
        doneReceiving = true;
      }
      else
      {
        radioBuffer += (char)character;
      }
    }
  } while (!doneReceiving);
}

void radioSerial(void *pvParameters)
{
  bool receiveOutput = true;
  
  while(true)
  {
    RadioIOParameters commandToSend;
    receiveOutput = true;
    
    if (xQueueReceive(radioInputQueue, &commandToSend, 0) == pdTRUE)
    {
      // Send to radio and await response
      Serial1.print(commandToSend.command);
      Serial1.print(COMMAND_TERMINATOR);
      receiveOutput = commandToSend.receiveOutput;

      // Pause between 10-100ms depending on when radio sends a response.
      for (int i = 0; i < 10; i++)
      {
        vTaskDelay( (10 * 1000) / portTICK_PERIOD_US );
        if (Serial1.available() > 0) break;
      }
    }

    if (Serial1.available() > 0)
    {
      // Output from the radio, handle accordingly.
      receiveFromRadio(receiveOutput);
    }
  }
}

void pcSerial(void *pvParameters)
{
  while(true)
  {
    RadioIOParameters commandToSend;
            
    while (SerialUSB.available() > 0)
    {
      int character = SerialUSB.read();
      if (character == COMMAND_TERMINATOR)
      {
        strcpy(commandToSend.command, computerBuffer.c_str());
        commandToSend.receiveOutput = true;

        // Send to radio thread and wait for response.
        xQueueSendToFront(radioInputQueue, &commandToSend, portMAX_DELAY);
        computerBuffer = "";
        if (xQueueReceive(pcOutputQueue, &commandToSend, portTICK_PERIOD_MS * 100) == pdTRUE)
        {
          // Received something back within 100ms.
          SerialUSB.print(commandToSend.command);
          SerialUSB.print(COMMAND_TERMINATOR);
        }
      }
      else
      {
        computerBuffer += (char)character;
      }
    }

    // Handle unsolicited output from the radio.
    if (xQueueReceive(pcOutputQueue, &commandToSend, 0) == pdTRUE)
    {
      SerialUSB.print(commandToSend.command);
      SerialUSB.print(COMMAND_TERMINATOR);
    }
  }
}

void pollRadio(TimerHandle_t timer)
{
  RadioIOParameters commandToSend;
  strcpy(commandToSend.command, RADIO_INFO_CMD);
  commandToSend.receiveOutput = false;

  // Post to queue. If the queue is full, ignore; we'll get called again eventually.
  xQueueSendToBack(radioInputQueue, &commandToSend, 0);
}

void setup() {
  // Set analog output pin to 0V.
  setBand(0);

  // Open serial ports
  SerialUSB.begin(COMPUTER_BAUD);
  SerialUSB.setTimeout(SERIAL_TIMEOUT_MS);
  Serial1.begin(RADIO_BAUD);
  Serial1.setTimeout(SERIAL_TIMEOUT_MS);

  // Create queues:
  // Radio Input (2 max, one for IF polling and one for PC traffic)
  // PC Output (1 max)
  radioInputQueue = xQueueCreate(2, sizeof(RadioIOParameters));
  pcOutputQueue = xQueueCreate(1, sizeof(RadioIOParameters));
  assert(radioInputQueue != NULL);
  assert(pcOutputQueue != NULL);

  // Create polling timer and start it.
  pollingTimer = xTimerCreate("Radio Polling", pdMS_TO_TICKS(POLLING_PERIOD_MS), pdTRUE, NULL, pollRadio);
  assert(pollingTimer != NULL);
  xTimerStart(pollingTimer, 0);
  
  // Enable FreeRTOS tasks:
  // 1. Arduino<->KX3 Serial
  // 2. PC<->Arduino Serial
  xTaskCreate(radioSerial, "Arduino<->KX3", 256, NULL, tskIDLE_PRIORITY + 1, &radioSerialTask);
  xTaskCreate(pcSerial, "PC<->Arduino", 256, NULL, tskIDLE_PRIORITY + 1, &pcSerialTask);

  // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();
}
  
void loop() 
{
  // RTOS idle loop; not currently used.
}

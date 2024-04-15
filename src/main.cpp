#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <HCSR04.h>

#include "MotorControl.h"
#include "MPU6050MadgwickAHRS.h"

//change it first
#define MOT1_ENC1 21
#define MOT1_DIRECT 20
#define MOT2_ENC1 19
#define MOT2_DIRECT 18

#define triggerPin 13
#define echoPin 12

UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);

volatile int MOT1_ENC1_TICKS = 0;
volatile int MOT2_ENC1_TICKS = 0;

#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN 7
#define CSN_PIN 8

RF24 radio(CE_PIN, CSN_PIN);

uint8_t address[][6] = { "1Node", "2Node" };

// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

bool role = false;  // true = TX role, false = RX role

struct PayloadStruct {
  char message[7];  // only using 6 characters for TX & ACK payloads
  uint8_t counter;
};
PayloadStruct payload;


void ISR_A() {
  if (digitalRead(MOT1_ENC1) == digitalRead(MOT1_DIRECT)) {
    MOT1_ENC1_TICKS++;
  } else {
    MOT1_ENC1_TICKS--;
  }
}

void ISR_B() {
  if (digitalRead(MOT2_ENC1) == digitalRead(MOT2_DIRECT)) {
    MOT2_ENC1_TICKS++;
  } else {
    MOT2_ENC1_TICKS--;
  }
}

float getDistance() {
  return distanceSensor.measureDistanceCm();
}

void setup() {
  attachInterrupt(digitalPinToInterrupt(MOT1_ENC1), ISR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(MOT2_ENC1), ISR_B, RISING);
  pwm1.begin();
  pwm1.setPWMFreq(60);

  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }

  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }

  // print example's introductory prompt
  Serial.println(F("RF24/examples/AcknowledgementPayloads"));

  // To set the radioNumber via the Serial monitor on startup
  Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
  while (!Serial.available()) {
    // wait for user input
  }
  char input = Serial.parseInt();
  radioNumber = input == 1;
  Serial.print(F("radioNumber = "));
  Serial.println((int)radioNumber);

  // role variable is hardcoded to RX behavior, inform the user of this
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // to use ACK payloads, we need to enable dynamic payload lengths (for all nodes)
  radio.enableDynamicPayloads();  // ACK payloads are dynamically sized

  // Acknowledgement packets have no payloads by default. We need to enable
  // this feature for all nodes (TX & RX) to use ACK payloads.
  radio.enableAckPayload();

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1

  // additional setup specific to the node's role
  if (role) {
    // setup the TX payload

    memcpy(payload.message, "Hello ", 6);  // set the payload message
    radio.stopListening();                 // put radio in TX mode
  } else {
    // setup the ACK payload & load the first response into the FIFO

    memcpy(payload.message, "World ", 6);  // set the payload message
    // load the payload for the first received transmission on pipe 0
    radio.writeAckPayload(1, &payload, sizeof(payload));

    radio.startListening();  // put radio in RX mode
  }

  // For debugging info
  // printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data
}

void loop() {
  // put your main code here, to run repeatedly:
  if (role) {
    // This device is a TX node

    unsigned long start_timer = micros();                  // start the timer
    bool report = radio.write(&payload, sizeof(payload));  // transmit & save the report
    unsigned long end_timer = micros();                    // end the timer

    if (report) {
      Serial.print(F("Transmission successful! "));  // payload was delivered
      Serial.print(F("Time to transmit = "));
      Serial.print(end_timer - start_timer);  // print the timer result
      Serial.print(F(" us. Sent: "));
      Serial.print(payload.message);  // print the outgoing message
      Serial.print(payload.counter);  // print the outgoing counter
      uint8_t pipe;
      if (radio.available(&pipe)) {  // is there an ACK payload? grab the pipe number that received it
        PayloadStruct received;
        radio.read(&received, sizeof(received));  // get incoming ACK payload
        Serial.print(F(" Recieved "));
        Serial.print(radio.getDynamicPayloadSize());  // print incoming payload size
        Serial.print(F(" bytes on pipe "));
        Serial.print(pipe);  // print pipe number that received the ACK
        Serial.print(F(": "));
        Serial.print(received.message);    // print incoming message
        Serial.println(received.counter);  // print incoming counter

        // save incoming counter & increment for next outgoing
        payload.counter = received.counter + 1;

      } else {
        Serial.println(F(" Recieved: an empty ACK packet"));  // empty ACK packet received
      }


    } else {
      Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
    }

    // to make this example readable in the serial monitor
    delay(1000);  // slow transmissions down by 1 second

  } else {
    // This device is a RX node

    uint8_t pipe;
    if (radio.available(&pipe)) {                     // is there a payload? get the pipe number that recieved it
      uint8_t bytes = radio.getDynamicPayloadSize();  // get the size of the payload
      PayloadStruct received;
      radio.read(&received, sizeof(received));  // get incoming payload
      Serial.print(F("Received "));
      Serial.print(bytes);  // print the size of the payload
      Serial.print(F(" bytes on pipe "));
      Serial.print(pipe);  // print the pipe number
      Serial.print(F(": "));
      Serial.print(received.message);  // print incoming message
      Serial.print(received.counter);  // print incoming counter
      Serial.print(F(" Sent: "));
      Serial.print(payload.message);    // print outgoing message
      Serial.println(payload.counter);  // print outgoing counter

      // save incoming counter & increment for next outgoing
      payload.counter = received.counter + 1;
      // load the payload for the first received transmission on pipe 0
      radio.writeAckPayload(1, &payload, sizeof(payload));
    }
  }  // role

  if (Serial.available()) {
    // change the role via the serial monitor

    char c = toupper(Serial.read());
    if (c == 'T' && !role) {
      // Become the TX node

      role = true;
      Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));

      memcpy(payload.message, "Hello ", 6);  // change payload message
      radio.stopListening();                 // this also discards any unused ACK payloads

    } else if (c == 'R' && role) {
      // Become the RX node

      role = false;
      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
      memcpy(payload.message, "World ", 6);  // change payload message

      // load the payload for the first received transmission on pipe 0
      radio.writeAckPayload(1, &payload, sizeof(payload));
      radio.startListening();
    }
  }
  delay(500);
}

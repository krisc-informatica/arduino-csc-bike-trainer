/**
 * Arduino Smart Bike Trainer
 * 
 * This Arduino project enables a connection between a non-smart digital bicyle trainer (like Tacx Flow T2200) and training software (Tacx trainer, Zwift, BKool,...)
 * using the BLE Cycling Speed and Cadence Profile.
 * This will not turn the trainer into a smart trainer, because no information is received from the software about the training context (grade, rolling resistance,...).
 * 
 * According to https://hackaday.io/project/164276-tacx-flow-ant-conversion the cable layout of the RJ10 connector on the Tacx Flow is as follows:
 *    1 Cadence signal to computer (one pulse per crank revolution), 3.3V pulses, pulled high (no pulse => 3.3V)
 *    2 GND/Common.
 *    3 PWM control signal to brake (2.6V pulses).
 *    4 AC power / synchronization signal to computer, ~23V AC signal which appears the +ve part is clipped at ~19.5V (see picture).
 *    5 +18V (~1.5V variation sawtooth profile).
 *    6 Speed signal to computer (4 pulses per brake axle revolution) 3.3V pulses, pulled high (no pulse => 3.3V)
 * 
 * BLE information that might be useful
 * 
 *    BLE Base UUID: 00000000-0000-1000-8000-00805F9B34FB
 *    16-bit UUID to 128-bit: 0000xxxx-0000-1000-8000-00805F9B34FB
 *    32-bit UUID to 128-bit: xxxxxxxx-0000-1000-8000-00805F9B34FB
 *    
 *    CSC (Cycling Speed and Cadence Sensor): 0x1816 (file:///C:/Users/krisc/Downloads/CSCP_SPEC_V10.pdf, https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Services/org.bluetooth.service.cycling_speed_and_cadence.xml)
 *      The profile specification document describes the CSC profile consisting of a Sensor which exposes two GATT services:
 *        - Cycling Speed and Cadence Service (mandatory)
 *          CSC Measurement (mandatory): 0x2A5B (11 bytes) [https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.csc_measurement.xml]
 *          CSC Feature (mandatory): 0x2A5C (2 bytes) [https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.csc_feature.xml]
 *          Sensor Location (conditional): 0x2A5D (1 byte) [https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.sensor_location.xml]
 *          SC Control Point (conditional): 0x2A55 [https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.sc_control_point.xml]
 *        - Device Information Service (optional)
 * 
 * @name   Arduino CSC Bike Trainer
 * @author Kris Cardinaels <kris@krisc-informatica.be>
 * @license MIT license <https://choosealicense.com/licenses/mit/>
 * @link krisc-informatica.be <https://www.krisc-informatica.be/tacx-flow-arduino-smart-trainer/>
 * @link Tacx Flow Ant+ Conversion <https://hackaday.io/project/164276-tacx-flow-ant-conversion>
 * @link Bicycle Odometer and Speedometer <https://create.arduino.cc/projecthub/alan_dewindt/bicycle-odometer-and-speedometer-with-99-lap-period-recorder-331d2b>
 */
#include <ArduinoBLE.h>

double WHEEL_SIZE = 2100; // Circumference of the wheel, to be defined by the rider

#define DEVICE_NAME_LONG "Arduino CSC Bike Trainer"
#define DEVICE_NAME_SHORT "KC-CSC"


/**
 * Cycling Speed and Cadence service, uuid 0x1816 or 00001816-0000-1000-8000-00805F9B34FB
 * 
 */
BLEService cyclingSpeedAndCadenceService("1816"); // CSC

// Service characteristics exposed by CSC
BLECharacteristic cscMeasurementCharacteristic("2A5B", BLENotify, 11);                                // CSC Measurement, mandatory, notify
BLECharacteristic cscFeatureCharacteristic("2A5C", BLERead, 2);                                       // CSC Feature, mandatory, read
// BLECharacteristic sensorLocation("2A5D", BLERead, 1);                                              // CSC Sensor Location, conditional, read
// BLECharacteristic scControlPoint("2A55", BLEWrite | BLEIndicate, 30);                              // CSC Control Point, conditional, write & indicate, 30 bytes chosen randomly


// Buffers used to write to the characteristics and initial values
unsigned char cscmBuffer[11] = {0,0,0,0,0,0,0,0,0,0,0};
unsigned char cscfBuffer[2] = {0b00000011, 0};                                                              // Features: 0 (Wheel revolution data), 1 (Crank revolution data)

/**
 * The client device
 */
BLEDevice central;

/**
 * Variables for the handling of writing statuses over BLE
 */
#define RED 22     
#define GREEN 23
#define BLUE 24     
int ble_connected = LOW;
const short NOTIFICATION_INTERVAL = 1000;
long previous_notification = 0;

/**
 * Speed and Cadence sensors
 */
#define SPEED   2
#define CADENCE 3
#define SYNC    12

volatile unsigned long speed_counter = 0;               // The incrementing counter of speed pulses from the brake
volatile unsigned long speed_timer = 0;                 // The last speed interrupt time
unsigned long speed_counter_csc =0;                     // The previous amount of speed pulses written to CSC measurement
unsigned long speed_timer_csc = 0;                      // The previous time written to csc
double instantaneous_speed = 0;                         // Global variable to hold the calculated speed

volatile unsigned long cadence_counter = 0;             // The incrementing counter of cranck revolutions
volatile unsigned long cadence_timer;
unsigned long cadence_counter_csc = 0;
unsigned long cadence_timer_csc = 0;
unsigned int instantaneous_cadence = 0;

// General variables
long current_millis;

void writeStatus(int red, int green, int blue) {
  analogWrite(RED, red);
  analogWrite(GREEN, green);
  analogWrite(BLUE, blue);
}

void setup() {
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  writeStatus(1024, 1024, 0);

  pinMode(SYNC, INPUT);
  attachInterrupt(digitalPinToInterrupt(SYNC), syncSignal, RISING);

  if (!BLE.begin()) { // Error starting the bluetooth module
    while (1) {
      writeStatus(0, 1024, 1024);
      delay(250);
      writeStatus(1024, 1024, 1024);
      delay(250);
    }
  }

  BLE.setDeviceName(DEVICE_NAME_LONG);
  BLE.setLocalName(DEVICE_NAME_SHORT);
  BLE.setAdvertisedService(cyclingSpeedAndCadenceService);
  cyclingSpeedAndCadenceService.addCharacteristic(cscMeasurementCharacteristic);
  cyclingSpeedAndCadenceService.addCharacteristic(cscFeatureCharacteristic);
  BLE.addService(cyclingSpeedAndCadenceService);

  // Write values to the characteristics that can be read
  cscFeatureCharacteristic.writeValue(cscfBuffer, 2);

  // start advertising
  BLE.advertise();
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // Speed and Cadence handling
  pinMode(SPEED, INPUT);
  pinMode(CADENCE, INPUT);
  attachInterrupt(digitalPinToInterrupt(SPEED), speedPulseInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(CADENCE), cadencePulseInterrupt, RISING);
}

void loop() {
  BLE.poll();

  central = BLE.central();
  if (central && central.connected()) {
    current_millis = millis();
    if (current_millis > previous_notification + NOTIFICATION_INTERVAL) { // A new notification should be done after the given period (1 second)
      writeCscMeasurement();
      previous_notification = current_millis;
    }
  }
}

/**
 * Writes the CSC Measurement characteristic. The speed information must be given in cumulative wheel revolutions.
 * The trainer value speed_counter is the cumulative brake revolutions (4 pulses per revolution).
 * The wheel revs = speed_counter2 = speed_counter X brake_size/wheel_size
 * Times must be given 1/1024 of a second
 * 
 * @return void
 */
void writeCscMeasurement() {
  cscmBuffer[0] = 0b00000011; // b0: Wheel revolution data present, b1: Cranck revolution data present

  unsigned long speed_counter2 = speed_counter * ( BRAKE_SIZE / WHEEL_SIZE);
  cscmBuffer[1] = speed_counter2 & 0xFF; // Cumulative wheel revolution
  cscmBuffer[2] = (speed_counter2 >> 8) & 0xFF;
  cscmBuffer[3] = (speed_counter2 >> 16) & 0xFF;
  cscmBuffer[4] = (speed_counter2 >> 32) & 0xFF;
  unsigned long speed_timer_1024 = speed_timer * 1000 / 1024;
  cscmBuffer[5] = speed_timer_1024 & 0xFF; // Last wheel event time
  cscmBuffer[6] = (speed_timer_1024 >> 8) & 0xFF;
  cscmBuffer[7] = cadence_counter & 0xFF; // Cumulative cranck revolution
  cscmBuffer[8] = (cadence_counter >> 8) & 0xFF;
  unsigned long cadence_timer_1024 = cadence_timer * 1000 / 1024;
  cscmBuffer[9] = cadence_timer_1024 & 0xFF; // Last cranck event time
  cscmBuffer[10] = (cadence_timer_1024 >> 8) & 0xFF;
  
  cscMeasurementCharacteristic.writeValue(cscmBuffer, 11);

  // CSC was written, so update the values
  speed_counter_csc = speed_counter;
  speed_timer_csc = speed_timer;
  cadence_counter_csc = cadence_counter;
  cadence_timer_csc = cadence_timer;

}

/*
 * BLE device connected and disconnected handlers
 */

/**
 * Lights the internal RGB LED to green on connection
 * 
 * @return void
 */
void blePeripheralConnectHandler(BLEDevice central) {
  ble_connected = HIGH;
  writeStatus(1024, 0, 1024);
}

/*
 * Lights the internal RGB LED to blue on disconnection
 * 
 * @return void
 */
void blePeripheralDisconnectHandler(BLEDevice central) {
  ble_connected = LOW;
  writeStatus(1024, 1024, 0);
}

void speedPulseInterrupt() {
  speed_counter++;
  speed_timer = millis();
}

void cadencePulseInterrupt() {
  cadence_counter++;
  cadence_timer = millis();
}

/**
 * Handles the synchronization signal:
 *   - sets the time in microseconds of the signal for the timing of the pwm
 *   - indicates that the signal can be generated if needed
 */
void syncSignal() {
  syncTime = micros();
}

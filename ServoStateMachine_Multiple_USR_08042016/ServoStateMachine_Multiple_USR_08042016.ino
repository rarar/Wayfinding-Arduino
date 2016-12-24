#include <VarSpeedServo.h>
#include <NewPing.h>

#define DIRECTIONS 4
#define SIZE 3
#define SERVO_STEP_SIZE 7

#define SONAR_NUM     4 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define SONAR_LOOP_INTERVAL 10 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
int inByte = 0;

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(3, 3, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(4, 4, MAX_DISTANCE),
  NewPing(5, 5, MAX_DISTANCE),
  NewPing(6, 6, MAX_DISTANCE)
};
long previousSonarMillis = 0;

VarSpeedServo cardinalDirections[DIRECTIONS];
long previousMillis = 0;
long interval = 8000;
long minTime = 2000;
int sequenceStart = 0;

// Serial Values
int currentValue = 0;
int values[] = {0, 0, 0, 0};
int autoToggle = 0;
int startingIndex = 0;
int manualPush = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  establishContact();
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  // Initialize Servos
  for (int i = 0; i < DIRECTIONS; i++) {
    cardinalDirections[i].attach(i + SERVO_STEP_SIZE);
    cardinalDirections[i].write(0, 255, false);
  }
}

void loop() {
  inByte = Serial.read();
  // If we have Auto mode, let's disable any manual triggers
  if (autoToggle == 1) {
    manualPush = 0;
  }
  unsigned long currentMillis = millis();

  // If it's the interval time OR if we've manually triggered and have done so without being trigger happy :)
  if (((currentMillis - previousMillis > interval) || ((manualPush == 1) && (currentMillis - previousMillis > minTime)))) {
    previousMillis = currentMillis;

    // Check to see if the power is off first!
    if (startingIndex < DIRECTIONS) runServoLoop();

    manualPush = 0;

  } else if (currentMillis - previousSonarMillis > PING_INTERVAL) {
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
      if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
        pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
        if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
        sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
        currentSensor = i;                          // Sensor being accessed.
        cm[currentSensor] = 0;           // Make distance zero in case there's no ping echo for this sensor.
        sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
      }
    }
    previousSonarMillis = currentMillis;
  }
}

// The meat and potatoes loop
void runServoLoop() {
  int i = startingIndex;
  do {
    cardinalDirections[i].write(130, 255, true);
    cardinalDirections[i].write(0, 255, true);
    i = (i == (int)(DIRECTIONS - 1)) ? 0 : i + 1;
  } while (i != startingIndex );
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  unsigned int minVal = MAX_DISTANCE;
  unsigned int sensor = 0;
  unsigned int avgVal = 0;
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    //        Serial.print(i);
    //        Serial.print("=");
    //        Serial.print(cm[i]);
    //        Serial.print("cm ");
    Serial.write(cm[i]);
    if (cm[i] == 0) {
      avgVal += MAX_DISTANCE;
    } else {
      avgVal += cm[i];
    }

    if (cm[i] < minVal && cm[i] != 0) {
      minVal = cm[i];
      sensor = i;
    }
  }
  //  Serial.print("min="); Serial.print(minVal); Serial.print(" from sensor="); Serial.print(sensor);
  //  Serial.println();
  startingIndex = sensor;
  avgVal = avgVal / SONAR_NUM;
  interval = map(avgVal, 0, MAX_DISTANCE, 5000, 18000);
  //  Serial.print("Avg Value="); Serial.print(avgVal);
  //  Serial.print(" | New Interval="); Serial.print(interval);
  //  Serial.print(" | Min Val="); Serial.print(minVal);
  //  Serial.println();
}

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('A');   // send a capital A
    delay(300);
  }
}

//// Read from serial
//void getValuesFromTouchOSC() {
//  if (Serial.available()) { //  Check if there is a new message
//    int incomingValue = Serial.read();
//    // Everything gets passed as a byte array
//    values[currentValue] = incomingValue;
//    currentValue++;
//    if (currentValue > 3) {
//      currentValue = 0;
//    }
//
//    // Error handling for the delay fader
//    if (values[0] < 0 || values[0] > 127) values[0] = 12;
//    interval = map(values[0], 0, 127, minTime, 30000); // map the fader value from our minimimum time interval (2.5 sec to 30 sec)
//    autoToggle = values[1];
//    startingIndex = values[3];
//    manualPush = values[2];
//  }
//
//}





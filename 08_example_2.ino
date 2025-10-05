// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

unsigned long last_sampling_time;   // unit: msec

int dutyFromDistance(float d) {
  if (d == 0.0 || d <= _DIST_MIN || d >= _DIST_MAX) return 255;  
  float duty;
  if (d <= 200.0) {
    // 100mm -> 255, 200mm -> 0
    duty = 255.0f * (200.0f - d) / (200.0f - _DIST_MIN);
  } else {
    // 200mm -> 0, 300mm -> 255
    duty = 255.0f * (d - 200.0f) / (_DIST_MAX - 200.0f);
  }
  if (duty < 0) duty = 0;
  if (duty > 255) duty = 255;
  return (int)(duty + 0.5f); 
}


void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 
  
  // initialize serial port
  Serial.begin(57600);
}


void loop() {
  if (millis() - last_sampling_time < INTERVAL) return;

  float distance = USS_measure(PIN_TRIG, PIN_ECHO);
  int duty = dutyFromDistance(distance);
  analogWrite(PIN_LED, duty);

  Serial.print("Min:");        Serial.print(_DIST_MIN);
  Serial.print(",distance:");  Serial.print(distance);
  Serial.print(",Max:");       Serial.print(_DIST_MAX);
  Serial.print(", duty:");     Serial.println(duty);

  last_sampling_time = millis(); 
}


// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  unsigned long us = pulseIn(ECHO, HIGH, (unsigned long)TIMEOUT); // 0이면 타임아웃
  return us * SCALE; // mm
}

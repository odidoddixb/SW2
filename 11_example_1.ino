#include <Servo.h>

// ---------- Pins ----------
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12
#define PIN_ECHO  13
#define PIN_SERVO 10

// ---------- Sonar ----------
#define SND_VEL 346.0
#define INTERVAL 25              // ms (샘플링 주기)
#define PULSE_DURATION 10        // us
#define _DIST_MIN 180.0          // mm (18 cm)
#define _DIST_MAX 360.0          // mm (36 cm)
#define TIMEOUT ((INTERVAL/2)*1000.0)
#define SCALE (0.001 * 0.5 * SND_VEL)   // us → mm

// ---------- Filters ----------
#define _EMA_ALPHA 0.30          

// ---------- Servo targets (mm) ----------
#define _TARGET_LOW  250.0
#define _TARGET_HIGH 290.0

// ---------- Servo calibration (us) ----------
#define _DUTY_MIN 500
#define _DUTY_NEU 1500
#define _DUTY_MAX 2500


// ---------- Smooth parameters ----------
const float   TARGET_LPF_ALPHA = 0.25f; 
const uint16_t SERVO_UPDATE_MS = 20;    
const float   SLEW_DEG_PER_SEC = 240.0; 

// -------------------------------------------------
Servo myservo;
float dist_prev = _DIST_MIN;
float dist_ema  = _DIST_MIN;

unsigned long last_sampling_time;
unsigned long last_servo_update;

float target_deg_lpf = 90.0f;  
float cmd_deg        = 90.0f;  

// 각도(0~180) → us
static inline int degToUs(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  float ratio = deg / 180.0f;
  return (int)(_DUTY_MIN + (_DUTY_MAX - _DUTY_MIN) * ratio);
}

// mm → 각도(0~180): 18cm -> 0°, 36cm -> 180° 
static inline float mmToDeg(float mm) {
  float deg = mm - 180.0f; // (mm-180)*(180/180)
  return constrain(deg, 0.0f, 180.0f);
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH); // active-low: OFF
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  Serial.begin(57600);

  dist_prev = _DIST_MIN;
  dist_ema  = dist_prev;
  target_deg_lpf = 90.0f;
  cmd_deg        = 90.0f;

  last_sampling_time = millis();
  last_servo_update  = millis();
}

void loop() {

  if (millis() - last_sampling_time >= INTERVAL) {
    last_sampling_time += INTERVAL;

    float dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

    float dist_filtered;
    if ((dist_raw == 0.0f) || (dist_raw > _DIST_MAX) || (dist_raw < _DIST_MIN)) {
      dist_filtered = dist_prev;
    } else {
      dist_filtered = dist_raw;
      dist_prev = dist_raw;
    }


    dist_ema = (_EMA_ALPHA * dist_filtered) + ((1.0f - _EMA_ALPHA) * dist_ema);

    
    bool in_range = (dist_filtered >= _DIST_MIN && dist_filtered <= _DIST_MAX);
    digitalWrite(PIN_LED, in_range ? LOW : HIGH);


    float target_deg =
      (dist_ema <= _DIST_MIN) ? 0.0f :
      (dist_ema >= _DIST_MAX) ? 180.0f :
                                mmToDeg(dist_ema);


    target_deg_lpf = (TARGET_LPF_ALPHA * target_deg) +
                     ((1.0f - TARGET_LPF_ALPHA) * target_deg_lpf);

    Serial.print("Min:");   Serial.print(_DIST_MIN);
    Serial.print(",dist:"); Serial.print(dist_raw);
    Serial.print(",ema:");  Serial.print(dist_ema);
    Serial.print(",Servo:");Serial.print(cmd_deg);
    Serial.print(",Max:");  Serial.print(_DIST_MAX);
    Serial.println();
  }

 
  if (millis() - last_servo_update >= SERVO_UPDATE_MS) {
    last_servo_update += SERVO_UPDATE_MS;

    const float maxStep = SLEW_DEG_PER_SEC * (SERVO_UPDATE_MS / 1000.0f);
    float diff = target_deg_lpf - cmd_deg;

    if (fabs(diff) <= maxStep) cmd_deg = target_deg_lpf;
    else                       cmd_deg += (diff > 0 ? maxStep : -maxStep);

    myservo.writeMicroseconds(degToUs(cmd_deg));
  }
}

// 초음파 센서 (mm)
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // mm
}

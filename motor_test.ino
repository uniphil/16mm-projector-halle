#define PWM 9
#define DIR A2
#define ADJ A4
#define GO 2
#define HALL 3

#define MIN_SPEED 30
#define MAX_SPEED 150
#define MAX_DIAL 1024

#define SCALE2 1

#define T(X) X >> SCALE2

enum State {
  stopped,
  going,
};

State state = stopped;
uint16_t last_speed = 0;
volatile unsigned long last_hall_trigger = 0;
volatile unsigned long hall_dt_micros;
volatile bool dirty = false;

void setup() {
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ADJ, INPUT);
  pinMode(GO, INPUT);
  pinMode(HALL, INPUT);
  // pull-ups
  digitalWrite(GO, HIGH);
  digitalWrite(HALL, HIGH);

  Serial.begin(9600 << SCALE2);  // << because it's a rate, not a delay

  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  CLKPR = (1<<CLKPCE);
  CLKPR = SCALE2;

  attachInterrupt(digitalPinToInterrupt(HALL), hall_trigger, RISING);
}

void loop() {
  if (state == stopped) {
    if (digitalRead(GO) == LOW) {
      start();
      state = going;
    } else {
      // still stopped
    }
  } else if (state == going) {
    if (digitalRead(GO) == HIGH) {
      stop_gently();
      state = stopped;
    } else {
      update_speed();
    }
  }
  
}

void hall_trigger() {
  unsigned long now = micros();
  hall_dt_micros = now - last_hall_trigger;
  last_hall_trigger = now;
  dirty = true;
}

uint8_t scale_speed(uint16_t pot) {
  return map(pot, 0, MAX_DIAL, MIN_SPEED, MAX_SPEED);
}

void start() {
  digitalWrite(DIR, HIGH);  // reverse for a moment
  analogWrite(PWM, 64);
  delay(T(60));
  analogWrite(PWM, 2);
  delay(T(120));
  digitalWrite(DIR, LOW);  // forward again
  analogWrite(PWM, 128);
  delay(T(100));
  analogWrite(PWM, MIN_SPEED);
}

void stop_gently() {
  int speed = scale_speed(last_speed);
  while (--speed) {
    analogWrite(PWM, speed);
    delay(1);
  }
  last_speed = speed;
}

void update_speed() {
  uint16_t next_speed = analogRead(ADJ);
  if (dirty) {
    Serial.println(1.0 / ((hall_dt_micros << SCALE2) / 1000000.0));
    dirty = false;
  }
  if (next_speed != last_speed) {
    last_speed = next_speed;
//    Serial.print("speed: ");
//    Serial.print(next_speed);
//    Serial.print("; ");
//    Serial.println(scale_speed(next_speed));
    analogWrite(PWM, scale_speed(next_speed));
  }

}


#define SCALE2 1
#define PWM 9
#define DIR A2
#define ADJ A4
#define GO 2
#define HALL 3

#define MIN_SPEED 30
#define MAX_SPEED 150
#define MAX_DIAL 1024

#define STOPPED 1000

#define SCALE2 1

#define T(X) X >> SCALE2

#define FPS 24
#define KP 0.02
#define KI 0.12
#define KD 0.09

enum State {
  stopped,
  going,
};

State state = stopped;
double last_speed = 0;
unsigned long last_update = 0;
volatile unsigned long last_hall_trigger = 0;
volatile unsigned long hall_dt_micros;
volatile bool dirty = false;

double sp = 1.0 / FPS * 1000000;

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
  long adjusted = map(pot, 0, MAX_DIAL, MIN_SPEED, MAX_SPEED);
  if (adjusted < MIN_SPEED) {
    return MIN_SPEED;
  } else if (adjusted > MAX_SPEED) {
    return MAX_SPEED;
  }
  return adjusted;
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

double err_sum = 0;
double last_err = 0;

void update_speed() {
  unsigned long now = millis();
  if (dirty) {
    dirty = false;

    double input = hall_dt_micros << SCALE2;
    double err = input - sp;

    Serial.print(1.0 / (input / 1000000));
    Serial.print("\t");
    Serial.print(err);

    err_sum += err * input / 1000000;
    if (err_sum > sp) {
      err_sum = sp;
    } else if (-err_sum > sp) {
      err_sum = -sp;
    }
    double d_err = (err - last_err) / input / 1000000;
    last_err = err;

    double next_speed = KP * err + KI * err_sum + KD * d_err;

    if (next_speed < 0) {
      next_speed = 0;
    }

    analogWrite(PWM, scale_speed(next_speed));

    Serial.print("\t");
    Serial.print(err_sum);
    Serial.print("\t");
    Serial.print(d_err);
    Serial.print("\t");
    Serial.println(scale_speed(next_speed));

    last_speed = next_speed;
    last_update = now;
  } else if (now - last_update > STOPPED) {
    analogWrite(PWM, MAX_SPEED);
  }
}



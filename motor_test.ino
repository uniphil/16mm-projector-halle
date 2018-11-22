// pins
#define VSYNC 2
#define HALL 3
#define ENC_PULSE_A 4
#define ENC_BLUE 5
#define ENC_GREEN 6
#define ENC_SW 7
#define SHUTTER_ENABLE 8
#define PWM 9  // TIMER1
#define ENC_RED 10  // TIMER1
#define DIR 11
#define ENC_PULSE_B 12
#define ADJ A0
#define SHUTTER A1
#define MODE A2
#define GO A3

// it's a resistor network into an analog input. numbers checked empirically:
#define MODE_0_LEVEL 14
#define MODE_1_LEVEL 244
#define MODE_2_LEVEL 396
#define MODE_3_LEVEL 568
#define MODE_4_LEVEL 1018
#define MODE_1_THRESH (MODE_0_LEVEL + MODE_1_LEVEL / 2)
#define MODE_2_THRESH (MODE_1_LEVEL + MODE_2_LEVEL / 2)
#define MODE_3_THRESH (MODE_2_LEVEL + MODE_3_LEVEL / 2)
#define MODE_4_THRESH (MODE_3_LEVEL + MODE_4_LEVEL / 2)

#define MIN_SPEED (44 << 2)
#define MAX_SPEED (140 << 2)

#define PULSE_PERIOD 200

#define STABLE_D_THRESH 4.0
#define STABLE_D_TIME 1300 // ms

//double shutter_phase = 0.55; // normalized 0-1
//double shutter_angle = 0.27;  // normalized 0-1

double shutter_phase = 0.52; // normalized 0-1
double shutter_angle = 0.22;  // normalized 0-1

double FPS = 23.95;
bool adjusting = false;
#define KP 4.0
#define KI 0.04
#define KD -38.0

#define KPHASE 500.0


enum State {
  stopped,
  starting,
  syncing,
  tracking,
  stopping,
  rewinding,
};

enum Mode {
  manual_mode,
  control_mode,
  vsync_mode,
  rewind_mode,
};

State state = stopped;
Mode mode = manual_mode;
double last_speed = 0;
unsigned long last_pid_update = 0;
unsigned long last_state_change = 0;
bool fresh_state_change = true;
volatile unsigned long vsync_last_trigger = 0;
volatile unsigned long vsync_last_last_trigger = 0;
volatile unsigned long hall_last_trigger = 0;
volatile unsigned long hall_dt_micros;

bool shutter_enabled = true;
volatile uint16_t ticks_to_shutter_close;
volatile uint16_t ticks_to_shutter_open;

volatile double hall_d_err = 0;
volatile double hall_last_err = 0;

volatile bool hall_maybe_skip = false;
volatile bool hall_maybe_bounce = false;

unsigned long hall_last_big_d = 0;

double sp = 1.0 / FPS * 1000;  // millis
unsigned long shutter_closed_millis = 1000 * shutter_angle / FPS;

unsigned long update_interval = sp / 10;  // 10x dead time

double err_sum = 0;
double last_output = 0;

typedef struct {
  unsigned long last_update;
  double kp;
  double ki;
  double kd;
  double err_acc;
  double last_err;
} pid;

#define vsync_div 2
volatile uint8_t vsync_count = 0;
void vsync_trigger() {
  vsync_count = (vsync_count + 1) % vsync_div;
  if (vsync_count != 0) {
    return;
  }
  vsync_last_last_trigger = vsync_last_trigger;
  vsync_last_trigger = micros();
}

void hall_trigger() {
  unsigned long now = micros();
  unsigned long dt_micros = now - hall_last_trigger;

  if (dt_micros > hall_dt_micros * 1.667) {
    hall_maybe_skip = true;
  } else if (dt_micros < hall_dt_micros / 3) {
    hall_maybe_bounce = true;
  }
  hall_dt_micros = dt_micros;
  hall_last_trigger = now;

  double input = hall_dt_micros / 1000;  // to millis
  double err = input - sp;
  hall_d_err = (err - hall_last_err) / input;
  hall_last_err = err;

  // set up for shutter close
  unsigned long micros_to_shutter_close = dt_micros * shutter_phase;
  ticks_to_shutter_close = micros_to_shutter_close / 1000;
  uint16_t rem = micros_to_shutter_close % 1000;
  uint16_t rem_ticks = rem / 4;
  OCR0A = TCNT0 + rem_ticks;  
  // if the timer overflowed while in this isr, drop a tick
  if (SREG & B00000001) {
    ticks_to_shutter_close++;
  }
}


ISR(TIMER0_COMPA_vect) {
  if (ticks_to_shutter_open > 0) {
    if (--ticks_to_shutter_open == 0) {
      digitalWrite(SHUTTER, LOW);
    }
  }
  if (ticks_to_shutter_close > 0) {
    if (--ticks_to_shutter_close == 0 && state != stopped && shutter_enabled) {
      digitalWrite(SHUTTER, HIGH);
    }
    // set up for shutter open
    if (mode == manual_mode) {
      unsigned long micros_to_shutter_open = hall_dt_micros * shutter_angle;
      ticks_to_shutter_open = micros_to_shutter_open / 1000;
      uint16_t rem = micros_to_shutter_open % 1000;
      uint16_t rem_ticks = rem / 4;
      OCR0A = TCNT0 + rem_ticks;
    } else {
      ticks_to_shutter_open = shutter_closed_millis;
    }
    // if the timer overflowed while in this isr, pick up an extra tick
    if (SREG & B00000001) {
      ticks_to_shutter_open++;
    }
  }
}


char* state_name(State state) {
  switch (state) {
    case (stopped):
      return "stopped";
    case (starting):
      return "starting";
    case (syncing):
      return "syncing";
    case (tracking):
      return "tracking";
    case (stopping):
      return "stopping";
    case (rewinding):
      return "rewinding";
    default:
      return "unknown";
  }
}

char* mode_name(Mode mode) {
  switch (mode) {
    case (manual_mode):
      return "manual";
    case (control_mode):
      return "control";
    case (vsync_mode):
      return "vsync";
    case (rewind_mode):
      return "rewind";
    default:
      return "unknown";
  }
}

void transition(State next_state) {
  Serial.print("transitioning to ");
  Serial.println(state_name(next_state));
  state = next_state;
  last_state_change = millis();
  fresh_state_change = true;
}

bool run_task(bool (*task)(unsigned long, bool), State next_state = -1) {
  unsigned long now = millis();
  bool completed = (*task)(now - last_state_change, fresh_state_change);
  if (fresh_state_change) fresh_state_change = false;
  if (completed & next_state != -1) {
    transition(next_state);
  }
  return completed;
}

void writePWM(uint16_t value) {
  if (value > 1024) {  // timer1 is configured for 10-bit pwm
    value = 1024;
  }
  OCR1A = value;
}

void encRGBWrite(uint8_t r, uint8_t g, uint8_t b) {
  OCR1B = (uint16_t)(255 - r) << 2;
  analogWrite(ENC_GREEN, 255 - g);
  analogWrite(ENC_BLUE, 255 - b);
}

bool drive(long kspeed, bool reverse = false) {
  bool in_range = true;
  if (kspeed == 0) {
    digitalWrite(DIR, false);  // slightly defensive: set direction to forward
    writePWM(0);
  } else {
    digitalWrite(DIR, reverse);
    if (kspeed > 1000) {
      kspeed = 1000;
      in_range = false;
    }
    writePWM(map(kspeed, 1, 1000, MIN_SPEED, MAX_SPEED));
  }
  return in_range;
}


bool pulse() {
  return (millis() % PULSE_PERIOD) < (PULSE_PERIOD * 2 / 3);
}


void update_mode(bool go) {
  uint16_t m = analogRead(MODE);
  Mode next_mode;
  if (m < MODE_1_THRESH) {
    next_mode = rewind_mode;
    go && pulse() ? encRGBWrite(64, 0, 1)
                  : encRGBWrite(128, 12, 0);
  } else if (m < MODE_2_THRESH) {
    adjusting = true;
    next_mode = control_mode;
    (state == syncing && pulse())
      ? encRGBWrite(0, 255, 0)
      : encRGBWrite(64, 0, 192);
  } else if (m < MODE_3_THRESH) {
    adjusting = false;
    next_mode = control_mode;
    (state == syncing && pulse())
      ? encRGBWrite(0, 255, 0)
      : encRGBWrite(0, 64, 127);
  } else {
    next_mode = manual_mode;
    encRGBWrite(127, 160, 0);
  }
  if (next_mode != mode) {
    mode = next_mode;
    Serial.print("changed to to ");
    Serial.println(mode_name(mode));
  }
}

void setup() {
  pinMode(VSYNC, INPUT_PULLUP);
  pinMode(HALL, INPUT_PULLUP);
  pinMode(ENC_PULSE_A, INPUT);
  pinMode(ENC_BLUE, OUTPUT);
  pinMode(ENC_GREEN, OUTPUT);
  pinMode(ENC_SW, INPUT);  // TODO: add a pull-down resistor
  pinMode(SHUTTER_ENABLE, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);
  pinMode(ENC_RED, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENC_PULSE_B, INPUT);
  pinMode(ADJ, INPUT);
  pinMode(SHUTTER, OUTPUT);
  pinMode(MODE, INPUT_PULLUP);
  pinMode(GO, INPUT_PULLUP);

  // shortcut pwm setup for timer1
  analogWrite(PWM, 1);
  analogWrite(ENC_RED, 1);

  // timer1 no prescale
  TCCR1B = (TCCR1B & 0b11111000) | 0b001;

  // timer1 fast 10-bit (mode 7, WGM0111, 16 MHz / 10 bits == 15.625 kHz)
  TCCR1A |= _BV(WGM10) | _BV(WGM11);
  TCCR1B |= _BV(WGM12);
  TCCR1B &= ~_BV(WGM13);

  attachInterrupt(digitalPinToInterrupt(VSYNC), vsync_trigger, FALLING);  // active-low
  attachInterrupt(digitalPinToInterrupt(HALL), hall_trigger, FALLING);  // active-low

  // timer0 interrupt for shutter closing
  TIMSK0 |= _BV(OCIE0A);

  Serial.begin(115200);
  while (!Serial);
  Serial.println("setup complete.");
  transition(stopped);
}

void loop() {
  bool go = digitalRead(GO) == LOW;
  update_mode(go);
  shutter_enabled = digitalRead(SHUTTER_ENABLE) == LOW;
  if (hall_maybe_skip) {
    Serial.println("HALL MAYBE SKIPPED ");
    hall_maybe_skip = false;
  }
  if (hall_maybe_bounce) {
    Serial.println("HALL MAYBE BOUNCED ");
    hall_maybe_bounce = false;
  }
  switch (state) {
  case stopped:
    if (go) {
      transition(starting);
    } else {
      run_task(&stop_task);
    }
    break;
  case starting:
    if (!go) {
      transition(stopping);
    } else {
      if (mode == rewind_mode) {
        transition(rewinding);
      }
      run_task(&start, syncing);
    }
    break;
  case syncing:
    if (!go) {
      transition(stopping);
    } else {
      if (mode == manual_mode) {
        transition(tracking);
        break;
      }
      if (mode == rewind_mode) {
        transition(stopping);
      }
      // only run stall check if we're not transitioning
      if (!run_task(&sync, tracking)) {
           run_task(&stall_check, stopped);
      }
    }
    break;
  case tracking:
    if (!go) {
      transition(stopping);
    } else {
      if (mode == manual_mode) {
        run_task(&manual_speed);
        break;
      }
      if (mode == rewind_mode) {
        transition(stopping);
      }
      // only run stall check if we're not transitioning
      if (!run_task(&track, syncing)) {
           run_task(&stall_check, stopped);
      }
    }
    break;
  case rewinding:
    if (!go) {
      transition(stopping);
    }
    if (mode != rewind_mode) {
      transition(starting);
    }
    run_task(&manual_rewind, stopped);
    break;
  case stopping:
    run_task(&gentle_stop, stopped);
    break;
  }
}

bool stop_task(unsigned long t, bool init) {
  if (init) {
    drive(0);
    digitalWrite(SHUTTER, LOW);
  }
  return true;
}

bool start(unsigned long t, bool init) {
  if (t < 60) {
    drive(map(t, 0, 60, 0, 150), true);
  } else if (t < 120) {
    drive(map(t, 60, 120, 150, 0), true);
  } else if (t < 160) {
    drive(0);
  } else if (t < 240) {
    drive(map(t, 160, 240, 0, 500));
  } else if (t < 400) {
    drive(map(t, 240, 400, 500, 400));
  } else {
    return true;
  }
  return false;
}

bool sync(unsigned long t, bool init) {
  // spin up or down to match frame rate (error from fps)
  if (init) {
    Serial.println("INIT SYNC");
    drive(400);
    err_sum = 400 / KI;  // kick-start
    last_pid_update = t;
    hall_last_big_d = t;
    return false;
  }
  if (t - last_pid_update >= update_interval) {
    double dt = t - last_pid_update;
    noInterrupts();
      unsigned long last_hall_dt_micros = hall_dt_micros;
      double last_hall_d_err = hall_d_err;
    interrupts();
    double input = last_hall_dt_micros / 1000;  // to millis
    double err = input - sp;
    double curr_err_sum = err_sum + err * dt;
    if (KI * curr_err_sum < 0) {
      Serial.print("WAT ");
      Serial.print(t);
      Serial.print("\t");
      Serial.println(last_pid_update);
    }
    Serial.print("sp:\t");
    Serial.print(sp);
    Serial.print("\tinput:\t");
    Serial.print(input);
    Serial.print("\tp:\t");
    Serial.print(KP * err);
    Serial.print("\ti:\t");
    Serial.print(KI * curr_err_sum);
    Serial.print("\td:\t");
    Serial.print(KD * last_hall_d_err);
    double output = KP * err + KI * curr_err_sum + KD * last_hall_d_err;
    bool output_in_range;
    if (output < 0) {
      drive(1);
      output_in_range = false;
    } else {
      output_in_range = drive(output);
    }
    Serial.print("\to:\t");
    Serial.print(output);
    Serial.print(" (");
    Serial.print(output_in_range);
    Serial.println(")");
    last_output = output;
    last_pid_update = t;
    if (output_in_range) {  // avoid integral wind-up
      err_sum = curr_err_sum;
    }
    if (abs(last_hall_d_err) > STABLE_D_THRESH) {
      hall_last_big_d = t;
    }
  }
  return (t - hall_last_big_d) > STABLE_D_TIME;
}


bool stall_check(unsigned long t, bool init) {
  noInterrupts();
    unsigned long last_hall_last_trigger = hall_last_trigger;
  interrupts();
  unsigned long now_raw_micros = micros();
  unsigned long dt_last = (now_raw_micros - last_hall_last_trigger) / 1000;
  if (dt_last > sp * 8) {
    Serial.print("stalled? ");
    Serial.print(dt_last);
    Serial.print("\n");
    Serial.print(now_raw_micros);
    Serial.print("\n");
    Serial.println(last_hall_last_trigger);
    return t > 300;
  }
  return false;
}


uint8_t err_count = 0;

double adj = 0;

bool track(unsigned long t, bool init) {
  // phase-lock
  if (init) {
    err_count = 0;
  }
  if (t - last_pid_update >= update_interval) {
    if (adjusting) {
      adj = analogRead(ADJ) / 1023.0 - 0.5;
    }
    double target_phase;
    if (mode == control_mode) {
      unsigned long now = micros();
      target_phase = (now % (unsigned int)((sp + adj) * 1000)) / ((sp + adj) * 1000);
    } else {
      noInterrupts();
        double set = (vsync_last_trigger - vsync_last_last_trigger) / 1000.0;
      interrupts();
      target_phase = (t % (unsigned int)(set + adj)) / (set + adj);
//      Serial.print("target ");
//      Serial.println(target_phase);
    }
    noInterrupts();
      unsigned long last_hall_last_trigger = hall_last_trigger;
      unsigned long last_hall_dt_micros = hall_dt_micros;
    interrupts();
    unsigned long measured_t = micros() - last_hall_last_trigger;
    double measured_phase = (measured_t % last_hall_dt_micros) / (double)last_hall_dt_micros;

    double phase_error = target_phase - measured_phase;
    if (phase_error > 0.5) {
      phase_error -= 1;
    } else if (phase_error < -0.5) {
      phase_error += 1;
    }

    drive(last_output + KPHASE * phase_error);

//    Serial.print(target_phase);
//    Serial.print("\t");
//    Serial.print(measured_phase);
//    Serial.print("\t");
//    Serial.print(phase_error);
//    Serial.print("\t");
//    Serial.print(KPHASE * phase_error); 

    last_pid_update = t;
    
    double input = last_hall_dt_micros / 1000;  // to millis
    double err = abs(input - sp) / sp;
    if (err > 0.1) {
      err_count++;
      Serial.print("\t");
      Serial.print(err);
      Serial.print("\t");
      Serial.println(err_count);
      return err_count > 23;  // cover at least 3 measurement cycles
    } else {
      err_count = 0;
    }

//    Serial.println();
  }
  return false;
}

unsigned long last_fps_update = 0;
bool manual_speed(unsigned long t, bool init) {
  uint16_t pot = 1023 - analogRead(ADJ);
  float normalized = pot / 1023.0;
  float curved = pow(normalized, 2);
  uint16_t level = map(curved * 1023, 0, 1023, 1, 1000);
  drive(level);
  if (t - last_fps_update > 500) {
    Serial.print(level);
    Serial.print('\t');
    Serial.println(1.0 / hall_dt_micros * 1000000);
    last_fps_update = t;
  }
  return false;
}

bool manual_rewind(unsigned long t, bool init) {
  uint16_t pot = 1023 - analogRead(ADJ);
  float normalized = pot / 1023.0;
  float curved = pow(normalized, 2);
  uint16_t level = map(curved * 1023, 0, 1023, 1, 1000);
  drive(level, true);
  if (t - last_fps_update > 500) {
    Serial.print(level);
    Serial.print('\t');
    Serial.println(1.0 / hall_dt_micros * 1000000);
    last_fps_update = t;
  }
  return false;
}

bool gentle_stop(unsigned long t, bool init) {
  if (t / 2 > MIN_SPEED) {
    return true;
  }
  writePWM(map(t / 2, 0, MIN_SPEED, MIN_SPEED, 0));
  return false;
}


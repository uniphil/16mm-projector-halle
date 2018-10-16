#define SCALE2 1
#define PWM 9
#define DIR A2
#define ADJ A4
#define VSYNC 2
#define HALL 3
#define GO 4

#define MIN_SPEED 30
#define MAX_SPEED 150

#define DEAD_TIME 42  // ms (based on 24 Hz)

#define SLOWDOWN 1  // ms per step down
#define STOPPED 500  // part of this means that we can't do frame rates approaching 2fps

#define STABLE_D_THRESH 4.0
#define STABLE_D_TIME 1300 // ms


#define SCALE2 1

#define TI(X) X >> SCALE2  // time delays (arduino -> real world)
#define TD(X) X << SCALE2  // time measures (real world -> arduino)

#define FPS 20
#define KP 12.0
#define KI 0.1
#define KD -75.0

#define KPHASE 500.0


enum State {
  stopped,
  starting,
  syncing,
  tracking,
  stopping,
};

State state = stopped;
double last_speed = 0;
unsigned long last_pid_update = 0;
unsigned long last_state_change = 0;
bool fresh_state_change = true;
volatile unsigned long vsync_last_trigger = 0;
volatile unsigned long vsync_last_last_trigger = 0;
volatile unsigned long hall_last_trigger = 0;
volatile unsigned long hall_dt_micros;

volatile double hall_d_err = 0;
volatile double hall_last_err = 0;
unsigned long hall_last_big_d = 0;

double sp = 1.0 / FPS * 1000;  // millis

unsigned long update_interval = sp / 10;  // 10x dead time

double err_sum = 0;
double last_err = 0;
double last_output = 0;

typedef struct {
  unsigned long last_update;
  double kp;
  double ki;
  double kd;
  double err_acc;
  double last_err;
} pid;

void vsync_trigger() {
  vsync_last_last_trigger = vsync_last_trigger;
  vsync_last_trigger = micros();
}


void hall_trigger() {
  unsigned long now = micros();
  hall_dt_micros = now - hall_last_trigger;
  hall_last_trigger = now;

  double input = TD(hall_dt_micros / 1000);  // to millis
  double err = input - sp;
  hall_d_err = (err - hall_last_err) / input;
  hall_last_err = err;

//  hall_unprocessed += 1;
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
  bool completed = (*task)(TD(now - last_state_change), fresh_state_change);
  if (fresh_state_change) fresh_state_change = false;
  if (completed & next_state != -1) {
    transition(next_state);
  }
  return completed;
}

//bool run_task(bool (*task)(unsigned long, bool), State next_state) {
//  bool completed = run_task(task);
//  if (completed) {
//  }
//  return completed;
//}

bool drive(long kspeed, bool reverse = false) {
  bool in_range = true;
  if (kspeed == 0) {
    digitalWrite(DIR, false);  // slightly defensive: set direction to forward
    analogWrite(PWM, 0);
  } else {
    digitalWrite(DIR, reverse);
    if (kspeed > 1000) {
      kspeed = 1000;
      in_range = false;
    }
    analogWrite(PWM, map(kspeed, 1, 1000, MIN_SPEED, MAX_SPEED));
  }
  return in_range;
}

void setup() {
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ADJ, INPUT);
  pinMode(VSYNC, INPUT);
  pinMode(HALL, INPUT);
  pinMode(GO, INPUT);
  // input pull-ups
  digitalWrite(VSYNC, HIGH);
  digitalWrite(HALL, HIGH);
  digitalWrite(GO, HIGH);

  // run arduino at half-speed (brings PWM into range for the controller)
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  CLKPR = (1<<CLKPCE);
  CLKPR = SCALE2;

  attachInterrupt(digitalPinToInterrupt(VSYNC), vsync_trigger, FALLING);  // active-low
  attachInterrupt(digitalPinToInterrupt(HALL), hall_trigger, FALLING);  // active-low

  Serial.begin(TD(115200));
  while (!Serial);
  Serial.println("setup complete.");
  transition(stopped);
}

void loop() {
  bool go = digitalRead(GO) == LOW;
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
      run_task(&start, syncing);
    }
    break;
  case syncing:
    if (!go) {
      transition(stopping);
    } else {
      run_task(&sync, tracking);
      run_task(&stall_check, stopped);
    }
    break;
  case tracking:
    if (!go) {
      transition(stopping);
    } else {
      run_task(&track, syncing);
      run_task(&stall_check, stopped);
    }
    break;
  case stopping:
    run_task(&gentle_stop, stopped);
    break;
  }
}

bool stop_task(unsigned long t, bool init) {
  if (init) drive(0);
  return true;
}

bool start(unsigned long t, bool init) {
  if (t < 60) {
    drive(map(t, 0, 60, 0, 200), true);
  } else if (t < 120) {
    drive(map(t, 60, 120, 200, 0), true);
  } else if (t < 160) {
    drive(0);
  } else if (t < 240) {
    drive(map(t, 160, 240, 0, 600));
  } else if (t < 400) {
    drive(map(t, 240, 400, 500, 333));
  } else {
    return true;
  }
  return false;
}

bool sync(unsigned long t, bool init) {
  // spin up or down to match frame rate (error from fps)
  if (init) {
    Serial.println("INIT SYNC");
    drive(200);
    last_err = 0;
    err_sum = 0;
    last_pid_update = t;
    hall_last_big_d = t;
//    hall_d_err = 0;
//    hall_last_err 0;
//    hall_unprocessed = 0;
    return false;
  }
  if (t - last_pid_update >= update_interval) {
    double dt = t - last_pid_update;
    double input = TD(hall_dt_micros / 1000);  // to millis
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
    Serial.print(KD * hall_d_err);
    double output = KP * err + KI * curr_err_sum + KD * hall_d_err;
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
    if (abs(hall_d_err) > STABLE_D_THRESH) {
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
  unsigned long dt_last = TD((now_raw_micros - last_hall_last_trigger) / 1000);
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
//  return (t > 300) && dt_last > sp * 8;  // we've missed 8 revs
}


uint8_t err_count = 0;

bool track(unsigned long t, bool init) {
  // phase-lock
  if (init) {
    err_count = 0;
  }
  if (t - last_pid_update >= update_interval) {
    double target_phase = (t % (unsigned int)sp) / sp;
    unsigned long measured_t = micros() - hall_last_trigger;
    double measured_phase = (measured_t % hall_dt_micros) / (double)hall_dt_micros;

    double phase_error = target_phase - measured_phase;
    if (phase_error > 0.5) {
      phase_error -= 1;
    } else if (phase_error < -0.5) {
      phase_error += 1;
    }

    drive(last_output + KPHASE * phase_error);

    Serial.print(target_phase);
    Serial.print("\t");
    Serial.print(measured_phase);
    Serial.print("\t");
    Serial.print(phase_error);
    Serial.print("\t");
    Serial.print(KPHASE * phase_error); 

    last_pid_update = t;
    
    double input = TD(hall_dt_micros / 1000);  // to millis
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

    Serial.println();
  }
  return false;
}

bool gentle_stop(unsigned long t, bool init) {
  if (t > MIN_SPEED) {
    return true;
  }
  drive(map(t, 0, MIN_SPEED, MIN_SPEED, 0));
  return false;
}

//void update_speed() {
//  unsigned long now = millis();
//  if (dirty) {
//    dirty = false;
//
//    double input = hall_dt_micros << SCALE2;
//    double err = sp - input;
//
//    Serial.print(1.0 / (input / 1000000));
//    Serial.print("\t");
//    Serial.print(err);
//
//    err_sum += err * input / 1000000;
//    if (err_sum > sp) {
//      err_sum = sp;
//    } else if (-err_sum > sp) {
//      err_sum = -sp;
//    }
//    double d_err = (err - last_err) / input / 1000000;
//    last_err = err;
//
//    double next_speed = KP * err + KI * err_sum + KD * d_err;
//
//    if (next_speed < 0) {
//      next_speed = 0;
//    }
//
//    analogWrite(PWM, scale_speed(next_speed));
//
//    Serial.print("\t");
//    Serial.print(err_sum);
//    Serial.print("\t");
//    Serial.print(d_err);
//    Serial.print("\t");
//    Serial.println(scale_speed(next_speed));
//
//    last_speed = next_speed;
//    last_update = now;
//  } else if (now - last_update > STOPPED) {
//    analogWrite(PWM, MAX_SPEED);
//  }
//}



// MARK: CONST
// ------------------------------------------------------------------------

// Serial
// ------------------------------------------------------------------------
constexpr uint32_t BAUD_RATE         = 9600;
constexpr size_t SERIAL_BUF_SIZE     = 64;
constexpr uint32_t FEEDBACK_INTERVAL = 50; // [ms]
constexpr uint8_t FEEDBACK_PRECISION = 6;  // [decimals]
constexpr char TERMINATOR            = '\n';

// Drive
// ------------------------------------------------------------------------
#define F_DRIVE_ENABLED 0

constexpr uint8_t DRIVE_FWD = 4;
constexpr uint8_t DRIVE_REV = 5;
// Needs to be PWM pin, UNO [D3, D5, D6, D9, D10, D11],
// MEGA [D3, D5, D6, D9, D10, D11, D13, D44, D45, D46, D47, D48, D49, D50]
constexpr uint8_t DRIVE_SPD = 6;

constexpr float MAX_LINEAR_VELOCITY = 2.3;  // [m/s] ~5 mph
constexpr float MAX_MOTOR_RPM       = 1000; // TODO: set to real value
// clamp motor speed
constexpr uint8_t MAX_PWM = 255; // [0-255]
constexpr uint8_t MIN_PWM = 2;   // [0-255]

#define F_HALL_ENABLED 0

// Needs to be interrupt pin, UNO [D2, D3], MEGA [D2, D3, D18, D19, D20, D21]
constexpr uint8_t HALL_PIN        = 18;
constexpr float WHEEL_RADIUS      = 0.1016; // [m] ~4 in
constexpr uint8_t MAGNETS_PER_REV = 1;
// ratio of motor shaft to wheel shaft
// TODO: set to real value
constexpr float DIFFERENTIAL_GEAR_RATIO = 1;
constexpr uint32_t SPEED_CALC_INTERVAL  = 100; // [ms]

// Steering
// ------------------------------------------------------------------------
#define F_STEER_ENABLED 1

// PID constants
constexpr float KP = 1;
constexpr float KD = 0.025;
constexpr float KI = 0.0;

constexpr uint8_t STEER_FWD = 7;
constexpr uint8_t STEER_REV = 6;
// Needs to be PWM pin, UNO [D3, D5, D6, D9, D10, D11],
// MEGA [D3, D5, D6, D9, D10, D11, D13, D44, D45, D46, D47, D48, D49, D50]
constexpr uint8_t STEER_SPD = 5;

constexpr float PINION_CMD_TOLERANCE = 0.5; // [rad] TODO: set to real value

// Needs to be interrupt pins, UNO [D2, D3], MEGA [D2, D3, D18, D19, D20, D21]
constexpr uint8_t ENCODER_A = 2; // YELLOW
constexpr uint8_t ENCODER_B = 3; // WHITE

constexpr uint8_t LIMIT_SWITCH = 13;

// LIGHTS
// ------------------------------------------------------------------------
#define F_LIGHTS_ENABLED 0

constexpr uint8_t LIGHTS          = 4;
constexpr uint32_t FLASH_INTERVAL = 1000; // [ms]

// MARK: STATE
// ------------------------------------------------------------------------

bool f_in_autonomous_mode = true;

// Command values from hardware interface
float g_pinion_cmd   = 0.0; // [rad] target angle
float g_velocity_cmd = 0.0; // [rad/s] target speed

// State values for hardware interface
float g_pinion_state   = 0.0; // [rad] actual angle
float g_velocity_state = 0.0; // [rad/s] actual speed

// previous accepted pinion command
float g_prev_pinion_cmd = PINION_CMD_TOLERANCE * 2; // [rad]

uint32_t g_last_steer_time = 0;
float g_previous_error     = 0;
float g_error_integral     = 0;

volatile int32_t g_encoder_position  = 0;
volatile uint32_t g_hall_pulse_count = 0;
uint32_t g_last_speed_calc_time      = 0;

// for light flashing
uint32_t g_last_light_time = 0;
int g_lights_state         = LOW;

// Serial
uint32_t g_last_feedback_time = 0;
char g_serial_buffer[SERIAL_BUF_SIZE];
size_t g_serial_len = 0;

// MARK: SETUP
// ------------------------------------------------------------------------
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) {} // Wait for USB
  Serial.println("Serial Connection Initialized");

#if F_DRIVE_ENABLED
  pinMode(DRIVE_FWD, OUTPUT);
  pinMode(DRIVE_REV, OUTPUT);
  pinMode(DRIVE_SPD, OUTPUT);
#endif

#if F_HALL_ENABLED
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), read_hall, RISING);
  g_last_speed_calc_time = millis();
#endif

#if F_STEER_ENABLED
  pinMode(STEER_FWD, OUTPUT);
  pinMode(STEER_REV, OUTPUT);
  pinMode(STEER_SPD, OUTPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), read_encoder_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), read_encoder_B, RISING);

  pinMode(LIMIT_SWITCH, INPUT);

  zero_steering();
#endif

#if F_LIGHTS_ENABLED
  pinMode(LIGHTS, OUTPUT);
#endif
}

// MARK: LOOP
// ------------------------------------------------------------------------
void loop() {

  // State 0: Powered on, not activated
  // ------------------------------------------------------------------------
  if (!f_in_autonomous_mode) {
    if (F_LIGHTS_ENABLED) { digitalWrite(LIGHTS, HIGH); }

  } else {
    // State 1: Activated
    // ------------------------------------------------------------------------

#if F_LIGHTS_ENABLED
    flash_lights();
#endif

    // TODO: Wireless Estop?

    read_commands();

    // only send new cmd if it is distinct enough from old cmd
    if (abs(g_pinion_cmd - g_prev_pinion_cmd) > PINION_CMD_TOLERANCE) {
      g_prev_pinion_cmd = g_pinion_cmd;
      update_steering();
    }
    update_drive();

    send_feedback();
  }
}

void set_motor(int direction, int speed, int spd_pin, int fwd_pin, int rev_pin) {
  analogWrite(spd_pin, speed);
  digitalWrite(fwd_pin, (direction == 1) ? HIGH : LOW);
  digitalWrite(rev_pin, (direction == 1) ? LOW : HIGH);
}

// MARK: STEER
// ------------------------------------------------------------------------
void update_steering() {

  // TODO: Calculate taget from cmd
  int target = 1200;

  // time difference
  unsigned long current_time = micros();
  float delta_time           = ((float)(current_time - g_last_steer_time)) / 1.0e6;
  g_last_steer_time          = current_time;

  // read position
  noInterrupts();
  int position = g_encoder_position;
  interrupts();

  int error           = position - target;
  float derivative_dt = (error - g_previous_error) / delta_time;
  g_error_integral    = g_error_integral + error * delta_time;

  float u = KP * error + KD * derivative_dt + KI * g_error_integral;

  float pwr = fabs(u);
  if (pwr > MAX_PWM) { pwr = MAX_PWM; }
  int dir = (u > 0) ? 1 : -1;

  set_motor(dir, pwr, STEER_SPD, STEER_FWD, STEER_REV);

  g_previous_error = error;

  // TODO: calculate state from position
  g_pinion_state = g_pinion_cmd;
}

// TODO: most things online seem to say this is wrong
void read_encoder_A() {
  g_encoder_position += (digitalRead(ENCODER_A) > 0) ? 1 : -1;
}

void read_encoder_B() {
  g_encoder_position += (digitalRead(ENCODER_B) > 0) ? 1 : -1;
}

void zero_steering() {
  // TODO: zeroing logic
  // Serial.println("Steering zeroed");
}

// MARK: DRIVE
// ------------------------------------------------------------------------
void update_drive() {

  if (g_velocity_cmd > 0 && F_DRIVE_ENABLED) {

    int direction = 1;
    if (g_velocity_cmd < 0) {
      direction      = -1;
      g_velocity_cmd = -g_velocity_cmd;
    }

    if (g_velocity_cmd > MAX_LINEAR_VELOCITY) {
      g_velocity_cmd = MAX_LINEAR_VELOCITY;
    }

    // (linear velocity -> rad/s) of wheel shaft to motor shaft to RPM
    float rpm = (g_velocity_cmd / WHEEL_RADIUS) * DIFFERENTIAL_GEAR_RATIO *
                (60.0 / (2 * PI));

    // TODO: verify this is correct
    // map RPM to PWM
    int pwm = (int)((rpm / MAX_MOTOR_RPM) * MAX_PWM);

    pwm = constrain(pwm, MIN_PWM, MAX_PWM);

    set_motor(direction, pwm, DRIVE_SPD, DRIVE_FWD, DRIVE_REV);
  }

  if (F_HALL_ENABLED) {
    calculate_speed();
  } else {
    g_velocity_state = g_velocity_cmd;
  }
}

void read_hall() { g_hall_pulse_count++; }

void calculate_speed() {
  unsigned long current_time = millis();

  if (current_time - g_last_speed_calc_time >= SPEED_CALC_INTERVAL) {
    noInterrupts();
    unsigned long pulses = g_hall_pulse_count;
    g_hall_pulse_count   = 0;
    interrupts();

    float delta_time = (current_time - g_last_speed_calc_time) / 1000.0;

    // (# of pulses / pulses per revolution) / delta time in minutes
    float rpm = (pulses / (float)MAGNETS_PER_REV) / (delta_time / 60.0);

    // (rpm -> rad/s) of motor shaft to wheel shaft to linear velocity
    g_velocity_state =
        (rpm * (2 * PI / 60.0)) * DIFFERENTIAL_GEAR_RATIO * WHEEL_RADIUS;

    g_last_speed_calc_time = current_time;
  }
}

// MARK: CMD
// ------------------------------------------------------------------------
void read_commands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == TERMINATOR) {
      parse_command(g_serial_buffer);
      g_serial_len = 0; // clear buffer

    } else if (g_serial_buffer < SERIAL_BUF_SIZE - 1) {
      g_serial_buffer[g_serial_len++] = c; // accumulate char

    } else {
      // overflow, flush buffer
      g_serial_len = 0;
    }
  }
}

void parse_command(String command) {
  // TODO: some kind of way to activate or Estop ?

  // Expected Format "C:P:<pinion_angle>,V:<velocity>"
  int p = command.indexOf("C:P:");
  int v = command.indexOf(",V:");

  if (p == -1 || v == -1) { return; }

  g_pinion_cmd   = command.substring(p + 4, v).toFloat();
  g_velocity_cmd = command.substring(v + 3).toFloat();
}

// MARK: FEED
// ------------------------------------------------------------------------
void send_feedback() {

  unsigned long current_time = millis();
  if (current_time - g_last_feedback_time >= FEEDBACK_INTERVAL) {
    // Format: "F:P:<pinion_angle>,V:<velocity>\n"
    Serial.print("F:P:");
    Serial.print(g_pinion_state, FEEDBACK_PRECISION);
    Serial.print(",V:");
    Serial.print(g_velocity_state, FEEDBACK_PRECISION);
    Serial.print(TERMINATOR);

    g_last_feedback_time = current_time;
  }
}

// MARK: LIGHTS
// ------------------------------------------------------------------------
void flash_lights() {
  unsigned long current_time = millis();
  if (current_time - g_last_light_time >= FLASH_INTERVAL) {
    digitalWrite(LIGHTS, (g_lights_state == LOW) ? HIGH : LOW);
    g_last_light_time = current_time;
  }
}
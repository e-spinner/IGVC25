#include <PID_v1.h>
#include <LedControl.h>

// MARK: TOGGLE
// ------------------------------------ //
// toggles used to enable and disable   //
// functions of the code at compilation //
// ------------------------------------ //

// 1 = on; 0 = off

#define T_DRIVE 1
#define T_HALL 1
#define T_STEER 1
#define T_LIGHTS 1
#define T_GRID 1

// MARK: CONST
// --------------------------------- //
// constants used to configure setup //
// --------------------------------- //

// Serial
constexpr uint32_t BAUD_RATE         = 115200;
constexpr size_t SERIAL_BUF_SIZE     = 64;
constexpr uint32_t FEEDBACK_INTERVAL = 50; // [ms]
constexpr uint8_t FEEDBACK_PRECISION = 6;  // [decimals]
constexpr char TERMINATOR            = '\n';

#define SABERTOOTH_SERIAL Serial3

#if T_DRIVE
constexpr float MAX_MOTOR_RAD_S = 15.0; // TODO: set to real value
#endif

#if T_HALL
// Needs to be interrupt pin, UNO [D2, D3], MEGA [D2, D3, D18, D19, D20, D21]
#define PIN_HALL 21
constexpr float WHEEL_RADIUS  = 0.1016; // [m] ~4 in.
constexpr uint8_t MAG_PER_REV = 1; // TODO: get real num
constexpr float DIFF_RATIO    = 1; // TODO: ^^^
constexpr uint32_t SPEED_CALC_INTERVAL = 100; // [ms]
#endif

#if T_STEER
// Needs to be interrupt pins, UNO [D2, D3], MEGA [D2, D3, D18, D19, D20, D21]
#define PIN_ENCODER_A 2
#define PIN_ENCODER_B 3
#define PIN_LEFT_LIMIT 18
#define PIN_RIGHT_LIMIT 19

constexpr float PINION_CMD_TOLERANCE = 0.5; // TODO: set to real value [rad]
constexpr float ENCODER_COUNTS_PER_RAD = 800.0; // TODO: from real hardware

// PID Constants
constexpr double Kp = 1.5;
constexpr double Kd = 0.010;
constexpr double Ki = 0.3;

// TODO: Kevin said something about which this should be, idk?
constexpr int LIMIT_SWITCH_ACTIVE = LOW; // Using INPUT_PULLUP
#endif

#if T_LIGHTS
#define PIN_LIGHTS 4
constexpr uint32_t FLASH_INTERVAL = 1000; // [ms]
#endif

#if T_GRID
// MAX7219
#define PIN_LED_DIN 12
#define PIN_LED_CLK 11
#define PIN_LED_CS 10

constexpr float PINION_DEADBAND_RAD    = 0.075f;
constexpr float VELOCITY_DEADBAND_RPS  = 0.75f;
#endif



// MARK: STATE
// -------------------------- //
// globals used for live data //
// -------------------------- //

bool f_in_auto_mode = true; // TODO: from hardware interface

// ========================================= //
// Command interface from hardware interface //
float COMMAND_pinion   = 0.0; // [rad] target angle
float COMMAND_velocity = 0.0; // [rad/s] target motor speed?

// ===================================== //
// state interface to hardware interface //
float STATE_pinion   = 0.0; // [rad] actual angle
float STATE_velocity = 0.0; // [rad/s] target motor speed?

// Serial
uint32_t g_last_feedback_time = 0;
char g_serial_buffer[SERIAL_BUF_SIZE];
size_t g_serial_len = 0;

#if T_HALL
volatile uint32_t VOL_hall_pulse_count = 0;
uint32_t g_last_speed_calc_time       = 0;
#endif

#if T_STEER
float g_prev_cmd_pinion = PINION_CMD_TOLERANCE * 2; // [rad]

double PID_setpoint = 0;
double PID_input = 0;
double PID_output = 0;

PID positionPID(&PID_input, &PID_output, &PID_setpoint, Kp, Ki, Kd, DIRECT);

// Motion shaping
double PID_ramped_output = 0;
double PID_max_step = 2;
double PID_braking_zone = 100;

volatile uint32_t VOL_encoder_count = 0;

// Limit switches
volatile bool VOL_left_limit  = false;
volatile bool VOL_right_limit = false;
int32_t g_left_limit_count = 0;
int32_t g_right_limit_count = 0;
int32_t g_center_count = 0;
bool g_homing_complete = false;
#endif

#if T_LIGHTS
uint32_t g_last_light_time = 0;
int g_lights_state         = LOW;
#endif

#if T_GRID
LedControl g_led_matrix = LedControl(PIN_LED_DIN, PIN_LED_CLK, PIN_LED_CS, 1);
#endif



// MARK: SETUP
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) {} // Wait for USB -- NOTE: blocking
  Serial.println("Serial Connection Initialized");

#if T_DRIVE || T_STEER
  SABERTOOTH_SERIAL.begin(9600);
  delay(2000);
  SABERTOOTH_SERIAL.write(64);  // Stop S1
  SABERTOOTH_SERIAL.write(192); // Stop S2
#endif

#if T_HALL
  pinMode(PIN_HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hall_ISR, RISING);
  g_last_speed_calc_time = millis();
#endif

#if T_STEER
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  // pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoder_ISR, RISING);

  pinMode(PIN_LEFT_LIMIT, INPUT_PULLUP);
  pinMode(PIN_RIGHT_LIMIT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_LIMIT), left_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_LIMIT), right_ISR, CHANGE);

  positionPID.SetOutputLimits(-63,63);
  positionPID.SetMode(AUTOMATIC);

  zero_steering();
#endif

#if T_LIGHTS
  pinMode(PIN_LIGHTS, OUTPUT);
#endif

#if T_GRID
  g_led_matrix.shutdown(0, false);
  g_led_matrix.setIntensity(0, 6); // brightness [0..15]
  g_led_matrix.clearDisplay(0);
#endif
}



// MARK: LOOP
void loop() {

  if (!f_in_auto_mode) {
    // Powered on, not activated

#if T_LIGHTS
    digitalWrite(PIN_LIGHTS, HIGH);
#endif

  } else {
    // Powered on, Activated

#if T_LIGHTS
    flash_lights();
#endif

    read_commands();

#if T_DRIVE
    update_drive();
#endif

#if T_STEER
    update_steering();
#endif

#if T_GRID
    update_led_matrix();
#endif

    send_feedback();
  }
}



// MARK: DRIVE
// --------------------------------------- //
// function defs to control drive system   //
// and read hall sensor for speed feedback //
// --------------------------------------- //
#if T_DRIVE
void send_motor_S2(int speed) {
  // Sabertooth Simplified Serial: 192 is STOP, 128 is Full Reverse, 255 is Full Forward
  // speed arrives as -63 to 63
  int command = 192 + speed;
  command = constrain(command, 128, 255);
  SABERTOOTH_SERIAL.write(command);
}

void update_drive() {

  if (COMMAND_velocity == 0.0f) {
    send_motor_S2(0);
  } else {

    float velocity_req = COMMAND_velocity;
    velocity_req = constrain(velocity_req, -MAX_MOTOR_RAD_S, MAX_MOTOR_RAD_S);

    // map rad/s -> [-63, 63] (Requested / Max) * 63
    int sabertooth_speed = (int)((velocity_req / MAX_MOTOR_RAD_S) * 63.0);

    send_motor_S2(sabertooth_speed);
  }

#if T_HALL
  calculate_speed();
#else
  STATE_velocity = COMMAND_velocity;
#endif
}

#endif

#if T_HALL
void hall_ISR() { VOL_hall_pulse_count++; }

void calculate_speed() {
  uint32_t current_time = millis();

  if (current_time - g_last_speed_calc_time >= SPEED_CALC_INTERVAL) {
    noInterrupts();
    uint32_t pulses = VOL_hall_pulse_count;
    VOL_hall_pulse_count = 0;
    interrupts();

    float delta_time = (current_time - g_last_speed_calc_time) / 1000.0;

    // (# of pulses / pulses per revolution) / dt in minutes -> rpm
    float rpm = (pulses / (float)MAG_PER_REV) / (delta_time / 60.0);

    // (rpm -> rad/s of drive motor)
    STATE_velocity = rpm * (2 * PI / 60.0);

    g_last_speed_calc_time = current_time;
  }

}
#endif



// MARK: STEER
// --------------------------------------- //
// function defs to control steer system   //
// and zero system using limit switches    //
// --------------------------------------- //
#if T_STEER
void send_motor_S1(int speed) {
  // Sabertooth Simplified Serial: 64 is STOP, 1 is Full Reverse, 127 is Full Forward
  // speed arrives as -63 to 63
  int command = 64 + speed;
  command = constrain(command, 1, 127);
  SABERTOOTH_SERIAL.write(command);
}

void update_steering() {
  noInterrupts();
  PID_input = (double)VOL_encoder_count;
  interrupts();

  // TODO: pick percentage
  // We want to stop 5% before the physical switch
  float safe_range_counts = abs(g_right_limit_count - g_left_limit_count) * 0.95;
  float max_rad = (safe_range_counts / 2.0) / ENCODER_COUNTS_PER_RAD;
  float constrained_pinion = constrain(COMMAND_pinion, -max_rad, max_rad);

  // Mapping COMMAND_pinion (rad) to encoder ticks for the PID setpoint
  PID_setpoint = (constrained_pinion * ENCODER_COUNTS_PER_RAD) + g_center_count;

  positionPID.Compute();
  double error = PID_setpoint - PID_input;

  // Braking zone
  double scaled_output = PID_output;
  if (abs(error) < PID_braking_zone) {
    double scale = (double)abs(error) / PID_braking_zone;
    scaled_output *= scale;
  }

  // ramp for smooth motion
  if (scaled_output > PID_ramped_output) PID_ramped_output += PID_max_step;
  else if (scaled_output < PID_ramped_output) PID_ramped_output -= PID_max_step;
  PID_ramped_output = constrain(PID_ramped_output, -63, 63);

  // check limit --- bools are atomic
  if (VOL_left_limit && PID_ramped_output < 0) {
    // If moving left (-speed) while left limit is active: STOP
    PID_ramped_output = 0;
    send_motor_S1(0);
  }
  else if (VOL_right_limit && PID_ramped_output > 0) {
    // If moving right (+speed) while right limit is active: STOP
    PID_ramped_output = 0;
    send_motor_S1(0);
  }

  if (abs(error) < 3) {
    // stop near target
    PID_ramped_output = 0;
    send_motor_S1(0); // Sends 64
  } else {
    // send command to sabertooth
    send_motor_S1((int)PID_ramped_output);
  }

  // Update state for feedback
  STATE_pinion = (PID_input - g_center_count) / ENCODER_COUNTS_PER_RAD;
}

void zero_steering() {
  // Move Left slowly until PIN_LEFT_LIMIT is triggered
  while (digitalRead(PIN_LEFT_LIMIT) != LIMIT_SWITCH_ACTIVE) {
    send_motor_S1(-20);
    delay(10);
  }
  send_motor_S1(0);
  delay(500);

  noInterrupts();
  g_left_limit_count = VOL_encoder_count;
  interrupts();

  // Move Right slowly until PIN_RIGHT_LIMIT is triggered
  while (digitalRead(PIN_RIGHT_LIMIT) != LIMIT_SWITCH_ACTIVE) {
    send_motor_S1(20);
    delay(10);
  }
  send_motor_S1(0);
  delay(500);

  noInterrupts();
  g_right_limit_count = VOL_encoder_count;
  interrupts();

  // Calculate Center
  g_center_count = (g_left_limit_count + g_right_limit_count) / 2;

  // Move to Center
  COMMAND_pinion = 0.0; // Set target to center for the PID to take over
}

void encoder_ISR() {
  if (digitalRead(PIN_ENCODER_B))
    VOL_encoder_count++;
  else
    VOL_encoder_count--;
}

void left_ISR() { VOL_left_limit = (digitalRead(PIN_LEFT_LIMIT) == LIMIT_SWITCH_ACTIVE); }
void right_ISR() { VOL_right_limit = (digitalRead(PIN_RIGHT_LIMIT) == LIMIT_SWITCH_ACTIVE); }

#endif



// MARK: SERIAL
// ------------------------------ //
// function defs to handle serial //
// communication with ros HI      //
// ------------------------------ //

void read_commands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == TERMINATOR) {
      g_serial_buffer[g_serial_len] = '\0';
      parse_command(String(g_serial_buffer));
      g_serial_len = 0; // clear buffer

    } else if (g_serial_len < SERIAL_BUF_SIZE - 1) {
      g_serial_buffer[g_serial_len++] = c; // accumulate char

    } else {
      g_serial_len = 0; // overflow, flush buffer
    }
  }
}

// TODO: Some concern with arduino string robustness
void parse_command(String command) {
  // TODO: some kind of way to activate or deactivate the auto mode

  // Expected Format "C:P:<pinion_angle>,V:<velocity>"
  int p = command.indexOf("C:P:");
  int v = command.indexOf(",V:");

  if (p == -1 || v == -1) { return; }

  COMMAND_pinion   = command.substring(p + 4, v).toFloat();
  COMMAND_velocity = command.substring(v + 3).toFloat();
}

void send_feedback() {
  unsigned long current_time = millis();

  if (current_time - g_last_feedback_time >= FEEDBACK_INTERVAL) {
    // Format: "F:P:<pinion_angle>,V:<velocity>\n"
    Serial.print("F:P:");
    Serial.print(STATE_pinion, FEEDBACK_PRECISION);
    Serial.print(",V:");
    Serial.print(STATE_velocity, FEEDBACK_PRECISION);
    Serial.print(TERMINATOR);

    g_last_feedback_time = current_time;
  }
}



// MARK: LIGHTS
#if T_LIGHTS
void flash_lights() {
  unsigned long current_time = millis();
  if (current_time - g_last_light_time >= FLASH_INTERVAL) {
    digitalWrite(PIN_LIGHTS, (g_lights_state == LOW) ? HIGH : LOW);
    g_last_light_time = current_time;
  }
}
#endif



// MARK: GRID
#if T_GRID
void update_led_matrix() {
  int x_slot = 1; // 0:left, 1:center, 2:right (pinion)
  if (COMMAND_pinion > PINION_DEADBAND_RAD) { x_slot = 0;
  } else if (COMMAND_pinion < -PINION_DEADBAND_RAD) { x_slot = 2; }

  int y_slot = 1; // 0:top, 1:center, 2:bottom (velocity)
  if (COMMAND_velocity > VELOCITY_DEADBAND_RPS) { y_slot = 0;
  } else if (COMMAND_velocity < -VELOCITY_DEADBAND_RPS) { y_slot = 2; }

  // Positions: left/center/right( and top/middle/bottom
  const uint8_t col_starts[3] = {1, 3, 5};
  const uint8_t row_starts[3] = {1, 3, 5};

  draw_block_2x2(row_starts[y_slot], col_starts[x_slot]);
}

void draw_block_2x2(uint8_t row_start, uint8_t col_start) {
  g_led_matrix.clearDisplay(0);
  for (uint8_t r = 0; r < 2; ++r) {
    for (uint8_t c = 0; c < 2; ++c) {
      g_led_matrix.setLed(0, row_start + r, col_start + c, true);
    }
  }
}
#endif
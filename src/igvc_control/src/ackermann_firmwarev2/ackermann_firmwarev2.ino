#include <QuickPID.h>
#include <sTune.h>
#include <AltSoftSerial.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// MARK: TOGGLE
// ------------------------------------ //
// toggles used to enable and disable   //
// functions of the code at compilation //
// ------------------------------------ //v

// 1 = on; 0 = off

#define T_DRIVE 1
#define T_STEER 1
#define T_LIGHTS 1

#define T_LOG_PID 0
#define T_LOG_PIN 0
#define T_LOG_SPD 0

#define T_HALL 0



// MARK: CONST
// --------------------------------- //
// constants used to configure setup //
// --------------------------------- //

// Serial
constexpr uint32_t BAUD_RATE         = 115200;
constexpr size_t SERIAL_BUF_SIZE     = 64;
constexpr uint32_t FEEDBACK_INTERVAL = 50; // [ms]
constexpr uint8_t FEEDBACK_PRECISION = 1;  // [decimals]
constexpr char TERMINATOR            = '\n';

AltSoftSerial SABERTOOTH_SERIAL; // uses 8 [RX], 9 [TX], and steals pwm timer from 10

#if T_DRIVE
constexpr float MAX_MOTOR_RAD_S = 1500 * ((2 * PI) / 60); // 3000 rpm -> rad/s
constexpr float DIFF_RATIO    = 2.85; // according to sean
#endif

constexpr uint32_t MOTION_LOG_INTERVAL_MS = 250;

#if T_HALL
// Needs to be interrupt pin, UNO [D2, D3], MEGA [D2, D3, D18, D19, D20, D21]
#define PIN_HALL 2
constexpr float WHEEL_RADIUS  = 0.1016; // [m] ~4 in.
constexpr uint8_t MAG_PER_REV = 1; // according to kevin
constexpr uint32_t SPEED_CALC_INTERVAL = 500; // [ms]
#endif

#if T_STEER
Encoder steer_enc(3, 2);

#define PIN_LEFT_LIMIT 5
#define PIN_RIGHT_LIMIT 6

constexpr float ENCODER_COUNTS_PER_RAD = 600 / (2 * PI); // according to kevin
constexpr float PINION_CMD_TOLERANCE = ENCODER_COUNTS_PER_RAD; // just setting it to encoder accuracy

constexpr int LIMIT_SWITCH_ACTIVE = LOW; // Using INPUT_PULLUP
#endif

#if T_LIGHTS
#define PIN_LIGHTS 7
constexpr uint32_t FLASH_INTERVAL = 250; // [ms]
#endif


// MARK: STATE
// -------------------------- //
// globals used for live data //
// -------------------------- //

bool f_in_auto_mode = true; // TO3DO: from hardware interface

// ========================================= //
// Command interface from hardware interface //
float COMMAND_pinion   = 0.0; // [rad] target angle
float COMMAND_velocity = 0.0; // [rad/s] target motor speed

// ===================================== //
// state interface to hardware interface //
float STATE_pinion   = 0.0; // [rad] actual angle
float STATE_velocity = 0.0; // [rad/s] actual motor speed

// Serial
uint32_t g_last_feedback_time = 0;
char g_serial_buffer[SERIAL_BUF_SIZE];
size_t g_serial_len = 0;

uint32_t g_last_motion_log = 0;

#if T_HALL
volatile uint32_t VOL_hall_pulse_count = 0;
uint32_t g_last_speed_calc_time       = 0;
#endif

#if T_STEER
float g_prev_cmd_pinion = PINION_CMD_TOLERANCE * 2; // [rad]


// PID Constants
float Kp = 0.3, Ki = 0.0, Kd = 0.015;
float PID_setpoint = 0, PID_input = 0, PID_output = 0;

QuickPID positionPID(&PID_input, &PID_output, &PID_setpoint, Kp, Ki, Kd, QuickPID::Action::direct);

// Motion shaping
double PID_ramped_output = 0;
double PID_max_step = 2;
double PID_braking_zone = 100;

// Limit switches
volatile bool f_left_limit  = false;
volatile bool f_right_limit = false;
int32_t g_left_limit_count = 0;
int32_t g_right_limit_count = 0;
int32_t g_center_count = 0;
bool g_homing_complete = false;
#endif

#if T_LIGHTS
uint32_t g_last_light_time = 0;
int g_lights_state         = LOW;
#endif



// MARK: SETUP
void setup() {


#if T_LIGHTS
  pinMode(PIN_LIGHTS, OUTPUT);
  digitalWrite(PIN_LIGHTS, HIGH);
#endif

  Serial.begin(BAUD_RATE);
  while (!Serial) {} // Wait for USB -- NOTE: blocking
  Serial.println("Serial Connection Initialized");

#if T_DRIVE || T_STEER
  SABERTOOTH_SERIAL.begin(9600);
  delay(2000);
  SABERTOOTH_SERIAL.write((uint8_t)0);
#endif

#if T_HALL
  pinMode(PIN_HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hall_ISR, RISING);
  g_last_speed_calc_time = millis();
#endif

#if T_STEER
  pinMode(PIN_LEFT_LIMIT, INPUT_PULLUP);
  pinMode(PIN_RIGHT_LIMIT, INPUT_PULLUP);

  positionPID.SetOutputLimits(-63, 63);
  positionPID.SetSampleTimeUs(10000); // 10ms sample time
  positionPID.SetMode(QuickPID::Control::automatic);
  zero_steering();

#endif

  Serial.println("setup complete");
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

    send_feedback();
  }
}



// MARK: DRIVE
// --------------------------------------- //
// function defs to control drive system   //
// and read hall sensor for speed feedback //
// --------------------------------------- //
#if T_DRIVE
int old_drive_spd = 100;
void send_motor_S2(int speed) {
  // Sabertooth Simplified Serial: 192 is STOP, 128 is Full Reverse, 255 is Full Forward
  // speed arrives as -63 to 63
  // -1 because for some reason that is stop
  if (speed != old_drive_spd) {
#if T_LOG_SPD
    Serial.println(speed);
#endif
    int command = 192 + speed -1;
    command = constrain(command, 128, 255);
    SABERTOOTH_SERIAL.write(command);
    old_drive_spd = command;
  }
}

constexpr float VELOCITY_CALC = (DIFF_RATIO / MAX_MOTOR_RAD_S) * 63.0;
void update_drive() {
  if (COMMAND_velocity == 0.0f) {
    send_motor_S2(0);
  } else {

    float velocity_req = COMMAND_velocity;
    velocity_req = constrain(velocity_req, -MAX_MOTOR_RAD_S, MAX_MOTOR_RAD_S);

    // map rad/s -> [-63, 63] (Requested * GR / Max) * 63
    int sabertooth_speed = (int)(velocity_req * VELOCITY_CALC);

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

constexpr float PULSE_TO_RADS = (2 * PI / 60.0);
void calculate_speed() {
  uint32_t current_time = millis();

  if (current_time - g_last_speed_calc_time >= SPEED_CALC_INTERVAL) {
    noInterrupts();
    uint32_t pulses = VOL_hall_pulse_count;
    VOL_hall_pulse_count = 0;
    interrupts();

    float delta_time = (current_time - g_last_speed_calc_time) / 1000.0;

    // (# of pulses / pulses per revolution) / dt in minutes -> rpm
    float rpm = pulses / ((float)MAG_PER_REV) / (delta_time / 60.0);

    // (rpm -> rad/s of drive motor)
    STATE_velocity = rpm * PULSE_TO_RADS;

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
int old_steer_spd = 100;
void send_motor_S1(int speed) {
  // Sabertooth Simplified Serial: 64 is STOP, 1 is Full Reverse, 127 is Full Forward
  // speed arrives as -63 to 63
  if (speed != old_steer_spd) {
    int command = 64 + (speed * -1);
    command = constrain(command, 1, 127);
    SABERTOOTH_SERIAL.write(command);
    old_steer_spd = command;
  }
}

void update_steering() {
  // Serial.println("update_steering");
  f_left_limit  = (digitalRead(PIN_LEFT_LIMIT) == LIMIT_SWITCH_ACTIVE);
  f_right_limit = (digitalRead(PIN_RIGHT_LIMIT) == LIMIT_SWITCH_ACTIVE);

  PID_input = (double)steer_enc.read();

  // We want to stop 5% before the physical switch
  float safe_range_counts = abs(g_right_limit_count - g_left_limit_count) * 0.95;
  float max_rad = (safe_range_counts / 2.0) / ENCODER_COUNTS_PER_RAD;
  float constrained_pinion = constrain(COMMAND_pinion, -max_rad, max_rad);

  // Mapping COMMAND_pinion (rad) to encoder ticks for the PID setpoint
  PID_setpoint = (constrained_pinion * ENCODER_COUNTS_PER_RAD) + g_center_count;

#if T_LOG_PIN
  Serial.print("cmd: ");
  Serial.print(COMMAND_pinion);
  Serial.print(", con: ");
  Serial.print(constrained_pinion);
  Serial.print(", set: ");
  Serial.println(PID_setpoint);
#endif

  // true if calculation was made
  if (positionPID.Compute()) {
    double error = PID_setpoint - PID_input;

    // Braking zone
    // double scaled_output = PID_output;
    // if (abs(error) < PID_braking_zone) {
    //   double scale = (double)abs(error) / PID_braking_zone;
    //   scaled_output *= scale;
    // }

    // // ramp for smooth motion
    // if (scaled_output > PID_ramped_output) PID_ramped_output += PID_max_step;
    // else if (scaled_output < PID_ramped_output) PID_ramped_output -= PID_max_step;
    // PID_ramped_output = constrain(PID_ramped_output, -63, 63);

    PID_ramped_output =  constrain(PID_output, -63, 63);

    // check limit --- bools are atomic
    if (f_left_limit && PID_ramped_output < 0) {
      // If moving left (-speed) while left limit is active: STOP
      PID_ramped_output = 0;
      send_motor_S1(0);
    }
    else if (f_right_limit && PID_ramped_output > 0) {
      // If moving right (+speed) while right limit is active: STOP
      PID_ramped_output = 0;
      send_motor_S1(0);
    }

    if (abs(error) < 30) {
      // stop near target
      PID_ramped_output = 0;
      send_motor_S1(0); // Sends 64
    } else {
      // send command to sabertooth
      send_motor_S1((int)PID_ramped_output);
    }
  }

  // Update state for feedback
  STATE_pinion = (PID_input - g_center_count) / ENCODER_COUNTS_PER_RAD;

#if T_LOG_PID
  if (abs((int)PID_ramped_output) > 0) {
    uint32_t now = millis();
    if (now - g_last_motion_log >= MOTION_LOG_INTERVAL_MS) {
      long cnt = (long)PID_input;
      long set = (long)PID_setpoint;
      long err = (long)(set - cnt);
      Serial.print("[ENC] cnt=");
      Serial.print(cnt);
      Serial.print(" set=");
      Serial.print(set);
      Serial.print(" err=");
      Serial.print(err);
      Serial.print(" out=");
      Serial.println((int)PID_ramped_output);
      g_last_motion_log = now;
    }
  }
#endif
}



// MARK: ZERO
void zero_steering() {
  Serial.println("zero_steering begin");

  // Move Left slowly until PIN_LEFT_LIMIT is triggered
  while (digitalRead(PIN_LEFT_LIMIT) != LIMIT_SWITCH_ACTIVE) {
    send_motor_S1(-10);
    delay(10);
  }
  send_motor_S1(0);
  delay(500);

  g_left_limit_count = 0;
  steer_enc.write(0);

  Serial.print("Left Lim: ");
  Serial.println(g_left_limit_count);

  // Move Right slowly until PIN_RIGHT_LIMIT is triggered
  while (digitalRead(PIN_RIGHT_LIMIT) != LIMIT_SWITCH_ACTIVE) {
    send_motor_S1(10);
    delay(10);
  }
  send_motor_S1(0);
  delay(500);

  g_right_limit_count = steer_enc.read();


  Serial.print("Right Lim: ");
  Serial.println(g_right_limit_count);

  // Calculate Center
  g_center_count = g_right_limit_count / 2;

  Serial.print("Center: ");
  Serial.println(g_center_count);

  // Move to Center
  // Serial.println("Moving to center...");
  // long target = g_center_count;
  // while (abs(steer_enc.read() - target) > 50) {
  //   long err = target - steer_enc.read();
  //   int spd = constrain((int)(err / 10), -20, 20);
  //   if (spd == 0) spd = (err > 0) ? 1 : -1; // minimum nudge
  //   send_motor_S1(spd);
  //   delay(10);
  // }
  // send_motor_S1(0);
  // delay(300); // let it settle
  // Serial.println("At center.");

  COMMAND_pinion = 0.0; // Set target to center for the PID to take over
  Serial.println("zero_steering complete");
}

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
      parse_command(g_serial_buffer);
      g_serial_len = 0; // clear buffer

    } else if (g_serial_len < SERIAL_BUF_SIZE - 1) {
      g_serial_buffer[g_serial_len++] = c; // accumulate char

    } else {
      g_serial_len = 0; // overflow, flush buffer
    }
  }
}

void parse_command(char* buf) {
  // Expected Format "C:P:<pinion_angle>,V:<velocity>"
  char* p_ptr = strstr(buf, "C:P:");
  char* v_ptr = strstr(buf, ",V:");

  if (p_ptr == NULL || v_ptr == NULL) { return; }

  COMMAND_pinion   = atof(p_ptr + 4);
  COMMAND_velocity = atof(v_ptr + 3);
}

void send_feedback() {
  unsigned long current_time = millis();

  if (current_time - g_last_feedback_time >= FEEDBACK_INTERVAL) {
    // Format: "F:P:<pinion_angle>,V:<velocity>\n"
    Serial.print(F("F:P:"));
    Serial.print(STATE_pinion, FEEDBACK_PRECISION);
    Serial.print(F(",V:"));
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
    digitalWrite(PIN_LIGHTS, g_lights_state = (g_lights_state == LOW) ? HIGH : LOW);
    g_last_light_time = current_time;
  }
}
#endif
// MARK: CONST
// ------------------------------------------------------------------------

// Serial
// ------------------------------------------------------------------------
const int BAUD_RATE                      = 115200;
const unsigned long FEEDBACK_INTERVAL_MS = 50; // [ms]

// decimals in feedback values
const int FEEDBACK_PRECISION = 6;
const char TERMINATOR        = '/n';

const double PINION_CMD_TOLERANCE = 0.5; // [rad] TODO: set to real value

// Drive
// ------------------------------------------------------------------------

// Pin definitions
const int DRIVE_FWD   = 4;
const int DRIVE_REV   = 5;
const int DRIVE_SPEED = 6;

const double MAX_LINEAR_VELOCITY = 2.3; // [m/s] ~5 mph

// TODO: Hall Sensor

// Steering
// ------------------------------------------------------------------------

// TODO: Pin definitions

// TODO: Encoder

// MARK: STATE
// ------------------------------------------------------------------------

bool f_in_autonomous_mode;

// Command values from hardware interface
double g_pinion_cmd   = 0.0; // [rad] target angle
double g_velocity_cmd = 0.0; // [rad/s] target speed

// State values for hardware interface
double g_pinion_state   = 0.0; // [rad] actual angle
double g_velocity_state = 0.0; // [rad/s] actual speed

// previous accepted pinion command
double g_prev_pinion_cmd = PINION_CMD_TOLERANCE * 2; // [rad]

// Serial
unsigned long g_last_feedback_time = 0;
String g_serial_buffer             = "";

// MARK: SETUP
// ------------------------------------------------------------------------
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) {} // Wait for USB

  zero_steering();
}

// MARK: LOOP
// ------------------------------------------------------------------------
void loop() {

  // State 0: Powered on, not activated
  // ------------------------------------------------------------------------
  if (!f_in_autonomous_mode) {
    // TODO: lights need to be solid
  } else {

    // TODO: lights need to be blinking

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

// STEER
// ------------------------------------------------------------------------
void update_steering() {
  // TODO: actual steering control
  g_pinion_state = g_pinion_cmd;
}

void zero_steering() {
  // TODO: zeroing logic
}

// DRIVE
// ------------------------------------------------------------------------
void update_drive() {
  // TODO: actual drive control
  g_velocity_state = g_velocity_cmd;
}

// MARK: CMD
// ------------------------------------------------------------------------
void read_commands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == TERMINATOR) {
      parse_command(g_serial_buffer);
      g_serial_buffer = ""; // clear buffer
    } else {
      g_serial_buffer += c; // accumulate char
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
  if (current_time - g_last_feedback_time >= FEEDBACK_INTERVAL_MS) {
    // Format: "F:P:<pinion_angle>,V:<velocity>\n"
    Serial.print("F:P:");
    Serial.print(g_pinion_state, FEEDBACK_PRECISION);
    Serial.print(",V:");
    Serial.print(g_velocity_state, FEEDBACK_PRECISION);
    Serial.print(TERMINATOR);

    g_last_feedback_time = current_time;
  }
}
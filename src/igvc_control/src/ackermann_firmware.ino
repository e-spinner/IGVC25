#include <Stepper.h>

// MARK: CONFIG
// -----------------------------------------------------------------------------

// TODO: setup sensor feedback

// Stepper motor configuration (28BYJ-48 with ULN2003)
const int STEPS_PER_REVOLUTION = 2048; // Steps per full revolution (28BYJ-48)
const int STEPPER_SPEED = 12; // Stepper motor speed (RPM) - reduced for reliability
const double PINION_GEAR_RATIO = 1.652;   // Pinion gear ratio
const double PINION_RADIUS     = 0.01905; // Pinion radius in meters

// Pin definitions (28BYJ-48 ULN2003 driver)
// -----------------------------------------------------------------------------
const int STEPPER_PIN1 = 8;  // IN1
const int STEPPER_PIN2 = 9;  // IN2
const int STEPPER_PIN3 = 10; // IN3
const int STEPPER_PIN4 = 11; // IN4

// DC motor pin definitions
const int MOTOR_DIR1      = 4; // Forward direction pin
const int MOTOR_DIR2      = 5; // Backward direction pin
const int MOTOR_SPEED_PIN = 6; // PWM speed control pin

// Motor control parameters
// -----------------------------------------------------------------------------
const int MAX_PWM         = 255;    // Maximum PWM value
const double WHEEL_RADIUS = 0.1524; // Wheel radius in meters

// Motor control parameters
const double MAX_LINEAR_VELOCITY = 2.0; // Maximum linear velocity in m/s

// Minimum step threshold to prevent tiny movements that cause vibration
const int MIN_STEPS_TO_MOVE = 10; // Only move if at least 10 steps difference

// Feedback update rate
const unsigned long FEEDBACK_INTERVAL_MS = 50; // Send feedback every 50ms

// MARK: OBJECTS
// -----------------------------------------------------------------------------

// Stepper motor object (28BYJ-48 pin order: IN1, IN3, IN2, IN4)
Stepper myStepper(STEPS_PER_REVOLUTION, STEPPER_PIN1, STEPPER_PIN3, STEPPER_PIN2,
                  STEPPER_PIN4);

// MARK: STATE
// -----------------------------------------------------------------------------

// Command values (received from hardware interface)
double m_pinion_angle_cmd = 0.0; // Commanded pinion angle in radians
double m_velocity_cmd     = 0.0; // Commanded velocity in rad/s

// Actual state (calculated from position/velocity)
long m_current_steps         = 0;   // Current absolute stepper position in steps
double m_pinion_angle_actual = 0.0; // Actual pinion angle in radians
double m_velocity_actual     = 0.0; // Actual velocity in rad/s

// Motor control state
int m_current_pwm = 0; // Current PWM value (0-255)

// Timing
unsigned long m_last_feedback_time = 0; // Last time feedback was sent

// Serial communication buffer
String m_serial_buffer        = "";
const char COMMAND_TERMINATOR = '\n';

// MARK: SETUP
// -----------------------------------------------------------------------------
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect (only needed for USB)
  }

  // Initialize stepper motor
  myStepper.setSpeed(STEPPER_SPEED);

  // Initialize DC motor pins
  pinMode(MOTOR_DIR1, OUTPUT);
  pinMode(MOTOR_DIR2, OUTPUT);
  pinMode(MOTOR_SPEED_PIN, OUTPUT);

  // Stop motors initially
  digitalWrite(MOTOR_DIR1, LOW);
  digitalWrite(MOTOR_DIR2, LOW);
  analogWrite(MOTOR_SPEED_PIN, 0);

  // Initialize state
  m_current_steps       = 0;
  m_pinion_angle_actual = 0.0;
  m_velocity_actual     = 0.0;
  m_last_feedback_time  = millis();

  Serial.println("Arduino Ackermann Steering Ready");
  Serial.print("Stepper: ");
  Serial.print(STEPS_PER_REVOLUTION);
  Serial.print(" steps/rev, ");
  Serial.print(STEPPER_SPEED);
  Serial.println(" RPM");
}

// MARK: LOOP
// -----------------------------------------------------------------------------
void loop() {
  // Read serial commands
  readSerialCommands();

  // Update motor positions based on current commands
  updateSteering();
  updateDriveMotor();

  // Send feedback periodically
  sendFeedback();
}

// MARK: SERIAL
// -----------------------------------------------------------------------------
void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == COMMAND_TERMINATOR) {
      // Process complete command
      parseCommand(m_serial_buffer);
      m_serial_buffer = ""; // Clear buffer
    } else {
      m_serial_buffer += c; // Add character to buffer
    }
  }
}

void parseCommand(String command) {
  // Expected format: "P:<pinion_angle>,V:<velocity>"
  // Example: "P:0.123456,V:1.234567"

  int pIndex = command.indexOf('P:');
  int vIndex = command.indexOf(",V:");

  if (pIndex == -1 || vIndex == -1) {
    // Invalid command format
    return;
  }

  // Extract pinion angle (radians)
  String pinionStr   = command.substring(pIndex + 2, vIndex);
  m_pinion_angle_cmd = pinionStr.toFloat();

  // Extract velocity (rad/s)
  String velocityStr = command.substring(vIndex + 3);
  m_velocity_cmd     = velocityStr.toFloat();
}

// MARK: STEERING
// -----------------------------------------------------------------------------
void updateSteering() {
  // Stepper is directly connected to pinion, so convert pinion angle directly to
  // steps Steps = (pinion_angle_rad / 2*PI) * STEPS_PER_REVOLUTION

  int target_steps =
      (int)((m_pinion_angle_cmd / (2.0 * 3.14159265359)) * STEPS_PER_REVOLUTION);

  // Calculate steps to move (relative to current position)
  int steps_to_move = target_steps - m_current_steps;

  // Only move if the difference is significant (prevents vibration)
  if (abs(steps_to_move) >= MIN_STEPS_TO_MOVE) {
    // Debug output (comment out if too verbose)
    static unsigned long last_debug = 0;
    if (millis() - last_debug > 500) { // Print every 500ms
      Serial.print("Stepper: angle=");
      Serial.print(m_pinion_angle_cmd, 4);
      Serial.print(" rad, steps=");
      Serial.print(steps_to_move);
      Serial.print(", target=");
      Serial.print(target_steps);
      Serial.print(", current=");
      Serial.println(m_current_steps);
      last_debug = millis();
    }

    // Move stepper motor
    myStepper.step(steps_to_move);
    m_current_steps += steps_to_move; // Update absolute position
  }

  // Calculate actual pinion angle from current position
  // Reverse: steps -> pinion angle
  m_pinion_angle_actual =
      ((double)m_current_steps / STEPS_PER_REVOLUTION) * (2.0 * 3.14159265359);
}

// MARK: DRIVE
// -----------------------------------------------------------------------------
void updateDriveMotor() {
  // Convert velocity (rad/s) to motor PWM and direction
  // Velocity in rad/s -> linear velocity (m/s) -> motor PWM

  // Linear velocity = angular velocity * wheel radius
  double linear_velocity = m_velocity_cmd * WHEEL_RADIUS;

  // Determine direction
  bool forward        = linear_velocity >= 0;
  double abs_velocity = abs(linear_velocity);

  // Convert to PWM (0-255)
  int pwm_value = (int)((abs_velocity / MAX_LINEAR_VELOCITY) * MAX_PWM);
  pwm_value     = constrain(pwm_value, 0, MAX_PWM);
  m_current_pwm = pwm_value; // Store for feedback

  // Set motor direction
  if (forward) {
    digitalWrite(MOTOR_DIR1, HIGH);
    digitalWrite(MOTOR_DIR2, LOW);
  } else {
    digitalWrite(MOTOR_DIR1, LOW);
    digitalWrite(MOTOR_DIR2, HIGH);
  }

  // Set motor speed
  analogWrite(MOTOR_SPEED_PIN, pwm_value);

  // Calculate actual velocity from PWM (reverse of PWM calculation)
  // This is an estimate - in production, use encoder feedback
  double linear_velocity_actual =
      ((double)m_current_pwm / MAX_PWM) * MAX_LINEAR_VELOCITY;
  if (!forward) { linear_velocity_actual = -linear_velocity_actual; }
  m_velocity_actual = linear_velocity_actual / WHEEL_RADIUS; // Convert to rad/s
}

// MARK: FEEDBACK
// -----------------------------------------------------------------------------
void sendFeedback() {
  // Send feedback at regular intervals
  unsigned long current_time = millis();
  if (current_time - m_last_feedback_time >= FEEDBACK_INTERVAL_MS) {
    // Format: "F:P:<pinion_angle>,V:<velocity>\n"
    // Example: "F:P:0.123456,V:1.234567\n"
    Serial.print("F:P:");
    Serial.print(m_pinion_angle_actual, 6);
    Serial.print(",V:");
    Serial.print(m_velocity_actual, 6);
    Serial.print("\n");

    m_last_feedback_time = current_time;
  }
}

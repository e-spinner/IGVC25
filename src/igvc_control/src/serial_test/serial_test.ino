// MARK: CONST
// ------------------------------------------------------------------------

#include <LedControl.h> // MAX7219 driver library

constexpr uint32_t BAUD_RATE         = 115200;
constexpr size_t SERIAL_BUF_SIZE     = 64;
constexpr uint32_t FEEDBACK_INTERVAL = 50; // [ms]
constexpr uint8_t FEEDBACK_PRECISION = 6;  // [decimals]
constexpr char TERMINATOR            = '\n';

// MAX7219 wiring (adjust to your pins)
constexpr uint8_t LED_DIN_PIN = 12; // data in to MAX7219
constexpr uint8_t LED_CLK_PIN = 11; // clock to MAX7219
constexpr uint8_t LED_CS_PIN  = 10; // chip select / load to MAX7219

// Sign-based display deadbands (within these -> treated as zero/center)
constexpr float PINION_DEADBAND_RAD    = 0.075f;
constexpr float VELOCITY_DEADBAND_RPS  = 0.75f;

// MARK: STATE
// ------------------------------------------------------------------------

bool f_in_autonomous_mode = true; // TODO: from hardware interface

// Command values from hardware interface
float g_pinion_cmd   = 0.0; // [rad] target angle
float g_velocity_cmd = 0.0; // [rad/s] target motor shaft speed

// State values for hardware interface
float g_pinion_state   = 0.0; // [rad] actual angle
float g_velocity_state = 0.0; // [rad/s] measured motor shaft speed


uint32_t g_last_feedback_time = 0;
char g_serial_buffer[SERIAL_BUF_SIZE];
size_t g_serial_len = 0;

// MAX7219 controller for a single 8x8 matrix
LedControl g_led_matrix = LedControl(LED_DIN_PIN, LED_CLK_PIN, LED_CS_PIN, 1);

// MARK: SETUP
// ------------------------------------------------------------------------
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) {} // Wait for USB
  Serial.println("Serial Connection Initialized");

  // Initialize LED matrix
  g_led_matrix.shutdown(0, false);
  g_led_matrix.setIntensity(0, 8); // brightness: 0..15
  g_led_matrix.clearDisplay(0);
}

// MARK: LOOP
// ------------------------------------------------------------------------
void loop() {
  read_commands();

  process_commands();

  update_led_matrix();

  send_feedback();

}

void set_motor(int direction, int speed, int spd_pin, int fwd_pin, int rev_pin) {
  analogWrite(spd_pin, speed);
  digitalWrite(fwd_pin, (direction == 1) ? HIGH : LOW);
  digitalWrite(rev_pin, (direction == 1) ? LOW : HIGH);
}

// MARK: LED MATRIX
// ------------------------------------------------------------------------


void draw_block_2x2(uint8_t row_start, uint8_t col_start) {
  g_led_matrix.clearDisplay(0);
  for (uint8_t r = 0; r < 2; ++r) {
    for (uint8_t c = 0; c < 2; ++c) {
      g_led_matrix.setLed(0, row_start + r, col_start + c, true);
    }
  }
}

void update_led_matrix() {
  // Sign-only mapping to a 2x2 block placed in a 3x3 grid of positions.
  // Deadband -> center (index 1), negative -> index 0 (left/up), positive -> index 2 (right/down).
  int x_slot = 1; // 0:left, 1:center, 2:right (pinion)
  if (g_pinion_state > PINION_DEADBAND_RAD) {
    x_slot = 0;
  } else if (g_pinion_state < -PINION_DEADBAND_RAD) {
    x_slot = 2;
  }

  int y_slot = 1; // 0:top, 1:center, 2:bottom (velocity)
  // Positive velocity shows "up" -> top slot
  if (g_velocity_state > VELOCITY_DEADBAND_RPS) {
    y_slot = 0;
  } else if (g_velocity_state < -VELOCITY_DEADBAND_RPS) {
    y_slot = 2;
  }

  // Starting indices for 2x2 block within 8x8
  // Positions: left(1), center(3), right(5) and top/middle/bottom similarly.
  const uint8_t col_starts[3] = {1, 3, 5};
  const uint8_t row_starts[3] = {1, 3, 5};

  draw_block_2x2(row_starts[y_slot], col_starts[x_slot]);
}

// MARK: CMD
// ------------------------------------------------------------------------
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
      // overflow, flush buffer
      g_serial_len = 0;
    }
  }
}

void parse_command(String command) {
  // TODO: some kind of way to activate or deactivate the system? [f_in_autonomous_mode]

  // Expected Format "C:P:<pinion_angle>,V:<velocity>"
  int p = command.indexOf("C:P:");
  int v = command.indexOf(",V:");

  if (p == -1 || v == -1) { return; }

  g_pinion_cmd   = command.substring(p + 4, v).toFloat();
  g_velocity_cmd = command.substring(v + 3).toFloat();
}

void process_commands() {
  g_pinion_state = g_pinion_cmd;
  g_velocity_state = g_velocity_cmd;
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
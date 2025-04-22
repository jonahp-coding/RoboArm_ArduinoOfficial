
// register addresses
#define config_register_addr 0x0
#define en_aa_register_addr 0x1
#define en_rx_addr_register_addr 0x2  // Not used in code!
#define setup_aw_register_addr 0x3  // Not used in code!
#define setup_retr_register_addr 0x4
#define rf_ch_register_addr 0x5
#define rf_setup_register_addr 0x6
#define status_register_addr 0x7
#define rx_addr_p0_register_addr 0xA
#define tx_addr_register_addr 0x10
#define rx_pw_p0_register_addr 0x11
#define feature_register_addr 0x1D

// Command Bytes/Bits
#define r_rx_payload_cmd 0x61
#define w_tx_payload_cmd 0xA0
#define r_rx_payload_wid_cmd 0x60
#define w_tx_payload_no_ack_cmd 0xB0
#define flush_tx_payload_cmd 0xE1
#define flush_rx_payload_cmd 0xE2

#define pca9685_default_addr 0x40
#define servo_frequency 50 // in Hertz(Hz)

#define servo_min_pulse_width 0.0005
#define servo_max_pulse_width 0.0032

// Bitmask
#define listen_button_bitmask 0b1

#define milli_to_second 1000

#define nrf_read 0
#define nrf_write 1
#define nrf_no_op -1

#include <SPI.h>
#include <Wire.h>
#include <My_PCA9685.h>

const uint8_t nrf_ce_pin = 9;
const uint8_t nrf_csn_pin = SS; // SS = 10 on Arduino Uno/Nano
const uint8_t nrf_irq_pin = 2;

// Creating Instances
SPISettings nrf24l01p_settings(7500000, MSBFIRST, SPI_MODE0);
PCA9685 my_pca9685((uint8_t) pca9685_default_addr, (uint16_t) servo_frequency);

uint8_t rx_buffer[13]; // button_byte(1-byte) + rotation_1(4-bytes) + rotation_2(4-bytes) + rotation_3(4-bytes)
uint8_t rx_buffer_length = (sizeof(rx_buffer) / sizeof(rx_buffer[0]));

uint8_t button_byte = 0;
float rotation_1, rotation_2, rotation_3;

bool button_1_state = false;

const uint8_t num_servos = 3;
const float angle_increment = 0.50; // 2-decimal places
float servo_pos[num_servos] = {55.0, 60.0, 60.0}; // Intial position

uint16_t write_delay = 50; // Microseconds!!!

uint16_t action_cooldown = 9; // Milliseconds

uint8_t warning_led = 7;
uint8_t cycle_led = 6;
uint8_t action_led = 5;

float control_gain_1 = 0.95;
float control_gain_2 = 1.40;
float control_gain_3 = 0.8;

// Time-keeping variables
uint32_t current_millis = millis();
uint32_t last_move_action = 0;

void setup() {

  Serial.begin(115200);
  Wire.begin();
  SPI.begin();

  pinMode(nrf_ce_pin, OUTPUT);
  pinMode(nrf_csn_pin, OUTPUT);
  pinMode(nrf_irq_pin, INPUT_PULLUP);

  pinMode(warning_led, OUTPUT);
  pinMode(cycle_led, OUTPUT);
  pinMode(action_led, OUTPUT);

  setup_nrf24l01p_receiver();
  my_pca9685.initialize_pca9685();

  digitalWrite(nrf_csn_pin, HIGH);
  digitalWrite(nrf_ce_pin, HIGH);

  digitalWrite(warning_led, LOW);
  digitalWrite(cycle_led, LOW);
  digitalWrite(action_led, LOW);

  my_pca9685.set_channel_pulse_width(0, map_float(servo_pos[0], (float) 0.0, (float) 180.0, (float) servo_min_pulse_width, (float) servo_max_pulse_width));
  my_pca9685.set_channel_pulse_width(1, map_float(servo_pos[1], (float) 0.0, (float) 180.0, (float) servo_min_pulse_width, (float) servo_max_pulse_width));
  my_pca9685.set_channel_pulse_width(2, map_float(servo_pos[2], (float) 0.0, (float) 180.0, (float) servo_min_pulse_width, (float) servo_max_pulse_width));

  delay(milli_to_second * 2);

}

void loop() {

  nrf24l01p_operation(r_rx_payload_cmd, nrf_read, rx_buffer_length, rx_buffer);
  for (uint8_t i = 0; i < rx_buffer_length; i++) {
    //Serial.println(rx_buffer[i]);
  }
  
  button_byte = rx_buffer[0];
  rotation_1 = bytesToFloat(rx_buffer[1], rx_buffer[2], rx_buffer[3], rx_buffer[4]);
  rotation_2 = bytesToFloat(rx_buffer[5], rx_buffer[6], rx_buffer[7], rx_buffer[8]);
  rotation_3 = bytesToFloat(rx_buffer[9], rx_buffer[10], rx_buffer[11], rx_buffer[12]);
  
  Serial.print("button_byte: "); Serial.println(button_byte, BIN);
  Serial.print("rotation_1: "); Serial.println(rotation_1, 4);
  Serial.print("rotation_2: "); Serial.println(rotation_2, 4);
  Serial.print("rotation_3: "); Serial.println(rotation_3, 4);

  digitalWrite(nrf_csn_pin, LOW);

  // Flush data from the RX payload
  SPI.beginTransaction(nrf24l01p_settings);
  SPI.transfer(flush_rx_payload_cmd); // Flush RX payload command
  SPI.endTransaction();

  digitalWrite(nrf_csn_pin, HIGH);

  if (button_byte & listen_button_bitmask) {
    button_1_state = true;
  } else {
    button_1_state = false;
  }

  // Motor control
  if (button_1_state) {
    Serial.println("\nListening...");

    if ((current_millis - last_move_action) > write_delay) {

      rotation_1 *= control_gain_1; // Control sensitivity of RoboArm Shoulder
      rotation_2 *= control_gain_2; // Control sensitivity of RoboArm Arm
      rotation_3 *= control_gain_3; // Control sensitivity of RoboArm ForeArm

      if (servo_pos[0] + rotation_1 > 180.0 * control_gain_1) {
        servo_pos[0] = 180.0 * control_gain_1;
      } else if (servo_pos[0] + rotation_1 < 0.0 * control_gain_1) {
        servo_pos[0] = 0.0 * control_gain_1;
      } else {
        servo_pos[0] += rotation_1;
      }

      if (servo_pos[1] + rotation_2 > 180.0 * control_gain_2) {
        servo_pos[1] = 180.0 * control_gain_2;
      } else if (servo_pos[1] + rotation_2 < 0.0 * control_gain_2) {
        servo_pos[1] = 0.0 * control_gain_2;
      } else {
        servo_pos[1] += rotation_2;
      }

      if (servo_pos[2] + rotation_3 > 180.0 * control_gain_3) {
        servo_pos[2] = 180.0 * control_gain_3;
      } else if (servo_pos[2] + rotation_3 < 0.0 * control_gain_3) {
        servo_pos[2] = 0.0 * control_gain_3;
      } else {
        servo_pos[2] += rotation_3;
      }

      my_pca9685.set_channel_pulse_width(0, map_float(servo_pos[0], (float) 0.0, (float) 180.0, (float) servo_min_pulse_width, (float) servo_max_pulse_width));
      my_pca9685.set_channel_pulse_width(1, map_float(servo_pos[1], (float) 0.0, (float) 180.0, (float) servo_min_pulse_width, (float) servo_max_pulse_width));
      my_pca9685.set_channel_pulse_width(2, map_float(servo_pos[2], (float) 0.0, (float) 180.0, (float) servo_min_pulse_width, (float) servo_max_pulse_width));

      Serial.println("Position(+)");
      Serial.print("Channel: 0");
      Serial.print(" / Angle: ");
      Serial.print(servo_pos[0], 2);
      Serial.println("°");
      Serial.print("Angle Interpolation: ");
      Serial.print(map_float(servo_pos[0], (float) 0.0 * control_gain_1, (float) 180.0 * control_gain_1, (float) servo_min_pulse_width, (float) servo_max_pulse_width) * 1000, 5);
      Serial.println("ms\n");

      ///////////

      Serial.println("Position(+)");
      Serial.print("Channel: 1");
      Serial.print(" / Angle: ");
      Serial.print(servo_pos[1], 2);
      Serial.println("°");
      Serial.print("Angle Interpolation: ");
      Serial.print(map_float(servo_pos[1], (float) 0.0 * control_gain_2, (float) 180.0 * control_gain_2, (float) servo_min_pulse_width, (float) servo_max_pulse_width) * 1000, 5);
      Serial.println("ms\n");

      ///////////

      Serial.println("Position(+)");
      Serial.print("Channel: 2");
      Serial.print(" / Angle: ");
      Serial.print(servo_pos[2], 2);
      Serial.println("°");
      Serial.print("Angle Interpolation: ");
      Serial.print(map_float(servo_pos[2], (float) 0.0 * control_gain_3, (float) 180.0 * control_gain_3, (float) servo_min_pulse_width, (float) servo_max_pulse_width) * 1000, 5);
      Serial.println("ms\n");

      last_move_action = current_millis;
    }
  }

  // Status checking
  /*if (servo_pos[channel_index] == (float) 0.0 || servo_pos[channel_index] == (float) 180.0) {
    digitalWrite(warning_led, HIGH);
  } else {
    digitalWrite(warning_led, LOW);
  }*/

  if ((current_millis - last_move_action) < action_cooldown) {
    digitalWrite(action_led, HIGH);
  } else {
    digitalWrite(action_led, LOW);
  }

  Serial.print("-------------------------------\n");

  current_millis = millis();
  delay(milli_to_second * 0.05); // program delay
}

float bytesToFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    uint32_t combinedBytes = ((uint32_t)b3 << 24) | ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | b0;
    float result;
    memcpy(&result, &combinedBytes, sizeof(result));
    return result;
}

uint8_t nrf24l01p_operation(uint8_t address, bool operation, uint8_t length, uint8_t *data_buffer) { // data is a pointer for data to send or store (depends on the operation)
  
  // Operations
  const uint8_t read_op = 0x0; 
  const uint8_t write_op = 0x20;

  uint8_t *buffer_pointer = data_buffer;
  
  digitalWrite(nrf_csn_pin, LOW);
  SPI.beginTransaction(nrf24l01p_settings);

  if (operation == nrf_write) {
    SPI.transfer(write_op + address);

    for (uint8_t i = 0; i < length; i++) {
      //Serial.println(*buffer_pointer, HEX);
      SPI.transfer(*buffer_pointer);        
      buffer_pointer++;
    }
  } else if (operation == nrf_read) {
    SPI.transfer(read_op + address);

    for (uint8_t i = 0; i < length; i++) {
      //Serial.println(*buffer_pointer, HEX);
      *buffer_pointer = SPI.transfer(0xFF);        
      buffer_pointer++;
    }
  } else if (operation == nrf_no_op) {
    // Commmand
    SPI.transfer(address);
  }

  SPI.endTransaction();
  digitalWrite(nrf_csn_pin, HIGH);
}

uint8_t clear_buffer(uint8_t *buffer, uint16_t length) {
  for (uint8_t i = 0; i < sizeof(length); i++) {
    *(buffer + i) = 0;
  }
}

bool setup_nrf24l01p_receiver() {

  const uint8_t rf_frequency = 0x3C;
  const uint8_t rx_payload_width = 0x20; // 20-Byte(s) wide
  const uint8_t rx_addr[5] = {0x77, 0x35, 0xF0, 0xD3, 0xE7}; // Example "RX Address" for Product Specification, Note LSByte to MSByte!!!
  
  // Setup the NRF24L01+ in Receiver Mode(RX)
  uint8_t information_buffer[32];

  information_buffer[0] = 0x0;  
  nrf24l01p_operation(en_aa_register_addr, nrf_write, 1, information_buffer);

  information_buffer[0] = 0x0;
  nrf24l01p_operation(setup_retr_register_addr, nrf_write, 1, information_buffer);

  information_buffer[0] = 0x33;
  nrf24l01p_operation(config_register_addr, nrf_write, 1, information_buffer);

  information_buffer[0] = rf_frequency;
  nrf24l01p_operation(rf_ch_register_addr, nrf_write, 1, information_buffer);

  information_buffer[0] = 0x26;
  nrf24l01p_operation(rf_setup_register_addr, nrf_write, 1, information_buffer);

  for (uint8_t i = 0; i < sizeof(rx_addr); i++) {
    information_buffer[i] = rx_addr[i];
  }
  nrf24l01p_operation(rx_addr_p0_register_addr, nrf_write, 5, information_buffer);
  clear_buffer(information_buffer, sizeof(information_buffer));

  information_buffer[0] = rx_payload_width;
  nrf24l01p_operation(rx_pw_p0_register_addr, nrf_write, 1, information_buffer);

  information_buffer[0] = 0x0;
  nrf24l01p_operation(feature_register_addr, nrf_write, 1, information_buffer);

  nrf24l01p_operation(flush_rx_payload_cmd, nrf_no_op, 0, information_buffer);

  return true;
}
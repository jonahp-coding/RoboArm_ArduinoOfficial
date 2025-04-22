
// register addresses
#define config_register_addr 0x0
#define en_aa_register_addr 0x1
#define en_rx_addr_register_addr 0x2  // Not used in code!
#define setup_aw_register_addr 0x3    // Not used in code!
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

#define nrf_read 0
#define nrf_write 1
#define nrf_no_op -1

#define milli_to_second 1000

#define __channel_deselect_all__ 0b0
#define __channel_select_1__ 0b1
#define __channel_select_2__ 0b10
#define __channel_select_3__ 0b100
#define __channel_select_4__ 0b1000
#define __channel_select_5__ 0b10000
#define __channel_select_6__ 0b100000
#define __channel_select_7__ 0b1000000
#define __channel_select_8__ 0b10000000

#include <SPI.h>
#include <Wire.h>
#include <MPU_6050.h>

uint8_t button1_pin = 3;
uint8_t button2_pin = 4;

uint8_t ready_led_pin = 8;

const uint8_t nrf_ce_pin = 9;
const uint8_t nrf_csn_pin = SS; // SS = 10
const uint8_t nrf_irq_pin = 2;

float gyro_y_1, gyro_y_2, gyro_y_3;
//float orientation_y_1, orientation_y_2, orientation_y_3;
float gyro_y_1_delta, gyro_y_2_delta, gyro_y_3_delta;

bool calibrate_gyros = false;

uint8_t tca9548a_addr = 0x70; // i.e. base_addr(0x70) + index(A2,A1,A0).

uint8_t mpu_addr = 0x68;

uint8_t mpu_6050_hz = 50;

uint8_t button_data = 0;
bool is_button1_pressed = false;

// Time-keeping variables
uint16_t current_millis = millis();
uint16_t last_mpu_sample = 0;

SPISettings nrf24l01p_settings(7500000, MSBFIRST, SPI_MODE0);

MPU_6050 mpu_1{mpu_addr};
MPU_6050 mpu_2{mpu_addr};
MPU_6050 mpu_3{mpu_addr};

void setup() {
  Serial.begin(115200);
  SPI.begin();
  Wire.begin();

  tca9548a_select_channel(__channel_select_1__);
  mpu_1.setup();

  tca9548a_select_channel(__channel_select_2__);
  mpu_2.setup();

  tca9548a_select_channel(__channel_select_3__);
  mpu_3.setup();

  pinMode(button1_pin, INPUT);
  pinMode(button2_pin, INPUT);

  pinMode(ready_led_pin, OUTPUT);

  pinMode(nrf_ce_pin, OUTPUT);
  pinMode(nrf_csn_pin, OUTPUT);
  pinMode(nrf_irq_pin, INPUT_PULLUP);

  digitalWrite(ready_led_pin, LOW);

  setup_nrf24l01p_transmitter();
  digitalWrite(nrf_ce_pin, HIGH);
  
  // Calibrating IMUs
  if (calibrate_gyros) {
    Serial.println("Calibrating IMU#1...");
    tca9548a_select_channel(__channel_select_1__);
    mpu_1.enable_calibration(true);
    //mpu_1.get_calibration();
    Serial.println("IMU#1 Calibrated.\n");

    Serial.println("Calibrating IMU#2...");
    tca9548a_select_channel(__channel_select_2__);
    mpu_2.enable_calibration(true);
    //mpu_2.get_calibration();
    Serial.println("IMU#2 Calibrated.\n");

    Serial.println("Calibrating IMU#3...");
    tca9548a_select_channel(__channel_select_3__);
    mpu_3.enable_calibration(true);
    //mpu_3.get_calibration();
    Serial.println("IMU#3 Calibrated.\n");
  }

  digitalWrite(ready_led_pin, HIGH);

  delay(milli_to_second);
}

void loop() {

  //Serial.println(digitalRead(button1_pin));
  //Serial.println(digitalRead(button2_pin));

  //button_data = (digitalRead(button1_pin) << 0) + (digitalRead(button2_pin) << 1);
  button_data = (digitalRead(button1_pin) << 0);
  Serial.print("button_data: "); Serial.print(button_data, BIN); Serial.print("\n");

  if ((current_millis - last_mpu_sample) > (milli_to_second/mpu_6050_hz)) {
    tca9548a_select_channel(__channel_select_1__);
    mpu_1.get_gyro_y(&gyro_y_1);

    tca9548a_select_channel(__channel_select_2__);
    mpu_2.get_gyro_y(&gyro_y_2);

    tca9548a_select_channel(__channel_select_3__);
    mpu_3.get_gyro_y(&gyro_y_3);

    //orientation_y_1 += gyro_y_1 / mpu_6050_hz;
    //orientation_y_2 += gyro_y_2 / mpu_6050_hz;
    //orientation_y_3 += gyro_y_3 / mpu_6050_hz;

    gyro_y_1_delta = gyro_y_1 / mpu_6050_hz;
    gyro_y_2_delta = gyro_y_2 / mpu_6050_hz;
    gyro_y_3_delta = gyro_y_3 / mpu_6050_hz;

    /*Serial.print("gyro_x_1: "); Serial.print(gyro_x_1, 4); Serial.print("°@"); Serial.print(mpu_6050_hz); Serial.print("Hz\n");
    Serial.print("gyro_x_2: "); Serial.print(gyro_x_2, 4); Serial.print("°@"); Serial.print(mpu_6050_hz); Serial.print("Hz\n");
    Serial.print("gyro_x_3: "); Serial.print(gyro_x_3, 4); Serial.print("°@"); Serial.print(mpu_6050_hz); Serial.print("Hz\n");*/

    //Serial.print("orientation_y_1: "); Serial.print(orientation_y_1, 4); Serial.print("°\n");
    //Serial.print("orientation_y_2: "); Serial.print(orientation_y_2, 4); Serial.print("°\n");
    //Serial.print("orientation_y_3: "); Serial.print(orientation_y_3, 4); Serial.print("°\n");

    Serial.print("gyro_y_1_delta: "); Serial.print(gyro_y_1_delta, 4); Serial.print("°@"); Serial.print(mpu_6050_hz); Serial.print("Hz\n");
    Serial.print("gyro_y_2_delta: "); Serial.print(gyro_y_2_delta, 4); Serial.print("°@"); Serial.print(mpu_6050_hz); Serial.print("Hz\n");
    Serial.print("gyro_y_3_delta: "); Serial.print(gyro_y_3_delta, 4); Serial.print("°@"); Serial.print(mpu_6050_hz); Serial.print("Hz\n");
    //Serial.print("\n");

    last_mpu_sample = current_millis;
  }

  if (digitalRead(button1_pin)) {
    if (!(is_button1_pressed)) {

      gyro_y_1_delta = 0.0;
      gyro_y_2_delta = 0.0;
      gyro_y_3_delta = 0.0;

      is_button1_pressed = true;
    }
  } else {
    if (is_button1_pressed) {
      is_button1_pressed = false;
    }
  }

  digitalWrite(nrf_csn_pin, LOW);

  // Writing data to the TX payload
  SPI.beginTransaction(nrf24l01p_settings);
  SPI.transfer(w_tx_payload_no_ack_cmd); // Write to TX payload command
  
  SPI.transfer(button_data);

  uint8_t *byte;
  
  byte = (uint8_t*) &gyro_y_1_delta;

  SPI.transfer(*(byte+0) & 0xFF);
  SPI.transfer(*(byte+1) & 0xFF);
  SPI.transfer(*(byte+2) & 0xFF);
  SPI.transfer(*(byte+3) & 0xFF);

  byte = (uint8_t*) &gyro_y_2_delta;

  SPI.transfer(*(byte+0) & 0xFF);
  SPI.transfer(*(byte+1) & 0xFF);
  SPI.transfer(*(byte+2) & 0xFF);
  SPI.transfer(*(byte+3) & 0xFF);

  byte = (uint8_t*) &gyro_y_3_delta;

  SPI.transfer(*(byte+0) & 0xFF);
  SPI.transfer(*(byte+1) & 0xFF);
  SPI.transfer(*(byte+2) & 0xFF);
  SPI.transfer(*(byte+3) & 0xFF);

  SPI.endTransaction();

  digitalWrite(nrf_csn_pin, HIGH);

  Serial.print("-------------------------------\n");

  delay(milli_to_second/mpu_6050_hz);
  current_millis = millis();

  //nrf24l01p_operation(flush_tx_payload_cmd, nrf_no_op, 0, tx_buffer); // Doesn't Work for some reason.

  // Flush data from the TX payload. 
  digitalWrite(nrf_csn_pin, LOW);
  SPI.beginTransaction(nrf24l01p_settings);
  SPI.transfer(flush_tx_payload_cmd); // Flush TX payload command
  SPI.endTransaction();
  digitalWrite(nrf_csn_pin, HIGH);
}

void tca9548a_select_channel(uint8_t channel) {
  Wire.beginTransmission(tca9548a_addr);
  Wire.write(channel);
  Wire.endTransmission();
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
    
    if (length > 0) {
      for (uint8_t i = 0; i < length; i++) {
        //Serial.println(*buffer_pointer, HEX);
        SPI.transfer(*buffer_pointer);        
        buffer_pointer++;
      }      
    }
  }

  SPI.endTransaction();
  digitalWrite(nrf_csn_pin, HIGH);
}

uint8_t clear_buffer(uint8_t *buffer, uint16_t length) {
  for (uint8_t i = 0; i < sizeof(length); i++) {
    *(buffer + i) = 0;
  }
}

bool setup_nrf24l01p_transmitter() {

  const uint8_t rf_frequency = 0x3C;
  const uint8_t tx_addr[5] = {0x77, 0x35, 0xF0, 0xD3, 0xE7}; // Example "TX Address" for Product Specification, Note LSByte to MSByte!!!
  
  // Setup the NRF24L01+ in Receiver Mode(RX)
  uint8_t information_buffer[32];

  information_buffer[0] = 0x0;  
  nrf24l01p_operation(en_aa_register_addr, nrf_write, 1, information_buffer);

  information_buffer[0] = 0x0;
  nrf24l01p_operation(setup_retr_register_addr, nrf_write, 1, information_buffer);

  information_buffer[0] = 0x52;
  nrf24l01p_operation(config_register_addr, nrf_write, 1, information_buffer);

  information_buffer[0] = rf_frequency;
  nrf24l01p_operation(rf_ch_register_addr, nrf_write, 1, information_buffer);

  information_buffer[0] = 0x26;
  nrf24l01p_operation(rf_setup_register_addr, nrf_write, 1, information_buffer);

  for (uint8_t i = 0; i < sizeof(tx_addr); i++) {
    information_buffer[i] = tx_addr[i];
  }
  nrf24l01p_operation(tx_addr_register_addr, nrf_write, sizeof(tx_addr), information_buffer);
  clear_buffer(information_buffer, sizeof(information_buffer));

  information_buffer[0] = 0x1;
  nrf24l01p_operation(feature_register_addr, nrf_write, 1, information_buffer);

  nrf24l01p_operation(flush_tx_payload_cmd, nrf_no_op, 0, information_buffer);

  return true;
}
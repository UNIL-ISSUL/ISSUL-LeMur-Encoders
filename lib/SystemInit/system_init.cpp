#include "system_init.h"

void set_DFR0972_mA(TCA9548 &multiplexer, uint8_t channel, float current_mA, uint16_t dac_4mA, uint16_t dac_20mA) {
  multiplexer.selectChannel(channel);
  delayMicroseconds(50);

  if (current_mA < CURRENT_MIN_MA) current_mA = CURRENT_MIN_MA;
  if (current_mA > CURRENT_MAX_MA) current_mA = CURRENT_MAX_MA;

  uint16_t dac_val = dac_4mA + ((current_mA - CURRENT_MIN_MA) * (float)(dac_20mA - dac_4mA) / 16.0f);

  Wire.beginTransmission(I2C_DAC_ADDR);
  Wire.write(0x02);
  Wire.write((dac_val << 4) & 0xFF);
  Wire.write((dac_val >> 4) & 0xFF);
  Wire.endTransmission();
}

bool system_hardware_init(
    TCA9548 &multiplexer,
    UNIT_EXT_ENCODER encoders[],
    size_t num_encoders,
    Lift &lift,
    ModbusRTUSlave &slave,
    bool coils[],
    uint16_t registers[],
    hw_timer_t *&timer_ref,
    void (*timerISR)()
) {
  Serial.begin(DEBUG_BAUDRATE);
  Serial.setTimeout(10);
  Serial.flush();
  delay(50);

  Serial.println();
  Serial.println("========================================");
  Serial.println("   M5Atom Boot @ " + String(DEBUG_BAUDRATE) + " baud");
  Serial.println("========================================");

  // 1. System & CPU Info
  Serial.println("[1/6] System & CPU Info...");
  Serial.println("      ESP32 CPU: " + String(ESP.getCpuFreqMHz()) + " MHz | Free Heap: " + String(ESP.getFreeHeap() / 1024) + " KB");

  // 2. Initialize Modbus RTU Slave
  Serial.println("[2/6] Initializing Modbus RTU Slave...");
  Serial2.begin(MODBUS_BAUDRATE, SERIAL_8E1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  slave.begin(MODBUS_BAUDRATE);
  slave.setCoils(coils, MODBUS_NUM_COILS);
  slave.setHoldingRegisters(registers, MODBUS_NUM_REGISTERS);
  Serial.println("      Modbus Slave (Addr " + String(MODBUS_SLAVE_ADDR) + " @ " + String(MODBUS_BAUDRATE) + " baud) OK");
  
  // Safe initialization delay (from commit 931106c) to let hardware and bus stabilize
  delay(1000);

  // 3. Initialize I2C bus & Multiplexer
  Serial.println("[3/6] Initializing I2C bus & Multiplexer...");
  Wire.setClock(I2C_FREQ_HZ);
  uint8_t i2c_error = 0;

  if (multiplexer.begin()) {
    Serial.println("      TCA9548 Multiplexer OK (0x70)");
  } else {
    Serial.println("      ❌ TCA9548 Multiplexer NOT FOUND (0x70)");
    i2c_error++;
  }

  // 4. Scan & Initialize Multiplexer Channels
  Serial.println("[4/6] Scanning Multiplexer Channels...");
  const char* enc_names[2] = {"Belt Enc", "Steps Enc"};
  for (size_t i = 0; i < num_encoders && i < 2; i++) {
    multiplexer.selectChannel(i);
    bool connected = multiplexer.isConnected(I2C_ENC_ADDR);
    if (connected) {
      encoders[i].init(&Wire, I2C_ENC_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
      Serial.println("      Channel " + String(i) + " (" + enc_names[i] + "): Found (0x" + String(I2C_ENC_ADDR, HEX) + ") | OK");
    } else {
      Serial.println("      ❌ Channel " + String(i) + " (" + enc_names[i] + "): NOT FOUND (0x" + String(I2C_ENC_ADDR, HEX) + ")");
      i2c_error++;
    }
  }

  const char* dac_names[2] = {"Speed DAC", "Incl DAC"};
  for (int i = 2; i < 4; i++) {
    multiplexer.selectChannel(i);
    if (multiplexer.isConnected(I2C_DAC_ADDR)) {
      if (i == 2) set_DFR0972_mA(multiplexer, 2, 4.0f, DFR0972_DAC_4MA_RAW, DFR0972_DAC_20MA_RAW);
      if (i == 3) set_DFR0972_mA(multiplexer, 3, 4.0f, DFR0972_DAC_4MA_RAW, DFR0972_DAC_20MA_RAW);
      Serial.println("      Port " + String(i) + " (" + dac_names[i - 2] + "): Found (0x" + String(I2C_DAC_ADDR, HEX) + ") | Output 4.00 mA");
    } else {
      Serial.println("      ❌ Port " + String(i) + " (" + dac_names[i - 2] + "): NOT FOUND (0x" + String(I2C_DAC_ADDR, HEX) + ")");
      i2c_error++;
    }
  }

  // Isolate PaHub channels from main I2C bus
  multiplexer.disableAllChannels();
  delayMicroseconds(50);

  // 5. Initialize Lift Hardware (Main I2C Bus via 3-way Hub)
  Serial.println("[5/6] Initializing Lift Hardware (Main I2C Bus)...");
  bool dac_ok = false, adc_ok = false;
  lift.checkHardware(dac_ok, adc_ok);
  if (dac_ok) {
    Serial.println("      GP8413 0-5V DAC found (0x" + String(I2C_LIFT_DAC_ADDR, HEX) + ")");
  } else {
    Serial.println("      ❌ GP8413 0-5V DAC NOT FOUND (0x" + String(I2C_LIFT_DAC_ADDR, HEX) + ")");
    i2c_error++;
  }

  if (adc_ok) {
    Serial.println("      ADS1110 ADC found (0x" + String(I2C_ADC_ADDR, HEX) + ")");
  } else {
    Serial.println("      ❌ ADS1110 ADC NOT FOUND (0x" + String(I2C_ADC_ADDR, HEX) + ")");
    i2c_error++;
  }

  if (dac_ok && adc_ok) {
    int lift_status = lift.init(DFRobot_GP8XXX::eOutputRange5V);
    if (lift_status != 0) {
      Serial.println("      ❌ Lift config failed: " + String(Lift::getStatusMessage(lift_status)));
      i2c_error++;
    } else {
      float init_volt = lift.getVoltage();
      float init_angle = lift.getInclinaison_deg();
      if (init_angle < LIFT_ANGLE_MIN_DEG || init_angle > LIFT_ANGLE_MAX_DEG) {
        Serial.println("      ⚠️ Warning: Initial: " + String(init_volt, 3) + " V -> " + String(init_angle, 2) + " deg (outside range [0-90 deg])");
      } else {
        Serial.println("      Initial: " + String(init_volt, 3) + " V -> " + String(init_angle, 2) + " deg (Valid [0-90 deg]) | Height: " + String(lift.getHeight_mm(), 1) + " mm");
      }
      Serial.println("      Motor Direction Relay Pins (UP: G22, DOWN: G19) OK");
      Serial.println("      Lift Hardware initialized successfully");
    }
  }

  // Halt on I2C error
  if (i2c_error > 0) {
    Serial.println("\n⛔ BOOT STOPPED: " + String(i2c_error) + " I2C error(s) detected.");
    Serial.println("   Please check wiring and power to the sensors/multiplexer.");
    M5.dis.drawpix(0, CRGB::Red);
    return false;
  }

  // Warm up Lift sensor
  for (int i = 0; i < 50; i++) {
    lift.update();
    delay(2);
  }
  uint16_t init_angle_centideg = (uint16_t)constrain(round(lift.getInclinaison_deg() * 100.0f), 0, 9000);
  registers[1] = init_angle_centideg;
  registers[2] = init_angle_centideg;

  // 6. Start 200 Hz Timer
  Serial.println("[6/6] Starting 200 Hz Real-Time Control Loop...");
  timer_ref = timerBegin(3, 80, true);
  timerAttachInterrupt(timer_ref, timerISR, true);
  timerAlarmWrite(timer_ref, UPDATE_PERIOD_US, true);
  timerAlarmEnable(timer_ref);

  M5.dis.drawpix(0, CRGB::Green);
  Serial.println("\n✅ System fully initialized and running @ 200 Hz!");
  Serial.println("   (Press M5 button to toggle Teleplot streaming)\n");

  return true;
}

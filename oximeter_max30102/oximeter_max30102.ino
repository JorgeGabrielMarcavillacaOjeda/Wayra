#include "MAX30102.h"
#include "Pulse.h"
#include <EEPROM.h>
#include <Wire.h>

MAX30102 sensor;
Pulse pulseIR;
Pulse pulseRed;
MAFilter bpm;

#define LED 2          // GPIO del LED en la placa ESP32
#define BUTTON 0       // Pin del botón
#define OPTIONS 7      // Dirección EEPROM para opciones

int beatAvg;
int SPO2, SPO2f;
bool filter_for_graph = false;
bool draw_Red = false;
uint8_t pcflag = 0;
uint8_t istate = 0;
uint8_t sleep_counter = 0;

void button() {
  pcflag = 1;
}

void checkbutton() {
  if (pcflag && !digitalRead(BUTTON)) {
    istate = (istate + 1) % 4;
    filter_for_graph = istate & 0x01;
    draw_Red = istate & 0x02;
    EEPROM.write(OPTIONS, filter_for_graph);
    EEPROM.write(OPTIONS + 1, draw_Red);
  }
  pcflag = 0;
}

void go_sleep() {
  sensor.off();
  esp_deep_sleep_start();
}

int getVCC() {
  return (int)((float)analogRead(A0) * 3.3 / 4095.0 * 100); // mide el voltaje de alimentación
}

// CONFIGURACIÓN MODIFICADA DEL SENSOR
void sensor_custom_setup() {
  sensor.writeRegister8(0x09, 0x40); // REG_MODE_CONFIG, reset
  delay(500);
  sensor.writeRegister8(0x02, 0x00); // REG_FIFO_WR_PTR
  sensor.writeRegister8(0x03, 0x00); // REG_OVF_COUNTER
  sensor.writeRegister8(0x04, 0x00); // REG_FIFO_RD_PTR
  sensor.writeRegister8(0x08, 0x4F); // REG_FIFO_CONFIG: sample avg=4, rollover=false, almost full=17
  sensor.writeRegister8(0x09, 0x03); // REG_MODE_CONFIG: SpO2 mode
  sensor.writeRegister8(0x0A, 0x27); // REG_SPO2_CONFIG: ADC range=4096nA, sampleRate=100Hz, pulseWidth=411us

  // ⭐ LED power más alto (~12.6 mA)
  sensor.writeRegister8(0x0C, 0x3F); // REG_LED1_PA: IR
  sensor.writeRegister8(0x0D, 0x4F); // REG_LED2_PA: Red
  sensor.writeRegister8(0x10, 0x1F); // REG_PILOT_PA
}

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  Wire.begin(21, 22);

  if (!sensor.begin()) {
    Serial.println("Error inicializando sensor MAX30102");
    while (1);
  }

  sensor_custom_setup(); // USAMOS NUEVA CONFIGURACIÓN

  filter_for_graph = EEPROM.read(OPTIONS);
  draw_Red = EEPROM.read(OPTIONS + 1);
  attachInterrupt(digitalPinToInterrupt(BUTTON), button, CHANGE);

  Serial.println("Iniciando...");
}

long lastBeat = 0;
bool led_on = false;

void loop() {
  sensor.check();
  long now = millis();

  if (!sensor.available()) return;

  uint32_t irValue = sensor.getIR();
  uint32_t redValue = sensor.getRed();
  sensor.nextSample();

  // DEBUG: mostrar datos crudos
  //Serial.print("IR: ");
  //Serial.print(irValue);
  //Serial.print(" | RED: ");
  //Serial.println(redValue);//

  // Nuevo umbral para detectar dedo
  if (irValue < 10000) {
    checkbutton();
    Serial.println("Coloca el dedo sobre el sensor...");
    delay(200);
    if (++sleep_counter > 100) go_sleep();
    return;
  }

  sleep_counter = 0;
  int16_t IR_signal, Red_signal;
  bool beatRed, beatIR;

  if (!filter_for_graph) {
    IR_signal = pulseIR.dc_filter(irValue);
    Red_signal = pulseRed.dc_filter(redValue);
    beatRed = pulseRed.isBeat(pulseRed.ma_filter(Red_signal));
    beatIR = pulseIR.isBeat(pulseIR.ma_filter(IR_signal));
  } else {
    IR_signal = pulseIR.ma_filter(pulseIR.dc_filter(irValue));
    Red_signal = pulseRed.ma_filter(pulseRed.dc_filter(redValue));
    beatRed = pulseRed.isBeat(Red_signal);
    beatIR = pulseIR.isBeat(IR_signal);
  }

  if (draw_Red ? beatRed : beatIR) {
    long btpm = 60000 / (now - lastBeat);
    if (btpm > 0 && btpm < 200) beatAvg = bpm.filter((int16_t)btpm);
    lastBeat = now;

    digitalWrite(LED, HIGH);
    led_on = true;

    long numerator = (pulseRed.avgAC() * pulseIR.avgDC()) / 256;
    long denominator = (pulseRed.avgDC() * pulseIR.avgAC()) / 256;
    int RX100 = (denominator > 0) ? (numerator * 100) / denominator : 999;

    SPO2f = (10400 - RX100 * 17 + 50) / 100;
    SPO2 = SPO2f;

    Serial.print("BPM: ");
    Serial.print(beatAvg);
    Serial.print(" | SpO2: ");
    Serial.print(SPO2f);
    Serial.println("%");
  }

  if (led_on && (now - lastBeat) > 25) {
    digitalWrite(LED, LOW);
    led_on = false;
  }
}
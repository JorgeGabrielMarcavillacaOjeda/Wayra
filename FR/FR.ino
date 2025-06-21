#include <Wire.h>
#include "MAX30105.h"

MAX30105 particleSensor;

// Configuración optimizada
#define SAMPLING_RATE 50
#define MOVING_AVG_SIZE 20 //tamaño del filtro 1era 25 2da 15 18
#define BREATH_THRESHOLD 18    // Aumentado para evitar falsos positivos umbral de amplitud 20-30  25 17 
#define CONTACT_THRESHOLD 500
#define REPORT_INTERVAL 20     // Aumentado a 20 segundos

// Variables mejoradas
float buffer[MOVING_AVG_SIZE];
int bufIndex = 0;
float baseline = 22000.0;
unsigned long lastReport = 0;
bool validContact = false;
unsigned long lastPeakTime = 0;
int peaksInPeriod = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("Error en sensor");
    while(1);
  }

  particleSensor.setup(0x0D); // Configuración media (33.8mA)
  particleSensor.setPulseWidth(411);
}

void loop() {
  static unsigned long lastSample = millis();
  
  if (millis() - lastSample >= (1000/SAMPLING_RATE)) {
    lastSample = millis();
    float raw = particleSensor.getIR();

    // 1. Detección de contacto mejorada
    static float maxRaw = 0, minRaw = 50000.0;
    maxRaw = max(maxRaw, raw);
    minRaw = min(minRaw, raw);
    float dynamicRange = maxRaw - minRaw;
    
    validContact = (dynamicRange > CONTACT_THRESHOLD);
    maxRaw *= 0.99; minRaw *= 1.01; // Decaimiento gradual

    if(validContact) {
      // 2. Filtrado adaptativo
      buffer[bufIndex] = raw;
      bufIndex = (bufIndex + 1) % MOVING_AVG_SIZE;
      
      float filtered = 0;
      for(int i=0; i<MOVING_AVG_SIZE; i++) filtered += buffer[i];
      filtered /= MOVING_AVG_SIZE;
      
      // 3. Normalización con histéresis
      baseline = baseline*0.95 + filtered*0.05;
      float norm = filtered - baseline;
      
      // 4. Detección de picos robusta
      static float lastNorm = 0;
      static float currentPeak = 0;
      
      if(norm > currentPeak) currentPeak = norm;
      
      if(lastNorm > 0 && norm <= 0) { // Cruce descendente
        if(currentPeak > BREATH_THRESHOLD && 
           millis() - lastPeakTime > 2000) { // Mínimo 2s entre respiraciones
          peaksInPeriod++;
          lastPeakTime = millis();
          Serial.print("Respiración válida. Amplitud: ");
          Serial.println(currentPeak, 1);
        }
        currentPeak = 0;
      }
      lastNorm = norm;
    } else {
      baseline = raw; // Reset para nuevo contacto
      peaksInPeriod = 0;
    }

    // 5. Reporte RPM mejorado
    if(millis() - lastReport >= (REPORT_INTERVAL*1000)) {
      float rpm = (peaksInPeriod/(float)REPORT_INTERVAL)*60.0;
      
      if(validContact) {
        if(rpm >= 8.0 && rpm <= 30.0) { // Rango más estricto
          Serial.print("RPM: ");
          Serial.println(rpm, 1);
        } else {
          Serial.print("RPM: ");
          Serial.print(rpm, 1);
          Serial.println(" (Revisar contacto)");
        }
      } else {
        Serial.println("RPM: 0.0 (Sin contacto adecuado)");
      }
      
      peaksInPeriod = 0;
      lastReport = millis();
      maxRaw = 0; minRaw = 50000.0; // Reset rango dinámico
    }
  }
}
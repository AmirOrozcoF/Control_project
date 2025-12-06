/*
 * CONTROLADOR PID CON FEEDFORWARD (BASE) - MODIFICADO PARA INPUT SERIAL
 * Base: 135 (Sostenimiento)
 * PID: 2.5, 1.3, 1.2
 * Input: Vía Monitor Serial
 */

// ===== PINES =====
const int trigPin = 7;
const int echoPin = 8;
const int PWM_PIN = 9;

// ===== FILTROS =====
const int NUM_READINGS = 5; 
float readings[NUM_READINGS];
int readIndex = 0;
const float ALPHA = 0.3; // Filtro suave
float filteredHeight = 0;

// ===== PID TUNING (TUS VALORES) =====
double Kp = 2.5;   // Reacción suave pero firme
double Ki = 1.3;   // Corrección fina de error estático
double Kd = 1.2;   // Freno para estabilidad

// ===== CONFIGURACIÓN FÍSICA =====
double pwm_base = 90.0; // Valor para flotar (Feedforward)

// ===== VARIABLES =====
double setpoint = 0;      // INICIA EN 0 POR SEGURIDAD
double input_height = 0;
double output_pwm = 0;

// PID Internals
double error, lastError = 0;
double integral = 0;
unsigned long lastTime = 0;
unsigned long startTime = 0;

// Límites
const int PWM_MIN = 110; 
const int PWM_MAX = 220; 

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  
  // Inicializar buffers del filtro
  float firstRead = readRawSonar();
  for (int i = 0; i < NUM_READINGS; i++) readings[i] = firstRead;
  filteredHeight = firstRead;
  
  // Motores apagados al inicio
  analogWrite(PWM_PIN, 0);
  
  delay(1000); 
  startTime = millis();
  
  Serial.println(F("SISTEMA LISTO."));
  Serial.println(F("Escribe la altura (ej: 15) y pulsa Enter."));
  Serial.println(F("Escribe 0 para APAGAR."));
  Serial.println(F("Tiempo(ms),Setpoint(cm),Altura(cm),PWM")); // Header para CSV
}

void loop() {
  // 1. LEER MONITOR SERIAL (NUEVO BLOQUE)
  if (Serial.available() > 0) {
    float val = Serial.parseFloat();
    // Limpiar buffer de saltos de línea
    while(Serial.available()) Serial.read();
    
    if (val < 2.0) { 
      // APAGADO DE EMERGENCIA O RESET
      setpoint = 0;
      integral = 0; // Importante: borrar memoria del PID
      output_pwm = 0;
      analogWrite(PWM_PIN, 0);
    } else {
      setpoint = val;
    }
  }

  // 2. LOOP DE CONTROL (50Hz)
  unsigned long now = millis();
  
  if (now - lastTime >= 20) { 
    double dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // --- LECTURA Y FILTRADO ---
    float raw = readRawSonar();
    float median = getMedian(raw);
    filteredHeight = (ALPHA * median) + ((1.0 - ALPHA) * filteredHeight);
    input_height = filteredHeight;

    // Si el setpoint es 0, mantenemos todo apagado y saltamos el cálculo PID
    if (setpoint == 0) {
        analogWrite(PWM_PIN, 0);
        // Seguimos enviando datos para verificar que el sensor lee bien en reposo
        Serial.print(now - startTime);
        Serial.print(",");
        Serial.print(0); // Setpoint 0
        Serial.print(",");
        Serial.print(input_height);
        Serial.println(",0"); // PWM 0
        return; 
    }

    // --- PID ---
    error = setpoint - input_height;

    // Anti-windup
    if (output_pwm < PWM_MAX && output_pwm > PWM_MIN) {
        integral += (error * dt);
    } 
    
    // Derivada
    double derivative = (error - lastError) / dt;
    
    // CALCULO FINAL: BASE + PID
    double PID_out = pwm_base + (Kp * error) + (Ki * integral) + (Kd * derivative);
    output_pwm = PID_out;

    // --- SALIDA ---
    if (output_pwm > PWM_MAX) output_pwm = PWM_MAX;
    if (output_pwm < PWM_MIN) output_pwm = PWM_MIN;

    analogWrite(PWM_PIN, (int)output_pwm);
    lastError = error;

    // --- LOG (Mismo formato CSV) ---
    Serial.print(now - startTime);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(input_height);
    Serial.print(",");
    Serial.println(output_pwm);
  }
}

// === AUXILIARES ===
float readRawSonar() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 18000); 
  if (duration == 0) return readings[readIndex]; 
  return (duration / 2.0) / 29.1 + 1.5; 
}

float getMedian(float newVal) {
  readings[readIndex] = newVal;
  readIndex = (readIndex + 1) % NUM_READINGS;
  float sorted[NUM_READINGS];
  for (int i=0; i<NUM_READINGS; i++) sorted[i] = readings[i];
  for (int i=0; i<NUM_READINGS-1; i++) {
    for (int j=0; j<NUM_READINGS-i-1; j++) {
      if (sorted[j] > sorted[j+1]) {
        float temp = sorted[j]; sorted[j] = sorted[j+1]; sorted[j+1] = temp;
      }
    }
  }
  return sorted[NUM_READINGS/2];
}
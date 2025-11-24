/*
 * CONTROLADOR PID CON FEEDFORWARD (BASE)
 * Solución al problema de "subida y bajada":
 * - Se añade un PWM_BASE de 135 (aprox lo necesario para flotar).
 * - El PID ahora trabaja mucho más relajado.
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

// ===== PID TUNING (RECALCULADO PARA BASE 135) =====
// Al tener una base, ya no necesitamos Kp gigantes.
double Kp = 2.5;   // Reacción suave pero firme
double Ki = 1.3;   // Corrección fina de error estático
double Kd = 1.2;   // Freno para estabilidad (ahora sí funcionará bien)

// ===== CONFIGURACIÓN FÍSICA =====
double pwm_base = 135.0; // ¡EL SECRETO! Valor aproximado para flotar (hover)
// Si el dron es muy pesado y no sube, aumenta esto a 140.
// Si sube solo sin parar, bájalo a 130.

// ===== VARIABLES =====
double setpoint = 15.0;   
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
  
  // Inicializar buffers
  float firstRead = readRawSonar();
  for (int i = 0; i < NUM_READINGS; i++) readings[i] = firstRead;
  filteredHeight = firstRead;
  
  delay(2000); 
  startTime = millis();
  
  Serial.println("Tiempo(ms),Setpoint(cm),Altura(cm),PWM");
}

void loop() {
  unsigned long now = millis();
  
  if (now - lastTime >= 20) { // 50Hz
    double dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // 1. LECTURA
    float raw = readRawSonar();
    float median = getMedian(raw);
    filteredHeight = (ALPHA * median) + ((1.0 - ALPHA) * filteredHeight);
    input_height = filteredHeight;

    // 2. PID
    error = setpoint - input_height;

    // Anti-windup
    if (output_pwm < PWM_MAX && output_pwm > PWM_MIN) {
        integral += (error * dt);
    } 
    
    // Derivada
    double derivative = (error - lastError) / dt;
    
    // CALCULO FINAL: BASE + PID
    // Aquí está la magia: Base sostiene, PID corrige.
    double PID_out = pwm_base + (Kp * error) + (Ki * integral) + (Kd * derivative);
    output_pwm = PID_out;

    // 3. SALIDA
    if (output_pwm > PWM_MAX) output_pwm = PWM_MAX;
    if (output_pwm < PWM_MIN) output_pwm = PWM_MIN;

    analogWrite(PWM_PIN, (int)output_pwm);
    lastError = error;

    // 4. LOG
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
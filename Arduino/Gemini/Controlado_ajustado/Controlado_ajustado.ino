/*
 * CONTROLADOR PID - VERSIÓN "RESPUESTA RÁPIDA"
 * Ajustes:
 * - Kp AUMENTADO: Para reacción inmediata al error.
 * - Kd REDUCIDO: Para evitar que el ruido frene los motores.
 * - Filtro Mediana: MANTENIDO (Es vital para tu sensor).
 */

// ===== PINES =====
const int trigPin = 7;
const int echoPin = 8;
const int PWM_PIN = 9;

// ===== FILTROS =====
const int NUM_READINGS = 5; 
float readings[NUM_READINGS];
int readIndex = 0;
// Filtro exponencial un poco más rápido (0.4) para que el PID vea el cambio antes
const float ALPHA = 0.4; 
float filteredHeight = 0;

// ===== PID TUNING (NUEVOS VALORES) =====
double Kp = 6;   // Antes 1.2 -> ¡Más fuerza de reacción!
double Ki = 0.8;   // Antes 0.35 -> Un poco más de empuje acumulado
double Kd = 8;   // Antes 4.0 -> ¡Menos freno! (Menos sensible al ruido)

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
const int PWM_MAX = 220; // Le damos un poco más de techo por si acaso

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

    // Anti-windup inteligente:
    // Si el PWM está al máximo y queremos subir más, NO acumular error (evita latigazo)
    // Si el PWM está al mínimo y queremos bajar más, NO acumular error
    if (output_pwm < PWM_MAX && output_pwm > PWM_MIN) {
        integral += (error * dt);
    } else if (output_pwm >= PWM_MAX && error < 0) { // Si saturó arriba pero el error baja, permitir salir
        integral += (error * dt);
    } else if (output_pwm <= PWM_MIN && error > 0) { // Si saturó abajo pero el error sube, permitir salir
        integral += (error * dt);
    }
    
    // Derivada (Calculada sobre la entrada filtrada para menos ruido "Derivative on Measurement")
    // Nota: Usualmente D = Kd * (error - lastError). 
    // Para reducir "patada" al cambiar setpoint, se usa -Kd * (input - lastInput).
    // Pero como tu setpoint es constante, da igual. Usaremos la estándar.
    double derivative = (error - lastError) / dt;
    
    double PID_out = (Kp * error) + (Ki * integral) + (Kd * derivative);
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
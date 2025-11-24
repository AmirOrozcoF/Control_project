/*
 * CONTROLADOR PID ESTABILIZADO - VERSIÓN "ANTI-BRINQUITOS"
 * Mejoras:
 * 1. Filtro de Mediana (Elimina los picos locos de 37cm)
 * 2. PID Suavizado (Menos agresivo)
 */

// ===== PINES =====
const int trigPin = 7;
const int echoPin = 8;
const int PWM_PIN = 9;

// ===== FILTRO DE MEDIANA (Para eliminar ruido) =====
const int NUM_READINGS = 5; // Ventana de 5 muestras
float readings[NUM_READINGS]; // Buffer
int readIndex = 0;

// ===== FILTRO EXPONENCIAL (Para suavizar) =====
const float ALPHA = 0.3; // 0.3 = Suave, 0.8 = Rápido
float filteredHeight = 0;

// ===== PID TUNING =====
double Kp = 1.2;   // Bajé un poco para evitar sobrereacción inicial
double Ki = 0.35;  // Corrección lenta pero segura
double Kd = 4.0;   // Amortiguación

// ===== VARIABLES =====
double setpoint = 15.0;   // Altura objetivo de prueba
double input_height = 0;
double output_pwm = 0;

// PID Internals
double error, lastError = 0;
double integral = 0;
unsigned long lastTime = 0;
unsigned long startTime = 0;

// Límites
const int PWM_MIN = 110; 
const int PWM_MAX = 200; 

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  
  // Inicializar el buffer del filtro con ceros o primera lectura
  float firstRead = readRawSonar();
  for (int i = 0; i < NUM_READINGS; i++) {
    readings[i] = firstRead;
  }
  filteredHeight = firstRead;
  
  delay(2000); // Tiempo para prepararse
  startTime = millis();
  
  Serial.println("Tiempo(ms),Setpoint(cm),Altura(cm),PWM");
}

void loop() {
  unsigned long now = millis();
  
  // 50Hz Loop (20ms)
  if (now - lastTime >= 20) {
    double dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // --- 1. LECTURA ROBUSTA ---
    float raw = readRawSonar();
    float median = getMedian(raw); // Paso 1: Eliminar picos
    
    // Paso 2: Filtro Suave (EMA)
    filteredHeight = (ALPHA * median) + ((1.0 - ALPHA) * filteredHeight);
    input_height = filteredHeight;

    // --- 2. CONTROL PID ---
    error = setpoint - input_height;

    // Anti-windup: Solo integrar si no estamos saturados al máximo
    // O si el error ayuda a salir de la saturación
    if (output_pwm < PWM_MAX || (error < 0)) {
       if (output_pwm > PWM_MIN || (error > 0)) {
          integral += (error * dt);
       }
    }
    
    double derivative = (error - lastError) / dt;
    
    // Calcular Salida
    double PID_out = (Kp * error) + (Ki * integral) + (Kd * derivative);
    output_pwm = PID_out;

    // --- 3. SATURACIÓN ---
    if (output_pwm > PWM_MAX) output_pwm = PWM_MAX;
    if (output_pwm < PWM_MIN) output_pwm = PWM_MIN;

    analogWrite(PWM_PIN, (int)output_pwm);
    lastError = error;

    // --- 4. DATOS ---
    Serial.print(now - startTime);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(input_height); // Enviamos la altura filtrada real
    Serial.print(",");
    Serial.println(output_pwm);
  }
}

// Lectura cruda del sensor
float readRawSonar() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 18000); // Timeout reducido (aprox 3 metros)
  
  if (duration == 0) return readings[readIndex]; // Si falla, devuelve el último dato del buffer
  
  float cm = (duration / 2.0) / 29.1;
  return cm + 1.5; 
}

// Algoritmo de Mediana (Ordena y saca el valor central)
float getMedian(float newVal) {
  // Insertar nuevo valor en buffer circular
  readings[readIndex] = newVal;
  readIndex = (readIndex + 1) % NUM_READINGS;
  
  // Copiar a array temporal para ordenar
  float sorted[NUM_READINGS];
  for (int i = 0; i < NUM_READINGS; i++) {
    sorted[i] = readings[i];
  }
  
  // Ordenar (Bubble sort simple para array pequeño)
  for (int i = 0; i < NUM_READINGS - 1; i++) {
    for (int j = 0; j < NUM_READINGS - i - 1; j++) {
      if (sorted[j] > sorted[j + 1]) {
        float temp = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = temp;
      }
    }
  }
  
  // Retornar el valor del medio
  return sorted[NUM_READINGS / 2];
}
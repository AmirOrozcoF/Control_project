/*
 * CONTROLADOR PID - MODO PRUEBA DE DATOS
 * Compatible con Auto_PWM_vs_H.py
 * * - Filtro: EMA (Idéntico a Nuevo_escalon_filtrado.ino)
 * - Salida: Formato CSV para Excel/Python
 * - Objetivo: 15cm (Automático, no requiere escribir en consola)
 */

// ===== PINES =====
const int trigPin = 7;
const int echoPin = 8;
const int PWM_PIN = 9;

// ===== PARÁMETROS DEL FILTRO (Copiados de tu código) =====
const float FS = 50.0;                    // 50 Hz
const float T_SAMPLE_S = 1.0 / FS;        // 0.02s
const float FC = 5.5;                     // Frecuencia de corte 5.5 Hz
const float TAU_F = 1.0 / (2.0 * PI * FC);
const float ALPHA = T_SAMPLE_S / (TAU_F + T_SAMPLE_S); 

// ===== PID TUNING (Ajustado para menos oscilación) =====
// He bajado Kp y Kd. Si sigue oscilando, bajaremos más Kp.
double Kp = 1.8;   // Antes 2.5
double Ki = 0.4;   // Integra el error (fuerza la llegada a la consigna)
double Kd = 3.5;   // Freno suave

// ===== VARIABLES =====
double setpoint = 15.0;   // <--- ALTURA FIJA DE PRUEBA (15 cm)
double input_height = 0;
double output_pwm = 0;
float filteredHeight = 0;

// Variables PID
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
  
  // Inicializar altura
  filteredHeight = readSonar();
  input_height = filteredHeight;
  
  // Espera de seguridad de 2 segundos antes de arrancar
  delay(2000);
  startTime = millis();
  
  // Encabezado para el CSV (Python lo guardará así)
  Serial.println("Tiempo(ms),Setpoint(cm),Altura(cm),PWM");
}

void loop() {
  unsigned long now = millis();
  
  // Ejecutar bucle cada 20ms (50Hz) aprox para coincidir con el filtro
  if (now - lastTime >= 20) {
    double dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // 1. LEER Y FILTRAR
    float rawHeight = readSonar();
    // Tu filtro original:
    filteredHeight = (ALPHA * rawHeight) + ((1.0 - ALPHA) * filteredHeight);
    input_height = filteredHeight;

    // 2. CONTROL PID
    error = setpoint - input_height;

    // Acumular integral solo si no estamos saturados (Anti-windup simple)
    if (output_pwm < PWM_MAX && output_pwm > PWM_MIN) {
      integral += (error * dt);
    }
    
    // Derivada
    double derivative = (error - lastError) / dt;
    
    // Salida PID
    double PID_out = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    output_pwm = PID_out;

    // 3. LIMITADORES (SATURACIÓN)
    if (output_pwm > PWM_MAX) output_pwm = PWM_MAX;
    if (output_pwm < PWM_MIN) output_pwm = PWM_MIN; // Mantiene rotación mínima
    
    // Si el error es muy grande (ej. al inicio), permitir arranque fuerte
    // Pero si estamos muy cerca y queremos apagar, se puede poner lógica extra.
    // Por ahora, control continuo.

    analogWrite(PWM_PIN, (int)output_pwm);
    lastError = error;

    // 4. ENVIAR DATOS A PYTHON
    Serial.print(now - startTime); // Tiempo relativo
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(input_height);
    Serial.print(",");
    Serial.println(output_pwm);
  }
}

float readSonar() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000); 
  if (duration == 0) return filteredHeight; 
  float cm = (duration / 2.0) / 29.1;
  return cm + 1.5; // Tu offset
}
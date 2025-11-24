/*
 * CONTROLADOR DE ALTURA PARA DRON - PID SIMPLE
 * * INSTRUCCIONES:
 * 1. Sube este código al Arduino.
 * 2. Abre el Monitor Serial (Asegúrate que esté a 115200 baudios).
 * 3. Escribe la altura deseada en cm y presiona Enter (ej: 20).
 * 4. Para apagar de emergencia, envía un 0.
 */

// ===== CONFIGURACIÓN DE PINES =====
const int trigPin = 7;
const int echoPin = 8;
const int PWM_PIN = 9;

// ===== PARÁMETROS DEL PID (SINTONIZACIÓN) =====
// Estos valores son estimados basándome en tus datos "Navidad".
// Si oscila mucho, baja Kp. Si es muy lento, sube Kp.
double Kp = 2.5;   // Fuerza de reacción inmediata
double Ki = 0.3;   // Corrección de error acumulado (elimina error constante)
double Kd = 5.0;   // Freno para evitar oscilaciones bruscas

// ===== LÍMITES DE SEGURIDAD =====
const int PWM_MIN = 110;  // PWM mínimo para que los motores giren (zona muerta)
const int PWM_MAX = 200;  // PWM máximo por seguridad (no quemar motores/golpear techo)
const int PWM_BASE = 0;   // Base, se ajustará sola con el PID

// ===== VARIABLES DEL SISTEMA =====
double setpoint = 0;      // Altura deseada (cm) - Inicia en 0 por seguridad
double input_height = 0;  // Altura actual medida
double output_pwm = 0;    // Señal enviada a los motores

// Variables internas del PID
double error, lastError = 0;
double integral = 0;
unsigned long lastTime = 0;

// Filtro (Tomado de tu código Nuevo_escalon_filtrado)
float filteredHeight = 0;
const float ALPHA = 0.2; // Factor de suavizado (0.0 a 1.0)

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  
  // Inicializar motores apagados
  analogWrite(PWM_PIN, 0);
  
  Serial.println(F("=== CONTROLADOR DE ALTURA LISTO ==="));
  Serial.println(F("Escribe una altura en cm (ej: 15.0) para iniciar."));
  Serial.println(F("Escribe 0 para APAGAR."));
  
  filteredHeight = readSonar(); // Lectura inicial
}

void loop() {
  // 1. LEER COMANDOS DEL USUARIO (MONITOR SERIAL)
  if (Serial.available() > 0) {
    float val = Serial.parseFloat();
    // Limpiar el buffer de cualquier caracter extra (como saltos de linea)
    while(Serial.available()) Serial.read(); 
    
    if (val <= 2.0) { // Si envían 0 o un valor muy bajo, apagamos todo
      setpoint = 0;
      integral = 0; // Reiniciar acumulador
      analogWrite(PWM_PIN, 0);
      Serial.println(F("SISTEMA APAGADO / MOTOR DETENIDO"));
    } else {
      setpoint = val;
      Serial.print(F("Nueva altura objetivo: "));
      Serial.print(setpoint);
      Serial.println(F(" cm"));
    }
  }

  // Si el setpoint es 0, no hacemos nada más
  if (setpoint == 0) {
    analogWrite(PWM_PIN, 0);
    delay(50);
    return;
  }

  // 2. MEDICIÓN DE TIEMPO
  unsigned long now = millis();
  double dt = (double)(now - lastTime) / 1000.0; // Tiempo en segundos
  
  // Ejecutar el PID cada 50ms (20Hz) aproximadamente para estabilidad
  if (dt >= 0.05) { 
    lastTime = now;

    // 3. LEER SENSOR Y FILTRAR
    float rawHeight = readSonar();
    // Filtro Exponencial (EMA) para reducir ruido del sensor
    filteredHeight = (ALPHA * rawHeight) + ((1.0 - ALPHA) * filteredHeight);
    input_height = filteredHeight;

    // 4. CÁLCULO DEL PID
    error = setpoint - input_height;
    
    // Término Integral (Acumula el error en el tiempo)
    // Anti-windup: limitamos la integral para que no crezca infinito si sostenemos el dron
    if(output_pwm < PWM_MAX && output_pwm > PWM_MIN) {
        integral += (error * dt);
    }

    // Término Derivativo (Cambio del error)
    double derivative = (error - lastError) / dt;

    // Fórmula PID completa
    double PID_output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    // Calcular PWM final
    output_pwm = PID_output; 

    // 5. SATURACIÓN Y SALIDA (Límites físicos)
    // Mapeamos o limitamos el valor. 
    // Nota: El PID da un valor de corrección. Si el sistema necesita un PWM base para despegar,
    // el término Integral se encargará de encontrarlo automáticamente.
    
    if (output_pwm > PWM_MAX) output_pwm = PWM_MAX;
    if (output_pwm < PWM_MIN) output_pwm = PWM_MIN; // Mantiene los motores girando mínimo

    // Seguridad extra: Si la altura objetivo es alcanzada y el error es muy bajo, mantener.
    // Pero si el usuario pidió 0, ya lo manejamos arriba.
    
    analogWrite(PWM_PIN, (int)output_pwm);
    lastError = error;

    // 6. DEBUG (Para ver en el Serial Plotter)
    Serial.print("Target:");
    Serial.print(setpoint);
    Serial.print(" Actual:");
    Serial.print(input_height);
    Serial.print(" PWM:");
    Serial.println(output_pwm);
  }
}

// Función auxiliar de lectura del sensor (basada en tu código)
float readSonar() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout 30ms para no bloquear
  if (duration == 0) return filteredHeight; // Si falla, retorna el último valor válido
  
  float cm = (duration / 2.0) / 29.1;
  // Corrección física (offset) si es necesario, 
  // tu código anterior tenía +1.5, puedes ajustarlo aquí:
  return cm + 1.5; 
}
const int trigPin = 7;
const int echoPin = 8;
const int PWM = 9;

// ===== CONFIGURACIÓN DEL EXPERIMENTO =====
const int PWM_HOVER = 183;
const int PWM_STEP = 193;     // Cambia a 155 para escalón negativo
const int PWM_OFF = 0;

// Tiempos en milisegundos
const unsigned long T_BASELINE = 10000;   // 10 segundos
const unsigned long T_STEP = 25000;       // hasta 25s
const unsigned long T_RETURN = 35000;     // hasta 35s
const unsigned long T_END = 40000;        // hasta 40s

// ===== MODOS DE OPERACIÓN =====
bool modoAutomatico = false;
unsigned long tiempoInicio = 0;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(PWM, OUTPUT);
  
  Serial.println("=== SISTEMA DE CONTROL DE DRON ===");
  Serial.println("Modo Manual: Ingrese duty cycle (0.0 - 1.0)");
  Serial.println("Modo Automatico: Escriba 'AUTO' para iniciar experimento");
  Serial.println("Escriba 'STOP' para detener y volver a manual");
}

void loop() {
  // ---- Medir distancia ----
  long duration;
  float cm;  // ← CAMBIADO A FLOAT
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  cm = microsecondsToCentimeters(duration);  // Ahora retorna float
  
  // ---- Determinar PWM según modo ----
  static int pwmValue = 0;
  unsigned long tiempoActual = millis();
  
  if (modoAutomatico) {
    unsigned long tiempoTranscurrido = tiempoActual - tiempoInicio;
    
    if (tiempoTranscurrido < T_BASELINE) {
      pwmValue = PWM_HOVER;
    }
    else if (tiempoTranscurrido < T_STEP) {
      pwmValue = PWM_STEP;
    }
    else if (tiempoTranscurrido < T_RETURN) {
      pwmValue = PWM_HOVER;
    }
    else if (tiempoTranscurrido < T_END) {
      pwmValue = PWM_OFF;
    }
    else {
      // Experimento terminado
      modoAutomatico = false;
      pwmValue = 0;
      Serial.println("\n=== EXPERIMENTO COMPLETADO ===");
      Serial.println("Volviendo a modo manual...");
    }
  }
  else {
    // Modo manual: leer desde Serial
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      
      if (input.equalsIgnoreCase("AUTO")) {
        modoAutomatico = true;
        tiempoInicio = millis();
        Serial.println("\n=== INICIANDO EXPERIMENTO AUTOMATICO ===");
        Serial.println("tiempo_ms, pwm, distancia_cm");
        return;
      }
      else if (input.equalsIgnoreCase("STOP")) {
        modoAutomatico = false;
        pwmValue = 0;
        Serial.println("\n=== EXPERIMENTO DETENIDO ===");
        return;
      }
      else {
        float dutyCycle = input.toFloat();
        if (dutyCycle >= 0.0 && dutyCycle <= 1.0) {
          pwmValue = (int)(dutyCycle * 255);
        }
      }
    }
  }
  
  // ---- Aplicar PWM ----
  analogWrite(PWM, pwmValue);
  
  // ---- Mostrar datos ----
  if (modoAutomatico) {
    // Formato CSV para MATLAB con decimales
    unsigned long tiempoExp = tiempoActual - tiempoInicio;
    Serial.print(tiempoExp);
    Serial.print(", ");
    Serial.print(pwmValue);  // PWM siempre es entero (0-255)
    Serial.print(", ");
    Serial.println(cm, 2);  // ← 2 decimales para distancia
  }
  else {
    // Formato legible humano
    Serial.print("Distancia: ");
    Serial.print(cm, 2);  // ← 2 decimales
    Serial.print(" cm | PWM: ");
    Serial.print(pwmValue);
    Serial.print(" (");
    Serial.print((pwmValue * 100.0) / 255.0, 1);  // ← 1 decimal
    Serial.println("%)");
  }
  
  delay(50);  // 20 Hz de muestreo
}

float microsecondsToCentimeters(long microseconds) {  // ← RETORNA FLOAT
  return microseconds / 29.0 / 2.0;  // ← División con decimales
}
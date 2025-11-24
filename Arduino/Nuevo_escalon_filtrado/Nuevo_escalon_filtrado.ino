// ============================================================================
// SISTEMA DE CONTROL - ESCALÓN ÚNICO 62% → 72%
// Con Filtro Digital EMA y Validaciones Mejoradas
// ============================================================================

// ===== PINES =====
const int trigPin = 7;
const int echoPin = 8;
const int PWM = 9;

// ===== CONFIGURACIÓN DEL EXPERIMENTO =====
const int PWM_INICIO = 133;  // 62% = 158/255
const int PWM_FINAL = 146;   // 70% = 178/255

// ===== PARÁMETROS DE MUESTREO =====
const float FS = 50.0;                    // Hz - Frecuencia de muestreo
const unsigned long T_SAMPLE_MS = 20;     // Exactamente 20 ms
const float T_SAMPLE_S = 0.02;            // Período en segundos

// ===== PARÁMETROS DEL FILTRO PASABAJOS (EMA) =====
const float FC = 5.5;                     // Hz - Frecuencia de corte
const float TAU_F = 1.0 / (2.0 * PI * FC); // Constante de tiempo del filtro
const float ALPHA = T_SAMPLE_S / (TAU_F + T_SAMPLE_S);  // Coeficiente EMA

// ===== CONFIGURACIÓN DEL SENSOR HC-SR04 =====
const float CM_MIN = 2.0;
const float CM_MAX = 30.0;                // Máximo 30 cm
const float CM_OFFSET = 1.5;
const float MAX_CAMBIO_CM = 4.0;          // Máximo cambio permitido entre lecturas

// ===== VARIABLES GLOBALES =====
unsigned long startTime = 0;
unsigned long ultimoMuestreo = 0;
int currentPWM = 133;
bool testCompleted = false;
bool dataLogging = false;

// Variables del filtro
float altura_filtrada = 0.0;
bool filtro_inicializado = false;

// Variables de robustez
float ultimaLecturaValida = 0.0;
float ultimaLecturaCruda = 0.0;

// ===== FUNCIÓN: CONVERSIÓN MICROSEGUNDOS A CENTÍMETROS =====
float microsecondsToCentimeters(long microseconds) {
  return microseconds / 29.0 / 2.0;
}

// ===== FUNCIÓN: LEER DISTANCIA CON VALIDACIÓN =====
float leerDistancia(bool *exitoso) {
  *exitoso = false;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  
  if (duration == 0) {
    return -1.0;
  }
  
  float cm = microsecondsToCentimeters(duration) + CM_OFFSET;
  
  // Validar rango (2-30 cm)
  if (cm < CM_MIN || cm > CM_MAX) {
    return -1.0;
  }
  
  // Validar cambios bruscos (más de 4 cm de diferencia)
  if (ultimaLecturaCruda > 0) {
    float cambio = abs(cm - ultimaLecturaCruda);
    if (cambio > MAX_CAMBIO_CM) {
      return -1.0;  // Rechazar lectura con cambio muy brusco
    }
  }
  
  *exitoso = true;
  ultimaLecturaCruda = cm;  // Actualizar última lectura cruda válida
  return cm;
}

// ===== FUNCIÓN: APLICAR FILTRO PASABAJOS (EMA) =====
float aplicarFiltro(float valor_crudo) {
  if (!filtro_inicializado) {
    altura_filtrada = valor_crudo;
    filtro_inicializado = true;
  } else {
    altura_filtrada = ALPHA * valor_crudo + (1.0 - ALPHA) * altura_filtrada;
  }
  
  return altura_filtrada;
}

// ===== FUNCIÓN: OBTENER ALTURA ROBUSTA =====
float obtenerAltura(bool *valida) {
  *valida = false;
  
  bool exitoso;
  float distancia = leerDistancia(&exitoso);
  
  if (exitoso) {
    ultimaLecturaValida = distancia;
    *valida = true;
    return aplicarFiltro(distancia);
  } else {
    // No hay lectura válida, pero no detenemos el experimento
    // Simplemente no imprimimos esta muestra
    return -1.0;
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(PWM, OUTPUT);
  
  // Esperar 5 segundos antes de iniciar
  delay(5000);
  
  // Ir a 100% durante 3 segundos
  analogWrite(PWM, 170);
  delay(3000);
  
  // Volver a 0%
  analogWrite(PWM, 0);
  delay(1000);
  
  // Ir a 62% durante 10 segundos sin tomar datos
  analogWrite(PWM, 133);
  delay(10000);
  
  // Iniciar toma de datos
  startTime = millis();
  ultimoMuestreo = startTime;
  dataLogging = true;
  
  Serial.println(F("Tiempo(ms),PWM,Altura(cm)"));
}

// ===== LOOP =====
void loop() {
  if (testCompleted) {
    return;
  }
  
  unsigned long currentTime = millis();
  
  // Control de frecuencia de muestreo a 50 Hz
  if (currentTime - ultimoMuestreo < T_SAMPLE_MS) {
    return;
  }
  ultimoMuestreo += T_SAMPLE_MS;
  
  unsigned long tiempoTranscurrido = currentTime - startTime;
  
  // Verificar si pasaron 10 segundos para subir a 70%
  if (tiempoTranscurrido >= 10000 && currentPWM == 133) {
    currentPWM = 146;
    analogWrite(PWM, currentPWM);
  }
  
  // Verificar si pasaron 60 segundos para terminar (10s + 50s)
  if (tiempoTranscurrido >= 60000) {
    testCompleted = true;
    analogWrite(PWM, 0);
    Serial.println();
    Serial.println(F("=== EXPERIMENTO COMPLETADO ==="));
    return;
  }
  
  // Tomar y guardar datos continuamente
  if (dataLogging) {
    bool valida;
    float altura = obtenerAltura(&valida);
    
    // Solo imprimir si la lectura es válida
    if (valida) {
      Serial.print(tiempoTranscurrido);
      Serial.print(",");
      Serial.print(currentPWM);
      Serial.print(",");
      Serial.println(altura, 2);
    }
    // Si no es válida, simplemente no imprime nada y continúa
  }
}
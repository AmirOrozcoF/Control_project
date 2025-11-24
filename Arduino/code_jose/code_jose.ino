// ============================================================================
// SISTEMA DE CONTROL Y ADQUISICIÓN DE DATOS - DRON CON ESTACAS
// Versión Mejorada con Filtro Digital y Muestreo Preciso
// ============================================================================
//
// PARÁMETROS PRINCIPALES DEL SISTEMA:
// -----------------------------------
// • Frecuencia de muestreo (Fs):     50 Hz (período = 20 ms)
// • Frecuencia de corte filtro (Fc): 5.5 Hz
// • Coeficiente filtro EMA (α):      ~0.26
// • PWM Hover (baseline):            183 (duty ~72%)
// • PWM Escalón positivo:            193 (duty ~76%)
// • PWM Escalón negativo:            173 (duty ~68%)
// • Rango sensor HC-SR04:            2-50 cm
// • Offset de calibración:           +1.5 cm
//
// SECUENCIA DEL EXPERIMENTO AUTOMÁTICO:
// --------------------------------------
// 1. Colocar el dron sobre las estacas
// 2. Conectar Arduino al PC y abrir Monitor Serial (115200 baud)
// 3. Enviar comando:
//    • "AUTO_UP"   → Experimento con escalón positivo (183→193→183→0)
//    • "AUTO_DOWN" → Experimento con escalón negativo (183→173→183→0)
// 4. El sistema ejecutará la secuencia automáticamente:
//    - 0-10s:  PWM en hover (183) - estabilización baseline
//    - 10-25s: PWM en escalón (193 o 173) - respuesta del sistema
//    - 25-35s: PWM retorna a hover (183) - recuperación
//    - 35-40s: PWM apagado (0) - finalización
// 5. Al terminar, copiar datos del Monitor Serial a archivo CSV
// 6. Guardar como: experimento_01.csv, experimento_02.csv, etc.
// 7. Formato de datos: tiempo_ms, pwm, altura_cm
//
// COMANDOS DISPONIBLES:
// ---------------------
// • AUTO_UP   : Iniciar experimento con escalón positivo
// • AUTO_DOWN : Iniciar experimento con escalón negativo
// • STOP      : Detener experimento en curso
// • 0.0-1.0   : Control manual (duty cycle), ej: 0.72 para hover
//
// ============================================================================

// ===== PINES =====
const int trigPin = 7;
const int echoPin = 8;
const int PWM = 9;

// ===== CONFIGURACIÓN DEL EXPERIMENTO =====
const int PWM_HOVER = 162;        // PWM de hover
const int PWM_STEP_UP = 173;      // Escalón positivo (+10)
const int PWM_STEP_DOWN = 153;    // Escalón negativo (-10)
const int PWM_OFF = 0;

// Tiempos en milisegundos
const unsigned long T_BASELINE = 10000;   // 10 segundos baseline
const unsigned long T_STEP = 25000;       // hasta 25s (escalón)
const unsigned long T_RETURN = 35000;     // hasta 35s (retorno hover)
const unsigned long T_END = 40000;        // hasta 40s (apagar)

// ===== PARÁMETROS DE MUESTREO (Diseño Teórico) =====
const float FS = 50.0;                    // Hz - Frecuencia de muestreo
const unsigned long T_SAMPLE_MS = 20;     // Exactamente 20 ms
const float T_SAMPLE_S = 0.02;            // Período en segundos

// ===== PARÁMETROS DEL FILTRO PASABAJOS (Diseño Teórico) =====
const float FC = 5.5;                     // Hz - Frecuencia de corte
const float TAU_F = 1.0 / (2.0 * PI * FC); // Constante de tiempo del filtro
const float ALPHA = T_SAMPLE_S / (TAU_F + T_SAMPLE_S);  // Coeficiente EMA

// ===== CONFIGURACIÓN DEL SENSOR HC-SR04 =====
const float CM_MIN = 2.0;                 // Distancia mínima válida (cm)
const float CM_MAX = 50.0;                // Distancia máxima válida (cm)
const float CM_OFFSET = 1.5;              // Offset de calibración (+1.5 cm)
const int MAX_FALLOS_CONSECUTIVOS = 5;    // Máximo de lecturas fallidas seguidas

// ===== VARIABLES GLOBALES =====
bool modoAutomatico = false;
unsigned long tiempoInicio = 0;
unsigned long ultimoMuestreo = 0;
int tipoEscalon = PWM_STEP_UP;            // Tipo de escalón a ejecutar

// Variables del filtro
float altura_filtrada = 0.0;
bool filtro_inicializado = false;

// Variables de robustez
int contadorFallos = 0;
float ultimaLecturaValida = 0.0;

// ===== FUNCIÓN: CONVERSIÓN MICROSEGUNDOS A CENTÍMETROS =====
float microsecondsToCentimeters(long microseconds) {
  return microseconds / 29.0 / 2.0;
}

// ===== FUNCIÓN: SETUP =====
void setup() {
  Serial.begin(115200);  // Aumentado a 115200 baud para mayor velocidad
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(PWM, OUTPUT);
  
  // Asegurar que el sistema empieza apagado
  analogWrite(PWM, 0);
  
  // Mensaje de bienvenida
  Serial.println(F("========================================"));
  Serial.println(F("  SISTEMA DE CONTROL DE DRON V2.0      "));
  Serial.println(F("  Con Filtro Digital y Muestreo Preciso"));
  Serial.println(F("========================================"));
  Serial.println();
  Serial.println(F("PARAMETROS DEL SISTEMA:"));
  Serial.print(F("  Frecuencia muestreo: ")); Serial.print(FS); Serial.println(F(" Hz"));
  Serial.print(F("  Periodo muestreo:    ")); Serial.print(T_SAMPLE_MS); Serial.println(F(" ms"));
  Serial.print(F("  Frecuencia corte:    ")); Serial.print(FC); Serial.println(F(" Hz"));
  Serial.print(F("  Coef. filtro (α):    ")); Serial.println(ALPHA, 4);
  Serial.print(F("  Offset sensor:       ")); Serial.print(CM_OFFSET); Serial.println(F(" cm"));
  Serial.println();
  Serial.println(F("COMANDOS DISPONIBLES:"));
  Serial.println(F("  AUTO_UP   : Experimento escalon positivo (183→193)"));
  Serial.println(F("  AUTO_DOWN : Experimento escalon negativo (183→173)"));
  Serial.println(F("  STOP      : Detener experimento"));
  Serial.println(F("  0.0-1.0   : Control manual (duty cycle)"));
  Serial.println();
  Serial.println(F("Listo para recibir comandos..."));
  Serial.println();
}

// ===== FUNCIÓN: LEER DISTANCIA CON VALIDACIÓN (MÉTODO CORREGIDO) =====
float leerDistancia(bool *exitoso) {
  *exitoso = false;
  
  // Generar pulso trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Leer duración del eco (sin timeout para coincidir con código de referencia)
  long duration = pulseIn(echoPin, HIGH);
  
  // Verificar lectura válida
  if (duration == 0) {
    return -1.0;  // Error: sin respuesta
  }
  
  // Convertir a centímetros con offset de calibración
  float cm = microsecondsToCentimeters(duration) + CM_OFFSET;
  
  // Validar rango
  if (cm < CM_MIN || cm > CM_MAX) {
    return -1.0;  // Error: fuera de rango
  }
  
  *exitoso = true;
  return cm;
}

// ===== FUNCIÓN: APLICAR FILTRO PASABAJOS (EMA) =====
float aplicarFiltro(float valor_crudo) {
  if (!filtro_inicializado) {
    // Primera lectura: inicializar filtro
    altura_filtrada = valor_crudo;
    filtro_inicializado = true;
  } else {
    // Filtro exponencial: y[n] = α·u[n] + (1-α)·y[n-1]
    altura_filtrada = ALPHA * valor_crudo + (1.0 - ALPHA) * altura_filtrada;
  }
  
  return altura_filtrada;
}

// ===== FUNCIÓN: OBTENER ALTURA ROBUSTA =====
float obtenerAltura() {
  bool exitoso;
  float distancia = leerDistancia(&exitoso);
  
  if (exitoso) {
    // Lectura válida
    contadorFallos = 0;
    ultimaLecturaValida = distancia;
    
    // Aplicar filtro
    return aplicarFiltro(distancia);
  } else {
    // Lectura fallida
    contadorFallos++;
    
    if (contadorFallos >= MAX_FALLOS_CONSECUTIVOS) {
      // Demasiados fallos consecutivos - error crítico
      Serial.println();
      Serial.println(F("ERROR CRITICO: Sensor no responde"));
      Serial.println(F("Verifique conexiones del HC-SR04"));
      modoAutomatico = false;
      analogWrite(PWM, 0);
      return -999.0;  // Valor de error
    }
    
    // Usar última lectura válida (sin actualizar filtro)
    return altura_filtrada;
  }
}

// ===== FUNCIÓN: LOOP PRINCIPAL =====
void loop() {
  unsigned long tiempoActual = millis();
  
  // ===== CONTROL DE FRECUENCIA DE MUESTREO =====
  if (modoAutomatico) {
    // En modo automático, muestrear a frecuencia precisa
    if (tiempoActual - ultimoMuestreo < T_SAMPLE_MS) {
      return;  // No es momento de muestrear
    }
    ultimoMuestreo += T_SAMPLE_MS;  // Incremento fijo para evitar drift
    
    // ===== MÁQUINA DE ESTADOS DEL EXPERIMENTO =====
    unsigned long tiempoTranscurrido = tiempoActual - tiempoInicio;
    int pwmValue = 0;
    
    // Determinar PWM según fase del experimento
    if (tiempoTranscurrido < T_BASELINE) {
      pwmValue = PWM_HOVER;
    }
    else if (tiempoTranscurrido < T_STEP) {
      pwmValue = tipoEscalon;  // PWM_STEP_UP o PWM_STEP_DOWN
    }
    else if (tiempoTranscurrido < T_RETURN) {
      pwmValue = PWM_HOVER;
    }
    else if (tiempoTranscurrido < T_END) {
      pwmValue = PWM_OFF;
    }
    else {
      // Experimento completado
      modoAutomatico = false;
      analogWrite(PWM, 0);
      Serial.println();
      Serial.println(F("=== EXPERIMENTO COMPLETADO ==="));
      Serial.println(F("Copie los datos y guardelos en archivo .csv"));
      Serial.println(F("Use 'experimento_01.csv', 'experimento_02.csv', etc."));
      Serial.println();
      
      // Reiniciar filtro para próximo experimento
      filtro_inicializado = false;
      contadorFallos = 0;
      return;
    }
    
    // Aplicar PWM
    analogWrite(PWM, pwmValue);
    
    // Leer altura (con filtro y validación)
    float altura = obtenerAltura();
    
    // Verificar error crítico
    if (altura == -999.0) {
      return;  // Error ya reportado en obtenerAltura()
    }
    
    // Imprimir datos en formato CSV
    Serial.print(tiempoTranscurrido);
    Serial.print(F(", "));
    Serial.print(pwmValue);
    Serial.print(F(", "));
    Serial.println(altura, 2);
  }
  else {
    // ===== MODO MANUAL =====
    
    // Verificar si hay comandos del usuario
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      input.toUpperCase();
      
      if (input.equals("AUTO_UP")) {
        // Iniciar experimento con escalón positivo
        Serial.println();
        Serial.println(F("=== EXPERIMENTO: ESCALON POSITIVO ==="));
        Serial.print(F("Secuencia: 183 → 193 → 183 → 0  |  fs = "));
        Serial.print(FS); Serial.println(F(" Hz"));
        Serial.println(F("tiempo_ms, pwm, altura_cm"));
        
        modoAutomatico = true;
        tiempoInicio = millis();
        ultimoMuestreo = tiempoInicio;
        tipoEscalon = PWM_STEP_UP;
        filtro_inicializado = false;
        contadorFallos = 0;
      }
      else if (input.equals("AUTO_DOWN")) {
        // Iniciar experimento con escalón negativo
        Serial.println();
        Serial.println(F("=== EXPERIMENTO: ESCALON NEGATIVO ==="));
        Serial.print(F("Secuencia: 183 → 173 → 183 → 0  |  fs = "));
        Serial.print(FS); Serial.println(F(" Hz"));
        Serial.println(F("tiempo_ms, pwm, altura_cm"));
        
        modoAutomatico = true;
        tiempoInicio = millis();
        ultimoMuestreo = tiempoInicio;
        tipoEscalon = PWM_STEP_DOWN;
        filtro_inicializado = false;
        contadorFallos = 0;
      }
      else if (input.equals("STOP")) {
        // Detener cualquier operación
        modoAutomatico = false;
        analogWrite(PWM, 0);
        Serial.println();
        Serial.println(F("=== SISTEMA DETENIDO ==="));
        Serial.println();
      }
      else {
        // Modo manual con duty cycle
        float dutyCycle = input.toFloat();
        if (dutyCycle >= 0.0 && dutyCycle <= 1.0) {
          int pwmValue = (int)(dutyCycle * 255);
          analogWrite(PWM, pwmValue);
          Serial.print(F("PWM ajustado a: "));
          Serial.print(pwmValue);
          Serial.print(F(" ("));
          Serial.print(dutyCycle * 100.0, 1);
          Serial.println(F("%)"));
        }
        else {
          Serial.println(F("Error: Ingrese valor entre 0.0 y 1.0"));
        }
      }
    }
    
    // En modo manual, mostrar información cada 500 ms
    static unsigned long ultimoReporte = 0;
    if (tiempoActual - ultimoReporte >= 500) {
      ultimoReporte = tiempoActual;
      
      bool exitoso;
      float distancia = leerDistancia(&exitoso);
      
      if (exitoso) {
        Serial.print(F("Distancia: "));
        Serial.print(distancia, 2);
        Serial.println(F(" cm"));
      } else {
        Serial.println(F("Error leyendo sensor"));
      }
    }
  }
}
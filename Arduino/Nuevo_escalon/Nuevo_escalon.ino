const int trigPin = 7;
const int echoPin = 8;
const int PWM = 9;

unsigned long startTime = 0;
float lastValidDistance = 0;
bool firstMeasurement = true;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(PWM, OUTPUT);
  
  // Subir a 100% durante 2 segundos (sin imprimir)
  analogWrite(PWM, 255);
  delay(2000);
  
  // Bajar a 0% durante 1 segundo (sin imprimir)
  analogWrite(PWM, 0);
  delay(1000);
  
  // Subir a 68% durante 30 segundos (imprimir solo últimos 2 segundos)
  analogWrite(PWM, 173); // 68% = 173/255
  delay(28000); // Esperar 28 segundos sin imprimir
  
  // Iniciar toma de datos para los últimos 2 segundos del 68%
  Serial.println("Tiempo(ms),PWM,Altura(cm)");
  startTime = millis();
  
  for (unsigned long elapsed = 0; elapsed < 2000; elapsed = millis() - startTime) {
    float cm = measureDistance();
    
    if (isValidMeasurement(cm)) {
      Serial.print(elapsed);
      Serial.print(",");
      Serial.print(173);
      Serial.print(",");
      Serial.println(cm, 2);
    }
    
    delay(33); // 30Hz
  }
  
  // Subir a 76% durante 30 segundos (imprimir todos los valores)
  analogWrite(PWM, 194); // 76% = 194/255
  startTime = millis();
  firstMeasurement = true; // Reiniciar para nueva etapa
  
  for (unsigned long elapsed = 0; elapsed < 30000; elapsed = millis() - startTime) {
    float cm = measureDistance();
    
    if (isValidMeasurement(cm)) {
      Serial.print(elapsed + 2000); // Continuar desde 2000ms
      Serial.print(",");
      Serial.print(194);
      Serial.print(",");
      Serial.println(cm, 2);
    }
    
    delay(33); // 30Hz
  }
  
  // Bajar a 0% y terminar
  analogWrite(PWM, 0);
}

void loop() {
  // No hacer nada más
}

float measureDistance() {
  long duration;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  
  float cm = microsecondsToCentimeters(duration) + 1.5;
  
  return cm;
}

float microsecondsToCentimeters(long microseconds) {
  return microseconds / 29.0 / 2.0;
}

bool isValidMeasurement(float cm) {
  // Ignorar mediciones sobre 30cm
  if (cm > 30.0) {
    return false;
  }
  
  // Si es la primera medición válida, aceptarla
  if (firstMeasurement) {
    lastValidDistance = cm;
    firstMeasurement = false;
    return true;
  }
  
  // Ignorar cambios mayores a 5cm respecto a la última medición válida
  if (abs(cm - lastValidDistance) > 5.0) {
    return false;
  }
  
  // Actualizar última distancia válida
  lastValidDistance = cm;
  return true;
}
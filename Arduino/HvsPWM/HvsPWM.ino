const int trigPin = 7;
const int echoPin = 8;
const int PWM = 9;

bool testCompleted = false;
int currentStep = 0;
const int totalSteps = 100;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(PWM, OUTPUT);
  
  // Esperar 5 segundos antes de iniciar
  delay(5000);
  
  // Ir a 100% durante 3 segundos
  analogWrite(PWM, 255);
  delay(3000);
  
  // Volver a 0%
  analogWrite(PWM, 0);
  delay(1000);
  
  Serial.println("PWM(%),Altura(cm)");
}

void loop() {
  if (testCompleted) {
    return; // No hacer nada más después de completar
  }
  
  // Calcular el valor de PWM para este paso (0% a 100%)
  float dutyCycle = (float)currentStep / (totalSteps - 1);
  int pwmValue = (int)(dutyCycle * 255);
  
  // Establecer el PWM
  analogWrite(PWM, pwmValue);
  
  // Tomar mediciones durante 3 segundos a 20Hz (50ms entre mediciones)
  const int numMeasurements = 60; // 3 segundos * 20 Hz
  const int delayMs = 50; // 1000ms / 20Hz
  float sumCm = 0.0;
  int validMeasurements = 0;
  
  for (int i = 0; i < numMeasurements; i++) {
    float cm = measureDistance();
    if (cm > 0) { // Solo contar mediciones válidas
      sumCm += cm;
      validMeasurements++;
    }
    delay(delayMs);
  }
  
  // Calcular el promedio
  float avgCm = 0.0;
  if (validMeasurements > 0) {
    avgCm = sumCm / validMeasurements;
  }
  
  // Imprimir resultado
  Serial.print(dutyCycle * 100, 2);
  Serial.print(",");
  Serial.println(avgCm, 2);
  
  // Avanzar al siguiente paso
  currentStep++;
  
  // Verificar si se completó el test
  if (currentStep >= totalSteps) {
    testCompleted = true;
    analogWrite(PWM, 0); // Apagar al terminar
  }
}

float measureDistance() {
  long duration;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  
  float cm = microsecondsToCentimeters(duration) + 1.5; // offset de +1.5 cm
  
  return cm;
}

float microsecondsToCentimeters(long microseconds) {
  return microseconds / 29.0 / 2.0;
}
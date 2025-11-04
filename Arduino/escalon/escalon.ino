const int trigPin = 7;
const int echoPin = 8;
const int PWM = 9;

unsigned long startTime = 0;
unsigned long lastStepTime = 0;
int currentPWM = 158; // 62% = 158/255
bool testCompleted = false;
bool dataLogging = false;

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
  
  // Ir a 62% durante 10 segundos sin tomar datos
  analogWrite(PWM, 158);
  delay(10000);
  
  // Iniciar toma de datos
  startTime = millis();
  lastStepTime = startTime;
  dataLogging = true;
  
  Serial.println("Tiempo(ms),PWM,Altura(cm)");
}

void loop() {
  if (testCompleted) {
    return;
  }
  
  unsigned long currentTime = millis();
  
  // Verificar si pasaron 50 segundos para subir a 72%
  if (currentTime - lastStepTime >= 50000) {
    currentPWM = 184; // 72% = 184/255
    analogWrite(PWM, currentPWM);
    lastStepTime = currentTime;
    
    // Después de otros 50 segundos, terminar
    delay(50000);
    testCompleted = true;
    analogWrite(PWM, 0); // Apagar al terminar
    return;
  }
  
  // Tomar y guardar datos continuamente
  if (dataLogging) {
    float cm = measureDistance();
    
    unsigned long timestamp = currentTime - startTime;
    
    Serial.print(timestamp);
    Serial.print(",");
    Serial.print(currentPWM);
    Serial.print(",");
    Serial.println(cm, 2);
    
    delay(50); // 20Hz
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
  
  float cm = microsecondsToCentimeters(duration) + 1.5;
  
  return cm;
}

float microsecondsToCentimeters(long microseconds) {
  return microseconds / 29.0 / 2.0;
}
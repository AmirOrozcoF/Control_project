const int trigPin = 7;
const int echoPin = 8;
const int PWM = 9;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(PWM, OUTPUT);
  Serial.println("Ingrese un valor de PWM entre 0 y 1:");
}

void loop() {
  // ---- medir distancia con ultrasonido ----
  long duration, inches, cm;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);

  // ---- leer PWM desde teclado ----
  static float dutyCycle = 0.0; // valor guardado si no se escribe nada nuevo
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    float newValue = input.toFloat();
    if (newValue >= 0.0 && newValue <= 1.0) {
      dutyCycle = newValue;
    }
  }

  int pwmValue = (int)(dutyCycle * 255);
  analogWrite(PWM, pwmValue);

  // ---- mostrar info en monitor serial ----
  Serial.print("Distancia: ");
  Serial.print(inches);
  Serial.print(" in, ");
  Serial.print(cm);
  Serial.print(" cm | Duty Cycle: ");
  Serial.print(dutyCycle * 100);
  Serial.print("% | PWM: ");
  Serial.println(pwmValue);

  delay(200);
}

long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

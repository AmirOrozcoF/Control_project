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
  long duration;
  float inches, cm;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  cm = microsecondsToCentimeters(duration) + 1.5; // offset de +1,5 cm
  inches = cm / 2.54; // recalcular pulgadas desde cm con offset

  // ---- leer PWM desde teclado ----
  static float dutyCycle = 0.0;
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
  Serial.print(inches, 2); // dos decimales
  Serial.print(" in, ");
  Serial.print(cm, 1);     // un decimal
  Serial.print(" cm | Duty Cycle: ");
  Serial.print(dutyCycle * 100, 1);
  Serial.print("% | PWM: ");
  Serial.println(pwmValue);

  delay(200);
}

float microsecondsToCentimeters(long microseconds) {
  return microseconds / 29.0 / 2.0;
}

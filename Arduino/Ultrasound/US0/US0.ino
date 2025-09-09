const int trigPin = 7;
const int echoPin = 8;
const int PWM = 9;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  pinMode(PWM,OUTPUT);
}

void loop() {
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:
  long duration, inches, cm;
  float roundedCm;
  float dutyCycle;
  int pwmValue;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  duration = pulseIn(echoPin, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);

  // Redondear la distancia a incrementos de 0.2 cm
  roundedCm = round((float)cm * 5.0) / 5.0;  // Redondea a múltiplos de 0.2
  
  // Calcular duty cycle basado en la distancia
  if (roundedCm <= 4.0) {
    // 4cm o menos = duty cycle 1.0 (100%)
    dutyCycle = 1.0;
    pwmValue = 255;
  } else if (roundedCm >= 55.0) {
    // 55cm o más = duty cycle 0.0 (0%)
    dutyCycle = 0.0;
    pwmValue = 0;
  } else {
    // Interpolación lineal entre 4cm y 55cm
    // dutyCycle va de 1.0 (en 4cm) a 0.0 (en 55cm)
    dutyCycle = 1.0 - ((roundedCm - 4.0) / (55.0 - 4.0));
    pwmValue = (int)(dutyCycle * 255);
  }
  
  // Aplicar PWM al pin 9
  analogWrite(PWM, pwmValue);
  
  // Mostrar información en monitor serial
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm, Rounded: ");
  Serial.print(roundedCm);
  Serial.print("cm, Duty Cycle: ");
  Serial.print(dutyCycle * 100);
  Serial.print("%, PWM: ");
  Serial.print(pwmValue);
  Serial.println();

  delay(100);
}

long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: https://www.parallax.com/package/ping-ultrasonic-distance-sensor-downloads/
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}
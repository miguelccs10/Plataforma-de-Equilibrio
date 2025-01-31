#include <Servo.h>
#include <PID_v1.h>

// Pinos e constantes
const int servoPin = 9;
const int trigPin = 5;
const int echoPin = 6;

const int setPoint = 6;  // Posição desejada
const int base = 90;     // Posição inicial do servo (90°)

double Input, Output, Setpoint;
double Kp = 0.8, Ki = 0.02, Kd = 0.75;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

// Servo
Servo myServo;

// Filtro de leitura
double cmFiltered = setPoint;
const double a = 0.5;  // Parâmetro do filtro exponencial

// Limites dinâmicos
int minLimit = 0;
int maxLimit = 180;

// Função para calibrar os limites do servo
void calibrateServo() {
  myServo.write(base);  // Inicializa o servo na posição central (90°)
  delay(1000);
  minLimit = 0;
  maxLimit = 180;
}

// Função para medir a posição (em cm)
double readPosition() {
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  long duration = pulseIn(echoPin, HIGH, 30000);  // Timeout de 30ms
  double cm = duration / 58.0;                   // Conversão para cm

  if (cm > 30 || cm <= 0) cm = 30;  // Limite máximo do sensor

  cmFiltered = a * cm + (1 - a) * cmFiltered;  // Filtro exponencial
  return cmFiltered;
}

// Função para mapear a saída do PID nos limites do servo
int mapServoOutput(double output) {
  return constrain(base + output, minLimit, maxLimit);
}

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);

  // Calibra o servo e inicializa na posição base
  calibrateServo();

  // Configura o PID
  Setpoint = setPoint;
  Input = readPosition();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(minLimit - base, maxLimit - base);
}

void loop() {
  Input = readPosition();
  myPID.Compute();

  int servoPosition = mapServoOutput(Output);
  myServo.write(servoPosition);

  // Exibe dados no Serial Monitor
  Serial.print("Distancia (cm): ");
  Serial.print(Input);
  Serial.print(" | Posicao Servo: ");
  Serial.println(servoPosition);

  delay(20);
}

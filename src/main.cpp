#include <Arduino.h>

#define si_pin A1

float si, e, so, integral, derivative, ePrevious = 0;
const int controlMode = 0;

const float sp = 300;

const float Kp = 2; // 5
const float Ki = 0.01;
const float Kd = 1.0;

const float b = 0;

#define driverPUL 11
#define driverDIR 10
#define brakeRelay 9
#define brakeSwitch 8
#define inductiveProx 12

int driverRelay = 7;

float stepperOut = 0;
float stepperPulseDelay = 0;

unsigned long lastSerialPrintTime = 0;
int brakeSwitchState = 0;
int steerPosCenter = 0;
int steerPosOK = 0;
int accumulated = 0;
int limitDirection = 0;
unsigned long lastSampling = 0;

int dir = 0;
int prevDir = 0;

int pulseAccumulated = 0;
int pulseLimit = 200;

void sendPulsesBlocking(uint8_t pulPin, uint8_t dirPin, int dir, unsigned long pulseCount, unsigned long pulseDelayMicros)
{
  digitalWrite(dirPin, dir);

  for (unsigned long i = 0; i < pulseCount; i++)
  {
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(5); // short pulse width
    digitalWrite(pulPin, LOW);
    delayMicroseconds(pulseDelayMicros);
  }

  
}

void readMS()
{
  si = analogRead(si_pin);

  // Proportional Control
  e = sp - si;

  // Integral Control
  // integral = integral + e;

  // integral = integral + e;

  if ((e > 0 && ePrevious < 0) || (e < 0 && ePrevious > 0))
  {
    // integral = 0; // Reset integral on error sign change
  }
  else if (integral >= 10000)
  {
    integral = 10000;
  }
  else if (integral <= 00000)
  {
    integral = -10000;
  }

  // Derivative Control
  derivative = ePrevious - e;
  ePrevious = e;

  switch (controlMode)
  {
  case 0: // Proportional control
    so = Kp * e + b;
    break;
  case 1: // Integral control
    so = Ki * integral + b;
    break;
  case 2: // Derivative control
    so = Kd * derivative + b;
    break;
  case 3: // Proportional + Integral control
    so = Kp * e + Ki * integral + b;
    break;
  case 4: // Proportional + Derivative control
    so = Kp * e + Kd * derivative + b;
    break;
  case 5: // Integral + Derivative control
    so = Ki * integral + Kd * derivative + b;
    break;
  case 6: // Proportional + Integral + Derivative control
    so = Kp * e + Ki * integral + Kd * derivative + b;
    break;
  default: // Invalid control mode
    so = 0;
    break;
  }

  // Serial.print("System Input: ");
  // Serial.println(si);
  // Serial.print("Error: ");
  // Serial.println(e);
  // Serial.print("System Output: ");
  // Serial.println(so);
}

void stepperControl()
{
  stepperOut = abs(so);
  if (stepperOut > 1023)
    stepperOut = 1023;

  stepperOut = map(abs(so), 0, 1023, 0, 1000); // 7000

  if (so > 0) // 0 = KANAN, 1 = KIRI
  {
    dir = 0; // so nya + -> kanan
  }
  else if (so < 0)
  {
    dir = 1; // so nya - -> kiri
  }

  Serial.print("SO: ");
  Serial.println(so);

  sendPulsesBlocking(driverPUL, driverDIR, dir, stepperOut, 500);
}

void printSerialData()
{
  unsigned long currentMillis = millis();
  if (currentMillis - lastSerialPrintTime >= 100)
  {
    lastSerialPrintTime = currentMillis;

    Serial.print("System Input: ");
    Serial.print(si);
    Serial.print("\tError: ");
    Serial.print(e);
    Serial.print("\tIntegral: ");
    Serial.print(integral);
    Serial.print("\tDerivative: ");
    Serial.print(derivative);
    Serial.print("\tSystem Output: ");
    Serial.print(so);
    Serial.print("\tStepper Output Freq: ");
    Serial.print(stepperOut);

    // Setir ke kanan, CCW. kiri CW
    if (so > 0)
    {
      // digitalWrite(driverDIR, LOW);
      // setFrequency(stepperOut, driverPUL);
      Serial.println("\tCW");
    }
    else if (so < 0)
    {
      // digitalWrite(driverDIR, HIGH);
      // setFrequency(stepperOut, driverPUL);
      Serial.println("\tCCW");
    }
    else
    {
      // setFrequency(0, driverPUL);
    }

    integral = integral + e;
  }
}

void setup()
{
  pinMode(driverPUL, OUTPUT);
  pinMode(driverDIR, OUTPUT);
  pinMode(brakeRelay, OUTPUT);
  pinMode(brakeSwitch, INPUT);
  pinMode(inductiveProx, INPUT);

  digitalWrite(brakeRelay, HIGH);
  pinMode(driverRelay, OUTPUT);
  digitalWrite(driverRelay, HIGH);

  Serial.begin(9600);
  Serial.println("Start");
  sei(); // Enable global interrupts

  steerPosCenter = digitalRead(inductiveProx);
  // sendPulsesNonBlocking(driverPUL, driverDIR, 1, 30000, 5);
  // sendPulsesBlocking(driverPUL, driverDIR, 1, 1000, 1000);

  // setFrequency(100, driverPUL);
  // digitalWrite(driverDIR, LOW);
}

void loop()
{
  steerPosCenter = digitalRead(inductiveProx);

  if (steerPosCenter == LOW)
  {
    steerPosOK = 1;
    pulseAccumulated = 0; // Reset accumulator
    accumulated = 0;      // Clear limit
    limitDirection = 0; // Clear direction

    // digitalWrite(driverRelay, LOW);
  }
  else if (steerPosOK == 0 && steerPosCenter == HIGH)
  {
    Serial.println("Please center the steering wheel");
  }

  if (steerPosOK)
  // if (true)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - lastSampling >= 100)
    // if (true)
    {
      lastSampling = currentMillis;
      // sendPulsesBlocking(driverPUL, driverDIR, 1, 100, 1000);
      readMS();
      stepperControl();
    }
  }
}

/*
Stepper Motor

Pin PUL dikasih pulse buat ngatur kecepatan, freq up, speed up
Pin DIR dikasih HIGH/LOW buat ngatur arah
Dri datasheet: Pulse Frequency Max = 200K (harusnya Hz, tpi di ds cm tulis 200K)

+/- dari so buat ngatur DIR, nilai dari so buat ngatur frekuensi pulse
Kalo terlalu lambat atau kurang cepat banting setirnya, Gain (K) nya diganti
*/

/*
Proportional Control

sp = Set Point
so = System Output (Sensor Sensed Value)
si = System Input
e = Error
k = Proportional Gain Factor
b = Bias
sp = 1.5V = 300 ADC Bit

so = K * e + b
e = sp - so

A+ A- B+ B-
GR BLK RED BLU
*/
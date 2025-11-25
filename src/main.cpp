#include <Arduino.h>

#define si_pin A1

float si;
float e = 0;
float so = 0;
float integral = 0;
float derivative = 0;
float ePrevious = 0;
const int controlMode = 0;

const float sp = 300; // 300

const float Kp = 0.008; // 0.08 Ok? 0.006 0075
const float Ki = 0.005; // 0.00085
const float Kd = 0.2;   // Terakhir 1.3 ||||| 0.0037

// 0.05 0.04 0.008 Junius

const float b = 0;

#define driverPUL 11
#define driverDIR 10
#define brakeRelay 9
#define brakeSwitch 8
#define inductiveProx 12

int driverRelay = 7;

float stepperOut = 0;
float stepperPulseDelay = 0;

unsigned long timeDelta = 0;
int brakeSwitchState = 0;
int steerPosCenter = 0;
int steerPosOK = 0;
int accumulated = 0;
int limitDirection = 0;
unsigned long lastSampling = 0;

int dir = 0;
int prevDir = 0;

int pulseAccumulated = 0;
int pulseLimit = 1800;

void setFrequency(uint32_t frequency, uint8_t pin)
{
  // === Special case: stop output ===
  if (frequency == 0)
  {
    // Disable Timer1
    TCCR1A = 0;
    TCCR1B = 0;

    // Ensure pin is not toggling anymore
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    // Serial.println("Output stopped.");
    return;
  }

  // Ensure the frequency is within limits
  if (frequency > 2000000)
  {
    // Serial.println("Frequency out of range (1Hz to 2MHz)");
    return;
  }

  // Calculate the timer settings
  uint16_t prescaler = 1; // Default prescaler
  uint32_t ocrValue = 16000000 / (2 * prescaler * frequency);

  // Adjust the prescaler and OCR value for different frequency ranges
  if (ocrValue > 65535)
  {
    prescaler = 8;
    ocrValue = 16000000 / (2 * prescaler * frequency);
  }
  if (ocrValue > 65535)
  {
    prescaler = 64;
    ocrValue = 16000000 / (2 * prescaler * frequency);
  }
  if (ocrValue > 65535)
  {
    prescaler = 256;
    ocrValue = 16000000 / (2 * prescaler * frequency);
  }
  if (ocrValue > 65535)
  {
    prescaler = 1024;
    ocrValue = 16000000 / (2 * prescaler * frequency);
  }

  if (ocrValue > 65535)
  {
    Serial.println("Frequency too low for this configuration.");
    return;
  }

  // Set the pin as output
  pinMode(pin, OUTPUT);

  // Configure Timer1
  TCCR1A = 0; // Clear Timer/Counter Control Registers
  TCCR1B = 0;
  TCCR1A = (1 << COM1A0); // Toggle pin on compare match
  TCCR1B = (1 << WGM12);  // CTC mode

  // Set the appropriate prescaler
  switch (prescaler)
  {
  case 1:
    TCCR1B |= (1 << CS10);
    break;
  case 8:
    TCCR1B |= (1 << CS11);
    break;
  case 64:
    TCCR1B |= (1 << CS11) | (1 << CS10);
    break;
  case 256:
    TCCR1B |= (1 << CS12);
    break;
  case 1024:
    TCCR1B |= (1 << CS12) | (1 << CS10);
    break;
  }

  // Set the output compare register value
  OCR1A = ocrValue - 1;

  // Attach the pin to Timer1 (only available on certain pins)
  if (pin == 11)
  {
    TCCR1A |= (1 << COM1A0); // Connect Timer1 to pin 11 (OC1A)
  }
  else if (pin == 12)
  {
    TCCR1A |= (1 << COM1B0); // Connect Timer1 to pin 12 (OC1B)
  }
  else
  {
    Serial.println("Invalid pin for Timer1 output.");
    return;
  }
}

void sendPulsesBlocking(uint8_t pulPin, uint8_t dirPin, int dir, unsigned long pulseCount, unsigned long pulseDelayMicros)
{
  digitalWrite(dirPin, dir);

  for (unsigned long i = 0; i < pulseCount; i++)
  {
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(5); // short pulse width
    digitalWrite(pulPin, LOW);
    delayMicroseconds(pulseDelayMicros);

    // if (dir == 0)
    // {
    //   pulseAccumulated += 1;
    // }
    // else if (dir == 1)
    // {
    //   pulseAccumulated -= 1;
    // }

    // if (pulseAccumulated > pulseLimit)
    // {
    //   break;
    // }
    // else if (pulseAccumulated < -pulseLimit)
    // {
    //   break;
    // }
  }
}

void readMS()
{
  si = analogRead(si_pin);

  // Proportional Control
  e = sp - si;

  // Integral Control
  integral += e;

  if ((e > 0 && ePrevious < 0) || (e < 0 && ePrevious > 0))
  {
    // integral = 0; // Reset integral on error sign change
  }
  else if (integral >= 6)
  {
    integral = 6;
  }
  else if (integral <= -6)
  {
    integral = -6;
  }

  // Derivative Control
  derivative = (e - ePrevious);
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

  // Serial.print("si: ");
  // Serial.print(si);
  // Serial.print("Error: ");
  // Serial.println(e);
  // Serial.print("System Output: ");
  // Serial.println(so);
}

void stepperControl()
{
  // stepperOut = abs(so);
  // if (stepperOut > 1023)
  //   stepperOut = 1023;

  // stepperOut = map(abs(so), 0, 1023, 0, 80); // 7000
  stepperOut = abs(round(so));

  // y = x^2
  // float normalized = 1 + abs(so) / 1023.0; // Normalize to 0–1 range
  // float exponent = 2.0;
  // float scaled = pow(abs(so), exponent);
  // stepperOut = 0.015 * scaled;
  // int roundedPulse = round(stepperOut);

  // Serial.print("\tSO: ");
  // Serial.print(so);
  // Serial.print("\tNormalized: ");
  // Serial.print(normalized);
  // Serial.print("\tScaled: ");
  // Serial.print(scaled);
  // Serial.print("\tStepperOut: ");
  // Serial.print(stepperOut);
  // Serial.print("\tRounded: ");
  // Serial.print(roundedPulse);

  if (so > 0) // 0 = KANAN, 1 = KIRI
  {
    dir = 0; // so nya + -> kanan
  }
  else if (so < 0)
  {
    dir = 1; // so nya - -> kiri
  }
  digitalWrite(driverDIR, dir);

  // if (pulseAccumulated > pulseLimit)
  // {
  //   accumulated = 1;
  //   limitDirection = 0; // right side limit
  //   Serial.println("Right limit reached");
  // }
  // else if (pulseAccumulated < -pulseLimit)
  // {
  //   accumulated = 1;
  //   limitDirection = 1; // left side limit
  //   Serial.println("Left limit reached");
  // }

  // if (accumulated == 1)
  // {
  //   if (limitDirection == 1 && dir == 1)
  //   {
  //     Serial.println("asdasdasd");
  //   }
  //   else if (limitDirection == 0 && dir == 0)
  //   {
  //     Serial.println("werwer");
  //   }
  //   else if (limitDirection == 1 && dir == 0)
  //   {
  //     digitalWrite(driverDIR, dir); // Klo ganti driver ini disesuaikan
  //     sendPulsesBlocking(driverPUL, driverDIR, dir, roundedPulse, 30);
  //     Serial.println("kjkjkjkjkjk");
  //   }
  //   else if (limitDirection == 0 && dir == 1)
  //   {
  //     digitalWrite(driverDIR, dir); // Klo ganti driver ini disesuaikan
  //     sendPulsesBlocking(driverPUL, driverDIR, dir, roundedPulse, 30);
  //     Serial.println("hthththththt");
  //   }
  // }
  // else if (accumulated == 0)
  // {
  //   digitalWrite(driverDIR, dir); // Klo ganti driver ini disesuaikan
  //   sendPulsesBlocking(driverPUL, driverDIR, dir, roundedPulse, 30);
  //   // Serial.println("poppopopop");
  // }
  // sendPulsesBlocking(driverPUL, driverDIR, dir, stepperOut, 10);
  // sendPulsesBlocking(driverPUL, driverDIR, dir, roundedPulse, 30);
  // y = 5x + 10x
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
  // sei(); // Enable global interrupts

  // steerPosCenter = digitalRead(inductiveProx);
  // sendPulsesNonBlocking(driverPUL, driverDIR, 1, 30000, 5);
  // sendPulsesBlocking(driverPUL, driverDIR, 1, 3000, 30);

  setFrequency(10000, driverPUL);
  // digitalWrite(driverDIR, LOW);
}

void print()
{
  Serial.print("si: ");
  Serial.print(si);
  Serial.print("\tderivative : ");
  Serial.print(derivative);
  Serial.print("\tintegral: ");
  Serial.print(integral);
  Serial.print("\tSO: ");
  Serial.print(so);
  Serial.print("\tStepperOut: ");
  Serial.println(stepperOut);
}

void loop()
{
  // digitalWrite(driverPUL, HIGH);
  // delayMicroseconds(5); // short pulse width
  // digitalWrite(driverPUL, LOW);
  // delayMicroseconds(10);

  // unsigned long currentMillis = millis();

  // if (currentMillis - lastSampling >= 50)
  // {
    // lastSampling = currentMillis;
    readMS();
    stepperControl();
  // }
}

/*
ini stepper lama

Pin PUL dikasih pulse buat ngatur kecepatan, freq up, speed up
Pin DIR dikasih HIGH/LOW buat ngatur arah
Dri datasheet: Pulse Frequency Max = 200K (harusnya Hz, tpi di ds cm tulis 200K)

+/- dari so buat ngatur DIR, nilai dari so buat ngatur frekuensi pulse
Kalo terlalu lambat atau kurang cepat banting setirnya, Gain (K) nya diganti
*/

/*
Proportional Control

sp = Set Point
so = System Output
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
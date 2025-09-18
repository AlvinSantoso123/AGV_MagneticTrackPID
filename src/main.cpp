#include <Arduino.h>

#define si_pin A1

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

float si, e, so, integral, derivative, ePrevious = 0;
const int controlMode = 0;

const float sp = 300;

const float Kp = 7.0; //5
const float Ki = 0.01;
const float Kd = 1.0;

const float b = 0;

/*
Stepper Motor

Pin PUL dikasih pulse buat ngatur kecepatan, freq up, speed up
Pin DIR dikasih HIGH/LOW buat ngatur arah
Dri datasheet: Pulse Frequency Max = 200K (harusnya Hz, tpi di ds cm tulis 200K)

+/- dari so buat ngatur DIR, nilai dari so buat ngatur frekuensi pulse
Kalo terlalu lambat atau kurang cepat banting setirnya, Gain (K) nya diganti
*/

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
float soAccumulated = 0;
unsigned long lastAccumulationTime = 0;

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
  else if (integral <= -10000)
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

  stepperOut = map(abs(so), 0, 1023, 0, 6000); // 7000

  unsigned long currentMillis = millis();
  if (currentMillis - lastAccumulationTime >= 20) // shorter interval → smoother
  {
    // Serial.print("SO: ");
    // Serial.println(so);
    lastAccumulationTime = currentMillis;
    // soAccumulated += (so / 20.0); //Klo mau limit, ini dinyalakan

    // Serial.print("SO Acc: ");
    // Serial.println(soAccumulated);

    // if (soAccumulated > 300)
    // {
    //   accumulated = 1;
    //   limitDirection = +1; // right side limit
    //   Serial.println(">>> Right limit reached <<<");
    // }
    // else if (soAccumulated < -300)
    // {
    //   accumulated = 1;
    //   limitDirection = -1; // left side limit
    //   Serial.println(">>> Left limit reached <<<");
    // }
  }

  // Safety logic
  // if (accumulated == 0)
  if (true)
  {
    Serial.println("OPERATIONAL");
    // Normal operation
    if (so > 0)
    {
      digitalWrite(driverDIR, LOW); // Klo ganti driver ini disesuaikan
      setFrequency(stepperOut, driverPUL);
      Serial.println("so>0");
    }
    else if (so < 0)
    {
      digitalWrite(driverDIR, HIGH); // Klo ganti driver ini disesuaikan
      setFrequency(stepperOut, driverPUL);
      Serial.println("so<0");
    }
    else
    {
      setFrequency(0, driverPUL);
      Serial.println("STOP");
    }
  }
  // else
  // {
  //   // Limit reached: only allow opposite direction
  //   if (limitDirection == +1 && so < 0)
  //   {
  //     // allow left movement
  //     digitalWrite(driverDIR, LOW);
  //     setFrequency(stepperOut, driverPUL);
  //   }
  //   else if (limitDirection == -1 && so > 0)
  //   {
  //     // allow right movement
  //     digitalWrite(driverDIR, HIGH);
  //     setFrequency(stepperOut, driverPUL);
  //   }
  //   else
  //   {
  //     // block further movement
  //     setFrequency(0, driverPUL);
  //   }
  // }
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

  steerPosCenter = digitalRead(inductiveProx);

  // setFrequency(2000, driverPUL);
  // digitalWrite(driverDIR, LOW);
}

void loop()
{
  steerPosCenter = digitalRead(inductiveProx);

  if (steerPosCenter == LOW)
  {
    steerPosOK = 1;
    soAccumulated = 0;  // Reset accumulator
    accumulated = 0;    // Clear limit
    limitDirection = 0; // Clear direction

    digitalWrite(driverRelay, LOW);
  }
  else if (steerPosOK == 0 && steerPosCenter == HIGH)
  {
    Serial.println("Please center the steering wheel");
  }

  if (steerPosOK)
  {
    readMS();
    stepperControl();
    printSerialData(); // Klo mau I, ini harus jalan
  }

  // steerPosCenter = digitalRead(inductiveProx);

  // if (steerPosCenter == LOW)
  // {
  //   steerPosOK = 1;
  //   soAccumulated = 0; // Reset accumulator
  //   accumulated = 0;
  // }
  // else if (steerPosOK == 0 && steerPosCenter == HIGH)
  // {
  //   Serial.println("Please center the steering wheel");
  // }

  // if (steerPosOK)
  // {
  //   readMS();
  //   stepperControl();
  //   // printSerialData();
  // }

  brakeSwitchState = digitalRead(brakeSwitch);

  if (brakeSwitchState == HIGH)
  {
    digitalWrite(brakeRelay, LOW);
  }
  else
  {
    digitalWrite(brakeRelay, HIGH);
  }
}

/*
Reference

INI BUAT DRIVER LAMA

Remarks from HBS86H Hybrid Stepper Servo Driver Manual
a. t1: ENA must be ahead of DIR by at least 5μ s. Usually, ENA+ and
ENA- are NC (not connected).
b. t2: DIR must be ahead of PUL active edge by 6μ s to ensure correct
direction;
c. t3: Pulse width not less than 2.5μs;
d. t4: Low level width not less than 2.5μs.

ArduinoPWM
https://docs.arduino.cc/learn/microcontrollers/analog-output/

Control Stepper Motor with Arduino by dronebotworkshop
https://dronebotworkshop.com/big-stepper-motors/
*/

#include <Arduino.h>

const bool SIMULATION = true;   // true = test with hard-coded inputs, false = real robot

const int SensorPins[5] = {34,35,32,33,25};
const int SensorPos[5] = {-2,-1,0,1,2};

const int ENA = 14;
const int IN1 = 27;
const int IN2 = 26;

const int ENB = 13;
const int IN3 = 12;
const int IN4 = 11;

// Value of ConstP, ConstD and BaseP aren't fixed and here used arbitrarily.
// The values are taken considering values of error, and the dx/dt, so that values, remain under control, although testing is required for actual values.
float ConstP = 15;
float ConstD = 0.05;

const int PWM_MAX = 255;
const int BaseP = 120;

const unsigned long LOOP_US = 5000;   // 5 ms
unsigned long lastLoop = 0;
bool wasLost = false;

float error = 0;
float previous_error = 0;
float last_valid_position = 0;
float derivative = 0;
float corection = 0;

const int PWM_FREQUENCY = 2000;
const int PWM_RESOLUTION = 8;

const int PWM_CH_A = 0;
const int PWM_CH_B = 1;

// Sample sensor inputs (each row = 5 sensors, 1 = line seen)
const int TestInputs[][5] = {
  {0,0,1,0,0},   // centered
  {0,0,1,1,0},   // slightly right
  {0,0,0,1,0},   // right
  {0,0,0,1,1},   // more right
  {0,0,0,0,1},   // far right
  {0,0,0,0,0},   // LOST (last seen on right)
  {0,0,0,0,0},   // still lost
  {0,1,0,0,0},   // line found again on the left
  {0,1,1,0,0},   // slightly left
  {1,0,0,0,0},   // far left
  {0,0,1,0,0}    // centered again
};

void setup()
{
  if (SIMULATION)
  {
    Serial.begin(115200);
  }

  for (int i = 0; i < 5; i++)
  {
    pinMode(SensorPins[i], INPUT);
  }

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(PWM_CH_A, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CH_A);

  ledcSetup(PWM_CH_B, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(ENB, PWM_CH_B);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  lastLoop = micros();
}

// Weighted centroid from a list of 5 sensor values
float Compute_Position(const int readings[5])
{
  int sum = 0;
  int weighted_sum = 0;

  for (int i = 0; i < 5; i++)
  {
    sum += readings[i];
    weighted_sum += readings[i] * SensorPos[i];
  }

  if (sum == 0)
    return 999.9;

  return float(weighted_sum) / sum;
}

// Reads the real sensors into an array
void Read_Sensors(int readings[5])
{
  for (int i = 0; i < 5; i++)
  {
    readings[i] = digitalRead(SensorPins[i]);   // use ! if sensors read LOW on the line
  }
}

void Set_Motor(int PWM_Channel, int IN1, int IN2, int Power)
{
  if(Power > PWM_MAX)
    Power = PWM_MAX;
  else if(Power < -PWM_MAX)
    Power = -PWM_MAX;

  if (Power > 0)
  {
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
  }
  else if (Power < 0)
  {
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
  }
  else
  {
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
  }

  ledcWrite(PWM_Channel, abs(Power));
}

void Set_Motors_(int Left_Power,int Right_Power)
{
  if (SIMULATION)
  {
    Serial.print("  motors: L=");
    Serial.print(Left_Power);
    Serial.print(" R=");
    Serial.println(Right_Power);
  }
  else
  {
    Set_Motor(PWM_CH_A,IN1,IN2,Left_Power);
    Set_Motor(PWM_CH_B,IN3,IN4,Right_Power);
  }
}

void Recover_Line()
{
  // Values used here are arbitrary.
  if (last_valid_position < 0)
    Set_Motors_(-80,80);
  else if (last_valid_position > 0)
    Set_Motors_(80,-80);
  else
    Set_Motors_(0,0);
}

// One control step: sensors in, motor commands out
void Control_Step(const int readings[5], float dt)
{
  float position = Compute_Position(readings);

  if (position == 999.9)
  {
    wasLost = true;
    if (SIMULATION)
    {
      Serial.println("  LOST -> recovery");
    }
    Recover_Line();
    return;
  }

  last_valid_position = position;

  error = -position;

  if (wasLost)
  {
    previous_error = error;   // avoids a derivative kick after recovery
    wasLost = false;
  }

  derivative = (error - previous_error)/dt;

  corection = ConstP*error + ConstD*derivative;

  previous_error = error;

  int Power_Left = BaseP - corection;
  int Power_Right = BaseP + corection;

  if (Power_Left > PWM_MAX)
    Power_Left = PWM_MAX;
  else if (Power_Left < -PWM_MAX)
    Power_Left = -PWM_MAX;

  if (Power_Right > PWM_MAX)
    Power_Right = PWM_MAX;
  else if (Power_Right < -PWM_MAX)
    Power_Right = -PWM_MAX;

  if (SIMULATION)
  {
    Serial.print("  position=");
    Serial.print(position);
    Serial.print(" error=");
    Serial.print(error);
    Serial.print(" correction=");
    Serial.println(corection);
  }

  Set_Motors_(Power_Left,Power_Right);
}

void loop()
{
  const float dt = LOOP_US / 1000000.0f;

  if (SIMULATION)
  {
    for (int i = 0; i < sizeof(TestInputs) / sizeof(TestInputs[0]); i++)
    {
      Serial.print("Test ");
      Serial.println(i);
      Control_Step(TestInputs[i], dt);
      delay(300);
    }
    delay(3000);
  }
  else
  {
    unsigned long now = micros();
    if (now - lastLoop < LOOP_US) return;
    lastLoop = now;

    int readings[5];
    Read_Sensors(readings);
    Control_Step(readings, dt);
  }
}

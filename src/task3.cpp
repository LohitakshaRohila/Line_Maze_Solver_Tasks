#include <Arduino.h>

const int SensorPins[5] = {13,12,14,27,26};

const int SensorPos[5] = {-2,-1,0,1,2};

const int PWM_CH_A = 0;
const int PWMA = 25;
const int IN1A = 18;
const int IN2A = 19;

const int PWM_CH_B = 1;
const int PWMB = 33;
const int IN1B = 32;
const int IN2B = 17;

// Value of ConstP, ConstD and BaseP aren't fixed and here used arbitrarily.
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

int STBY = 23;

const int PWM_FREQUENCY = 2000;
const int PWM_RESOLUTION = 8;

void setup()
{
  for (int i = 0; i < 5; i++)
  {
    pinMode(SensorPins[i], INPUT);
  }

  pinMode(PWMA, OUTPUT);
  pinMode(IN1A, OUTPUT);
  pinMode(IN2A, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(IN1B, OUTPUT);
  pinMode(IN2B, OUTPUT);

  pinMode(STBY,OUTPUT);

  ledcSetup(PWM_CH_A, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWMA, PWM_CH_A);

  ledcSetup(PWM_CH_B, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWMB, PWM_CH_B);

  digitalWrite(IN1A, LOW);
  digitalWrite(IN2A, LOW);

  digitalWrite(IN1B, LOW);
  digitalWrite(IN2B, LOW);

  digitalWrite(STBY,HIGH);

  lastLoop = micros();

}


float Read_Line_Position()
{
  int sum = 0;
  int weighted_sum = 0;

  for (int i = 0; i < 5;i++)
  {
    int value = digitalRead(SensorPins[i]);
    sum += value;
    weighted_sum += value*SensorPos[i];
  }

  if (sum == 0)
    return 999.9;

    return float(weighted_sum)/(sum);  
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
  Set_Motor(PWM_CH_A,IN1A,IN2A,Left_Power);
  Set_Motor(PWM_CH_B,IN1B,IN2B,Right_Power);
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

void loop()
{
  unsigned long now = micros();
  if (now - lastLoop < LOOP_US) return;
  lastLoop = now;

  const float dt = LOOP_US/ 1000000.0f;

  float position = Read_Line_Position();

  if (position == 999.9)
  {
    wasLost = true;
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

  Set_Motors_(Power_Left,Power_Right);
}

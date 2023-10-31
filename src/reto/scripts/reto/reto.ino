#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;
// Encoder #1
const int pinCanalA = 2;
const int pinCanalB = 4;

// Control Motor 1
const int motorPin1 = 10;
const int motorPin2 = 11;

float dt = 0.1;
float tiempo = 0;
float contador = 0;
float pos = 0;
float pos_ant = 0;
float vel = 0;

int input = 0;

std_msgs::Float32 output_msg;

int MaxOutput = 255;

void inputCb(const std_msgs::Float32 &input_msg)
{

  input = (int)(input_msg.data * MaxOutput);

  if (input > MaxOutput)
  {
    input = MaxOutput;
  }
  else if (input < -MaxOutput)
  {
    input = -MaxOutput;
  }
}

ros::Subscriber<std_msgs::Float32> sub("motor_input", &inputCb);
ros::Publisher pub("motor_output", &output_msg);

void setup()
{
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  Serial.begin(9600);
  nh.initNode();
  nh.subscribe(sub);

  attachInterrupt(digitalPinToInterrupt(pinCanalA), Encoder, RISING);
  pinMode(pinCanalA, INPUT);
  pinMode(pinCanalB, INPUT);
  nh.advertise(pub);
}

void loop()
{

  if (micros() - tiempo > dt * 1000000)
  {

    tiempo = micros();
    pos = contador;

    vel = (pos - pos_ant) / 408 * (2 * 3.141592 / dt);

    output_msg.data = vel;

    if (input < 0)
    {
      analogWrite(motorPin1, -input);
      analogWrite(motorPin2, 0);
    }

    else
    {
      analogWrite(motorPin1, 0);
      analogWrite(motorPin2, input);
    }

    Serial.println(vel);
    pos_ant = pos;

    pub.publish(&output_msg);
  }

  nh.spinOnce();
  delay(1);
}

// Interrupcion de Encoder 1
void Encoder()
{
  if (digitalRead(pinCanalA) == digitalRead(pinCanalB))
  {
    contador++;
  }
  else
  {
    contador--;
  }
}

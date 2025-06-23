/*
* Calibração da IMU
* Autor: Alex Cani
*/

#include <Mpu6050.h>
#include <Servo.h>
/*
* Este código serve para determinar os valores de offset
* para calibração da IMU.
* 
* Utilize o Monitor Serial ou o Plotter Serial para determinar
* os valores que devem ser SOMADOS às variáveis de modo a zerá-las.
*/
Mpu6050 imu;
Servo esc;
Servo servo;

float offset_posicao_x = (13.9-10.4)/2-3;
float offset_posicao_y = 2;
float offset_velocidade_x = 10;
float offset_velocidade_y = -0.3;

const int pin_esc = 6;
int pwm_min = 1000;
int pwm_max = 2000;
char data, old_data;
unsigned long count = 0;

const int pin_servo = 9;
int pos = 90;
const int pos_init = 90;
// Itere sobre estes valores e quando estiver correto anote-os e use-os
// no código de controle.
// Mais opções de offset estão disponíveis no código da biblioteca Mpu6050
// Na prática, apenas a velocidade e posição em um dos eixos será utilizada.
// Este código permite alterar os offsets tanto em x quanto em y prevendo o 
// uso da IMU rotacionada em 90º no chassi do veículo.
// Identifique qual é o eixo correto (dada a construção do veículo e as marcações na IMU
// e preocupe-se apenas com este.



void setup() {
  Serial.begin(115200); // Inicia serial em baud rate 115200



  imu.Begin();

  // Aplica os offsets
  imu.SetXPosOffset(offset_posicao_x);
  imu.SetYPosOffset(offset_posicao_y); 
  imu.SetXSpeedOffset(offset_velocidade_x);
  imu.SetYSpeedOffset(offset_velocidade_y);
  
  esc.attach(pin_esc,pwm_min,pwm_max);
  esc.writeMicroseconds(pwm_min);

  servo.attach(pin_servo);
}

void loop() {
  imu.Update(); // Pega os valores atuais

  Rotation rot = imu.GetRotation(); // Pega rotação e velocidade
  Velocity vel = imu.GetVelocity();

  // Imprime os valores, comente a linha para desabilitar
  Serial.print(rot.y); Serial.print('\t');
  Serial.print(vel.y); Serial.print('\t');
  Serial.print(rot.x); Serial.print('\t');
  Serial.print(vel.x); Serial.print('\t');
  Serial.print(servo.read()); Serial.print('\t');
  Serial.print(15); Serial.print('\t');
  Serial.print(-15); Serial.print('\t');

  Serial.println();

  if(Serial.available() > 0){
    switch (Serial.read()) {
      case 'a':
        pos = pos + 10;
        break;
      case 'd':
        pos = pos - 10;
        break;
      case 'r':
        pos = pos_init;
        break;
      case 'i':
        esc.writeMicroseconds(pwm_max);
        Serial.print(count); Serial.println(": PWM set to MAX");
        break;
      case 'o':
        esc.writeMicroseconds(pwm_min);
        Serial.print(count); Serial.println(": PWM set to MIN");
        break;
    }
    servo.write(pos);
  }

  delay(10);
}
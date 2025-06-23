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

//(14.1-11.5)/2-3;
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
const int pos_init = 90;
// Itere sobre estes valores e quando estiver correto anote-os e use-os
// no código de controle.
// Mais opções de offset estão disponíveis no código da biblioteca Mpu6050
// Na prática, apenas a velocidade e posição em um dos eixos será utilizada.
// Este código permite alterar os offsets tanto em x quanto em y prevendo o 
// uso da IMU rotacionada em 90º no chassi do veículo.
// Identifique qual é o eixo correto (dada a construção do veículo e as marcações na IMU
// e preocupe-se apenas com este.


//____________________ ALTERAÇÕES ___________________________
const int servo_maximo = 130; //  Limite superior da atuação do servo
const int servo_minimo = 50;  //  Limite inferior da atuação do servo
const bool inverter_direcao_servo = false; 
const bool EIXO_X = true;
const bool inverter_imu = false;

unsigned long tempo_ultimo_controle;
float x1, x2, x3;
float du, dt;
float theta = 0;
float u;
int atuacao;

const float kGrausParaRadianos = M_PI/180.0;
const float kRadianosParaGraus = 180.0/M_PI;

const bool plotar_referencia = true;
const bool plotar_x1 = true;
const bool plotar_x2 = true;
const bool plotar_x3 = true;
const bool plotar_atuacao_min_max = false;
const bool plotar_atuacao = false;

const int Ts = 5;
const float K1 = -3.60;
const float K2 = -0.58;
const float K3 = -0.88;

bool controle = false;
//_______________________________________________



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
  servo.write(pos_init);

  tempo_ultimo_controle = millis();
}

void loop() {
  imu.Update(); // Pega os valores atuais
  dt = millis() - tempo_ultimo_controle;

  
  if (dt >= Ts) { // Loop de controle
    Rotation rot = imu.GetRotation();
    Velocity vel = imu.GetVelocity();
    
    x1 = EIXO_X ? rot.x : rot.y;
    x1 *= (inverter_imu ? -1 : 1);
    x1 *= kGrausParaRadianos;
    
    x2 = theta;
  c
  o
  i
  
    x3 = EIXO_X ? vel.x : vel.y;
    x3 *= (inverter_imu ? -1 : 1);
    x3 *= kGrausParaRadianos;

    if (controle == true){
      u = -K1*x1 -K2*x2 -K3*x3;

      theta += (u*dt)/1000.0; //  Converter dt para segundos
      // Atua
      atuacao = static_cast<int>((inverter_direcao_servo ? -1 : 1)*kRadianosParaGraus*theta + pos_init);
      if (atuacao > servo_maximo) {
        atuacao = servo_maximo;
        theta = kGrausParaRadianos*(servo_maximo-pos_init); //  Não pode deixar theta ficar crescendo!
      } else if (atuacao < servo_minimo) {
        atuacao = servo_minimo;
        theta = kGrausParaRadianos*(servo_minimo-pos_init);
      }
        
        servo.write(atuacao);
        tempo_ultimo_controle = millis();
    } else{
    servo.write(pos_init);
    theta = 0;
    }
  }
  

  if(Serial.available() > 0){
    switch (Serial.read()) {
      case 'i':
        esc.writeMicroseconds(pwm_max);
        Serial.print(count); Serial.println(": PWM set to MAX");
        break;
      case 'o':
        esc.writeMicroseconds(pwm_min);
        Serial.print(count); Serial.println(": PWM set to MIN");
        break;
       case 'c':
        controle = !controle;
        Serial.println("CONTROLE");
        break;
    }
  }



  // Plots
  if (plotar_referencia) {
    Serial.print(0);
    Serial.print('\t');
    Serial.print(13);
    Serial.print('\t');
    Serial.print(-13);
    Serial.print('\t');
  }
  if (plotar_x1) {
    Serial.print(x1*kRadianosParaGraus);
    Serial.print('\t');
  }
  if (plotar_x2) {
    Serial.print(x2*kRadianosParaGraus);
    Serial.print('\t');
  }
  if (plotar_x3) {
    Serial.print(x3*kRadianosParaGraus);
    Serial.print('\t');
  }
  if (plotar_atuacao_min_max) {
    Serial.print(kGrausParaRadianos*servo_maximo);
    Serial.print('\t');
    Serial.print(kGrausParaRadianos*servo_minimo);
    Serial.print('\t');
  }
  if (plotar_atuacao){
    Serial.print(atuacao - pos_init);
    Serial.print('\t');
  }
  Serial.println();

  delay(10);
}
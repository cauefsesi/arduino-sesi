#include <AFMotor.h>

//Definição dos pinos do Motor na Shield
AF_DCMotor motor_E(1);
AF_DCMotor motor_D(3);


//Declaração de Constantes
const int lineFollowSensor1 = A15;
const int lineFollowSensor2 = A14;
const int lineFollowSensor3 = A13;
const int lineFollowSensor4 = A9;
const int lineFollowSensor5 = A12;
const int lineFollowSensor6 = A10;
const int lineFollowSensor7 = A11;
const int lineFollowSensor8 = A8;

const int sensor_ref = 200;


//Declaração de Variáveis
int LFSensor [9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float erro = 0;
int previousError = 0;
int P = 0;
int Kp = 25;
int Prop_value = 0;
int Pot_mot_D = 0;
int Pot_mot_E = 0;
int Pot_base = 110;
int Pot_limite_sup = 255;
int Pot_limite_inf = 10;
int tempo_delay = 10; 
int contador = 0;


void leitura_sensores()
{
    LFSensor [1] = analogRead (lineFollowSensor1);
    LFSensor [2] = analogRead (lineFollowSensor2);
    LFSensor [3] = analogRead (lineFollowSensor3);
    LFSensor [4] = analogRead (lineFollowSensor4);
    LFSensor [5] = analogRead (lineFollowSensor5);
    LFSensor [6] = analogRead (lineFollowSensor6);
    LFSensor [7] = analogRead (lineFollowSensor7);
    LFSensor [8] = analogRead (lineFollowSensor8);

    
    //if      ((LFSensor [1] >= sensor_ref)  && (LFSensor [2] <  sensor_ref)  && (LFSensor [3] <  sensor_ref)  && (LFSensor [4] <  sensor_ref)  && (LFSensor [5] <  sensor_ref) && (LFSensor [6] <  sensor_ref) && (LFSensor [7] <  sensor_ref)) erro =  7;    //(1 0 0 0 0 0 0)
    //else if ((LFSensor [1] >= sensor_ref)  && (LFSensor [2] >= sensor_ref)  && (LFSensor [3] <  sensor_ref)  && (LFSensor [4] <  sensor_ref)  && (LFSensor [5] <  sensor_ref) && (LFSensor [6] <  sensor_ref) && (LFSensor [7] <  sensor_ref)) erro =  6;    //(1 1 0 0 0 0 0)
    if      ((LFSensor [1] <  sensor_ref)  && (LFSensor [2] >= sensor_ref)  && (LFSensor [3] <  sensor_ref)  && (LFSensor [4] <  sensor_ref)  && (LFSensor [5] <  sensor_ref) && (LFSensor [6] <  sensor_ref) && (LFSensor [7] <  sensor_ref)) erro =  5;    //(0 1 0 0 0 0 0)
    else if ((LFSensor [1] <  sensor_ref)  && (LFSensor [2] >= sensor_ref)  && (LFSensor [3] >= sensor_ref)  && (LFSensor [4] < sensor_ref)   && (LFSensor [5] <  sensor_ref) && (LFSensor [6] <  sensor_ref) && (LFSensor [7] <  sensor_ref)) erro =  4;    //(0 1 1 0 0 0 0)
    else if ((LFSensor [1] <  sensor_ref)  && (LFSensor [2] <  sensor_ref)  && (LFSensor [3] >= sensor_ref)  && (LFSensor [4] < sensor_ref)   && (LFSensor [5] <  sensor_ref) && (LFSensor [6] <  sensor_ref) && (LFSensor [7] <  sensor_ref)) erro =  3;    //(0 0 1 0 0 0 0)
    else if ((LFSensor [1] <  sensor_ref)  && (LFSensor [2] <  sensor_ref)  && (LFSensor [3] >= sensor_ref)  && (LFSensor [4] >= sensor_ref)  && (LFSensor [5] <  sensor_ref) && (LFSensor [6] <  sensor_ref) && (LFSensor [7] <  sensor_ref)) erro =  2.5;  //(0 0 1 1 0 0 0)

    else if ((LFSensor [1] <  sensor_ref)  && (LFSensor [2] <  sensor_ref)  && (LFSensor [3] <  sensor_ref)  && (LFSensor [4] >= sensor_ref)  && (LFSensor [5] <  sensor_ref) && (LFSensor [6] <  sensor_ref) && (LFSensor [7] <  sensor_ref)) erro =  0;     //(0 0 0 1 0 0 0)
    else if ((LFSensor [1] <  sensor_ref)  && (LFSensor [2] <  sensor_ref)  && (LFSensor [3] <  sensor_ref)  && (LFSensor [4] <  sensor_ref)  && (LFSensor [5] <  sensor_ref) && (LFSensor [6] <  sensor_ref) && (LFSensor [7] <  sensor_ref)) erro =  0;     //(0 0 0 0 0 0 0)

    else if ((LFSensor [1] <  sensor_ref)  && (LFSensor [2] <  sensor_ref)  && (LFSensor [3] < sensor_ref)  && (LFSensor [4] >= sensor_ref)  && (LFSensor [5] >=  sensor_ref) && (LFSensor [6] <  sensor_ref) && (LFSensor [7] <  sensor_ref)) erro =  -2.5;  //(0 0 0 1 1 0 0)
    else if ((LFSensor [1] <  sensor_ref)  && (LFSensor [2] <  sensor_ref)  && (LFSensor [3] < sensor_ref)  && (LFSensor [4] <  sensor_ref)  && (LFSensor [5] >=  sensor_ref) && (LFSensor [6] <  sensor_ref) && (LFSensor [7] <  sensor_ref)) erro =  -3;    //(0 0 0 0 1 0 0)
    else if ((LFSensor [1] <  sensor_ref)  && (LFSensor [2] <  sensor_ref)  && (LFSensor [3] < sensor_ref)  && (LFSensor [4] <  sensor_ref)  && (LFSensor [5] >=  sensor_ref) && (LFSensor [6] >= sensor_ref) && (LFSensor [7] <  sensor_ref)) erro =  -4;    //(0 0 0 0 1 1 0)
    else if ((LFSensor [1] <  sensor_ref)  && (LFSensor [2] <  sensor_ref)  && (LFSensor [3] < sensor_ref)  && (LFSensor [4] <  sensor_ref)  && (LFSensor [5] <   sensor_ref) && (LFSensor [6] >  sensor_ref) && (LFSensor [7] <  sensor_ref)) erro =  -5;    //(0 0 0 0 0 1 0)
    //else if ((LFSensor [1] <  sensor_ref)  && (LFSensor [2] <  sensor_ref)  && (LFSensor [3] < sensor_ref)  && (LFSensor [4] <  sensor_ref)  && (LFSensor [5] <   sensor_ref) && (LFSensor [6] >= sensor_ref) && (LFSensor [7] >= sensor_ref)) erro =  -6;    //(0 0 0 0 0 1 1)
    //else if ((LFSensor [1] <  sensor_ref)  && (LFSensor [2] <  sensor_ref)  && (LFSensor [3] < sensor_ref)  && (LFSensor [4] <  sensor_ref)  && (LFSensor [5] <   sensor_ref) && (LFSensor [6] <  sensor_ref) && (LFSensor [7] >  sensor_ref)) erro =  -7;    //(0 0 0 0 0 0 1)

    else if ((LFSensor [1] >= sensor_ref)  && (LFSensor [2] >= sensor_ref)  && (LFSensor [3] >= sensor_ref) && (LFSensor [4] <  sensor_ref)  && (LFSensor [5] <  sensor_ref)  && (LFSensor [6] <  sensor_ref) && (LFSensor [7] <  sensor_ref))  //90 g esq 
   {
   motor_E.setSpeed(150);
     motor_E.run(BACKWARD);
     motor_D.setSpeed(150);
     motor_D.run(BACKWARD);
     delay(2900); 
     LFSensor [8] = 0;
   
            while (LFSensor [8] <  sensor_ref)
            {
            motor_E.setSpeed(150);
            motor_E.run(FORWARD);
            motor_D.setSpeed(150);
            motor_D.run(BACKWARD);
            delay(10); 
            LFSensor [8] = analogRead (lineFollowSensor8);
            }
             motor_E.setSpeed(150);
            motor_E.run(FORWARD);
            motor_D.setSpeed(150);
            motor_D.run(BACKWARD);
            delay(500);
     motor_E.setSpeed(150);
     motor_E.run(FORWARD);
     motor_D.setSpeed(150);
     motor_D.run(FORWARD);
     delay(2500); 
      
    }  
    else if ((LFSensor [1] <  sensor_ref)  && (LFSensor [2] <  sensor_ref)  && (LFSensor [3] <  sensor_ref) && (LFSensor [4] <  sensor_ref)  && (LFSensor [5] >= sensor_ref)  && (LFSensor [6] >= sensor_ref) && (LFSensor [7] >= sensor_ref)) //90 g dir
  {
       motor_E.setSpeed(150);
     motor_E.run(BACKWARD);
     motor_D.setSpeed(150);
     motor_D.run(BACKWARD);
     delay(2900); 
    LFSensor [8] = 0;
   
            while (LFSensor [8] <  sensor_ref)
            {
            motor_E.setSpeed(150);
            motor_E.run(BACKWARD);
            motor_D.setSpeed(150);
            motor_D.run(FORWARD);
            delay(10); 
            LFSensor [8] = analogRead (lineFollowSensor8);
            }
            motor_E.setSpeed(150);
            motor_E.run(FORWARD);
            motor_D.setSpeed(150);
            motor_D.run(BACKWARD); 
             delay(500);
     motor_E.setSpeed(150);
     motor_E.run(BACKWARD);
     motor_D.setSpeed(150);
     motor_D.run(BACKWARD);
     delay(2500);  
     
  }

    /*for (int i=0;i<7;i++)
    {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" :");
    Serial.println(LFSensor[i]);
    }*/

}

void Prop_control()
{ 
  leitura_sensores();
   
    P = Kp * erro;
    Pot_mot_D = Pot_base + P;
    Pot_mot_E = Pot_base - P + 10;
    
    
    if ((Pot_mot_D) > Pot_limite_sup) 
       Pot_mot_D = Pot_limite_sup;
       
    if ((Pot_mot_D) < Pot_limite_inf) 
       Pot_mot_D = Pot_limite_inf;
       
    if ((Pot_mot_E) > Pot_limite_sup) 
       Pot_mot_E = Pot_limite_sup;
       
    if ((Pot_mot_E) < Pot_limite_inf)  
       Pot_mot_E = Pot_limite_inf;
       
    Serial.print("Valor do erro:");
  
}

void motor_P_control()
{
    Prop_control();
    motor_E.run(BACKWARD);
    motor_D.run(BACKWARD);
    motor_E.setSpeed(Pot_mot_E);
    motor_D.setSpeed(Pot_mot_D);
    delay(tempo_delay);
}

void setup() 
{

    pinMode(lineFollowSensor1,INPUT);
    pinMode(lineFollowSensor2,INPUT);
    pinMode(lineFollowSensor3,INPUT);
    pinMode(lineFollowSensor4,INPUT);
    pinMode(lineFollowSensor5,INPUT);
    pinMode(lineFollowSensor6,INPUT);
    pinMode(lineFollowSensor7,INPUT);
    pinMode(lineFollowSensor8,INPUT);
    
    //Quebra de inércia dos motores
    motor_E.run(BACKWARD);
    motor_D.run(BACKWARD);
    motor_E.setSpeed(100);
    motor_D.setSpeed(100);
    motor_E.run(RELEASE);
    motor_D.run(RELEASE);

    Serial.begin(9600);//Inicializa a serial com velocidade de comunicação de 9600.
}

void loop() 
{
  motor_P_control();
}
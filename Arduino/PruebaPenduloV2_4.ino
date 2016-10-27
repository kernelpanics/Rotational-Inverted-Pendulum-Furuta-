int val; 
//******************************Encoder de la base*******************************:
int encoder1PinA = 7;
int encoder1PinB = 8;
volatile int encoder0Count = 0;
volatile float angulo0 = 0;
volatile float velocidad0 = 0;
volatile float angulo0_previo = 0;
volatile float angulo0_post = 0;
//**************************Sensor de posición del pendulo**************************:
volatile float angulo1 = 0;
volatile float velocidad2 = 0;
volatile float angulo1_previo = 0;
volatile float angulo1_post = 0;

volatile float k1;
volatile float k2;
volatile float k3;
volatile float k4;

volatile float k1_1;
volatile float k2_1;
volatile float k3_1;
volatile float k4_1;

//******************************Control del pendulo LQR*******************************:
//float k[]={-20.000,-260,-2,-8};
//{-25.000,-265,-0.91,-6.9};
//{-20.000,-265,-2,-8};
//{-20.000,-260,-2,-8};Mejor
//{-31.6228,-153.2589,-18.9058,-0.188588};
//{-2.0000,-279.085,-10.296,-0.1741};mejor
//{-11.1803,-56.2337,-7.4964,-6.8986};
//{-11.1803,-401.9054,-39.1665,-49.5090};
//{-10.0000,-359.5967,-35.1447,-44.2937};
//{-3.1623,-114.7889,-11.8729,-14.1166};
//{-3.1623,-350.3832,-33.4577,-43.1580};
//{-1.0000,-279.085,-27.296,-2.1741};

float u = 0;
float velocidad_motor = 0;
int velocidad_motor_MAX = 255; // velocidad maxima del PWM es 255


int n = LOW;
int in1=22;
int in2=24;
int ena=9;
int pos;
int i; 
int pulso;
long rpm;
float pi = 3.141592654;
unsigned long previo=0;
const long intervalo = 1000;
volatile unsigned long actual;
unsigned long tiempo;


 void setup() { 
  pinMode (encoder1PinA,INPUT);
  pinMode (encoder1PinB,INPUT);
  Serial.begin (250000);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(ena,OUTPUT);
  //REG_ADC_MR=0x10380280;
  
  attachInterrupt(7,doEncoder1A,CHANGE);
  attachInterrupt(8,doEncoder1B,CHANGE);
  //attachInterrupt(A8,constantes,CHANGE);
  //attachInterrupt(actual,constantes,CHANGE);
 } 

//LECTURA DE POSICION DEL BRAZO
void doEncoder1A(){
  if (digitalRead(encoder1PinA) == HIGH){
    if (digitalRead(encoder1PinB) == LOW){
      encoder0Count = encoder0Count + 1;
      } 
    else {
      encoder0Count = encoder0Count - 1;
      }
  }
  else {                                     
    if (digitalRead(encoder1PinB) == HIGH) { 
      encoder0Count = encoder0Count + 1;
    } 
    else {
      encoder0Count = encoder0Count - 1;
    }
  }
  angulo0 = 0.000523598*encoder0Count;   //El encoder tiene 400 pulsos por revolucion, la cuenta entre 400*2pi;  En radianes
}

void doEncoder1B(){  
  if (digitalRead(encoder1PinB) == HIGH) {  
    if (digitalRead(encoder1PinA) == HIGH) { 
      encoder0Count = encoder0Count + 1;
    } 
    else {
      encoder0Count = encoder0Count - 1;
    }
  }
  else {                                   
    if (digitalRead(encoder1PinA) == LOW) { 
      encoder0Count = encoder0Count + 1;
    } 
    else {
      encoder0Count = encoder0Count - 1;
    }
  }
  angulo0 = 0.000523598*encoder0Count;  //El encoder tiene 400 pulsos por revolucion, la cuenta entre 400*2pi;  En radianes
}//0.01571*

//LECTURA DE POSICION DEL PENDULO

void posPendulo(){
  pos=analogRead(A0);
  angulo1=(pos-238)*0.006551809; //angulo pendulo en radianes
  
}
//-(3.926990817-(pos-510)*0.01095904414)+1.570796327
//angulo1=(pos)*0.014612-10.655; //angulo1=(pos-697)*0.006756; En radianes
//angulo1=(pos-705)*0.007038123; //angulo1=(pos-697)*0.006756; En radianes
//(pos-710)*0.006756+0.5*0.006756;
//angulo1=(pos-704.5)*0.006637167; //angulo1=(pos-697)*0.006756; En radianes

void constantes(){
  k1_1=analogRead(A8);
  k2_1=analogRead(A9);
  k3_1=analogRead(A10);
  k4_1=analogRead(A11); 
  
}

void loop(){
  constantes();
  k1=-k1_1*0.1953125;
  k2=-k2_1*0.1953125;
//  k1=-k1_1*0.09765625;
//  k2=-100-k2_1*0.29296875;
  k3=-k3_1*0.01953125;
  k4=-k4_1*0.01953125; 
//  k2=-265;
//  k3=-2;
//  k4=-8; 
  posPendulo();
  actual=millis();
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);

//  if((actual-tiempo > 3)){
//    tiempo=actual;
//  }
//
//  switch(actual-tiempo){
//    case 1:
//      angulo0_previo = angulo0;
//      angulo1_previo = angulo1;
//    case 2:
//      angulo0_post = angulo0;  
//      angulo1_post = angulo1;
//    case 3:
//      velocidad0 = (angulo0_post - angulo0_previo)*500;  // Velocidad en rad/s *****brazo
//      velocidad2 = (angulo1_post - angulo1_previo)*500;  // Velocidad en rad/s *****Pendulo
//  }

  if((actual-tiempo > 6)){
    tiempo=actual;
  }

  //VELOCIDAD
  if (actual-tiempo == 0){
    angulo0_previo = angulo0;
    angulo1_previo = angulo1;
  }
  if (actual-tiempo == 3){
    angulo0_post = angulo0;  
    angulo1_post = angulo1;
  }
  if (actual-tiempo == 6){
    velocidad0 = (angulo0_post - angulo0_previo)*166.7;  // Velocidad en rad/s *****brazo
    velocidad2 = (angulo1_post - angulo1_previo)*166.7;  // Velocidad en rad/s *****Pendulo
  }
  u = -(k2*angulo1+k1*angulo0+k3*velocidad0+k4*velocidad2); //angulo0=brazo,angulo1=pendulo 
  //u = -(k[1]*angulo1+k[2]*angulo0+k[3]*velocidad0+k[4]*velocidad2); //angulo0=brazo,angulo1=pendulo 
  velocidad_motor = velocidad_motor_MAX*0.138889*u; // El motor funciona con 7.2V, la velocidad maxima es 255(señal PWM), velocidad=u*255/7.2
//  //velocidad_motor = 35.417*u; // El motor funciona con 7.2V, la velocidad maxima es 255(señal PWM), velocidad=u*255/7.2

//  Serial.print(k1);
//  Serial.print("||");
//  Serial.print(k2);
//  Serial.print("||");
//  Serial.print(k3);
//  Serial.print("||");
//  Serial.print(k4);
//  Serial.print("||");
//  Serial.print(velocidad_motor);
//  Serial.print("||");
//  Serial.print(angulo0);
//  Serial.print("||");
//  Serial.print(angulo1);
//  Serial.print("||");
//  Serial.print(velocidad0);
//  Serial.print("||");
//  Serial.print(velocidad2);
//  Serial.print("||");
//  Serial.print(pos);
//  Serial.print("||");
//  Serial.println(actual-tiempo);
  
  if (velocidad_motor > 0 ){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    
  }
  if (velocidad_motor < 0){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  
  if (velocidad_motor >= velocidad_motor_MAX){
    velocidad_motor = velocidad_motor_MAX;
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  if (velocidad_motor <= -velocidad_motor_MAX){
    velocidad_motor = -velocidad_motor_MAX;
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  analogWrite(ena,abs(velocidad_motor)); // Velocidad maxima del motor es 255
}





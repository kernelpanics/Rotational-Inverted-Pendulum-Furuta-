volatile float k1;
volatile float k2;
volatile float k3;
volatile float k4;

volatile float k1_1;
volatile float k2_1;
volatile float k3_1;
volatile float k4_1;
 void setup() { 
  Serial.begin (250000);
 } 

void constantes(){
  k1_1=analogRead(A8);
  k2_1=analogRead(A9);
  k3_1=analogRead(A10);
  k4_1=analogRead(A11); 
  k1=-k1_1*0.09765625;
  k2=-100-k2_1*0.29296875;
  k3=-k3_1*0.01953125;
  k4=-k4_1*0.01953125; 
}



void loop(){
constantes();


  Serial.print(k1);
  Serial.print("||");
  Serial.print(k2);
  Serial.print("||");
  Serial.print(k3);
  Serial.print("||");
  Serial.println(k4);
}





char a;
int b;
#include<SoftwareSerial.h>

int In1=7;
int In2=8; 
int In3=12;
int In4=13;
int ENA=5;
int ENB=11;
int SPED=90;
int SPED1=120;
int sensor = A1;
int green = 2;
int red = 3;
bool p = true;
void setup()
{ 
  Serial.begin(9600);
  pinMode(In1,OUTPUT);
  pinMode(In2,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(In3,OUTPUT);
  pinMode(In4,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(sensor, INPUT);
  pinMode(green, OUTPUT);
  pinMode(red, OUTPUT);
  
}

void loop() {

  b = analogRead(sensor);
  a = Serial.read();
  if (a == 'R')
      Red();
  else if(a == 'G')
      Green();
  else if( a == 'f')
      forward();
  else if(a == 'b')
    backward();
  else if(a == 'l')
    left();
  else if (a == 'r')
    right();
  else if (a == 'L')
    Left();
  if (b <= 200)
  {
    Stop();
    if(p == true)
        Serial.write('d');   
    p = false;
    
  }
  if(b>200 && p == false)
  {
    p = true;
  }
 
}
void forward()
  { 
     digitalWrite(In1,HIGH);
     digitalWrite(In2,LOW);
     analogWrite(ENA,SPED1);

     digitalWrite(In3,HIGH);
     digitalWrite(In4,LOW);
     analogWrite(ENB,SPED);
  }
  
  
  void backward()
  {  
 
   digitalWrite(In1,LOW);
  digitalWrite(In2,HIGH);
  analogWrite(ENA,SPED1);

 digitalWrite(In3,LOW);
  digitalWrite(In4,HIGH);
  analogWrite(ENB,SPED);
 }
  
  
  void Left()
{  
 
    digitalWrite(In1,HIGH);
  digitalWrite(In2,LOW);
  analogWrite(ENA,SPED1);
  digitalWrite(In3,LOW);
  digitalWrite(In4,HIGH);
  analogWrite(ENB,SPED);
 

}


 void left()
 {

  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  analogWrite(ENA, SPED1 - 10);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
  analogWrite(ENB, 0);
  
 }

 void right()
 {
 
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  analogWrite(ENB, SPED - 10);
  
 }

 
void Red()
{
 
  digitalWrite(red, HIGH);
  delay(100);
  digitalWrite(red, LOW);
}

void Green()
{
  digitalWrite(green, HIGH);
  delay(100);
  digitalWrite(green, LOW);
}
 void Stop()
 { 
    digitalWrite(In1,LOW);
    digitalWrite(In2,LOW);
    digitalWrite(In3,LOW);
    digitalWrite(In4,LOW);
    analogWrite(ENA,0);
    analogWrite(ENB,0); 
  }
  

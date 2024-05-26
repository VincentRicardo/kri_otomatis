#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40); 

#define SERVOMIN  125                                                 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  625                                                 // this is the 'maximum' pulse length count (out of 4096)

int sudut[20];
unsigned long myTime;
unsigned long currentTime;
bool flag = true;

void tahan_waktu(int waktu){
  myTime = millis();
  while(flag == true){
    currentTime = millis();
    if(currentTime - myTime < waktu){
      flag = true;
    }
    else if(currentTime - myTime >= waktu)
    {
      flag = false;
    }
  }
  flag = true;
}

void berdiri(){
  board1.setPWM(0, 0, angleToPulse(30));
  board1.setPWM(1, 0, angleToPulse(80));
  board1.setPWM(2, 0, angleToPulse(5));
  board1.setPWM(3, 0, angleToPulse(55)); //100
  board1.setPWM(4, 0, angleToPulse(105));    
  board1.setPWM(5, 0, angleToPulse(180));
  board1.setPWM(6, 0, angleToPulse(80)); //35
  board1.setPWM(7, 0, angleToPulse(100));
  board1.setPWM(8, 0, angleToPulse(0));
}

void tangga(){
  board1.setPWM(0, 0, angleToPulse(30));
  board1.setPWM(1, 0, angleToPulse(80));
  board1.setPWM(2, 0, angleToPulse(5));
  board1.setPWM(3, 0, angleToPulse(55));
  board1.setPWM(4, 0, angleToPulse(105));    
  board1.setPWM(5, 0, angleToPulse(180));
  //board1.setPWM(6, 0, angleToPulse(80));
  ///board1.setPWM(7, 0, angleToPulse(65));
  board1.setPWM(6, 0, angleToPulse(90));
  board1.setPWM(7, 0, angleToPulse(75));
  board1.setPWM(8, 0, angleToPulse(0));
}

void stand_awal(){
  board1.setPWM(0, 0, angleToPulse(30));
  board1.setPWM(3, 0, angleToPulse(55));
  board1.setPWM(6, 0, angleToPulse(80));
  
  //berubah
  board1.setPWM(1, 0, angleToPulse(20));
  board1.setPWM(4, 0, angleToPulse(160));
  board1.setPWM(7, 0, angleToPulse(20));
  //sisanya berdiri
  board1.setPWM(2, 0, angleToPulse(5));
  board1.setPWM(5, 0, angleToPulse(180));
  board1.setPWM(8, 0, angleToPulse(0));

  tahan_waktu(700);
  board1.setPWM(1, 0, angleToPulse(80));
  board1.setPWM(4, 0, angleToPulse(105));
  board1.setPWM(7, 0, angleToPulse(100));
}

void setup() {
  Serial.begin(9600);
  board1.begin();
  board1.setPWMFreq(60);
  stand_awal();
  
}


void loop(){
  if (Serial.available() > 0){
    String data = Serial.readStringUntil('\n');
    sscanf(data.c_str(), "%d %d %d %d %d %d %d %d %d %d", &sudut[0], &sudut[1], &sudut[2], &sudut[3], &sudut[4], &sudut[5], &sudut[6], &sudut[7], &sudut[8], &sudut[9]);
    if(sudut[0] == 1){
      berdiri();
      Serial.write(18);
    }
    else if(sudut[0] == 0){
      board1.setPWM(1, 0, angleToPulse(20));
      board1.setPWM(4, 0, angleToPulse(160));
      board1.setPWM(7, 0, angleToPulse(20));
      tahan_waktu(400);
      board1.setPWM(0, 0, angleToPulse(sudut[1]));
      board1.setPWM(3, 0, angleToPulse(sudut[4]));
      board1.setPWM(6, 0, angleToPulse(sudut[7]));

      tahan_waktu(100);

      board1.setPWM(1, 0, angleToPulse(sudut[2]));
      board1.setPWM(4, 0, angleToPulse(sudut[5]));
      board1.setPWM(7, 0, angleToPulse(sudut[8]));

      board1.setPWM(2, 0, angleToPulse(sudut[3]));
      board1.setPWM(5, 0, angleToPulse(sudut[6]));
      board1.setPWM(8, 0, angleToPulse(sudut[9]));
      
      tahan_waktu(600);
      berdiri();
      Serial.write(18);
    }else if (sudut[0] == 2) {
      board1.setPWM(1, 0, angleToPulse(20));
      board1.setPWM(4, 0, angleToPulse(160));
      board1.setPWM(7, 0, angleToPulse(20));
      tahan_waktu(400);
      board1.setPWM(0, 0, angleToPulse(sudut[1]));
      board1.setPWM(3, 0, angleToPulse(sudut[4]));
      board1.setPWM(6, 0, angleToPulse(sudut[7]));

      tahan_waktu(100);

      board1.setPWM(1, 0, angleToPulse(sudut[2]));
      board1.setPWM(4, 0, angleToPulse(sudut[5]));
      board1.setPWM(7, 0, angleToPulse(sudut[8]));

      board1.setPWM(2, 0, angleToPulse(sudut[3]));
      board1.setPWM(5, 0, angleToPulse(sudut[6]));
      board1.setPWM(8, 0, angleToPulse(sudut[9]));
      
      tahan_waktu(600);
      berdiri();
      Serial.write(18);
    }else if (sudut[0] == 3) {
      board1.setPWM(1, 0, angleToPulse(20));
      board1.setPWM(4, 0, angleToPulse(160));
      board1.setPWM(7, 0, angleToPulse(20));
      tahan_waktu(400);
      board1.setPWM(0, 0, angleToPulse(sudut[1]));
      board1.setPWM(3, 0, angleToPulse(sudut[4]));
      board1.setPWM(6, 0, angleToPulse(sudut[7]));

      tahan_waktu(100);

      board1.setPWM(1, 0, angleToPulse(sudut[2]));
      board1.setPWM(4, 0, angleToPulse(sudut[5]));
      board1.setPWM(7, 0, angleToPulse(sudut[8]));
      board1.setPWM(2, 0, angleToPulse(sudut[3]));
      board1.setPWM(5, 0, angleToPulse(sudut[6]));
      board1.setPWM(8, 0, angleToPulse(sudut[9]));
      tahan_waktu(600);
      tangga();
      Serial.write(18);
    }else if (sudut[0] == 4){
      Serial.write(18);
    }else if (sudut[0]==5){
      //paha ngangkat dikit
      board1.setPWM(4, 0, angleToPulse(120)); // nambah 15
      board1.setPWM(7, 0, angleToPulse(60)); // ngurang 15
      tahan_waktu(500);
      //paha turun, betis ngelebar
      board1.setPWM(4, 0, angleToPulse(sudut[5])); //kurang 15 erajat
      board1.setPWM(7, 0, angleToPulse(sudut[8])); // nambah 15
      board1.setPWM(5, 0, angleToPulse(sudut[6])); // ngurang 20
      board1.setPWM(8, 0, angleToPulse(sudut[9])); //nambah 20
      tahan_waktu(1500);
      // balik ke posisi awal

      board1.setPWM(4, 0, angleToPulse(105));    
      board1.setPWM(5, 0, angleToPulse(180));
      board1.setPWM(7, 0, angleToPulse(100));
      board1.setPWM(8, 0, angleToPulse(25));
      //kaki sebelah ngelebar
      board1.setPWM(1, 0, angleToPulse(sudut[2])); //nambah 15
      board1.setPWM(2, 0, angleToPulse(sudut[3])); //nambah 20
      
      tahan_waktu(1500);
      //balik posisi awal
      board1.setPWM(1, 0, angleToPulse(90));
      board1.setPWM(2, 0, angleToPulse(20));
      berdiri();
      Serial.write(18);
      
    }else if (sudut[0] == 6){
      tahan_waktu(1700);
      board1.setPWM(1, 0, angleToPulse(20));
      board1.setPWM(4, 0, angleToPulse(160));
      board1.setPWM(7, 0, angleToPulse(20));
      tahan_waktu(500);
      //turun lagi
      berdiri();
      Serial.write(18);
    }
  }
}


int angleToPulse(int ang)                             //gets angle in degree and returns the pulse width
  {  int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max 
     return pulse;
  }

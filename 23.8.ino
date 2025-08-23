float Kp=50, ki=0, kd=35;
float error=0, P=0, I=0, D=0, PID_value=0;
float previous_error = 0 ;
int gia_tri_ban_dau = 100;
int PID_phai,PID_trai;
int angle;
#define sensor1 2
#define sensor2 A4
#define sensor3 A3
#define sensor4 11
#define sensor5 12
#define In1 4
#define In2 5
#define In3 8
#define In4 9
#define ENA 6
#define ENB 10
#define R_S A1 //ir sensor Right
#define echo A2    //Echo pin
#define trigger A3 //Trigger pin
void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);
void dung();
void chay_thang();

void setup()
{
  #include <Servo.h>
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
   pinMode(sensor1, INPUT);
    pinMode(sensor2, INPUT);
     pinMode(sensor3, INPUT);
      pinMode(sensor4, INPUT);
       pinMode(sensor5, INPUT);
  Serial.begin(9600);
}


void read_sensor_values()
{  
  if(digitalRead(sensor1)==1&&digitalRead(sensor2)==1&&digitalRead(sensor3)==1&&digitalRead(sensor4)==1&&digitalRead(sensor5)==1)
{digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
} 
if(digitalRead(sensor1)==0&&digitalRead(sensor2)==0&&digitalRead(sensor3)==0&&digitalRead(sensor4)==0&&digitalRead(sensor5)==1)
error=6;  
else if(digitalRead(sensor1)==0&&digitalRead(sensor2)==0&&digitalRead(sensor3)==0&&digitalRead(sensor4)==1&&digitalRead(sensor5)==1)
error=4;  
else if(digitalRead(sensor1)==0&&digitalRead(sensor2)==0&&digitalRead(sensor3)==1&&digitalRead(sensor4)==1&&digitalRead(sensor5)==0)
error=2;
else if(digitalRead(sensor1)==0&&digitalRead(sensor2)==0&&digitalRead(sensor3)==1&&digitalRead(sensor4)==0&&digitalRead(sensor5)==0)
error=0;
else if(digitalRead(sensor1)==0&&digitalRead(sensor2)==1&&digitalRead(sensor3)==1&&digitalRead(sensor4)==0&&digitalRead(sensor5)==0)
error=-2; 
if(digitalRead(sensor1)==1&&digitalRead(sensor2)==1&&digitalRead(sensor3)==0&&digitalRead(sensor4)==0&&digitalRead(sensor5)==0)
error=-4;
 else if(digitalRead(sensor1)==1&&digitalRead(sensor2)==0&&digitalRead(sensor3)==0&&digitalRead(sensor4)==0&&digitalRead(sensor5)==0)
 error=-6; 
  P = error;
  I = I+error;
  D = error-previous_error;
  PID_value = (Kp*P) + (ki*I) + (kd*D);
  previous_error = error;
  // Luôn cho hai bánh tiến
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);

  int base_speed = 30; // tốc độ tối thiểu để bánh vẫn quay
  int max_speed = 150;

  PID_phai = gia_tri_ban_dau - PID_value;
  PID_trai = gia_tri_ban_dau + PID_value;

  // Giữ trong khoảng [base_speed, max_speed]
  PID_phai = constrain(PID_phai, base_speed, max_speed);
  PID_trai = constrain(PID_trai, base_speed, max_speed);

  analogWrite(ENB, PID_phai);
  analogWrite(ENA, PID_trai);
}

void loop()
{
  read_sensor_values();
}

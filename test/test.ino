#include <Wire.h>
#include <MsTimer2.h>      //需要从库管理器中下载
#include <SoftwareSerial.h>


//宏定义
//编码器引脚定义
#define ENCODER_L 2 
#define DIRECTION_L 4
#define ENCODER_R 3
#define DIRECTION_R 13

//TB6612FNG驱动模块控制信号 共6个
#define IN1 6   
#define IN2 7
#define IN3 8
#define IN4 9
#define PWMA 5
#define PWMB 10

#define BTRX 11
#define BTTX 12


//SoftwareSerial BTserial(BTRX,BTTX);

//PID 相关
float Kp1=1,Ki1=0.0,Kd1=0,/*第一级的PID系数*/
      Kp2=0.45,Ki2=0.1,Kd2=0.08;/*第二级的PID系数*/

//计算速度有关:
int L_cnt=0,R_cnt=0,
    L_velocity=0,R_velocity=0,
    Aim_velocity=0,Aim_derction=0;

float accgyo[3];   //只需要三个数据，依次为x轴加速度、z轴加速度、y轴角速度
float Angle=0,Aim_angle=0,Gyo=0;    //小车倾角，和角速度

//陀螺仪初始化
void Mpu6050_Init()
{
  Wire.beginTransmission(0b1101000);  //Mpu6050硬件地址
  Wire.write(0x6b);                   //Mpu6050电源管理寄存器
  Wire.write(0);                      //写零开启Mpu6050
  Wire.endTransmission();

  //还可以设置量程//根据数据手册
  Wire.beginTransmission(0b1101000);  //Mpu6050硬件地址
  Wire.write(0x1b);                   //角速度设置寄存器
  Wire.write(8);                      //设置角速度量程为500°/s（加速度默认为2g）
  Wire.endTransmission();
}


//获得加速度和角速度数据
void GetMpuData(float accgyo[])
{
  short data[6]; //储存原始数据，两字节
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x3B);   
  Wire.endTransmission(); 
  Wire.requestFrom(0b1101000,14);

  while(Wire.available() < 14);    
  data[0] = Wire.read()<<8|Wire.read();  //x轴加速度  
  data[1] = Wire.read()<<8|Wire.read();
  data[2] = Wire.read()<<8|Wire.read();
  Wire.read();Wire.read();
  data[3] = Wire.read()<<8|Wire.read();
  data[4] = Wire.read()<<8|Wire.read();
  data[5] = Wire.read()<<8|Wire.read();
  
  accgyo[0]=-data[1]*0.000598154;
  accgyo[1]=data[2]*0.000598154;
  accgyo[2]=-data[3]*0.015259;
}

//角速度和加速度融合计算倾角，尽量简单实用
void CalculateAngle()
{
  static float gyo_last=0;
  
  accgyo[2]+=0.0;//校准，需要实测
  
  Gyo=0.5*(accgyo[2]+gyo_last);   //角速度一阶滤波
  gyo_last=accgyo[2];
  
  Angle=0.005*(57.296*atan(accgyo[0]/accgyo[1])+(-1.8)/*调零需要实测*/)+0.995*(Angle+(accgyo[2])*0.005);
}

//编码器中断处理函数。 正负，加减，高低电平需要实测
void L_encoder()
{
  //该函数力求简单，提高执行速度
  if(digitalRead(ENCODER_L)==HIGH)
  {
    if(digitalRead(DIRECTION_L)==LOW)L_cnt++;
    else L_cnt--;
  }
  else
  {
    if(digitalRead(DIRECTION_L)==HIGH)L_cnt++;
    else L_cnt--;
  }
}

void R_encoder()
{
   if(digitalRead(ENCODER_R)==HIGH)
  {
    if(digitalRead(DIRECTION_R)==LOW)
      R_cnt++;
    else R_cnt--;
  }
  else
  {
    if(digitalRead(DIRECTION_R)==HIGH)
      R_cnt++;
    else R_cnt--;
  }
}

//设定电机的pwm 范围为-255到+255
void Set_pwm(int L_pwm,int R_pwm)
{
  //倾倒检测，有必要，该倾倒处理需要倾倒后重启小车
  if(Angle>30||Angle<-30)
  {
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    digitalWrite(PWMA,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
    digitalWrite(PWMB,LOW);
    return;
  }
  
  //限幅
  if(L_pwm<-255)L_pwm=-255;
  if(L_pwm>255)L_pwm=255;
  if(R_pwm<-255)R_pwm=-255;
  if(R_pwm>255)R_pwm=255;


  if(L_pwm>=0)
  {
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    analogWrite(PWMA,L_pwm);
  }
  else
  {
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(PWMA,-L_pwm);
  }
  if(R_pwm>=0)
  {
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    analogWrite(PWMB,R_pwm);
  }
  else
  {
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    analogWrite(PWMB,-R_pwm);
  }
}

//计算轮子的速度
void Get_velocity()
{
  static int L_lastcnt=0,R_lastcnt=0;
  L_velocity=L_cnt-L_lastcnt;
  R_velocity=R_cnt-R_lastcnt;
  L_lastcnt=L_cnt;
  R_lastcnt=R_cnt;
}

int Balance_pwm=0;
//直立pid控制，采用串级pid
void Balance_control()
{
  static float I1=0/*为计算第一级积分*/,I2=0/*为计算第二级积分*/,last1=0,/*为计算第一级微分*/last2=0/*为计算第二级的微分*/,out1=0/*第一级的输出*/;
  float angle_error=0;

  angle_error=Angle-Aim_angle;
  
  I1+=angle_error;
  //积分限幅
  //if(I1>100)I1=100;
  //if(I1<-100)I1=-100;
  
  //过零点消除积分
  //if(angle_last*Angle<=0)I1=0;
  //angle_last=Angle;
  
  out1=-(angle_error*Kp1+I1*Ki1+(angle_error-last1)*Kd1);
  last1=angle_error;
  
  I2+=(Gyo-out1);
  Balance_pwm=(Gyo-out1)*Kp2+I2*Ki2+((Gyo-out1)-last2)*Kd2;
  last2=(Gyo-out1);
}

//方向控制
int Direction_pwm=0;
void Direction_control()
{
  float p=0.3;
  L_cnt+=Aim_derction;
  L_cnt-=Aim_derction;
  Direction_pwm=(L_cnt-R_cnt)*p;
}

int Velocity_pwm=0;
void Velocity_control()
{
  float p=0.05;
  Aim_angle=(L_velocity+R_velocity-Aim_velocity)*p;
}

//刷新输出
void Out_Flash()
{
  int L_pwm=Balance_pwm+Velocity_pwm-Direction_pwm,
      R_pwm=Balance_pwm+Direction_pwm+Velocity_pwm;

  //去掉死区,死去大小要实测
  if(L_pwm>0)L_pwm+=15;
  else L_pwm-=15;
  if(R_pwm>0)R_pwm+=15;
  else R_pwm-=15;
  
  Set_pwm(L_pwm,R_pwm);
}

long t1=0;//控制函数计时
//主要内核函数
void Kernel()
{
   sei();   //再次打开中断，不然会出现问题；该函数已不需要

   //打印中断时间
   //Serial.println(millis()-t1); 
   //t1=millis();
   
   static int cnt=0;   //中断次数计数器
   
   GetMpuData(accgyo);
   CalculateAngle();

   Direction_control();
   
   //20ms执行一次
   if(cnt%10==0)
   {
    Get_velocity();
    //Direction_control();
    Velocity_control();
   }
   
   Balance_control();
   Out_Flash(); 
   cnt++;
}

long t;   //记录中断运行时
void setup() {
  // put your setup code here, to run once:

  //编码器引脚初始化
  pinMode(ENCODER_L,INPUT);
  pinMode(DIRECTION_L,INPUT);
  pinMode(ENCODER_R,INPUT);
  pinMode(DIRECTION_R,INPUT);

  //电机引脚初始化
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(PWMB,OUTPUT);
  
  Serial.begin(2000000);   //串口调试,波特率的高低决定串口打印的时间
//  BTserial.begin(9600);
  
  //BTserial.println("AT+NAMEBalanceCar0");
  //delay(1000);
  
  Wire.begin();          //初始化IIC连接
  
  Mpu6050_Init();        //mpu6050初始化（设置）

  //编码器中断初始化
  attachInterrupt(0,L_encoder,CHANGE);
  attachInterrupt(1,R_encoder,CHANGE);

  MsTimer2::set(5,Kernel);  //定时中断dt ms, Kernel()主要控制函数
  MsTimer2::start();
  
}

char a,num[8],ch;
void loop() {
  // put your main code here, to run repeatedly:
//    if(BTserial.available())
//    {
//      a=BTserial.read(); 
//      //Serial.print(a);
//      if(a=='{')
//      {
//        char p=0;
//        
//        ch=BTserial.read();
//        BTserial.read();
//        
//         while(BTserial.available())
//         {
//           num[p++]=BTserial.read();
//         }
//         num[--p]='\0';
//         float nu=atol(num);
//         Serial.println(nu);
//         switch(ch)
//         {
//          case '0':Kp1+=(nu-50000)/200000;
//          case '1':Ki1+=(nu-50000)/200000;
//          case '2':Kd1+=(nu-50000)/200000;
//          case '3':Kp2+=(nu-50000)/200000;
//          case '4':Ki2+=(nu-50000)/200000;
//          case '5':Kd2+=(nu-50000)/200000;
//         }
//         
//      }
//      switch (a)
//      {
//        case 'A':Aim_velocity=-5;Aim_derction=0;break;
//        case 'B':Aim_velocity=-5;Aim_derction=1;break;
//        case 'C':Aim_velocity=0;Aim_derction=1;break;
//        case 'D':Aim_velocity=5;Aim_derction=1;break;
//        case 'E':Aim_velocity=5;Aim_derction=0;break;
//        case 'F':Aim_velocity=5;Aim_derction=-1;break;
//        case 'G':Aim_velocity=0;Aim_derction=-1;break;
//        case 'H':Aim_velocity=-5;Aim_derction=-1;break;
//        case 'Z':Aim_velocity=0;Aim_derction=0;break;
//      }
//    }
    
}


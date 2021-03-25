//编码器单圈11脉冲，减速比30

//要实现的作用：从上位机得到一个前进多少厘米/旋转多少角度的指令(占用0/1的USB串口)，然后准确的施行

//左轮和右轮分别对位置PID？首先要能走直线，其次要保证两者的步调尽可能一致（修正到同步之后需不需要补偿？如果修正的太慢会有左右偏差）

#define IN11    3       //IN1
#define IN12    2       //IN2    左轮
#define IN13    5       //IN3
#define IN14    4       //IN4    右轮
#define IN21    6       //IN1
#define IN22    7       //IN2    左轮
#define IN23    9       //IN3
#define IN24    8       //IN4    右轮


/*#define ENA1    3       //左
#define ENB1    5       //右
#define ENA2    6       //左
#define ENB2    9       //右
*/
#define forward {analogWrite(IN11,pwm);digitalWrite(IN12,LOW);analogWrite(IN13,pwm);digitalWrite(IN14,LOW);analogWrite(IN21,pwm);digitalWrite(IN22,LOW);analogWrite(IN23,255-pwm);digitalWrite(IN24,HIGH);}//正正正反
#define back {analogWrite(IN11,255-pwm);digitalWrite(IN12,HIGH);analogWrite(IN13,255-pwm);digitalWrite(IN14,HIGH);analogWrite(IN21,255-pwm);digitalWrite(IN22,HIGH);analogWrite(IN23,pwm);digitalWrite(IN24,LOW);}
#define left_turn {analogWrite(IN11,255-pwm);digitalWrite(IN12,HIGH);analogWrite(IN13,pwm);digitalWrite(IN14,LOW);analogWrite(IN21,pwm);digitalWrite(IN22,LOW);analogWrite(IN23,pwm);digitalWrite(IN24,LOW);}
#define right_turn {analogWrite(IN11,pwm);digitalWrite(IN12,LOW);analogWrite(IN13,255-pwm);digitalWrite(IN14,HIGH);analogWrite(IN21,255-pwm);digitalWrite(IN22,HIGH);analogWrite(IN23,255-pwm);;digitalWrite(IN24,HIGH);}
#define left {analogWrite(IN11,255-pwm);digitalWrite(IN12,HIGH);analogWrite(IN13,pwm);digitalWrite(IN14,LOW);analogWrite(IN21,255-pwm);digitalWrite(IN22,HIGH);analogWrite(IN23,255-pwm);digitalWrite(IN24,HIGH);}
#define right {analogWrite(IN11,pwm);digitalWrite(IN12,LOW);analogWrite(IN13,255-pwm);digitalWrite(IN14,HIGH);analogWrite(IN21,pwm);digitalWrite(IN22,LOW);analogWrite(IN23,pwm);digitalWrite(IN24,LOW);}
#define stopc {analogWrite(IN11,0);digitalWrite(IN12,0);analogWrite(IN13,0);digitalWrite(IN14,0);analogWrite(IN21,0);digitalWrite(IN22,0);analogWrite(IN23,0);digitalWrite(IN24,0);}

#define Sx {v1+=vx;v2+=vx;v3+=vx;v4-=vx;}
#define Sy {v1-=vy;v2+=vy;v3-=vy;v4-=vy;}
#define Delta {v1-=w;v2+=w;v3+=w;v4+=w;}

#define stopc {analogWrite(IN11,0);digitalWrite(IN12,0);analogWrite(IN13,0);digitalWrite(IN14,0);analogWrite(IN21,0);digitalWrite(IN22,0);analogWrite(IN23,0);digitalWrite(IN24,0);}
int v1=0,v2=0,v3=0,v4=0;
int w  =  120; //速度常数
int pix=6000; //像素差
int flag = 0;
struct PID  //PID参数
{
  double kp=0.26;
  double kd=0.0;
  double ki=0.0;
  double last_error;
  double integrator;
  double imax=300;
}self; 
double Outputp, Outputd,Outputi,derivative;                          //pid参数
double get_pid(PID self, double error);
void pid_forward();        //获得pid的函数
int pwm=100;                                   
int first=0;
char z;
int m=5000;
int vx  =   200;    
int vy   =   127; 
int a[2] = {5000,5000};
int i = 0;
int deriva =0 ;
int shu=0;

void setup(){
  pinMode(IN11,OUTPUT);
  pinMode(IN12,OUTPUT);
  pinMode(IN13,OUTPUT);
  pinMode(IN14,OUTPUT);
  pinMode(IN21,OUTPUT);
  pinMode(IN22,OUTPUT);
  pinMode(IN23,OUTPUT);
  pinMode(IN24,OUTPUT);
  pinMode(13,OUTPUT);
  Serial.begin(9600); //设置波特率

}

void loop(){
   if(pix==6000)
   {
     stopc;
   }
  

 
if(Serial.available())
{
   if(first==0)
   {
     pwm=200;
     forward;
     delay(1600);
     first=1;
   }
  m=0;
  while(Serial.available())
  {
    delay(10);
    z = Serial.read();
    if(z == '-')
    {
      flag = 1;
      continue;
    }
    m = m*10 + z - '0';
  }
  if (flag == 1) pix = -m;
  else pix=m;
   flag = 0; 
   a[0]=a[1];
   a[1] = pix;
   deriva = a[1]-a[0];
}
if(deriva>4800&&pix==5000&&shu>=70)
{
  pwm = 200;
  back;
  delay(1200);
  shu=0;
}
  if(pix==5000)       //没有pix,小车逆时针自转
  {
    pwm = 100;
    left_turn;
    shu=0;
  }
  else if((pix>300||pix<-200)&&pix!=6000)     //有pix,
  {
     pwm = get_pid(self,pix);
     if(pwm>100)pwm=100;
     if(pwm<-100)pwm=-100;
     if(pwm>0)
     {
     left_turn;
     }
     else 
     {
      pwm=-pwm;
     right_turn;
     }
     shu=0;
  }
  else if(pix!=6000)
  {
    w=get_pid(self,pix);
    if(w>100)w=100;
     if(w<-100)w=-100;
    Sx;
    Delta;
    pid_forward();
    shu++;
  
  }
  
  delay(5);
}

double get_pid(PID self, double error)
{
  
  //P
  Outputp = error*self.kp;
  //D
  derivative = (error - self.last_error);
  self.last_error = error;
  Outputd = self.kd * derivative;
  //I
  if(error>-150&&error<150)
  self.integrator += error;
  if(self.integrator<-self.imax||self.integrator>self.imax)
  {self.integrator = 0;}
  Outputi = self.integrator * self.ki;
  return Outputp+Outputi+Outputd;
}

void pid_forward(){
  if(v1>0){
      analogWrite(IN11,v1);
      digitalWrite(IN12,LOW);
    }else{
      analogWrite(IN11,255+v1);
      digitalWrite(IN12,HIGH);
      }
   if(v2>0){
      analogWrite(IN13,v2);
      digitalWrite(IN14,LOW);
    }else{
      analogWrite(IN13,255+v2);
      digitalWrite(IN14,HIGH);
      }
   if(v3>0){
      analogWrite(IN21,v3);
      digitalWrite(IN22,LOW);
    }else{
      analogWrite(IN21,255+v3);
      digitalWrite(IN22,HIGH);
      }
   if(v4>0){
      analogWrite(IN23,v4);
      digitalWrite(IN24,LOW);
    }else{
      analogWrite(IN23,255+v4);
      digitalWrite(IN24,HIGH);
      }
    //digitalWrite(13,HIGH);digitalWrite(13,HIGH);
  v1=0;
  v2=0;
  v3=0;
  v4=0;
  }

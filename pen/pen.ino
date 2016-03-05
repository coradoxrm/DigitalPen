#include "ov7670cfg.h"

#define WRST 28 //a0
#define SCCB_SCL 21 //c4
#define SCCB_SDA 20//c5
#define RCLK 25//a1
#define RRST 26//a4
#define OE 27//a11
#define WEN 29//a12
#define VSYNC 2//a15

#define D0 37//PC0
#define D1 36//PC1
#define D2 35//PC2
#define D3 34//PC3
#define D4 33//PC4
#define D5 32//PC5
#define D6 31//PC6
#define D7 30//PC7

#define TAKE 11 
//#define ROTA 1
//#define AT 9

#define L 8
#define R 9



int done = 0;
int state = 0;

void SCCB_Init()
{
  pinMode(SCCB_SCL,OUTPUT);
  pinMode(SCCB_SDA,OUTPUT);
}

void SCCB_Start()
{
    /*SCCB_SCCB_SDA=1;     //数据线高电平	   
    SCCB_SCCB_SCL=1;	    //在时钟线高的时候数据线由高至低
    delay_us(50);  
    SCCB_SCCB_SDA=0;
    delay_us(50);	 
    SCCB_SCCB_SCL=0;	    //数据线恢复低电平，单操作函数必要	  */
  digitalWrite(SCCB_SDA,HIGH);
  digitalWrite(SCCB_SCL,HIGH);
  delayMicroseconds(50);
  digitalWrite(SCCB_SDA,LOW);
  delayMicroseconds(50);
  digitalWrite(SCCB_SCL,LOW);
}

void SCCB_Stop(){
  
  /*void SCCB_Stop(void)
{
    SCCB_SCCB_SDA=0;
    delay_us(50);	 
    SCCB_SCCB_SCL=1;	
    delay_us(50); 
    SCCB_SCCB_SDA=1;	
    delay_us(50);
} */
  digitalWrite(SCCB_SDA,LOW);
  delayMicroseconds(50);
  digitalWrite(SCCB_SCL,HIGH);
  delayMicroseconds(50);
  digitalWrite(SCCB_SDA,HIGH);
  delayMicroseconds(50);
}

void SCCB_No_Ack(void)
{
	delayMicroseconds(50);//delay_us(50);
	digitalWrite(SCCB_SDA,HIGH);//SCCB_SCCB_SDA=1;	
	digitalWrite(SCCB_SCL,HIGH);//SCCB_SCCB_SCL=1;	
	delayMicroseconds(50);//delay_us(50);
	digitalWrite(SCCB_SCL,LOW);//SCCB_SCCB_SCL=0;	
	delayMicroseconds(50);//delay_us(50);
	digitalWrite(SCCB_SDA,LOW);//SCCB_SCCB_SDA=0;	
	delayMicroseconds(50);//delay_us(50);
}

byte SCCB_RD_Byte()
{
	byte temp=0,j;    
	pinMode(SCCB_SDA,INPUT);//SCCB_SCCB_SDA_IN();		//设置SCCB_SDA为输入  
	for(j=8;j>0;j--) 	//循环8次接收数据
	{		     	  
		delayMicroseconds(50);//delay_us(50);
		digitalWrite(SCCB_SCL,HIGH);//SCCB_SCCB_SCL=1;
		temp=temp<<1;
		if(digitalRead(SCCB_SDA))temp++;   
		delayMicroseconds(50);//delay_us(50);
		digitalWrite(SCCB_SCL,LOW);//SCCB_SCCB_SCL=0;
	}	
	pinMode(SCCB_SDA,OUTPUT);//SCCB_SCCB_SDA_OUT();		//设置SCCB_SDA为输出    
	return temp;
} 



byte SCCB_RD_Reg(byte reg)
{
	byte val=0;
	SCCB_Start(); 				//启动SCCB传输
	SCCB_WR_Byte(0x42);		//写器件ID	  
	delayMicroseconds(100);//delay_us(100);	 
  	SCCB_WR_Byte(reg);			//写寄存器地址	  
	delayMicroseconds(100);//delay_us(100);	  
	SCCB_Stop();   
	delayMicroseconds(100);//delay_us(100);	   
	//设置寄存器地址后，才是读
	SCCB_Start();
	SCCB_WR_Byte(0x43);	//发送读命令	  
	delayMicroseconds(100);//delay_us(100);
  	val=SCCB_RD_Byte();		 	//读取数据
  	SCCB_No_Ack();
  	SCCB_Stop();
  	return val;
}

byte SCCB_WR_Byte(byte dat)
{
	byte j,res;	 
	for(j=0;j<8;j++) //循环8次发送数据
	{
		if(dat&0x80)digitalWrite(SCCB_SDA,HIGH);	
		else digitalWrite(SCCB_SDA,LOW);
		dat<<=1;
		delayMicroseconds(50);
		digitalWrite(SCCB_SCL,HIGH);	
		delayMicroseconds(50);
		digitalWrite(SCCB_SCL,LOW);//SCCB_SCL=0;		   
	}			 
	pinMode(SCCB_SDA,INPUT);//SCCB_SCCB_SDA_IN();		//设置SCCB_SDA为输入 
	delayMicroseconds(50);//delay_us(50);
	digitalWrite(SCCB_SCL,HIGH);//SCCB_SCL=1;			//接收第九位,以判断是否发送成功
	delayMicroseconds(50);//delay_us(50);
	if(digitalRead(SCCB_SDA))res=1;  //SCCB_SDA=1发送失败，返回1
	else res=0;         //SCCB_SDA=0发送成功，返回0
	digitalWrite(SCCB_SCL,LOW);//SCCB_SCL=0;		 
	pinMode(SCCB_SDA,OUTPUT);//SCCB_SCCB_SDA_OUT();		//设置SCCB_SDA为输出    
	return res;  
}	

	

byte SCCB_WR_Reg(byte reg,byte data)
{
	byte res=0;
	SCCB_Start(); 					//启动SCCB传输
	if(SCCB_WR_Byte(0x42))res=1;	//写器件ID	  
	delayMicroseconds(100);//delay_us(100);
  	if(SCCB_WR_Byte(reg))res=1;		//写寄存器地址	  
	delayMicroseconds(100);//delay_us(100);
  	if(SCCB_WR_Byte(data))res=1; 	//写数据	 
  	SCCB_Stop();	  
  	return	res;
}	

byte OV7670_Init()
{
  //RCC->APB2ENR|=1<<2;		//先使能外设PORTA时钟
//	RCC->APB2ENR|=1<<3;		//先使能外设PORTB时钟

//PA0/1/4 输出
byte temp;


pinMode(WRST,OUTPUT);
pinMode(RCLK,OUTPUT);
pinMode(RRST,OUTPUT);
//PA15 输入、PA11/12/    14输出 **
pinMode(VSYNC,INPUT);

pinMode(OE,OUTPUT);
pinMode(WEN,OUTPUT);
pinMode(RRST,OUTPUT);

//JTAG_Set(SWD_ENABLE);
SCCB_Init();
//Serial.print("before");Serial.println(SCCB_RD_Reg(0x12));
if(SCCB_WR_Reg(0x12,0x80))return 1;
delay(50);
//Serial.print("after");Serial.println(SCCB_RD_Reg(0x12));
temp=SCCB_RD_Reg(0x0b);
if(temp!=0x73)return 2;
temp=SCCB_RD_Reg(0x0a);   
if(temp!=0x76)return 2;

for(int i=0;i<sizeof(ov7670_init_reg_tbl)/sizeof(ov7670_init_reg_tbl[0]);i++)
{
	   	SCCB_WR_Reg(ov7670_init_reg_tbl[i][0],ov7670_init_reg_tbl[i][1]);
		delay(2);
}
return 0x00;

}

volatile int OV_State=0;

void watcher(){
  /*
                if(ov_sta<2)
		{
			if(ov_sta==0)
			{
				OV7670_WRST=0;	 	//复位写指针		  		 
				OV7670_WRST=1;	
				OV7670_WREN=1;		//允许写入FIFO
			}else OV7670_WREN=0;	//禁止写入FIFO 	 
			ov_sta++;
		}
  */
  if(OV_State == 0)				//判断状态，第一次下降沿
  {
    OV_State = 1;
    digitalWrite(WRST,LOW);
    digitalWrite(WRST,HIGH);				//状态变为1
    digitalWrite(WEN,HIGH);		//使能WEN，写
  }
  else if(OV_State == 1)		//状态1，
  {
    digitalWrite(WEN,LOW);		//关闭写使能，禁止写
    OV_State = 2;				//状态变为2，准备读数据
    detachInterrupt(0);			//关闭外部中断 
  }
  
}


void readReset(){
  digitalWrite(OE,LOW);
  digitalWrite(RRST,LOW);
  digitalWrite(RCLK,LOW);
  digitalWrite(RCLK,HIGH);
  digitalWrite(RCLK,LOW);
  digitalWrite(RRST,HIGH);
  digitalWrite(RCLK,HIGH);
  
 /* OV7670_CS=0;	 
 		OV7670_RRST=0;				//开始复位读指针 
		OV7670_RCK=0;
		OV7670_RCK=1;
		OV7670_RCK=0;
		OV7670_RRST=1;				//复位读指针结束 
		OV7670_RCK=1;  */


}


void writeReset()
{
    pinMode(VSYNC,INPUT);
//digitalWrite(WRST,HIGH);
}


void camera_refresh(){
  
  while(OV_State != 2){
  }
  
  readReset();
  //digitalWrite(OE,LOW);//!!
  
	  //unsigned short colordata = 0;
          unsigned int colordata = 0;
          unsigned int sum = 0;
	  for(int x=0; x<240; x++)
	  {
		  for (int y = 0; y<320;y++)
		  {
                          //GPIOB->CRL=0X88888888;***********************************
                          
                         
			  digitalWrite(RCLK,LOW);
			  colordata=PINC;		//读高位
			  digitalWrite(RCLK,HIGH);
                          colordata<<=8;
			  digitalWrite(RCLK,LOW);
			  colordata|=PINC;			//读低位
			  digitalWrite(RCLK,HIGH);
                          //GPIOB->CRL=0X33333333;
                        /* if(x > 115 && y>155 && x<= 125 && y <= 165)
                         {
                           sum = sum + colordata/100;
                           //Serial.print(colordata);Serial.print(',');
                         }*/
			  //TFTscreen.drawPixel(x,y,colordata);  //RGB565
                          Serial.print(colordata);
                          if(y < 319)
                          Serial.print(",");
                          else
                          Serial.print('\n');
		  }
	  } 
  

  //Serial.print("cc");Serial.print(63488);Serial.print('c');
  
  digitalWrite(OE,HIGH);
  /*OV7670_CS=1; 							 
		OV7670_RCK=0; 
		OV7670_RCK=1;*/
  digitalWrite(RCLK,LOW);
  digitalWrite(RCLK,HIGH);

  delayMicroseconds(20);
  OV_State=0;
}


void setup()
{
// Stm32_Clock_Init(9);	//系统时钟设置
//uart_init(72,9600);	 	//串口初始化为9600
Serial.begin(115200);
pinMode(TAKE,INPUT);


 byte t;
 int num =0;


//pinMode(LED,OUTPUT);
      // pinMode(AT,OUTPUT);
       //digitalWrite(AT,HIGH);
       //Serial.begin(115200);//这里应该和你的模块通信波特率一致
       /*delay(100);
       Serial.println("AT");
       delay(100);
       Serial.println("AT+NAME=OPENJUMPER-Bluetooth");//命名模块名
       delay(100);
       AT;
       AT+ROLE=0;
       AT+PSWD=1234;
       AT+UART=115200,0,0;
       AT+RMAAD;
       Serial.println("AT+ROLE=0");//设置主从模式：0从机，1主机
       delay(100);
       Serial.println("AT+PSWD=1234");//设置配对密码，如1234
       delay(100);
       Serial.println("AT+UART=115200,0,0");//设置波特率9600，停止位1，校验位无
       delay(100);
       Serial.println("AT+RMAAD");//清空配对列表*/



//	delay_init(72);	   	 	//延时初始化 
// OV7670_Init();
 

 //delay(2000);
 while(t = OV7670_Init())//初始化OV7670
 {
                 //Serial.println(t);
		//LCD_ShowString(60,150,200,200,16,"OV7670 Error!!");
		//delay_ms(200);
	        //LCD_Fill(60,230,239,246,WHITE);
		//delay(2000);
                num++;
 }
 //Serial.println("init ok");
 //Serial.println(num);
done = 0;
writeReset();
attachInterrupt(0,watcher,RISING);

}

int val;
int temp;
int out = 0;

int l;
int r;

void loop()
{
 // val = digitalRead(TAKE);
  
  if(done == 0)//(val == HIGH)
  {
    //delay(2000);   
    //Serial.println("take");   
    //attachInterrupt(0,watcher,RISING);
    camera_refresh();
    done = 1;
    
   /* while(1)
    {
      l = digitalRead(L);
      r = digitalRead(R);
      if( l == HIGH)
      Serial.print('l');
      if( r == HIGH)
      Serial.print('r'); 
      delay(100);
    }*/
   
    //Serial.println("one pic ok");
   /* while(1)
    {
      temp = analogRead(ROTA);
  if(abs(out - temp) > 100)
  {
    if(temp > out)Serial.print('r');
    else Serial.print('l'); 
    out = temp;    
    //Serial.println(out);
  }
  }*/
      
   // }
    
    
  }
  
  /*temp = analogRead(ROTA);
  if(abs(out - temp) > 100)
  {
    if(temp > out)Serial.print('r');
    else Serial.print('l'); 
    out = temp;    
    //Serial.println(out);
  }*/
  
  
}

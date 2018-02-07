#include <stm32f4xx.h>
#include <math.h>


#define E   10
#define RS  11
#define D4  12
#define D5  13
#define D6  14
#define D7  15

int TimingDelay;
int hz10=1;

int dist=0;
int dist_prev;

int acc_x;
int acc_y;
int acc_z;

int Ready=0;

int acc_x0;
int acc_y0;
//int acc_z0;

int acc_x_predicted;
int acc_x_p_predicted;
int acc_x_p=1;
int acc_y_predicted;
int acc_y_p_predicted;
int acc_y_p=1;
int F=1; //dynamic
int Q=5; //filter
int H=1; //
int R=25; //noise
int FirstMeasure=1;

int height=40;

short mag_x;
short mag_y;
short mag_z;

int m1,m2,m3,m4; //motors: 0-3400
//     m1(clockwise)
//
// m2  1   m3
//
//     m4

int test=0;

short AC1,AC2,AC3,B1,B2,MB,MC,MD; //baro calibration variables
unsigned short AC4,AC5,AC6;       //baro calibration variables
long UT;
long UP;
long T,P,B5;
int oss;

int THROTTLE=0;
int YAW;
int PITCH;
int ROLL;

float acc_pid_p=1.0; //PID REGULATORS
float acc_pid_i=0.5;
float acc_pid_d=0.01;
float son_pid_p=1.0;
float son_pid_i=1.0;
float son_pid_d=0.1;
int son_pid;
int acc_x_pid;
int acc_y_pid;

long current_us=0;
long previous_acc_us=0;
long previous_son_us=0;

int main(void)
{
	  SCB->CPACR|= ((3UL << 10*2)|(3UL << 11*2));
	  SystemCoreClockUpdate();
      SysTick_Config(SystemCoreClock/1000000);

      InitRCC();
      InitGPIO();
      initLCD();
	  RCC->APB1RSTR&=~RCC_APB1RSTR_I2C2RST;

	  InitTIM();
	  InitIIC();
	  InitUSART();
	  InitIRQ();


	  InitMag();
	  InitAcc();
	  //InitBaro();

	  lcdwrstr("hello");
	  char buff[16];

      Delay_ms(3000);
      m1=0;m2=0;m3=0;m4=0;
      SetMot(m1,m2,m3,m4);
      Delay_ms(5000);

      acc_x0=acc_x;
      acc_y0=acc_y;

      SetMot(650,650,650,650);
      Delay_ms(200);
      SetMot(0,0,0,0);
      Delay_ms(200);
      SetMot(650,650,650,650);
      Delay_ms(200);
      SetMot(0,0,0,0);
      Delay_ms(200);
      SetMot(650,650,650,650);
      Delay_ms(200);
      SetMot(0,0,0,0);
      Delay_ms(200);
     // SetMot(500,500,500,500);

      Ready=1;

      //GetBaroPressAndTemp();

	  while(1)
	  {
		//
		// GetBaro(baro);
       //

        sprintf(buff,"%d",dist_prev);
        lcdwrstr(buff);
        // lcdwrstr_(", ");
        //sprintf(buff,"%d",mag_y);
        //lcdwrstr_(buff);

        Delay_ms(300);
	  }
}

void TIM4_IRQHandler()
{
 	if (TIM4->SR&TIM_SR_UIF)
	{
		TIM4->SR&=~TIM_SR_UIF;

		if (Ready==1)
		{
		if (hz10==4) hz10=1; else hz10++;

		int acc_x_1;
		int acc_y_1;

		//reading and filteration

		if (FirstMeasure==1)
		{
			GetAccXYZ(&acc_x,&acc_y,&acc_z);
			GetMagXYZ(&mag_x,&mag_y,&mag_z);
			acc_x_1=acc_x;
			acc_y_1=acc_y;
			FirstMeasure=0;
		} else {

		acc_x_1=acc_x;
		acc_y_1=acc_y;

		GetAccXYZ(&acc_x,&acc_y,&acc_z); }
 		GetMagXYZ(&mag_x,&mag_y,&mag_z);

		acc_x_predicted=F*acc_x_1;
		acc_x_p_predicted=F*acc_x_p*F+Q;

		float K = H*acc_x_p_predicted/(1.0f*H*H*acc_x_p_predicted+R);
		acc_x=(int)acc_x_predicted+K*(acc_x-H*acc_x_predicted);
		acc_x_p=(int)(1-K*H)*acc_x_p_predicted;

		acc_y_predicted=F*acc_y_1;
		acc_y_p_predicted=F*acc_y_p*F+Q;

		K = H*acc_y_p_predicted/(1.0f*H*H*acc_y_p_predicted+R);
		acc_y=(int)acc_y_predicted+K*(acc_y-H*acc_y_predicted);
		acc_y_p=(int)(1-K*H)*acc_y_p_predicted;

		//kinematics
		int last_us=current_us-previous_acc_us;
		acc_x_pid=acc_pid_p*abs(acc_x)+acc_pid_i*(abs(acc_x)+acc_x_1)*last_us/1000000+acc_pid_d*(abs(acc_x)-acc_x_1)*10000000/last_us;
		acc_y_pid=acc_pid_p*abs(acc_y)+acc_pid_i*(abs(acc_y)+acc_y_1)*last_us/1000000+acc_pid_d*(abs(acc_y)-acc_y_1)*10000000/last_us;
		previous_acc_us=current_us;

		if (hz10==4)
			{   last_us=current_us-previous_son_us;
				dist_prev=dist;

				dist=get_dist(); //11.1ms
				if (dist==65535) dist=dist_prev;
				dist=-dist+height;
				son_pid=son_pid_p*dist+son_pid_i*(dist+dist_prev)*last_us/1000000+son_pid_d*(dist-dist_prev)*10000000/last_us;
				previous_son_us=current_us;

				//THROTTLE+=son_pid;
				//THROTTLE=650;
			}
		THROTTLE=450;
	    m1=THROTTLE;
	    if (acc_x>0) m1+=acc_x_pid;
	    m2=THROTTLE;
	    if (acc_y>0) m2+=acc_y_pid;
	    m3=THROTTLE;
	    if (acc_y<0) m3+=acc_y_pid;
	    m4=THROTTLE;
	    if (acc_x<0) m4+=acc_x_pid;
	    SetMot(m1,m2,m3,m4);
		}
	}
}

void InitRCC()
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;
	RCC->APB1ENR|=RCC_APB1ENR_TIM3EN;
	RCC->APB1ENR|=RCC_APB1ENR_TIM4EN;
	RCC->APB1ENR|=RCC_APB1ENR_I2C2EN;
	RCC->APB1ENR|=RCC_APB1ENR_USART2EN;
}

void InitGPIO()
{
	GPIOD->MODER=0x55500055;
	GPIOD->OTYPER=0x00000000;
	GPIOD->OSPEEDR=0x00000000;
	GPIOD->PUPDR=0x00000000;
	GPIOD->ODR=0x0C00;

	GPIOB->MODER=0x00A00000;
	GPIOB->OTYPER|=0x0C00;
	GPIOB->OSPEEDR=0x00000000;
	GPIOB->PUPDR=0x00000000;
	GPIOB->AFR[1]|=0x00004400;

	GPIOA->MODER|=0x000000a0;
	GPIOA->OTYPER|=0x0000;
	GPIOA->OSPEEDR=0x00000000;
	GPIOA->PUPDR|=0x00000000;
	GPIOA->AFR[0]|=0x00007700;
}

void InitTIM()
{
	TIM3->ARR=40000;
	TIM3->PSC=40;
	TIM3->DIER|=TIM_DIER_CC4IE|TIM_DIER_CC3IE|TIM_DIER_CC2IE|TIM_DIER_CC1IE|TIM_DIER_UIE;
	TIM3->CR1|=TIM_CR1_CEN|TIM_CR1_ARPE;

	TIM4->ARR=40000;
	TIM4->PSC=100;
	TIM4->DIER|=TIM_DIER_UIE;
	TIM4->CR1|=TIM_CR1_CEN|TIM_CR1_ARPE;
}

void InitIIC()
{
	I2C2->CR2|=40;
	I2C2->CCR|=(I2C_CCR_CCR&0x004)|I2C_CCR_FS|I2C_CCR_DUTY;
	I2C2->TRISE|=I2C_TRISE_TRISE&0x0D;
	I2C2->CR1|=I2C_CR1_PE;
}

void InitUSART()
{
	USART2->BRR|=0x1047;
	USART2->CR1|=USART_CR1_UE|USART_CR1_TE| USART_CR1_RE;
}

void InitIRQ()
{
	NVIC_SetPriority(TIM3_IRQn,0);
	NVIC_SetPriority(TIM4_IRQn,1);
	NVIC_EnableIRQ(TIM3_IRQn);
	Delay_ms(5);
	NVIC_EnableIRQ(TIM4_IRQn);
}

void send4(char gg)
{
	GPIOD->ODR&=~((0<<E)|(0<<RS)|(1<<D4)|(1<<D5)|(1<<D6)|(1<<D7));
	GPIOD->ODR|=(gg&0x0f)<<12;
	GPIOD->ODR|=(1<<E);
	Delay_us(1);
	GPIOD->ODR&=~(1<<E);
	Delay_us(40);
}

void sendByte(char gg)
{
	GPIOD->ODR&=~((1<<E)|(0<<RS)|(1<<D4)|(1<<D5)|(1<<D6)|(1<<D7));
	GPIOD->ODR|=(gg&0xf0)<<8;
	GPIOD->ODR|=(1<<E);
	Delay_us(1);
	GPIOD->ODR&=~(1<<E);
	Delay_us(40);
	GPIOD->ODR&=~((0<<E)|(0<<RS)|(1<<D4)|(1<<D5)|(1<<D6)|(1<<D7));
	GPIOD->ODR|=(gg&0x0f)<<12;
	GPIOD->ODR|=(1<<E);
	Delay_us(1);
	GPIOD->ODR&=~(1<<E);
	Delay_us(40);
}

void initLCD()
{
	Delay_ms(50);
	GPIOD->ODR&=~(1<<RS);
	GPIOD->ODR&=~(1<<E);
	send4(0x03);
	Delay_us(4500);
	send4(0x03);
	Delay_us(4500);
	send4(0x03);
	Delay_us(150);
	send4(0x02);
	sendByte(0x28);
	sendByte(0x01);
	Delay_us(2000);
	sendByte(0x06);
	sendByte(0x0C);
	sendByte(0x01);
	Delay_us(2000);
}

void lcdwrch(uint8_t symb)
{
	GPIOD->ODR|=(1<<RS);
	sendByte(symb);
	GPIOD->ODR&=~(1<<RS);
}

void lcdwrstr(char k[])
{
	int n=0;
	sendByte(0x01);
	Delay_us(2000);
	while (k[n]!=0)
	 {
		lcdwrch(k[n++]);
	 }
}

void lcdwrstr_(char k[])
{   int n=0;
	while (k[n]!=0)
	 {
		lcdwrch(k[n++]);
	 }
}

int get_temp()
{
	usart_send(0x11);
	usart_send(0x00);
	usart_send(0x00);
	usart_send(0x11);

	int t1=usart_get();
	int t2=usart_get();
	int t3=usart_get();
	int t4=usart_get();

	int temperature;
	if (t2&0xF0) temperature= (((t2&0x7F)<<8)|t3)/10;
	else temperature= (((t2)<<8)|t3)/10;
	return temperature;
}

int get_dist()
{
	usart_send(0x22);
	usart_send(0x00);
	usart_send(0x00);
	usart_send(0x22);

	int t1=usart_get();
	int t2=usart_get();
	int t3=usart_get();
	int t4=usart_get();

	return (t2<<8)|t3;
}

void usart_send(unsigned char g)
{

	while(!(USART2->SR & USART_SR_TXE)) {}
	(void) USART2->SR;
	USART2->DR  =  g;

}

int usart_get()
{
	while(!(USART2->SR & USART_SR_RXNE)) {}
    return   USART2->DR;
}

void InitAcc()
{
	I2C2->CR1|=I2C_CR1_START;

	while (!(I2C2->SR1&I2C_SR1_SB)){}

	(void)I2C2->SR1;

	I2C2->DR=0x82;

	while (!(I2C2->SR1&I2C_SR1_ADDR)){}

	(void)I2C2->SR1;
	(void)I2C2->SR2;

	I2C2->DR=0x0D;

	while (!(I2C2->SR1&I2C_SR1_BTF)){}
	while (!(I2C2->SR1&I2C_SR1_TXE)){}

	I2C2->DR=0x10;

	I2C2->CR1|=I2C_CR1_STOP;

	Delay_ms(10);

	I2C2->CR1|=I2C_CR1_START;

	while (!(I2C2->SR1&I2C_SR1_SB)){}

	(void)I2C2->SR1;

	I2C2->DR=0x82;

	while (!(I2C2->SR1&I2C_SR1_ADDR)){}

	(void)I2C2->SR1;
	(void)I2C2->SR2;

	I2C2->DR=0x20;

	while (!(I2C2->SR1&I2C_SR1_BTF)){}
	while (!(I2C2->SR1&I2C_SR1_TXE)){}

	I2C2->DR=0x08;

	I2C2->CR1|=I2C_CR1_STOP;

	Delay_ms(10);

	I2C2->CR1|=I2C_CR1_START;

	while (!(I2C2->SR1&I2C_SR1_SB)){}

	(void)I2C2->SR1;

	I2C2->DR=0x82;

	while (!(I2C2->SR1&I2C_SR1_ADDR)){}

	(void)I2C2->SR1;
	(void)I2C2->SR2;

	I2C2->DR=0x35;

	while (!(I2C2->SR1&I2C_SR1_BTF)){}
	while (!(I2C2->SR1&I2C_SR1_TXE)){}

	I2C2->DR=0x02;

	I2C2->CR1|=I2C_CR1_STOP;

	Delay_ms(10);
}

void InitMag()
{
	    I2C2->CR1|=I2C_CR1_START;

		while (!(I2C2->SR1&I2C_SR1_SB)){}

		(void)I2C2->SR1;

		I2C2->DR=0x3C;

		while (!(I2C2->SR1&I2C_SR1_ADDR)){}

		(void)I2C2->SR1;
		(void)I2C2->SR2;

		I2C2->DR=0x02;

		while (!(I2C2->SR1&I2C_SR1_BTF)){}
		while (!(I2C2->SR1&I2C_SR1_TXE)){}

		I2C2->DR=0x00;

		I2C2->CR1|=I2C_CR1_STOP;
}

void GetAccXYZ(int *x,int *y, int *z)
{
	int t7=GetIICdata(0x41,0x02);
	int t6=GetIICdata(0x41,0x03);

	if (t6&0x80) t6=0-0x7FF^(((t6&0x7C)<<4)|(t7>>2));
	else t6=((t6&0x7C)<<4)|(t7>>2);
	*x=t6;

	t7=GetIICdata(0x41,0x04);
	t6=GetIICdata(0x41,0x05);

	if (t6&0x80) t6=0-0x7FF^(((t6&0x7C)<<4)|(t7>>2));
	else t6=((t6&0x7C)<<4)|(t7>>2);
	*y=t6;

	t7=GetIICdata(0x41,0x06);
	t6=GetIICdata(0x41,0x07);

	if (t6&0x80) t6=0-0x7FF^(((t6&0x7C)<<4)|(t7>>2));
	else t6=((t6&0x7C)<<4)|(t7>>2);
	*z=t6;
}

void GetMagXYZ(int *x,int *y, int *z)
{
	int t7=GetIICdata(0x1E,0x03);
	int t6=GetIICdata(0x1E,0x04);

	*x=(t7<<8)|t6;

	t7=GetIICdata(0x1E,0x05);
	t6=GetIICdata(0x1E,0x06);

	*y=(t7<<8)|t6;

	t7=GetIICdata(0x1E,0x07);
	t6=GetIICdata(0x1E,0x08);

	*z=(t7<<8)|t6;
}

void InitBaro()
{
	GetBaroParam();
	GetBaroUT();
	oss=3;
	GetBaroUP();

}

void GetBaroPressAndTemp()
{
	GetBaroUT();
	GetBaroUP();
	GetBaroTT();
	GetBaroTP();
}

void GetBaroParam()
{
	int x1=GetIICdata(0x77,0xaa);
	int x2=GetIICdata(0x77,0xab);
	AC1=(short)x1<<8|x2;
		x1=GetIICdata(0x77,0xac);
		x2=GetIICdata(0x77,0xad);
	AC2=(short)x1<<8|x2;
		x1=GetIICdata(0x77,0xae);
		x2=GetIICdata(0x77,0xaf);
	AC3=(short)x1<<8|x2;
		x1=GetIICdata(0x77,0xb0);
		x2=GetIICdata(0x77,0xb1);
	AC4=(unsigned short)x1<<8|x2;
		x1=GetIICdata(0x77,0xb2);
		x2=GetIICdata(0x77,0xb3);
	AC5=(unsigned short)x1<<8|x2;
		x1=GetIICdata(0x77,0xb4);
		x2=GetIICdata(0x77,0xb5);
	AC6=(unsigned short)x1<<8|x2;
		x1=GetIICdata(0x77,0xb6);
		x2=GetIICdata(0x77,0xb7);
	B1=(short)x1<<8|x2;
		x1=GetIICdata(0x77,0xb8);
		x2=GetIICdata(0x77,0xb9);
	B2=(short)x1<<8|x2;
		x1=GetIICdata(0x77,0xba);
		x2=GetIICdata(0x77,0xbb);
	MB=(short)x1<<8|x2;
		x1=GetIICdata(0x77,0xbc);
		x2=GetIICdata(0x77,0xbd);
	MC=(short)x1<<8|x2;
		x1=GetIICdata(0x77,0xbe);
		x2=GetIICdata(0x77,0xbf);
	MD=(short)x1<<8|x2;
}

void GetBaroUT()
{
	I2C2->CR1|=I2C_CR1_START;

	while (!(I2C2->SR1&I2C_SR1_SB)){}

	(void)I2C2->SR1;

	I2C2->DR=0xEE;

	while (!(I2C2->SR1&I2C_SR1_ADDR)){}

	(void)I2C2->SR1;
	(void)I2C2->SR2;

	I2C2->DR=0xF4;

	while (!(I2C2->SR1&I2C_SR1_BTF)){}
	while (!(I2C2->SR1&I2C_SR1_TXE)){}

	I2C2->DR=0x2E;

	I2C2->CR1|=I2C_CR1_STOP;

	Delay_us(4500);


	I2C2->CR1|=I2C_CR1_START;

	while (!(I2C2->SR1&I2C_SR1_SB)){}

	(void)I2C2->SR1;

	I2C2->DR=0xEE;

	while (!(I2C2->SR1&I2C_SR1_ADDR)){}

	(void)I2C2->SR1;
	(void)I2C2->SR2;

	I2C2->DR=0xF6;

	while (!(I2C2->SR1&I2C_SR1_BTF)){}

	I2C2->CR1|=I2C_CR1_START;

	while (!(I2C2->SR1&I2C_SR1_SB)){}

	(void)I2C2->SR1;

	I2C2->DR=0xEF;

	while (!(I2C2->SR1&I2C_SR1_ADDR)){}

	(void)I2C2->SR1;
	(void)I2C2->SR2;

	while (!(I2C2->SR1&I2C_SR1_RXNE)){}

	I2C2->CR1|=I2C_CR1_ACK;

	unsigned char as1=I2C2->DR;

	I2C2->CR1&=~I2C_CR1_ACK;

	unsigned char as2=I2C2->DR;

	I2C2->CR1|=I2C_CR1_STOP;

	UT=(((long)as1<<8)|((long)as2));
}

void GetBaroUP()
{
	I2C2->CR1|=I2C_CR1_START;

	while (!(I2C2->SR1&I2C_SR1_SB)){}

	(void)I2C2->SR1;

	I2C2->DR=0xEE;

	while (!(I2C2->SR1&I2C_SR1_ADDR)){}

	(void)I2C2->SR1;
	(void)I2C2->SR2;

	I2C2->DR=0xF4;

	while (!(I2C2->SR1&I2C_SR1_BTF)){}
	while (!(I2C2->SR1&I2C_SR1_TXE)){}

	I2C2->DR=0xF4;

	I2C2->CR1|=I2C_CR1_STOP;

	Delay_us(25500);

	I2C2->CR1|=I2C_CR1_START;

	while (!(I2C2->SR1&I2C_SR1_SB)){}

	(void)I2C2->SR1;

	I2C2->DR=0xEE;

	while (!(I2C2->SR1&I2C_SR1_ADDR)){}

	(void)I2C2->SR1;
	(void)I2C2->SR2;

	I2C2->DR=0xF6;

	while (!(I2C2->SR1&I2C_SR1_BTF)){}

	I2C2->CR1|=I2C_CR1_START;

	while (!(I2C2->SR1&I2C_SR1_SB)){}

	(void)I2C2->SR1;

	I2C2->DR=0xEF;

	while (!(I2C2->SR1&I2C_SR1_ADDR)){}

	(void)I2C2->SR1;
	(void)I2C2->SR2;

	while (!(I2C2->SR1&I2C_SR1_RXNE)){}

	I2C2->CR1|=I2C_CR1_ACK;

	unsigned char as1=I2C2->DR;

	unsigned char as2=I2C2->DR;

	I2C2->CR1&=~I2C_CR1_ACK;

	unsigned char as3=I2C2->DR;

	I2C2->CR1|=I2C_CR1_STOP;

	UP=((long)as1<<16|as2<<8|as3)>>(8-oss);
}

void GetBaroTT()
{
	long X1,X2;
	X1=((long)UT-AC6)*AC5>>15;
	X2=((long)MC<<11)/(X1+MD);
	B5=X1+X2;
	T=(B5+8)>>4;
}

void GetBaroTP()
{
	long B6,X1,X2,X3,B3,B7;
	unsigned long B4;
	B6=B5-(long)4000;
	X1=(B2*((B6*B6)>>12))>>11;
	X2=AC2*B6>>11;
	X3=X1+X2;
	B3=((AC1*4+X3)<<oss+2)>>2;
	X1=AC3*B6>>13;
	X2=(B1*(B6*B6>>12))>>16;
	X3=((X1+X2)+2)>>2;
	B4=(AC4*(unsigned long)(X3+32768))>>15;
	B7=((unsigned long)UP-B3)*(50000>>oss);
	if (B7<0x80000000) P=B7*2/B4; else P=(B7/B4)*2;
	X1=(P>>8)*(P>>8);
	X1=(X1*3038)>>16;
	X2=(-7357*P)>>16;
	P=P+((X1+X2+3791)/16);
}

int GetPressInMm()
{
	return (int)P/133.3f;
}

int GetAltiInCm()
{
	return (int)4433000.0f*(1.0f-powf((float)P/101325.0f,(float)1.0f/5.255f));
}

int GetIICdata(int adr,int adr2)
{
	I2C2->CR1|=I2C_CR1_START;

	while (!(I2C2->SR1&I2C_SR1_SB)){}

	(void)I2C2->SR1;

	I2C2->DR=(adr<<1);

	while (!(I2C2->SR1&I2C_SR1_ADDR)){}

	(void)I2C2->SR1;
	(void)I2C2->SR2;

	I2C2->DR=adr2;

	while (!(I2C2->SR1&I2C_SR1_BTF)){}

	I2C2->CR1|=I2C_CR1_START;

	while (!(I2C2->SR1&I2C_SR1_SB)){}

	(void)I2C2->SR1;

	I2C2->DR=(adr<<1)|0x01;

	while (!(I2C2->SR1&I2C_SR1_ADDR)){}

	(void)I2C2->SR1;
	(void)I2C2->SR2;

	while (!(I2C2->SR1&I2C_SR1_RXNE)){}

	int tt=I2C2->DR;

	I2C2->CR1|=I2C_CR1_STOP;
	return tt;
}

void SetMot(int x1,int x3,int x2, int x4)
{
	 TIM3->CCR1=1500+x1;//xmax=3400
	 TIM3->CCR2=1500+x2;
	 TIM3->CCR3=1500+x3;
	 TIM3->CCR4=1500+x4;
}

void Delay_ms(__IO uint32_t nTime)
{
  TimingDelay = 1000*nTime;
  while(TimingDelay != 0);
}

void Delay_us(__IO uint32_t nTime)
{
  TimingDelay = nTime;
  while(TimingDelay != 0);
}

void SysTick_Handler()
{
	current_us++;
	if (TimingDelay != 0x00)
	  {
	    TimingDelay--;
	  }
}

void TIM3_IRQHandler()
{
	if (TIM3->SR&TIM_SR_CC4IF)
		{
			TIM3->SR&=~TIM_SR_CC4IF;
			GPIOD->ODR&=0xFFF7;
		}
	if (TIM3->SR&TIM_SR_CC3IF)
		{
			TIM3->SR&=~TIM_SR_CC3IF;
			GPIOD->ODR&=0xFFFB;
		}
	if (TIM3->SR&TIM_SR_CC2IF)
		{
			TIM3->SR&=~TIM_SR_CC2IF;
			GPIOD->ODR&=0xFFFD;
		}
	if (TIM3->SR&TIM_SR_CC1IF)
		{
			TIM3->SR&=~TIM_SR_CC1IF;
			GPIOD->ODR&=0xFFFE;
		}
	if (TIM3->SR&TIM_SR_UIF)
		{
			TIM3->SR&=~TIM_SR_UIF;
			GPIOD->ODR|=0x000F;
		}
}



void assert_failed(uint8_t* file, uint32_t line)
{
    lcdwrstr("err");
	while(1){}
}




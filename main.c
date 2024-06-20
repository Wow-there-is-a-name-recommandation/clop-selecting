/*
 * selecting.c
 *
 * Created: 2024-05-22 오전 10:27:54
 * Author : user
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <math.h>
#include <compat/ina90.h>

////////////////////////////////////////////////////ADC/////////////////////////////////////////////////////////////
void ADC_Init(void);
int adc_data[8] = {0};	// temp3, light4/ light5
int adc_SetChannel(unsigned char Adc_input);
double get_thermister();
void IR_cali();
void normalization(volatile int *adc,volatile int *adcmax,volatile int *adcmin,volatile int *normal,volatile int *flag);

volatile int IR_max[2] = {0,},IR_min[2] = {1023,1023}, IR_norm[2] = {0,}, IR_flag[2] = {0,};


////////////////////////////////////////////////////UART//////////////////////////////////////////////////////////////////
void Uart_Init(void);
void Uart_Receive(void);
void Uart_trans(unsigned char data);
void Uart_Num(int nNum);
void Uart_String(unsigned char sString[]);

////////////////////////////////////////////////PWM/////////////////////////////////////////////////////////////
void PWM_Init(void);
volatile int g_cnt = 0, s_cnt = 0;

////////////////////////////////////////////////////////////I2C&IMU/////////////////////////////////////////////////
void I2C_Init(void);
void write(unsigned char add,unsigned char dat); // 쓰기
unsigned char read(char addr); // 읽기

// x축 센서값 변수
double gx = 0;
double ax = 0;
double ay = 0;
double az = 0;
double gx_temp = 0;
double ax_temp = 0;
double ay_temp = 0;
double az_temp = 0;
unsigned char data[50];
double dt = 0.01;

double angle_accel = 0;
double angle_gyro = 0;
double last_angle_gyro = 0;
double roll=0;

//double getkalman(double acc, double gyro, double dt);

//////////////////////////// Filter //////////////////////////////////////////////////////
#define N 5			//필터 차수
const double fir_coeffs[N] = {0.2, 0.2, 0.2, 0.2, 0.2};		//필터 계수

double thermal_buffer[N] = {0,};	//이전 입력 저장

double fir_filter(double input, double *buffer);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(void)
{
	//Init
	ADC_Init();
	Uart_Init();
	PWM_Init();
	I2C_Init();
	
	DDRA = 0xff;
	PORTA = 0xff;
	DDRG = 0xff;			//LED on
	PORTG = 0b11111101;
	
	sei();
	
	//MPU setting
	write(0x6B, 0x00);	//센서 ON
	write(0x6C, 0x00);
	write(0x1B, 0x08);	// FS SEL=1 -> sensitivity: gyro 65.5
	write(0x1C, 0x08);	// AFS SEL=1 -> sensitivity: accel 8192
	write(0x1A, 0x05);	// DLPF 10Hz로 설정
	PORTA = 0b01010101;
	
    while (1) 
    {
		if(1)	//box done
		{
			
			/*
			PORTA = ~PORTA;
			////////////////////////IMU 받는것도 타이머로 할까
			gx_temp = (read(0x43)<<8)|read(0x44);
			ax_temp = (read(0x3B)<<8)|read(0x3C);
			ay_temp = (read(0x3D)<<8)|read(0x3E);
			az_temp = (read(0x3F)<<8)|read(0x40);
			gx = gx_temp / 65.5;
			ax = ax_temp / 8192;
			ay = ay_temp / 8192;
			az = az_temp / 8192;
			
			last_angle_gyro = angle_accel;	//자이로 각 계산용(칼만시 미사용)-자이로 각은 적분 오차 있으니 엑셀 각이나 상보 각으로 보정 필요
			angle_accel = atan(ay/sqrt(pow(ax,2)+pow(az,2)))*180/3.141592;	//가속도 각은 잘 나오는 것 확인
			angle_gyro = gx * dt + last_angle_gyro;							//자이로 각은 속도 문제 때문에 타이머 쪽에서 확인하는게 좋을 듯(빠른 주기 필요 예상)
			
			unsigned char gxt[8], axt[8];
			
			dtostrf(angle_gyro,5,2,gxt);
			dtostrf(angle_accel,5,2,axt);
			
			sprintf(data,"%s %s %s %s\n\r", "GyroX: ",gxt, "AccelX: ", axt);
			Uart_String(data);
			*/
			
			unsigned char gxt[8], axt[8];
			
			dtostrf(angle_gyro,5,2,gxt);
			dtostrf(angle_accel,5,2,axt);
			
			sprintf(data,"%s %s %s %s\n\r", "GyroX: ",gxt, "AccelX: ", axt);
			Uart_String(data);
		}
		else	//wait box
		{
			IR_cali();
		}
    }
}

///////////////////////////////////////////////ADC///////////////////////////////////////////////////////////////
void ADC_Init(void){
	DDRF = 0x00;
	ADMUX = 0b01000000;
	ADCSRA = 0b10000111;	//128 prescaler
}

int adc_SetChannel(unsigned char Adc_input){
	ADMUX = (Adc_input | 0x40);
	ADCSRA |= (1<<ADSC);
	while(!(ADCSRA & (1<<ADIF)));
	return ADC;
}

double get_thermister(){
	ADMUX = 0x03;
	
	ADCSRA |= (1<<ADSC);
	while(!(ADCSRA & ((1<<ADIF))));
	
	int adc = ADC;
	double Vadc = adc * 5.0 / 1023.0;
	
	double Rth = (5.0 / Vadc) * 4700 - 4700;
	double T = 1 / (0.0033 + (0.000274)*log(Rth/1000));
	T = T - 273.15;
	
	return T;
}

void IR_cali()
{
	for(int i=0;i<2;i++){
		ADMUX=i+4;

		ADCSRA|=(1<<ADSC);
		while(!(ADCSRA&(1<<ADIF)));
		adc_data[i]=ADC;
	}
	
	for(int i=0;i<2;i++){
		if(adc_data[i]>=IR_max[i])	IR_max[i]=adc_data[i];
		if(adc_data[i]<=IR_min[i])	IR_min[i]=adc_data[i];
	}
}

void normalization(volatile int *adc,volatile int *adcmax,volatile int *adcmin,volatile int *normal,volatile int *flag){
	for(int i=0;i<2;i++){
		normal[i]=(double)(adc[i]-adcmin[i])/(adcmax[i]-adcmin[i])*100;
	}
	for(int i=0;i<2;i++){
		if(normal[i]>90)		flag[i]=0;
		else					flag[i]=1;
	}
}

void get_IR(){
	for(int i=0;i<2;i++){
		ADMUX=i+4;

		ADCSRA|=(1<<ADSC);
		while(!(ADCSRA&(1<<ADIF)));
		adc_data[i]=ADC;
	}
}

///////////////////////////////////////////////////////////UART///////////////////////////////////////////////////////////////
void Uart_Init(void){
	DDRE=0x02;
	UCSR1A = 0x00;
	UCSR1B = (1<<RXEN1) | (1<<TXEN1);//uart 송수신 기능 허용
	UCSR1C = (1<<UCSZ11) | (1<<UCSZ10);//데이터의 길이는 8bit
	
	UBRR1H = 0;
	UBRR1L = 103;//속도 9600bps
}

void Uart_Receive(void){
	while(!(UCSR1A & (1<<RXC1)));
	//PORTA=0b11111110;
}

void Uart_trans(unsigned char data)
{
	while(!(UCSR1A &(1<<UDRE1)));
	UDR1 = data;
}

void Uart_Num(int nNum){
	Uart_trans(nNum / 10000 + 48);
	Uart_trans((nNum % 10000) / 1000 + 48);
	Uart_trans((nNum % 1000) / 100 + 48);
	Uart_trans((nNum % 100) / 10 + 48);
	Uart_trans((nNum % 10) + 48);
}

void Uart_String(unsigned char sString[])
{
	size_t i = 0;
	size_t len = strlen(sString);
	
	for(i=0; i<len; i++){
		while( !( UCSR1A & (1<<UDRE1) ) );
		UDR1 = sString[i];
	}
}
////////////////////////////////////////////////PWM/////////////////////////////////////////////////////////////
void PWM_Init(void)
{
	DDRB=0xff;
	DDRE=0b01101000;
	PORTE=0b00100000;
	
	TCCR2 = (0<<FOC2)|(1<<WGM21)|(1<<WGM20)|(1<<COM21)|(0<<COM20)|(1<<CS22)|(0<<CS21)|(1<<CS20);
	TIMSK = (1<<TOIE2);
	
	TCNT2 = 256-131;
	
	TCCR3A=(1<<COM3A1)|(1<<COM3B1)|(1<<WGM31);	//모터가 약하니 좀더 쎈거 찾아보자
	TCCR3B=(1<<WGM33)|(1<<WGM32)|(1<<CS30);

	ICR3=799;
	OCR3A=0;
}

ISR(TIMER2_OVF_vect){
	
	if (s_cnt >500)
	{
		g_cnt ++;
		
		if(g_cnt == 10){
			g_cnt = 0;
			
			/*************** Get Sensor ****************/
			double adc_thermistor = get_thermister();
			get_IR();
			normalization(adc_data, IR_max, IR_min, IR_norm,IR_flag);
			
			/*
			gx_temp = (read(0x43)<<8)|read(0x44);
			ax_temp = (read(0x3B)<<8)|read(0x3C);
			ay_temp = (read(0x3D)<<8)|read(0x3E);
			az_temp = (read(0x3F)<<8)|read(0x40);
			gx = gx_temp / 65.5;
			ax = ax_temp / 8192;
			ay = ay_temp / 8192;
			az = az_temp / 8192;
			
			last_angle_gyro = angle_accel;
			angle_accel = atan(ay/sqrt(pow(ax,2)+pow(az,2)))*180/3.141592;	//가속도 각은 잘 나오는 것 확인
			angle_gyro = gx * dt + last_angle_gyro;							//자이로 각은 속도 문제 때문에 타이머 쪽에서 확인하는게 좋을 듯(빠른 주기 필요 예상)
			//타이머에서 각도 처리 -> 필터 적용 필요
			*/
			
			Uart_Num(adc_thermistor);	Uart_trans(44);
			Uart_Num(IR_norm[0]);		Uart_trans(44);
			Uart_Num(IR_norm[1]);
			Uart_trans(13);
			
			/*******************Do sensor***************************/
			if(adc_thermistor >29)
			{
				OCR3A = 700;
			}
			else
			{
				OCR3A = 0;
			}
			
		
	}
	else{
		s_cnt ++;
		IR_cali();	//캘리용 실제 적용시 s_cnt 말고 초음파 플래그로
	}
	
	TCNT2 = 255 - 131;
}

///////////////////////////////////////////////////////IMU/////////////////////////////////////////////////////////////////
void I2C_Init(void){
	
	TWCR=(1<<TWINT)|(1<<TWEN); // TWI 활성화
	TWSR=0x00; // Prescaler : 1, 상태 초기화
	TWBR=12; // 00001100, Fscl = 400KHz(Fcpu/(16+2*TWBR*Prescaler) = Fscl)
}

void write(unsigned char add,unsigned char dat) //자이로 센서 설정 
{
	//PORTA = 0x02;//write확인
	_delay_us(50);  // 50us
	TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN); // S 10100100
	while(!(TWCR&(1<<TWINT))); // 전송 대기
	while((TWSR&0xF8)!=0x08); //신호 대기 00001000

	TWDR=0xD0; // AD+W저장11010000 slave 주소 + w? 뭘까
	TWCR=(1<<TWINT)|(1<<TWEN); // 전송10000100
	while(!(TWCR&(1<<TWINT))); //전송대기 여기서 멈춤
	while((TWSR&0xF8)!=0x18); //ACK대기
	//PORTA = 0b01101100;
	TWDR=add; // RA
	TWCR=0x84; // 전송
	while(!(TWCR&(1<<TWINT)));
	while((TWSR&0xF8)!=0x28); // ACK
	
	TWDR=dat; // DATA
	TWCR=0x84; // 전송
	while(!(TWCR&(1<<TWINT)));
	while((TWSR&0xF8)!=0x28); // ACK
	
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); // P
	_delay_us(50);  // 50us
	//PORTA = 0x02;//write확인
}

unsigned char read(char addr) //자이로 센서 값 읽어오기
{
	//PORTA |= 0x02;  //read확인
	unsigned char data; // data넣을 변수

	TWCR=0xA4; // S
	while((TWCR&0x80)==0x00); //통신대기
	while((TWSR&0xF8)!=0x08); //신호대기
	TWDR=0xD0; // AD+W
	TWCR=0x84; // 전송

	while((TWCR&0x80)==0x00); //통신대기
	while((TWSR&0xF8)!=0x18); //ACK
	TWDR=addr; // RA
	TWCR=0x84; //전송

	while((TWCR&0x80)==0x00); //통신대기
	while((TWSR&0xF8)!=0x28); //ACK
	TWCR=0xA4; // RS

	while((TWCR&0x80)==0x00); //통신대기
	while((TWSR&0xF8)!=0x10); //ACK
	TWDR=0xD1; // AD+R
	TWCR=0x84; //전송
	
	while((TWCR&0x80)==0x00); //통신대기
	while((TWSR&0xF8)!=0x40); // ACK
	TWCR=0x84;//전송

	while((TWCR&0x80)==0x00); //통신대기
	while((TWSR&0xF8)!=0x58); //ACK
	data=TWDR;
	TWCR=0x94; // P
	_delay_us(50);  // 50us
	PORTA = ~PORTA;
	return data;
}

////////////////////////// Filter ////////////////////////////////////////////
double fir_filter(double input, double *buffer)
{
	for (int i = N - 1; i > 0; i--) {
		buffer[i] = buffer[i - 1];
	}
	buffer[0] = input;

	double output = 0;
	for (int i = 0; i < N; i++) {
		output += fir_coeffs[i] * buffer[i];
	}

	return output;
}


/*
double getkalman(double acc,double gyro,double dt)
{
	angle += dt * (gyro - bias);

	P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
	P[0][1] -= dt * P[1][1] ;
	P[1][0] -= dt * P[1][1] ;
	P[1][1] += Q_gyro * dt ;

	double S = P[0][0] + R_measure;
	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;

	double y = acc - angle;
	angle += K[0] * y;
	bias += K[1] * y;

	double P_temp[2] = {P[0][0], P[0][1]};
	P[0][0] -= K[0] * P_temp[0];
	P[0][1] -= K[0] * P_temp[1];
	P[1][0] -= K[1] * P_temp[0];
	P[1][1] -= K[1] * P_temp[1];

	return angle;
};
*/
/*
#define F_CPU 16000000UL
#define sbi(PORTX,bitX) PORTX|=(1<<bitX)
#define cbi(PORTX,bitX) PORTX&=~(1<<bitX)
#define tbi(PORTX,bitX) PORTX^=(1<<bitX)

#define FS_SEL 131

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

void write(unsigned char add,unsigned char dat); // 쓰기
unsigned char read(char addr); // 읽기
void Uart_trans(unsigned char data);
void Uart_trans_int(int data);
void get_raw_data();
void calibrate();

volatile double dt = 0.000;
volatile int temp;
volatile unsigned char a_x_l, a_x_h, a_y_l, a_y_h, a_z_l, a_z_h;
volatile unsigned char g_x_l, g_x_h, g_y_l, g_y_h, g_z_l, g_z_h;
volatile double bas_a_x, bas_a_y, bas_a_z;
volatile double bas_g_x, bas_g_y, bas_g_z;
volatile double a_x, a_y, a_z;
volatile double g_x, g_y, g_z;
volatile double las_angle_gx, las_angle_gy, las_angle_gz;
volatile double angle_ax,  angle_ay,  angle_az;  
volatile double angle_gx,  angle_gy,  angle_gz;  
volatile double roll, pitch, yaw;
volatile double alpha;

int main()
{
	UCSR0A = 0x00;
	UCSR0B = (1<<TXEN0);
	UCSR0C = (3<<UCSZ00);
	UBRR0H = 0;
	UBRR0L = 103;
	
	TWCR=(1<<TWINT)|(1<<TWEN); // TWI 활성화
	TWSR=0x00; // Prescaler : 1, 상태 초기화
	TWBR=12; // 00001100, Fscl = 400KHz(Fcpu/(16+2*TWBR*Prescaler) = Fscl)
	
	TCCR2 = (1<<CS22)|(1<<CS01);
	TCNT2 = 256-125;
	TIMSK = (1<<TOIE2);
	
	DDRA = 0xff;
	PORTA = 0xff;
	
	write(0x6B, 0x00); //센서 ON
	write(0x6C, 0x00);
	PORTA = 7;
	write(0x1B, 0x08); // gyro set - 500/s로 설정= 65.4 LSB
	write(0x1A, 0x05); // DLPF 10Hz로 설정
	
	calibrate();
	sei();
	
	while(1)
	{
		PORTA = ~PORTA;
		get_raw_data();
		
		las_angle_gx = roll;
		las_angle_gy = pitch;
		las_angle_gz = yaw;
		
		temp = (a_x_h<<8)|a_x_l;
		a_x = -temp - 16383;
		temp = (a_y_h<<8)|a_y_l;
		a_y = -temp;
		temp = (a_z_h<<8)|a_z_l;
		a_z = temp;
		temp = (g_x_h<<8)|g_x_l;
		g_x = temp;
		temp = (g_y_h<<8)|g_y_l;
		g_y = temp;
		temp = (g_z_h<<8)|g_z_l;
		g_z = temp;
		
		g_x = (g_x - bas_g_x)/FS_SEL;
		g_y = (g_y - bas_g_y)/FS_SEL;
		g_z = (g_z - bas_g_z)/FS_SEL;
		
		angle_ax = atan(-1.000 * a_y/sqrt(pow(a_x,2)+pow(a_z,2))) * 180/3.141592;
		angle_ay = atan(a_x/sqrt(pow(a_y,2) + pow(a_z,2))) * 180/3.141592;
		
		angle_gx = g_x * dt + las_angle_gx;
		angle_gy = g_y * dt + las_angle_gy;
		angle_gz = g_z * dt + las_angle_gz;
		
		dt = 0.000;
		
		alpha = 0.96;
		roll = alpha*angle_gx + (1.000-alpha)*angle_ax;
		pitch = alpha*angle_gy + (1.000-alpha)*angle_ay;
		yaw = angle_gz;
		
		Uart_trans_int(roll);
		Uart_trans('\t');
		Uart_trans_int(pitch);
		Uart_trans('\t');
		Uart_trans_int(yaw);
		Uart_trans('\n');
		Uart_trans('\r');
		
		_delay_ms(100);
	}
}

ISR(TIMER2_OVF_vect){
	dt += 0.002;
	TCNT2 = 256-125;
}

void write(unsigned char add,unsigned char dat) // 쓰기
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR & 0xF8) != 0x08);
	
	TWDR = 0b11010000;
	TWCR = (1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR & 0xF8) != 0x18);
	
	TWDR = add;
	TWCR = (1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR & 0xF8) != 0x28);
	
	TWDR = dat;
	TWCR = (1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR & 0xF8) != 0x28);
	
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

unsigned char read(char addr) // 읽기
{
	unsigned char data;
	
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR & 0xF8) != 0x08);
	
	TWDR = 0b11010000;
	TWCR = (1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR & 0xF8) != 0x18);
	
	TWDR = addr;
	TWCR = (1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR & 0xF8) != 0x28);
	
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR & 0xF8) != 0x10);
	
	TWDR = 0b11010001;
	TWCR = (1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR & 0xF8) != 0x40);
	
	TWCR = (1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));
	while((TWSR & 0xF8) != 0x58);
	
	data = TWDR;
	
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
	
	return data;
}

void Uart_trans(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

void Uart_trans_int(int data)
{
	if(data < 0)
	{
		data = -data;
		Uart_trans('-');
	}
	else
		Uart_trans(' ');
		
	int temp = 0;
	temp = data / 10000;
	Uart_trans(temp + 48);
	temp = (data % 10000)/1000;
	Uart_trans(temp + 48);
	temp = (data % 1000)/100;
	Uart_trans(temp + 48);
	temp = (data % 100)/10;
	Uart_trans(temp + 48);
	temp = data % 10;
	Uart_trans(temp + 48);
}
void get_raw_data()
{
	
	a_x_h = read(0x3B);
	a_x_l = read(0x3C);
	a_y_h = read(0x3D);
	a_y_l = read(0x3E);
	a_z_h = read(0x3F);
	a_z_l = read(0x40);
	g_x_h = read(0x43);
	g_x_l = read(0x44);
	g_y_h = read(0x45);
	g_y_l = read(0x46);
	g_z_h = read(0x47);
	g_z_l = read(0x48);

}

void calibrate()
{
	int cal = 10;
	
	for(int i = 0; i<cal; i++)
	{
		PORTA = 7;
		get_raw_data();
		
		temp = (a_x_h<<8)|a_x_l;
		a_x += -temp - 16383;
		temp = (a_y_h<<8)|a_y_l;
		a_y += -temp;
		temp = (a_z_h<<8)|a_z_l;
		a_z += temp;
		temp = (g_x_h<<8)|g_x_l;
		g_x += temp;
		temp = (g_y_h<<8)|g_y_l;
		g_y += temp;
		temp = (g_z_h<<8)|g_z_l;
		g_z += temp;
		
		_delay_ms(100);
	}
	
	a_x /= cal;
	a_y /= cal;
	a_z /= cal;
	g_x /= cal;
	g_y /= cal;
	g_z /= cal;
	
	bas_a_x = a_x;
	bas_a_y = a_y;
	bas_a_z = a_z;
	bas_g_x = g_x;
	bas_g_y = g_y;
	bas_g_z = g_z;
}
*/

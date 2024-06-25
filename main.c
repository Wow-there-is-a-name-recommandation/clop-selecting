/*
 * BLDC.c
 *
 * Created: 2024-04-29 오후 1:19:58
 * Author : user
 */ 

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
volatile int end_flag = 0;

double get_Potentiometer(void);
double speed = 0;

double get_FSR402();
const double PRESSURE_THRESHOLDmin = 10;
const double PRESSURE_THRESHOLD = 200; // 임계값 (변경 가능)
const int NUM_SAMPLES = 10; // 샘플 개수
double adc_fsr402 = 0;

double read_distance_hc_sr04_sensor1();
double read_distance_hc_sr04_sensor2();
void distance_sensor_init();
const double DISTANCE_THRESHOLD = 5.0; // 일정 거리 임계값
volatile uint8_t run_flag = 0; // 플래그 변수

////////////////////////////////////////////////////UART//////////////////////////////////////////////////////////////////
void Uart_Init(void);
void Uart_Receive(void);
void Uart_trans(unsigned char data);
void Uart_Num(int nNum);
void Uart_String(unsigned char sString[]);

////////////////////////////////////////////////PWM/////////////////////////////////////////////////////////////
void PWM_Init(void);
unsigned int set_servo1(double angle);
unsigned int set_servo2(double angle);

volatile int g_cnt = 0, s_cnt = 0;

double angle1 = 0.0; // 서보 모터 1의 각도
double angle2 = 0.0; // 서보 모터 2의 각도

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
double dt = 0.01, alpha = 0.96;

double angle_accel = 0;
double angle_gyro = 0;
double last_angle_gyro = 0;
double roll=0;

//////////////////////////// Filter //////////////////////////////////////////////////////
#define N 10			//필터 차수
const double fir_coeffs[N] = {0.01, 0.0249, 0.0668, 0.1249, 0.1756, 0.1957, 0.1756, 0.1249, 0.0668, 0.0249, 0.01};		//필터 계수

double thermal_buffer[N] = {0,};	//이전 입력 저장
double IR1_buffer[N] = {0,};
double IR2_buffer[N] = {0,};
double poten_buffer[N] = {0,};
double press_buffer[N] = {0,};
double dis1_buffer[N] = {0,};
double dis2_buffer[N] = {0,};
double IR_filt[2] = {0,}, filt_thermo = 0, filt_poten = 0, filt_press, dis_filt[2] = {0,};

double fir_filter(double input, double *buffer);


double Q_angle = 0.001;
double Q_gyro = 0.003;
double R_measure = 0.05;

double Kal_angle = 0;
double angle = 0;
double rate = 0;
double bias = 0;

double P[2][2] = {0,};
double K[2];
double getkalman(double acc,double gyro,double dt);

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
	
    while (1) 
    {
		/*
			//선풍기
			if(filt_thermo >32)			{OCR3A = 790;}
			else if(filt_thermo >29)	{OCR3A = 700;}
			else						{OCR3A = 0;}
			
			//분류
			if (run_flag){
				
				if (filt_poten>3)
				{
					while(roll>20){
						PORTD &= 0b10101111;
						OCR3B = 700;
					}
					OCR3B = 0;
				}
				else{
					while(roll<20){
						PORTD &= 0b01011111;
						OCR3B = 700;
					}
					OCR3B = 0;
				}
				
				
				if (filt_press > PRESSURE_THRESHOLD) {
					PORTA = 0x0f;
					angle2 = 90.0; // 압력 센서가 임계값 이상이면 서보 모터 2를 90도로 회전
					set_servo2(angle2);
					_delay_ms(500);
					angle1 = 90.0;
					set_servo1(angle1);
					}
				else if (filt_press > PRESSURE_THRESHOLDmin) { // 압력이 가해지면 서보 모터 1이 180도 회전
					PORTA = 0xf0;
					angle1 = 90.0;
					set_servo1(angle1);
					}
				
				while (!end_flag);
				set_servo1(0);
				set_servo2(0);
				end_flag = 0;
				run_flag = 0;
			}
			else{
				IR_cali();
				set_servo1(0);
				set_servo2(0);
				
			}
				*/
		unsigned char gxt[8], axt[8];
		
		dtostrf(angle_accel,5,2,axt);
		dtostrf(Kal_angle,5,2,gxt);
		
		sprintf(data,"%s %s\n\r",axt, gxt);
		Uart_String(data);
    }
}

///////////////////////////////////////////////ADC///////////////////////////////////////////////////////////////
void ADC_Init(void){
	DDRF = 0x00;
	DDRC = 0b01010101;		//DC 방향 + 초음파
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

double get_Potentiometer(void){
	ADMUX = 0x00;
	
	ADCSRA |= (1<<ADSC);
	while(!(ADCSRA & ((1<<ADIF))));
	
	int adc = ADC;
	double Vadc = adc * 5.0 / 1023.0;
	
	return Vadc;
}

double get_FSR402() {
	uint32_t sum = 0;
	for (int i = 0; i < NUM_SAMPLES; i++) {
		sum += adc_SetChannel(7); // Read ADC value from channel 7
		_delay_ms(5); // 소프트웨어 디바운싱을 위해 약간의 지연 추가
	}
	double avg = sum / NUM_SAMPLES;
	double Vadc = avg * 5.0 / 1023.0;
	return Vadc;
}

double read_distance_hc_sr04_sensor1() {
	// Sensor 1 (PE4: 트리거, PE5: 에코)
	PORTC &= ~(1 << PC4); // 트리거 핀을 LOW로 설정하여 초기화
	_delay_us(2);

	PORTC |= (1 << PC4); // 트리거 핀을 HIGH로 설정하여 초음파 펄스를 발생
	_delay_us(10);
	PORTC &= ~(1 << PC4);
	uint32_t count = 0;
	while (!(PINC & (1 << PC5))); // 에코 핀에서 반사된 펄스 대기
	while (PINC & (1 << PC5)) {
		count++;
		_delay_us(1);
	}

	double distance = (count / 2.0) * 0.0343; // 거리 계산 (사운드 속도: 343 m/s)
	return distance;
}

double read_distance_hc_sr04_sensor2() {
	// Sensor 2 (PE6: 트리거, PE7: 에코)
	PORTC &= ~(1 << PC6); // 트리거 핀을 LOW로 설정하여 초기화
	_delay_us(2);

	PORTC |= (1 << PC6); // 트리거 핀을 HIGH로 설정하여 초음파 펄스를 발생
	_delay_us(10);
	PORTC &= ~(1 << PC6);

	uint32_t count = 0;
	while (!(PINC & (1 << PC7))); // 에코 핀에서 반사된 펄스 대기
	while (PINC & (1 << PC7)) {
		count++;
		_delay_us(1);
	}

	double distance = (count / 2.0) * 0.0343; // 거리 계산 (사운드 속도: 343 m/s)
	return distance;
}

void distance_sensor_init()
{
	double distance_sensor1 = read_distance_hc_sr04_sensor1(); // Sensor 1 값 읽기 (PE4: 트리거, PE5: 에코)
	double distance_sensor2 = read_distance_hc_sr04_sensor2(); // Sensor 2 값 읽기 (PE6: 트리거, PE7: 에코)
	dis_filt[0] = fir_filter(distance_sensor1, dis1_buffer);
	dis_filt[1] = fir_filter(distance_sensor2, dis2_buffer);
	
	//Uart_Num(distance_sensor1);	Uart_trans(44);
	//Uart_Num(distance_sensor2);	Uart_trans(44);
	if (dis_filt[0] < DISTANCE_THRESHOLD && dis_filt[1] < DISTANCE_THRESHOLD) {
		run_flag = 1; // 플래그 설정
		} else {
		run_flag = 0; // 플래그 해제
	}
	//Uart_Num(run_flag);	Uart_trans(13);
}

///////////////////////////////////////////////////////////UART///////////////////////////////////////////////////////////////
void Uart_Init(void){
	DDRE=0b00001000;				//uart
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
	DDRB=0xff;	//servo
	DDRE=0b00011000;	//DC
	PORTD&=0b10101111;	//ocr + 모터 방향 설정 차례
	
	TCCR2 = (0<<FOC2)|(1<<WGM21)|(1<<WGM20)|(1<<COM21)|(0<<COM20)|(1<<CS22)|(0<<CS21)|(1<<CS20);
	TIMSK = (1<<TOIE2);
	
	TCNT2 = 256-131;
	
	TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (1 << CS11) | (1 << CS10);
	ICR1 = 4999;
	
	angle1 = 0.0; // 초기 각도 설정
	set_servo1(angle1);
	angle2 = 45.0; // 초기 각도 설정
	set_servo2(angle2);
	
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
				/*
				double adc_thermistor = get_thermister();
				filt_thermo = fir_filter(adc_thermistor,thermal_buffer);
				get_IR();
				normalization(adc_data, IR_max, IR_min, IR_norm,IR_flag);
				IR_filt[0] = fir_filter(IR_norm[0], IR1_buffer);
				IR_filt[1] = fir_filter(IR_norm[1], IR2_buffer);
			
				if (IR_filt[0]<87|IR_filt[1]<87)
				{
					end_flag = 1;
				}
				distance_sensor_init();
				
				speed = get_Potentiometer();
				filt_poten = fir_filter(speed, poten_buffer);
				
				adc_fsr402 = get_FSR402()*1000;
				filt_press = fir_filter(adc_fsr402, press_buffer);
				*/
				gx_temp = (read(0x43)<<8)|read(0x44);
				ax_temp = (read(0x3B)<<8)|read(0x3C);
				ay_temp = (read(0x3D)<<8)|read(0x3E);
				az_temp = (read(0x3F)<<8)|read(0x40);
				gx = gx_temp / 65.5;
				ax = ax_temp / 8192;
				ay = ay_temp / 8192;
				az = az_temp / 8192;
			PORTA=5;
				//last_angle_gyro = roll;
				angle_accel = atan(ay/sqrt(pow(ax,2)+pow(az,2)))*180/3.141592;
				//angle_gyro = gx * dt + last_angle_gyro;
				//roll = alpha*angle_gyro + (1-alpha)*angle_accel;
				Kal_angle = getkalman(angle_accel, gx, dt);	
								PORTA=~PORTA;
			}
	}
	else{
		s_cnt ++;
	}
	
	TCNT2 = 255 - 131;
}

unsigned int set_servo1(double angle) {
	double width;
	double duty;
	width = (angle / 90.0) + 0.5;
	duty = (width / 20.0) * 100.0;
	OCR1A = (int)(duty / 100.0 * ICR1);
	return OCR1A;
}

unsigned int set_servo2(double angle) {
	double width;
	double duty;
	width = (angle / 90.0) + 0.5;
	duty = (width / 20.0) * 100.0;
	OCR1B = (int)(duty / 100.0 * ICR1);
	return OCR1B;
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
	//PORTA = ~PORTA;
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


/*
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned int i=0;
unsigned int j=0;
int mode = 0;

ISR(TIMER1_OVF_vect)
{
	/*
	j++;	//시동 중 1주기당 i 1개씩 증가
	if(j>=250)     // 한 주기가 20ms이므로 250주기(5초)가 지나면
	{
		PORTA = (1<<DDA1);   // 시동과정 끝나고 구동 시 LED 2개
		OCR1A = 2000;   // 1.5ms Pulse Width (약하게만 구동)
		OCR1B = 2000;
	}
	else    // 처음 실행 후 5초가 지나기 전까진
	{
		PORTA = ~PORTA; //led 1
	}
	
}

ISR(TIMER3_OVF_vect)
{
	/*
	i++;	//시동 중 1주기당 i 1개씩 증가
	if(i>=250)     // 한 주기가 20ms이므로 250주기(5초)가 지나면
	{
		PORTA = 1<<DDA3;   // 시동과정 끝나고 구동 시 LED 2개
		OCR3A = 2000;   // 1.5ms Pulse Width (약하게만 구동)
		OCR3B = 2000;
	}
	else    // 처음 실행 후 5초가 지나기 전까진
	{
		PORTA = ~PORTA; //led 1
	}
	
}

ISR(INT0_vect){
	switch(mode){
	case 0:
		PORTA = 0b11111011;
		OCR3A = 2000; //1.1ms
		OCR3B = 2000;

		OCR1A = 2000; //1.1ms
		OCR1B = 2000; //1.1ms
		
		mode = 1;
		break;
	case 1:
		PORTA = 0b11111101;
		OCR3A = OCR3A+20; //1.1ms
		OCR3B = OCR3B+20;

		OCR1A = OCR1A+20; //1.1ms
		OCR1B = OCR1B+20; //1.1ms
		break;
	default:
		break;
	}
	
}

void PWM_Setting()
{
	
	DDRB = 0xff;  // OC1ABC Output direction
	DDRE = 0x0f;
	
	PORTE = 0xaa;
	
	TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);  // 0 clear at OCR1A, high signal set at TOP(ICR1)
	TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS00);  // Fast PWM ICRn Top, 8 scaler,
	
	ICR1 = 799;       // 16Mhz / (8*40000) = 50 Hz, 20ms.
	OCR1A = 790;
}

void UART_setting()
{
	//수신
	UCSR1A=0x00;
	UCSR1B = (1<<RXEN1)|(1<<TXEN1)|(0<<UCSZ12);
	UCSR1C = (0<<UMSEL1)|(0<<UPM11)|(0<<UPM10)|(0<<USBS1)|(1<<UCSZ11)|(1<<UCSZ10);

	UBRR1H = 0;   //통신 속도 조절 레지스터
	UBRR1L = 103;//통신 속도 조절 레지스터ft7
}

unsigned char UART_recv(void)  // 데이터를 외부에서 받는 함수
{
	while(!(UCSR1A & (1<<RXC1)));
	//if(UCSR1A&(1<<RXC1))
	return UDR1;
	//else return 'q';
}

void LED_Setting()
{
	DDRA = 0xff;
}

void interrupt_Setting(void)
{
	DDRD = 0x00;
	
	EIMSK = (1<<INT0);
	EICRA = (1<<ISC01)|(0<<ISC00);

}

int main(void)
{
    /* Replace with your application code 
	PWM_Setting();
	LED_Setting();
	UART_setting();
	interrupt_Setting();
	
	
	//char sen;
	
	sei();
	
    while (1) 
    {
		/*
	    sen=UART_recv();

	    if(sen=='u')//시동 킨다
	    {
		    _delay_ms(5000);
		    OCR3B=3400;
		    OCR3C=3400;
	    }
	    else if(sen=='d')//시동 끈다
	    {
		    OCR3B=2000;
		    OCR3C=2000;
	    }
	    else if(sen=='l')//좌회전
	    {
		    OCR3A=4000;
	    }
	    else if(sen=='r')//우회전
	    {
		    OCR3A=2000;
	    }
	    else if(sen=='f')//전방
	    {
		    OCR3A=3000;//전방
	    }
	    else if(sen=='g')//추진 킨다
	    {
		    _delay_ms(5000);
		    OCR1A = 2500;
	    }
	    else if(sen=='s')//추진 끈다
	    {
		    OCR1A = 2000;
	    }
		
    }
}
*/

/*
	#define F_CPU 16000000UL
#define UBRR 103

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>

double angle1 = 0.0; // 서보 모터 1의 각도
double angle2 = 0.0; // 서보 모터 2의 각도

volatile int g_cnt = 0;
//volatile int last_tick = 0;

const double PRESSURE_THRESHOLDmin = 1.2;
const double PRESSURE_THRESHOLD = 2; // 임계값 (변경 가능)
const int NUM_SAMPLES = 10; // 샘플 개수
const int DELAY_SECONDS = 2.5;
volatile uint32_t pressure_time = 0;

const double DISTANCE_THRESHOLD = 5.0; // 일정 거리 임계값
volatile uint8_t run_flag = 0; // 플래그 변수

volatile double adc_fsr402 = 0.0;
//volatile double distance_sensor1 = 0.0;
//volatile double distance_sensor2 = 0.0;
   
void USART0_TX_vect(unsigned char data){
   while(!(UCSR0A &(1<<UDRE0)));
   UDR0 = data;
}

unsigned char USART0_RX(void){
   while(!(UCSR0A & (1<<RXC0)));
   return UDR0;
}

void USART0_NUM(int nNum){
   USART0_TX_vect(nNum / 10000 + 48);
   USART0_TX_vect((nNum % 10000) / 1000 + 48);
   USART0_TX_vect((nNum % 1000) / 100 + 48);
   USART0_TX_vect((nNum % 100) / 10 + 48);
   USART0_TX_vect((nNum % 10) + 48);
}
void USART0_STR(const char* str) {
   while(*str) {
      USART0_TX_vect(*str++);
   }
}

/////////////////////////////////adc///////////////////////////////////

void adc_init() {
   DDRF = 0x00;
   ADMUX = (1 << REFS0); // AREF = AVcc
   ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC Enable and prescaler of 128
}

uint16_t adc_read(int ch) {
   ADMUX = (ADMUX & 0xF8) | ch;
   ADCSRA |= (1 << ADSC); // Start single conversion
   while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
   return ADC;
}

double get_FSR402() {
   uint32_t sum = 0;
   for (int i = 0; i < NUM_SAMPLES; i++) {
      sum += adc_read(7); // Read ADC value from channel 7
      _delay_ms(5); // 소프트웨어 디바운싱을 위해 약간의 지연 추가
   }
   double avg = sum / NUM_SAMPLES;
   double Vadc = avg * 5.0 / 1023.0;
   return Vadc;
}
///////////////////////////servo//////////////////////////////////////////////////
unsigned int set_servo1(double angle) {
   double width;
   double duty;
   width = (angle / 90.0) + 0.5;
   duty = (width / 20.0) * 100.0;
   OCR1A = (int)(duty / 100.0 * ICR1);
   return OCR1A;
}

unsigned int set_servo2(double angle) {
   double width;
   double duty;
   width = (angle / 90.0) + 0.5;
   duty = (width / 20.0) * 100.0;
   OCR1B = (int)(duty / 100.0 * ICR1);
   return OCR1B;
}
///////////////////////////////////psd//////////////////////////////////////////////////
double read_distance_hc_sr04_sensor1() {
   // Sensor 1 (PE4: 트리거, PE5: 에코)
   PORTE &= ~(1 << PE4); // 트리거 핀을 LOW로 설정하여 초기화
   _delay_us(2);

   PORTE |= (1 << PE4); // 트리거 핀을 HIGH로 설정하여 초음파 펄스를 발생
   _delay_us(10);
   PORTE &= ~(1 << PE4);

   uint32_t count = 0;
   while (!(PINE & (1 << PE5))); // 에코 핀에서 반사된 펄스 대기
   while (PINE & (1 << PE5)) {
      count++;
      _delay_us(1);
   }

   double distance = (count / 2.0) * 0.0343; // 거리 계산 (사운드 속도: 343 m/s)
   return distance;
}

double read_distance_hc_sr04_sensor2() {
   // Sensor 2 (PE6: 트리거, PE7: 에코)
   PORTE &= ~(1 << PE6); // 트리거 핀을 LOW로 설정하여 초기화
   _delay_us(2);

   PORTE |= (1 << PE6); // 트리거 핀을 HIGH로 설정하여 초음파 펄스를 발생
   _delay_us(10);
   PORTE &= ~(1 << PE6);

   uint32_t count = 0;
   while (!(PINE & (1 << PE7))); // 에코 핀에서 반사된 펄스 대기
   while (PINE & (1 << PE7)) {
      count++;
      _delay_us(1);
   }

   double distance = (count / 2.0) * 0.0343; // 거리 계산 (사운드 속도: 343 m/s)
   return distance;
}
void distance_sensor_init()
{
   
   double distance_sensor1 = read_distance_hc_sr04_sensor1(); // Sensor 1 값 읽기 (PE4: 트리거, PE5: 에코)
   double distance_sensor2 = read_distance_hc_sr04_sensor2(); // Sensor 2 값 읽기 (PE6: 트리거, PE7: 에코)

   if (distance_sensor1 < DISTANCE_THRESHOLD || distance_sensor2 < DISTANCE_THRESHOLD) {
      run_flag = 1; // 플래그 설정
      } else {
      run_flag = 0; // 플래그 해제
   }

}

/*void send_distance_data() {
   double distance1 = read_distance_hc_sr04_sensor1();
   double distance2 = read_distance_hc_sr04_sensor2();

   char buffer[32];
   dtostrf(distance1, 6, 2, buffer);
   USART0_STR(buffer);
   USART0_STR(" \n\r");

   dtostrf(distance2, 6, 2, buffer);
   USART0_STR("Distance Sensor 2: ");
   USART0_STR(buffer);
   USART0_STR(" cm\n");
}

///////////////////////////////////press&servo//////////////////////////////////////////////
ISR(TIMER2_OVF_vect) {
   
   TCNT2 = 255 - 156;
   
   g_cnt++;
   if (g_cnt == 10) {
      g_cnt = 0;
      distance_sensor_init();
      double adc_fsr402 = get_FSR402();
   
   USART0_NUM((int)(adc_fsr402 * 1000)); // 소수점 아래 세 자리까지 전송
   USART0_TX_vect('\n');
   USART0_TX_vect('\r');
   
     }
   
}
////////////////////////////////timer////////////////////////////////////
void timer2_init() {
   TCCR2 = (1 << WGM21) | (1 << WGM20) | (1 << COM21) | (1 << CS22) | (1 << CS20);
   TIMSK = (1 << TOIE2);
   TCNT2 = 256 - 156;
} 

int main(void) {
   
   DDRB = 0xFF;
   DDRF = 0x00;
   DDRE = (1 << PE4) | (1 << PE6); // 초음파 센서 트리거 핀 출력 설정
   DDRE &= ~((1 << PE5) | (1 << PE7)); // 초음파 센서 에코 핀 설정
   

   TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0) | (1 << WGM11);
   TCCR1B = (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (1 << CS11) | (1 << CS10);
   ICR1 = 4999;
   
   angle1 = 0.0; // 초기 각도 설정
   angle2 = 45.0; // 초기 각도 설정
   
   UBRR0L = 103; // (unsigned char)UBRR;
   UBRR0H = 0; // (unsigned char)(UBRR>>8);

   UCSR0A = 0x00;
   UCSR0B = (1 << RXEN0) | (1 << TXEN0);
   UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

   adc_init();
   timer2_init();

   
   sei();

   while (1) {
      // 메인 루프
      static uint32_t last_tick = 0;
      last_tick++;
      
      if (run_flag){
         
         double adc_fsr402 = get_FSR402();
         if (adc_fsr402 > PRESSURE_THRESHOLDmin) { // 압력이 가해지면 서보 모터 1이 180도 회전
            angle1 = 180.0;
            pressure_time = last_tick; // 현재 시간을 기록            
            } else {
            if (last_tick - pressure_time > DELAY_SECONDS * (F_CPU / 1024 / 256)) {
               angle1 = 0.0;
            }
         }

         if (adc_fsr402 > PRESSURE_THRESHOLD) {
            angle2 = 90.0; // 압력 센서가 임계값 이상이면 서보 모터 2를 90도로 회전
            } else {
            if (last_tick - pressure_time > DELAY_SECONDS * (F_CPU / 1024 / 256)) {
               angle2 = 45.0; // 임계값 미만일 때 일정 시간이 지난 후 서보 모터 2를 45도로 회전
            }
         }
         set_servo1(angle1);
         set_servo2(angle2);
         //send_distance_data();
      }
      
   }
}
*/
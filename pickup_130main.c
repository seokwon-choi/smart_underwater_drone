/*	Purpose : ������ ������� Embedded Robot Project
	Target Name : MPro EMB485CAN - T37  (Number 74)
	Compiler : MDK-ARM Compiler V5
	MCU : Cortex-M0 NUC130
	CMSIS : NUC130_CMSIS302_LIBRARY
	Document : ������κ�_MPro_T37_�о�α�.txt
	D:\develop\M_Series\MPro\MPro_EMB485CAN_2019\Firmware_Code_UnderShip
	Information : 
		- 2020�� �ܱ�����б� ���ȣ, �ּ���, õ����, ������
	---------------------------------------------------------------
	Copyright by FirmwareBank Inc,. All Rights Reserved. */

/* ��� 
- DC Mptor
- Servo Motor
- RF400MHz
- Joy Stick
*/

/* ��� 
- 
*/

/*
���� : ���̽�ƽ ����
- 
- 
- 
- 

*/

/* ����

*/

// Target ����� ���� ���� ȸ��
// NUC130		  Interface
// PB15				Internal, 

// x				ENC(4) : x
// PA12				ENC(3) : Motor Encode 
// GND				ENC(2) : Encode GND 
// VCC				ENC(1) : Encode VCC 5V

// PD15/			1 Limit(4) : Switch
// PD14/			2 Limit(3) : Switch
// GND				LCD Port(2) : GND 
// VCC				LCD Port (1) : VCC 5V

// PB2				U35(5) : 400MHZ SET
// PB0/UART0 Rx		U35(4) : 400MHZ Tx
// PB1/UART0 Tx		U35(3) : 400MHZ Rx
// GND				U35(2) : 400MHZ GND
// VCC				U35(1) : 400MHZ VCC


/* ���ǻ���

*/

#include "Undersh.h"

// Geaneral Value
unsigned long ulpass;
unsigned short uspass;
unsigned char ucpass;
unsigned long runflag; 

struct ring{   
	volatile unsigned long		wp;
	volatile unsigned long 		rp;
	volatile unsigned char		buffer[QSIZE];
}; 

struct ring q0;					// for ring buffer for UART0
struct ring q1;					// for ring buffer for UART1
//struct ring q2;					// for ring buffer for UART2

// for UART
unsigned char get_char0, get_char1;

// timer
volatile unsigned long timer0tick, timer1tick; // Timer Value.
volatile unsigned long timer_buzzer;// buzzer on time
volatile unsigned long timer_redled;// LED Red Count time

// PWM
unsigned long pwmspeed[11];
unsigned long mot1_speed, mot2_speed; // motor speed
unsigned long speed;
unsigned long up_speed = 38, down_speed = 38;
unsigned long CMR0_speed, CMR1_speed, CMR2_speed, CMR3_speed;

// Running Processor
signed long motor_encode, motor_encode_backup;
volatile unsigned long time_limit_sound; // buzzer, beep sound auto off 

unsigned char switchcom_400, switchcom_rfid;//, switchcom_lcd;
volatile unsigned long timercom_400, timercom_rfid;//, timercom_lcd;
unsigned char uccommand_pccode, ucdevice;// command code, device
//unsigned char uc_checksum_400;
//unsigned char ret_pc[9]; 
unsigned char pc_data[4];	
unsigned char send_to_pc[9]; 

unsigned long rfid4byte, rfid4byte_temp;
unsigned long long ullrfid10BYTE;// 64Bit // RFCard ID

unsigned long ulfunc_Command;// ���� ��ɾ� ó��


// Proto Type
// timer
void time0_delay(unsigned long T0);//10m
void time1_delay(unsigned long T1);//1S

// Utility
// ���� ����
void back_left_motor_control(unsigned long back_left_motor_speed);
void back_right_motor_control(unsigned long back_right_motor_speed);
void top_left_AND_bottom_right(unsigned long up_down_speed);
void top_right_AND_bottom_left(unsigned long up_down_speed);
void motor_initialization(void);
void motor_stop(void);

// ���� ��� ����
void go_straight(unsigned long back_motor_speed);
void go_back(unsigned long back_motor_speed);
void turn_left(unsigned long back_left_motor_speed, unsigned long back_right_motor_speed);
void turn_right(unsigned long back_left_motor_speed, unsigned long back_right_motor_speed);
void go_down(unsigned long down_speed);
void go_up(unsigned long up_speed);
void hovering(unsigned long up_speed, unsigned long down_speed);

void beep_sound_timer(unsigned long time_sound);
void beep_sound(void);
unsigned char hex2ascii(unsigned char tohex);
unsigned long isflag(unsigned long flag);
void setflag(unsigned long flag);
void clrflag(unsigned long flag);
void togflag(unsigned long flag);

// system
void setup(void);

// uart
unsigned char check_pop0(void);
unsigned char check_pop1(void);

void ring_ini0(void);
void ring_ini1(void);

void putstring0(unsigned char *pui8Buffer);
void putstring1(unsigned char *pui8Buffer);

void Processor_pop0_400(void); // return F_ACT_400
void Processor_pop1(void); // return 
	
// Project, Running
extern void rf433mhz_ini(unsigned char ucdeviceaddress);
//void motor_under_robot(unsigned char mot_device,  unsigned char mot_direction, unsigned long pwmarray); 

unsigned char dip_read4(void){
unsigned char returndipvalue;

	returndipvalue =0;
	if(DIP1 ==0) returndipvalue |= (1<<3);
	if(DIP2 ==0) returndipvalue |= (1<<2);
	if(DIP3 ==0) returndipvalue |= (1<<1);
	if(DIP4 ==0) returndipvalue |= (1<<0);
	return returndipvalue;
}


		
void undership_action(void){
// for test
// Joy stick infor
//80 : x ����
//40 : x ������
//20 : Y -
//10 : Y +
//08 : Button D
//04 : Button B
//02 : Button A
//01 : Button C
	switch(uccommand_pccode){
		case 0: 
			motor_stop();// ��ư ���� ���� ���� 2�� ���� BUT ȣ������ ��� ����
			time0_delay(TM_100mS);
		break;
		
		case 0x12:// A��ư ������ ���̽�ƽ ������, ����
			hovering(up_speed, down_speed);
			motor_stop();
			time0_delay(TM_100mS);
			go_straight(40);
			time0_delay(TM_100mS);
		break;	

		case 0x22:// A��ư ������ ���̽�ƽ �ڷ�, ����
			hovering(up_speed, down_speed);
			motor_stop();
			time0_delay(TM_100mS);
			go_back(36);
			time0_delay(TM_100mS); // 		
		break;	

		case 0x42:// A��ư ������ ���̽�ƽ ����������, ��ȸ��
			hovering(up_speed, down_speed);
			motor_stop();
			time0_delay(TM_100mS);
			turn_right(40, 36);
			time0_delay(TM_100mS); // 	
		break;	

		case 0x82:// A��ư ������ ���̽�ƽ ��������, ��ȸ��
			hovering(up_speed, down_speed);
			motor_stop();
			time0_delay(TM_100mS);
			turn_left(36, 40);
			time0_delay(TM_100mS); // 		
		break;	

		case 0x14:// B��ư ������ ���̽�ƽ ������, top left+bottom right speed up
			up_speed++;// ���� �����̱� ������ ����
			time0_delay(TM_100mS); // 
		break;	

		case 0x24:// B��ư ������ ���̽�ƽ �ڷ�, top left+bottom right speed down 
			up_speed--;// ���� �����̱� ������ ����
			time0_delay(TM_100mS); // 			
		break;	

		case 0x44:// B��ư ������ ���̽�ƽ ���������� , top right+bottom left speed up
			down_speed++;// ���� �����̱� ������ ����
			time0_delay(TM_100mS); // 
		break;	

		case 0x84:// B��ư ������ ���̽�ƽ �������� , top right+bottom left speed down
			down_speed--;// ���� �����̱� ������ ����
			time0_delay(TM_100mS); // 			
		break;	

		case 0x11:// C��ư ������ ���̽�ƽ ������ , ��������(ȣ����)
			motor_stop();// �������� ����
			hovering(up_speed, down_speed);// ȣ���� ����
			time0_delay(TM_100mS); // 			
		break;	
		
	}
	
} 


int main(void){
    setup();
	
	// IP System
	UART_Open(UART_400MHz, 9600); // RF 400Mhz
	UART_Open(UART1, 9600); // s1 made
	
	ring_ini0();
	ring_ini1();
	
	// Software
	runflag = 0; // flag initial
	
	switchcom_400 = 0;// switch ini
	
	// Hardware

	LEDGF(0);// OFF
	LEDRF(0);// OFF
	
	// 400MHz
	rf433mhz_ini(DEVICE_ADDRESS);
	
	// Sound
	beep_sound_timer(TM_500mS);// auto off
	speed = 38;
	up_speed = 38;
	down_speed = 38;//�������� ���� ���·� �ʱ�ȭ
	motor_initialization();// ���� �������·� �ʱ�ȭ
	
	while(2){	
		// Joy Stick ��� ó��
		Processor_pop0_400(); // return F_ACT_400
		
	if(isflag(F_ACT_400)){ // good Receive RF, read uccommand_pccode
		undership_action();  
		clrflag(F_ACT_400);	
		}
		
		if(IN6 == 0){ // push ON
			
			LEDGF(1);// ON
			LEDRF(1);// ON					
			time0_delay(TM_1000mS); // delay time		
		}
		else{
			LEDGF(0);// 
			LEDRF(0);// 			
		}
		
		ucpass = dip_read4();
	}
}

void setup(void){
	SYS_UnlockReg();
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);
	CLK_WaitClockReady( CLK_CLKSTATUS_IRC22M_STB_Msk);
	CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC,CLK_CLKDIV_HCLK(1));// 
	
	// Enable IP clock
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_EnableModuleClock(TMR1_MODULE); 
	CLK_EnableModuleClock(UART0_MODULE);
	CLK_EnableModuleClock(UART1_MODULE);
	CLK_EnableModuleClock(PWM01_MODULE);//pwm ����
	CLK_EnableModuleClock(PWM23_MODULE);
	CLK_EnableModuleClock(PWM45_MODULE);
	CLK_EnableModuleClock(PWM67_MODULE);
	
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HIRC, 0);// 
	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HIRC, 0);//
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));
	CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));
	CLK_SetModuleClock(PWM01_MODULE, CLK_CLKSEL1_PWM01_S_HCLK, 0);//pwm set module Clock ����-CLK.h�� ����
	CLK_SetModuleClock(PWM23_MODULE, CLK_CLKSEL1_PWM23_S_HCLK, 0);
	CLK_SetModuleClock(PWM45_MODULE, CLK_CLKSEL2_PWM45_S_HCLK, 0);
	CLK_SetModuleClock(PWM67_MODULE, CLK_CLKSEL2_PWM67_S_HCLK, 0);
	
	SystemCoreClockUpdate(); 
    SYS_LockReg();

    SYS->ALT_MFP = 0x00000000;
    SYS->ALT_MFP1 = 0x00000000;
    SYS->ALT_MFP2 = 0x00000000;
	
	// pwm sys ����
	SYS->GPA_MFP = SYS_GPA_MFP_PA12_PWM0 | SYS_GPA_MFP_PA13_PWM1 | SYS_GPA_MFP_PA14_PWM2 | SYS_GPA_MFP_PA15_PWM3;
	
	// Port B
	SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD | SYS_GPB_MFP_PB4_UART1_RXD | SYS_GPB_MFP_PB5_UART1_TXD;
	
	// Port C
	SYS->GPC_MFP = 0;
	
	// Port D
	SYS->GPD_MFP = 0; 
	
	// LED Red, Green
	GPIO_SetMode(PB, BIT12 | BIT8, GPIO_PMD_OUTPUT);//low->led on, 12=red led, 8= green led
		
	// Buzzer
	GPIO_SetMode(PC, BIT6, GPIO_PMD_OUTPUT);
	
	// Button, IN6
	GPIO_SetMode(PB, BIT3, GPIO_PMD_QUASI);
	
	// DIP
	GPIO_SetMode(PA, BIT7 | BIT6 | BIT5, GPIO_PMD_QUASI);
	GPIO_SetMode(PC, BIT7, GPIO_PMD_QUASI);
	
	// MOT2, 3
	GPIO_SetMode(PA, BIT3 | BIT2 | BIT1 | BIT0, GPIO_PMD_OUTPUT);
	
	// MOT6, 7
	GPIO_SetMode(PA, BIT11 | BIT10 | BIT9 | BIT8, GPIO_PMD_OUTPUT);
	
	// M8 OUT define
	GPIO_SetMode(PB, BIT15 | BIT2, GPIO_PMD_OUTPUT);
	
	// MOT P4
	//GPIO_SetMode(PA, BIT12, GPIO_PMD_OUTPUT);
	//GPIO_SetMode(PC, BIT0, GPIO_PMD_OUTPUT);
	
	// MOT P3
	//GPIO_SetMode(PA, BIT13, GPIO_PMD_OUTPUT);
	//GPIO_SetMode(PC, BIT1, GPIO_PMD_OUTPUT);
	
	// Timer
	TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 100); // 10mS
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);//TMR0_IRQHandler
	TIMER_Start(TIMER0);
	
	TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1); //  1 Sec
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);//TMR1_IRQHandler
	TIMER_Start(TIMER1);
	
	// UART
	UART_EnableInt(UART0,(UART_IER_RDA_IEN_Msk | UART_IER_TOUT_IEN_Msk));
	NVIC_EnableIRQ(UART02_IRQn);
	
	UART_ENABLE_INT(UART1,(UART_IER_RDA_IEN_Msk | UART_IER_TOUT_IEN_Msk));
	NVIC_EnableIRQ(UART1_IRQn);	
	
	// pwm	
	ulpass = 2000; // 2000HZ
	
	//pwm(0, 1, 2, 3) ��� ����-54 3210
	PWM_EnableOutput(PWMA, 0x01); //pwm0
    PWM_Start(PWMA, 0x01);
	PWM_EnableOutput(PWMA, 0x02); //pwm1
    PWM_Start(PWMA, 0x02);
	PWM_EnableOutput(PWMA, 0x04); //pwm2
    PWM_Start(PWMA, 0x04);
	PWM_EnableOutput(PWMA, 0x08); //pwm3
    PWM_Start(PWMA, 0x08);
	// calibration�� BLDC���Ͱ� ESC�� ����ϱ� ������ �ִ�, �ּ� duty�� ó���� ������ �ִ� �κ��̴�.
	//pwm0 calibration
	PWM_ConfigOutputChannel(PWMA, 0, ulpass, 60);
	CMR0_speed =  PWMA->CMR0; //NUC100Series.h 6270��° ��
	PWM_SET_CMR(PWMA, 0, CMR0_speed);
	PWM_ConfigOutputChannel(PWMA, 0, ulpass, 20);
	CMR0_speed =  PWMA->CMR0;
	PWM_SET_CMR(PWMA, 0, CMR0_speed);
	
	//pwm1 calibration
	PWM_ConfigOutputChannel(PWMA, 1, ulpass, 60);
	CMR1_speed =  PWMA->CMR1; //NUC100Series.h 6270��° ��
	PWM_SET_CMR(PWMA, 1, CMR1_speed);
	PWM_ConfigOutputChannel(PWMA, 1, ulpass, 20);
	CMR1_speed =  PWMA->CMR3;
	PWM_SET_CMR(PWMA, 1, CMR1_speed);
	
	//pwm2 calibration
	PWM_ConfigOutputChannel(PWMA, 2, ulpass, 60);
	CMR2_speed =  PWMA->CMR2; //NUC100Series.h 6270��° ��
	PWM_SET_CMR(PWMA, 2, CMR2_speed);
	PWM_ConfigOutputChannel(PWMA, 2, ulpass, 20);
	CMR2_speed =  PWMA->CMR2;
	PWM_SET_CMR(PWMA, 2, CMR2_speed);
	
	//pwm3 calibration
	PWM_ConfigOutputChannel(PWMA, 3, ulpass, 60);
	CMR3_speed =  PWMA->CMR3; //NUC100Series.h 6270��° ��
	PWM_SET_CMR(PWMA, 3, CMR3_speed);
	PWM_ConfigOutputChannel(PWMA, 3, ulpass, 20);
	CMR3_speed =  PWMA->CMR3;
	PWM_SET_CMR(PWMA, 3, CMR3_speed);
	
}

void Processor_pop0_400(void){ // return F_ACT_400
	// ���̽�ƽ ����� ���� �ۼ��� �ڵ尡 �ƴϹǷ� ���� �� ����
}
void Processor_pop1(void){ // return
	// ���̽�ƽ ����� ���� �ۼ��� �ڵ尡 �ƴϹǷ� ���� �� ����
}


// timer
void TMR0_IRQHandler(void) {// 10ms
	TIMER_ClearIntFlag(TIMER0);
	timer0tick++;		
	
	if(isflag(F_BEEP_ON)){
		time_limit_sound--;
		if(time_limit_sound ==0){
			BUZF(0); // Off
			clrflag(F_BEEP_ON);
		}
	}
	
}

void TMR1_IRQHandler(void) {//  1sec
	TIMER_ClearIntFlag(TIMER1);
	timer1tick++;	
	
	if(isflag(F_COMTIME_400)){
		timercom_400++;
		if(timercom_400 > 3){//3 Sec �̻� �Ǹ� ��� ���̺� ����
			switchcom_400 = 0; // Home
			clrflag(F_COMTIME_400);// cancle
		}
	}		
		
}

void time0_delay(unsigned long T0) { 
	timer0tick =0;
	while(timer0tick < T0);
}

void time1_delay(unsigned long T1) {  
	timer1tick =0;
	while(timer1tick < T1);
}

// UART
// CommunicationUtility
unsigned char check_pop0(void){
	// ���̽�ƽ ����� ���� �ۼ��� �ڵ尡 �ƴϹǷ� ������ �� ����
} //end check_pop

unsigned char check_pop1(void){
	// ���̽�ƽ ����� ���� �ۼ��� �ڵ尡 �ƴϹǷ� ������ �� ����
} //end check_pop

void ring_ini0(void){
	// ���̽�ƽ ����� ���� �ۼ��� �ڵ尡 �ƴϹǷ� ������ �� ����
}	
void ring_ini1(void){
	// ���̽�ƽ ����� ���� �ۼ��� �ڵ尡 �ƴϹǷ� ������ �� ����
}	

// uart0
//UART02_IRQn
void UART02_IRQHandler(uint32_t param){ // UART0_IRQHandler
	// ���̽�ƽ ����� ���� �ۼ��� �ڵ尡 �ƴϹǷ� ������ �� ����
} 

// uart1
void UART1_IRQHandler(uint32_t param){ // UART1_IRQHandler
	// ���̽�ƽ ����� ���� �ۼ��� �ڵ尡 �ƴϹǷ� ������ �� ����
} 

void putstring0(unsigned char *pui8Buffer){     // 
	// ���̽�ƽ ����� ���� �ۼ��� �ڵ尡 �ƴϹǷ� ������ �� ����
}	

void putstring1(unsigned char *pui8Buffer){     // 
	// ���̽�ƽ ����� ���� �ۼ��� �ڵ尡 �ƴϹǷ� ������ �� ����
}	

// Utility
//���� ����
//����ü ������ pwm0, ����ü �������� pwm1, ����ϰ� ���� ������+�Ʒ��������� pwm2, ����ϰ� ���� ��������+�Ʒ������� pwm3
void back_left_motor_control(unsigned long back_left_motor_speed){
	PWM_ConfigOutputChannel(PWMA, 0, ulpass, back_left_motor_speed);
	CMR0_speed =  PWMA->CMR0;
	PWM_SET_CMR(PWMA, 0,CMR0_speed);
}

void back_right_motor_control(unsigned long back_right_motor){
	PWM_ConfigOutputChannel(PWMA, 1, ulpass, back_right_motor);
	CMR1_speed =  PWMA->CMR1;
	PWM_SET_CMR(PWMA, 1,CMR1_speed);
}

void top_left_AND_bottom_right(unsigned long up_down_speed){
	PWM_ConfigOutputChannel(PWMA, 2, ulpass, up_down_speed);
	CMR2_speed =  PWMA->CMR2;
	PWM_SET_CMR(PWMA, 2,CMR2_speed);
}

void top_right_AND_bottom_left(unsigned long up_down_speed){
	PWM_ConfigOutputChannel(PWMA, 3, ulpass, up_down_speed);
	CMR3_speed =  PWMA->CMR3;
	PWM_SET_CMR(PWMA, 3,CMR3_speed);
}

//����ü ���� ����
void motor_stop(void){
	back_left_motor_control(38);
	back_right_motor_control(38);
}

// ���� �ʱ�ȭ
void motor_initialization(void){
	back_left_motor_control(38);
	back_right_motor_control(38);
	top_left_AND_bottom_right(38);
	top_right_AND_bottom_left(38);
}

// ���� ��� ����: ����, ����, ��ȸ��, ��ȸ��, (���, �ϰ�,) �������� ����ϰ� �����
void go_straight(unsigned long back_motor_speed){ //����
	back_left_motor_control(back_motor_speed);
	back_right_motor_control(back_motor_speed);
}

void go_back(unsigned long back_motor_speed){ //����
	back_left_motor_control(back_motor_speed);
	back_right_motor_control(back_motor_speed);
}

void turn_left(unsigned long back_left_motor_speed, unsigned long back_right_motor_speed){// ��ȸ��
	back_left_motor_control(back_left_motor_speed);
	back_right_motor_control(back_right_motor_speed);
}

void turn_right(unsigned long back_left_motor_speed, unsigned long back_right_motor_speed){// ��ȸ��
	back_left_motor_control(back_left_motor_speed);
	back_right_motor_control(back_right_motor_speed);
}

void go_down(unsigned long down_speed){//�ϰ�-> �ʿ� ���� ����
	top_left_AND_bottom_right(down_speed);
	top_right_AND_bottom_left(down_speed);
}

void go_up(unsigned long up_speed){//���-> �ʿ� ���� ����
	top_left_AND_bottom_right(up_speed);
	top_right_AND_bottom_left(up_speed);
}

void hovering(unsigned long up_speed, unsigned long down_speed){
	top_left_AND_bottom_right(up_speed);
	top_right_AND_bottom_left(down_speed);
}


void beep_sound_timer(unsigned long time_sound){
	BUZF(1);
	time_limit_sound = time_sound;
	setflag(F_BEEP_ON);
}

void beep_sound(void){
	BUZF(1);
	time0_delay(TM_200mS);	
	BUZF(0);	
}

unsigned char hex2ascii(unsigned char tohex){
    if (tohex<0x0A)    tohex += 0x30; //'0'
    else        tohex += 0x37; 
    return (tohex);
}

unsigned long isflag(unsigned long flag){
	return (runflag & flag);
}

void setflag(unsigned long flag){
	 runflag |= flag;
}

void clrflag(unsigned long flag){
	runflag &= ~flag;
}

void togflag(unsigned long flag){
	runflag ^= flag;
}



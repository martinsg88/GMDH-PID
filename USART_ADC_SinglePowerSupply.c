////////////////////////////////////////////////
// USART with ADC test (Single power supply) //
///////////////////////////////////////////////
/*Include headerfiles*/
#include <p18f2550.h>
#include <stdio.h>
#include <stdlib.h>
#include <delays.h>
#include <usart.h>
#include <sw_uart.h>
#include <timers.h>
#include <adc.h>
#include <pwm.h>

#include <math.h>

/*Setting Configuration bits*/
//[Help]->[Topics]->[PIC18 Config Settings]
//--- CLOCK Settings ---
#pragma config PLLDIV = 5, CPUDIV = OSC1_PLL2, USBDIV = 2, FOSC = HSPLL_HS, VREGEN=OFF
//--- CCP Setting ---
#pragma config CCP2MX=OFF
//--- PORT Setting ---
#pragma config PBADEN=OFF
//--- Another Settings ---
#pragma config MCLRE=ON, PWRT=ON, WDT=OFF, STVREN=OFF, LVP=OFF  

/*Prototype declaration*/
int initUI(int);
void Output(void);
float ControllerOutput(void);
float SystemOutput(void);
void val_print(float);
float N_Adaline(float x1, float x2, float* w);

/*Defines*/
#define BAUD 9600 //baud rate for uart
#define CLOCK 48000000		//クロック周波数[Hz]
#define AD_RefVoltage 5.0	//A/D変換器の基準電圧
#define MULTI 100;

/*Grobal variable numbers*/
unsigned char i;
unsigned int j;
int endstep = 400;
//A/D Convertor
unsigned int voltage;
//Control
float Ts = 1.0;					//Sampling time[s]
unsigned long Ts_counter = 0;	//counter for sampling
int t = 0;
int	ADresult;		//[ADC]signed 16bit integer(0~1024)
int endstep;
unsigned char ControlMode = 0;
unsigned char ConfMode = 0;
float StepValue;
unsigned char r_index = 0;
float r[4]={1, 1.5, 2, 0.5};
float y[3]= {0.0, 0.0 , 0.0};
float u[3]={0.0, 0.0 , 0.0}; 
float e[3]={0.0, 0.0 , 0.0};
float x[3]={0.0, 0.0 , 0.0};
float Delta_e;
//GMDH-PID
float x1, x2;
char layer1_SelectInputs_Kp, layer1_SelectInputs_Ki, layer1_SelectInputs_Kd;

#pragma idata large_idata
float w_Kp1[6] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
float w_Ki1[6] = {0.7, 0.8, 0.9, 0.1, 0.2, 0.3}; 
float w_Kd1[6] = {0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
float w_Kp2[6] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6}; 
float w_Ki2[6] = {0.7, 0.8, 0.9, 0.1, 0.2, 0.3}; 
float w_Kd2[6] = {0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
float w_Kp3[6] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
float w_Ki3[6] = {0.7, 0.8, 0.9, 0.1, 0.2, 0.3};
float w_Kd3[6] = {0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
#pragma idata

//PID Gains
float Kp = 0.0494, Ki = 0.0542, Kd = 0.0255;


long left = 0;
unsigned long right = 0; 

#pragma interrupt controller

#pragma code isrcode = 0x08

void isr_direct(void){_asm GOTO controller _endasm}

void controller(void){
	INTCONbits.TMR0IF = 0;		
	printf("TIMER TIMER TIMER\n\r");
	WriteTimer0(Ts_counter);
	SystemOutput();
	ControllerOutput();
	//Update variables
	for(j=2; j>0; j--){
			y[j] = y[j-1];
			u[j] = u[j-1];
			e[j] = e[j-1];
		}
		Output();
		t++;
	if(t == 401){
		INTCONbits.TMR0IF = 0;
		INTCONbits.TMR0IE = 0;
	}
}

//===================== Main function =====================
void main(void){
	//Variable declaration
	unsigned int timex;
	float check;
	int i = 0,j = 0;
	int input = 0;
	//---------- Initial Settings of SFR ----------
	/*CLOCK settings*/
	OSCCON = 0x00;
	/*IO settings*/
	PORTA = 0x00;
	PORTB = 0x00;
	TRISB = 0x00;
	//+++++ A/D Convertor Settings +++++
	TRISA = 0xFF;	//TRISA is all input mode
	RCONbits.IPEN = 0;
	INTCONbits.GIE = 1;	
	OpenTimer0(TIMER_INT_OFF & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);
	//A/D open(Tad=Fosc/64, right-align, 16Tad, CH0_selected, Interapt_OFF, Ref = Vdd & Vss, AN0~AN5:Analog input)
	OpenADC(ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_16_TAD,
			ADC_CH0 & ADC_INT_OFF & ADC_REF_VDD_VSS, 0x09);
	
	/*USART settings*/
	timex = 77;
  	OpenUSART (USART_TX_INT_OFF &
              	USART_RX_INT_ON &
              	USART_ASYNCH_MODE &
              	USART_EIGHT_BIT &
              	USART_CONT_RX &
             	USART_BRGH_LOW, timex); //For baud rate 9600
	TRISCbits.RC7 = 1;
	
	ControlMode = initUI(input);
	ControlMode = ControlMode - 48;
	printf("%d\n\r",ControlMode);
	
	//---------- Simulation ----------
	//Delay10KTCYx(256);
	printf(" %%t, r,  y, u, e, Delta_e, Kp, Ki, Kd \n\r");
	INTCONbits.GIE = 1;		
	INTCONbits.TMR0IE = 1;		
	while(INTCONbits.TMR0IE == 1);
	CloseUSART();
	while(1);


}

//-------------- System Output --------------------
float SystemOutput(){
		int	ADresult;		//[ADC]signed 16bit integer(0~1024)
		//+++++ A/D Convert +++++
		SetChanADC(ADC_CH0);	//Select chanel
		ConvertADC();			//Start ADC
		while(BusyADC());		//Wate finished ADC
		ADresult = ReadADC();
		y[0] = (ADresult*255.0)/1024.0;
}

//-------------- Controller --------------------
float ControllerOutput(void)
{
	//+++++ Change SetValues +++++
	if(t%100 == 0 && t!=0)
		r_index++;

	//+++++ Caluculate error +++++
	e[0] = r[r_index] - y[0];
	Delta_e = e[0] - e[1];

	//+++++ Controller selects +++++
	switch(ControlMode)
	{
		case 0:
			printf("Hello0\n\n\r\r");
			//----- Step output ----- 
			u[0] = StepValue;
			break;
		case 1:
			printf("Hello1\n\n\r\r");
			//----- Fixed PID controller output ----- 
			u[0] = u[1] + Kp*(e[0]-e[1]) + Ki*e[0] + Kd*(e[0]-2.0*e[1]+e[2]);
			break;
		case 2:
			printf("Hello2\n\n\r\r");
			//----- GMDH-PID controller output -----
			//*** Kp ***
			//layer1
			switch(layer1_SelectInputs_Kp)
			{
				case 0:
					x1 = N_Adaline(r[r_index], r[r_index], w_Kp1);
					x2 = N_Adaline(r[r_index], e[0], w_Kp2);
					break;
				case 1:
					x1 = N_Adaline(r[r_index], r[r_index], w_Kp1);
					x2 = N_Adaline(r[r_index], Delta_e, w_Kp2);
					break;
				case 2:
					x1 = N_Adaline(r[r_index], e[0], w_Kp1);
					x2 = N_Adaline(r[r_index], Delta_e, w_Kp2);
					break;
			}
			//layer2
			Kp = N_Adaline(x1, x2, w_Kp3);
	
			//*** Ki ***
			//layer1
			switch(layer1_SelectInputs_Ki)
			{
				case 0:
					x1 = N_Adaline(r[r_index], r[r_index], w_Ki1);
					x2 = N_Adaline(r[r_index], e[0], w_Ki2);
					break;
				case 1:
					x1 = N_Adaline(r[r_index], r[r_index], w_Ki1);
					x2 = N_Adaline(r[r_index], Delta_e, w_Ki2);
					break;
				case 2:
					x1 = N_Adaline(r[r_index], e[0], w_Ki1);
					x2 = N_Adaline(r[r_index], Delta_e, w_Ki2);
					break;
			}
			//layer2
			Ki = N_Adaline(x1, x2, w_Ki3);
	
			//*** Kd ***
			//layer1
			switch(layer1_SelectInputs_Kd)
			{
				case 0:
					x1 = N_Adaline(r[r_index], r[r_index], w_Kd1);
					x2 = N_Adaline(r[r_index], e[0], w_Kd2);
					break;
				case 1:
					x1 = N_Adaline(r[r_index], r[r_index], w_Kd1);
					x2 = N_Adaline(r[r_index], Delta_e, w_Kd2);
					break;
				case 2:
					x1 = N_Adaline(r[r_index], e[0], w_Kd1);
					x2 = N_Adaline(r[r_index], Delta_e, w_Kd2);
					break;
			}
			//layer2
			Kd = N_Adaline(x1, x2, w_Kd3);
			//----- PID controller output ----- 
			u[0] = u[1] + Kp*(e[0]-e[1]) + Ki*e[0] + Kd*(e[0]-2.0*e[1]+e[2]);
			break;
	}
}


//-------------- Calculation of N-Adaline's output --------------------
float N_Adaline(float x1, float x2, float* w)
{
	//Variable declaration
	float z;
	float sq_x1, sq_x2, x1x2;	
	
	//Calculate z
	sq_x1 = pow(x1, 2.0);
	x1x2 = x1*x2;
	sq_x2 = pow(x2, 2.0);
	z = w[0]*x1 + w[1]*sq_x1 + w[2]*x1x2 + w[3]*sq_x2 + w[4]*x2 + w[5];
	
	return z;
}

//-------------- USART print --------------------
void val_print(float alpha){
	float num;
	num = fabs(alpha);
	left = (long)((float)num);
	right =(long)((float)num*100)-left*100;
	if(alpha < 0){
		printf((far char*)"-%lu.%d%lu",left,0,right);
	}
	else{
		if(!(right < 0)){
			if(right < 10){
				printf((far char*)"%lu.%d%lu",left,0,right);
			}
			else{
				printf((far char*)"%lu.%lu",left,right);
			}
		}
		else{
			printf("here!");
			right = right*-1;
			if(right < 10){
				printf((far char*)"%lu.%d%lu",left,0,right);
			}
			else{
				printf((far char*)"%lu.%lu",left,right);
			}
		}
	}
}

int initUI(int input){
	int sent = 0;
	sent = input;
	printf("Please Select 0 - 2\n\r");
	printf("0 for Step Output\n\r");
	printf("1 for Fixed PID controller output\n\r");
	printf("2 for GMDH-PID controller output\n\r");
	INTCONbits.TMR0IF = 0;
	while(BusyUSART());
	while(!DataRdyUSART());
	input = ReadUSART();
	for(i = 0; i < 10; i++){Nop();}
	sent = input;
	Delay10KTCYx(256);
	printf("Your Selection: %d\n\r\n\r", sent);
	INTCONbits.TMR0IF = 1;
	return(sent);
}
void Output(void){

		e[0] = 0;
		Delta_e = 0;
		u[0] = 0;
		//sending data
		printf("%d, ",t);
		val_print(r[r_index]);
		printf(", ");
		val_print(y[0]);
		printf(", ");
		val_print(u[0]);
		printf(", ");
		val_print(e[0]);
		printf(", ");
		val_print(Delta_e);
		printf(", ");
		val_print(Kp);
		printf(", ");
		val_print(Ki);
		printf(", ");
		val_print(Kd);
		printf("\n\r");
}	
//Lab 6

/* Sample code for main function to read the compass and ranger */ 
#include <c8051_SDCC.h>
#include <stdlib.h>// needed for abs function 
#include <stdio.h>
#include <i2c.h>
#define Motor_Neut 2765	
#define Motor_Max 3502	
#define Motor_Min 2028	

//************************
//Initialization Functions 
void Port_Init(void); 
void PCA_Init(void); 
void ADC_Init(void); 
void PCA_ISR(void) __interrupt 9; 
void XBR0_Init(void);
void SMB0_Init(void);
unsigned char Read_AD_Input (unsigned char n);

//Compass and Servo
void Pick_Heading_higher(void);
void Pick_Heading_lower(void);
unsigned int Read_Compass(void);

//Ranger Reading
unsigned int ReadRanger(void);

//angle stuff
void Set_Angle(void);

//Moves and calculates the truning
void Set_distance_PW(unsigned int range);
void steering(unsigned int new_desired_heading);

//Timer/Flags

unsigned int heading_count=1;
int Flag=0;
unsigned int counts=0;
int heading_flag=1;
int state_flag=1;
unsigned int h_counts=0;

//ranger and range
unsigned int range=0;

//heading
__xdata unsigned int new_desired_heading=0;
unsigned int actual_heading=0;
unsigned int desired_heading_higher=0;
unsigned int desired_heading_lower=0;

//error
signed int error=0;
signed int prev_error=0;

//GAINS
float der_gain=0;
float prop_gain=0;

//ALGORITHM
signed long Algorithm_value=0;

//Battery
unsigned int volt=0;


//Main Function
void main(void) 
{ 
	
	Sys_Init(); // initialize board 
	putchar('  '); 
	Port_Init(); 
	PCA_Init(); 
	XBR0_Init();
	SMB0_Init();
	ADC_Init();

	lcd_clear();
	PCA0CP3=0xFFFF-Motor_Neut;
	PCA0CP2=0xFFFF-Motor_Neut;
	PCA0CP0=0xFFFF-Motor_Neut;
	//Set gain values and haeding
	printf("\n Set Initial heading with keypad and range with potentiometer \r\n");
	Pick_Heading_higher();

	printf("\n Set Initial heading with keypad and range with potentiometer \r\n");
	Pick_Heading_lower();

	

	
	//Choose Proportional Gain
	lcd_clear();
	lcd_print("\nEnter proportional Gain(0-100):\r\n");
	prop_gain= kpd_input(1)/10;

	//Choose Derivative Gain
	lcd_clear();
	lcd_print("\nEnter derivative gain(ex. 0-1000):\r\n");
	der_gain = kpd_input(1)/10;

	
	lcd_clear();

	Set_Angle();

	printf_fast_f("\n A_Head,	D_Head,	Voltage,	error,	Kp,	Kd,	range,	Algorithm_value\r\n");
		
		
	//range= ReadRanger();
	//Set_distance_PW(range);
	while(1){
		
		if(heading_flag){
			if(state_flag){
				new_desired_heading=desired_heading_higher;
				state_flag=0;
				heading_flag=0;
			}else{
				new_desired_heading=desired_heading_lower;
				state_flag=1;
				heading_flag=0;
			}
				
		}
		
		steering(new_desired_heading);

		
		
	}

}
//----------------------------------------------------------------------------
// SET INITIAL DESIRED HEADING WITHOUT RANGER 
//----------------------------------------------------------------------------

//Set Desired Heading
void Pick_Heading_higher(void)
{
	while (counts < 1);
	lcd_clear();
	lcd_print("Enter 1st heading:\n(180-360)\n");
	desired_heading_higher = kpd_input(1) * 10;
	//printf( "\rDesired heading = %u  \n", heading_aim);
}


void Pick_Heading_lower(void)
{
	while (counts < 1);
	lcd_clear();
	lcd_print("Enter 1st heading:\n(0-180)\n");
	desired_heading_lower = kpd_input(1) * 10;
	//printf( "\rDesired heading = %u  \n", heading_aim);
}
//----------------------------------------------------------------------------
// READ COMPASS
//----------------------------------------------------------------------------

//********************************
//Read Compass
unsigned int Read_Compass(void)
{
	unsigned char addr_C = 0xC0; // the address of the sensor, 0xC0 for the compass
	unsigned char Data[2]; // Data is an array with a length of 2
	i2c_read_data(addr_C, 2, Data, 2); // read two byte, starting at reg 2
	actual_heading =(((unsigned int)Data[0] << 8) | Data[1]);
	return actual_heading; // the heading returned in degrees between 0 and 3599
}



//----------------------------------------------------------------------------
// READ RANGER
//----------------------------------------------------------------------------

//************************************
//Gets range
unsigned int ReadRanger(void)
	{
		unsigned char Data[2];
		unsigned char addr=0xE0; // the address of the ranger is 0xE0

		i2c_read_data(addr, 2, Data, 2); // read two bytes, starting at reg 2
		range = (((unsigned int)Data[0] << 8) | Data[1]);
		Data[0] = 0x51; // write 0x51 to reg 0 of the ranger:
		i2c_write_data(addr, 0, Data, 1); // write one byte of data to reg 0 at addr

		counts = 0;
    	while(counts < 5);

		return range;
	}




//---------------------------------------------------------------------------------------------------
//ININITIALIZATIONS FOR PORTS, XBAR, SMBus
//---------------------------------------------------------------------------------------------------

//********************************
//Port Ininitialization
void Port_Init()
{
	P0MDOUT &= ~0xF0; 		//Set inputs to open drain, P0.4, P0.5, P0.6 and P0.7,
	P0 |= 0xF0; 			//Set inputs to high impedance

}
//CrossBar Initialization
void XBR0_Init()
{
	XBR0 = 0x25; // cross bar at 0x25
}
	
//********************************
//SMBus Initialization
void SMB0_Init(void)
{
	SMB0CR=0x93;		//set SCL to 100KHz (actual freq ~ 94,594Hz)
	ENSMB=1;			//bit 6 of SMB0CN, enable the SMBus
}

//-------------------------------------------------------------------------------------------------------------------------------------
//ADC
//-------------------------------------------------------------------------------------------------------------------------------------

//********************************
//Analog / Digital Conversion Initialization
void ADC_Init(void)
{
	REF0CN = 0x03;		//Set vref = to internal reference voltage (2.4v)
	ADC1CN = 0x80;		//Enables ADC converter
	ADC1CF |= 0x01;		//Gain is equal to one
}

//********************************
//Read Analog / Digital Conversion Pin
unsigned char Read_AD_Input (unsigned char n)
{
	AMX1SL = n; 							//Set P1.n as the analog input for ADC1
	ADC1CN &= ~0x20; 						//Clear the Conversion Completed flag
	ADC1CN |= 0x10;							//Initiate A/D conversion
	while ((ADC1CN & 0x20) == 0x00); 		//Wait for conversion to complete
	return ADC1; 							//Return digital value in ADC1 register
}

//-----------------------------------------------------------------------------------------------------------
//PCA
//-----------------------------------------------------------------------------------------------------------

//********************************
//Programmable Counter Array Initialization
void PCA_Init(void)
{
	PCA0MD = 0x81;					//SYS/12, CF interrupts enabled
	PCA0CPM0 = 0xC2; 		// 16 bit, enable CCM0
	PCA0CPM1 = 0xC2;		//16 bit, enable CCM1
	PCA0CPM2 = 0xC2;		// 16 bit, enable CCM2
	PCA0CPM3= 0xC2;			//16 bit, enable CCM3
	PCA0CN |= 0x40;					//Enable PCA counter
	EIE1 |= 0x08;					//PCA interupt
	EA = 1;							//Global variables
	IE |= 0x02;					//Includes Interrupt Initializations
}
	
//********************************
//Interrupt Service Routine for Programmable Counter Array Overflow Interrupt 
void PCA_ISR(void) __interrupt 9 
{ 
	
	if (CF) 
	{ 	
		CF = 0; 				//Clear overflow indicator 
		PCA0 = 28672; 			//20 ms 
		h_counts++;				//Heading Counter
		counts++;
		heading_count++;
		if (h_counts>= 3)        // every 3rd count
		{
			Flag = 1;
			h_counts = 0;
`	  	}	
		if (heading_count>= 300){ // 6s instead of 3 because it takes too long to settle 
			heading_flag=1;
			heading_count=0;
		}
	

	} 
	PCA0CN &= 0x40; 
}

//------------------------------------------------------------------------------------------
// FUNCTIONS THAT CONTROL AND CALCULATE STEERING OF GONDOLA 
//------------------------------------------------------------------------------------------

//***************************************************
//STEERS THE SIDE FANS IN THE CORRECT DIRECTIONS
void steering(unsigned int new_desired_heading){	
	if(Flag){
		actual_heading=Read_Compass();
		error=new_desired_heading-actual_heading; 		//calculates the error
		error=error/10; 								// divides by ten to put it in decimal form 
	
		volt = Read_AD_Input (3); //battery voltage in mV
		printf_fast_f("\n\r	%d,	%d,	%d,	%d,	%4.2f,	%4.2f,	%d", actual_heading, new_desired_heading, volt, error, prop_gain, der_gain, range);
		
		
		//Check if error is in range of (180 to -180)
		if(error >180){				//if above 180
			error-=180;
			error=-180+error;
		}
		if (error <-180){			// if less than -180
			error +=180;
			error=180+error; 	
		}
		//printf("\n error: %d \r\n",error);
		Algorithm_value= (signed long) Motor_Neut + (signed long)prop_gain*(signed long)(error) + (signed long)der_gain*(signed long)(error - prev_error);
		//printf("\n Algorithm_value: %ld \r\n", Algorithm_value);
		//equation to calculate the algorithm value it is put in unsigned long just incase the values for the error are too large for 
		// int value

		if (Algorithm_value>Motor_Max){		//safety check to make sure pw doesn't go above max
			Algorithm_value=Motor_Max;
		}
		else if (Algorithm_value<Motor_Min){	//safety check to make sure pw doesn't go under min
			Algorithm_value=Motor_Min;
		}
		printf("\n\r ,	%ld", Algorithm_value);

		prev_error=error;

		PCA0CP0=0xFFFF-Algorithm_value;

		Flag=0;			//resets flag
	}

}

//*************************************************
//CALCULATES THE DISTANCE/ RANGE OF THE RANGER AND CONVERTS THE DESIRED HEADING INTO A NEW DESIRED HEADING 
void Set_distance_PW(unsigned int range){
	if (range>100){		//checks if range is above 100 or not if above doesn't matter
		range=100;
	}
	
	if (range>0 && range<=100){
		desired_heading_higher=4*range+(desired_heading_higher-180);	//increases desired heading linearly with the range 
		desired_heading_lower=4*range+(desired_heading_lower-180);
		//makes sure the new desired heading is in the range 
		if(desired_heading_higher< 1){
			desired_heading_higher+=3600;
		}
		if(desired_heading_lower< 1){
			desired_heading_lower+=3600;
		}
		if(desired_heading_higher> 3599){
			desired_heading_higher= desired_heading_higher % 3600;
		}
		if(desired_heading_lower> 3599){
			desired_heading_lower= desired_heading_lower % 3600;
		}

	}
}



//-----------------------------------------------------------------------------------------
//Setting the Angle 
//----------------------------------------------------------------------------------------
//Function to use ranger to adjust the thruster fan angle
//holding a fixed range value for ~5 seconds locks in the corresponding angle.
void Set_Angle(void)
{
    char set_angle, count;
    unsigned int adj, previous_adj, angle;

    count = 0;
    previous_adj = 0;
    set_angle = 1;

    while(set_angle)
    {
        adj = ReadRanger();
        angle = 2765 + (adj - 40)*10;               //May change: 40 nominal height
                                                    //            10 gain on rotation
        if (angle < Motor_Min)
			{ angle = Motor_Min;

			}
        else if (angle > Motor_Max) 
		{
		angle = Motor_Max;
		}
        PCA0CP1 = 0xFFFF - angle;                   //CEX1 is thrust fan servo


        if(abs(previous_adj - adj) < 8)
			{ count++;    //Adjust depending on how noisy data is
			}
		else 
		{count = 0;
		}
        if(count > 62) {
			set_angle = 0;
			}               //Assuming ranger reads every 80ms
			previous_adj = adj;                         //
			printf("\r Range = %u   ", adj);
			
    }
}




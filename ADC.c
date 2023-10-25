/********************************************************************
SETTING UP NEW KL05Z UNITS.
STEP 1: LOAD INTO BOOT MODE ON A WINDOWS 7 MACHINE. DO THIS BY HOLDING DOWN
THE RESET BUTTON WHILE PLUGGING THE DEVICE IN.
STEP 2: MOVE THE "MSD-DEBUG-FRDM-KL05Z_Pemicro_v118" FILE FROM THE 
PREMICRO OPENSDA DRIVE INTO THE BOOTSTRAPER.
STEP 3: TO DEPLOY CODE TO THE DEVICE, COPY THE .srec FILE IN THE OBJECT FOLDER 
ONTO THE BOARD WHILE NOT IN BOOT MODE.

COPYING FILES VIA SSH:
- scp ADC_VARIANT_1.srec pi@129.21.124.61:Documents/Programs/UART_IO/TRANSFER_TMP/filename
********************************************************************/
#include "MKL05Z4.h"
#include "ADC.h"
#include <math.h>

#define FALSE      (0)
#define TRUE       (1)

#define MAX_STRING (79)

#define ADC0_SC1_VAL 	(0x0u)
#define ADC0_CFG1_VAL	(0x45u)
#define ADC0_CFG2_VAL (0x10u)
#define ADC0_SC2_VAL	(0x00u)
#define ADC0_SC3_VAL 	(0x8Cu)

#define ADC0_SC3_SINGLE (0x00u)
#define ADC0_SC3_CAL (ADC_SC3_CAL_MASK | ADC_SC3_AVGE_MASK | \
                                           ADC_SC3_AVGS_MASK)
																					 
#define VOLTAGE_PORT 			13
#define VOLTAGE_SIZE			2
#define TITLE_NUMBER 			0x0000000A
#define DIVISION_COUNT		0x00000004
#define TITLE_NUM_SIZE		4
#define DIVISION_CNT_SIZE	4

#define X_POS 						1
#define	Y_POS							3
#define Z_POS							5

#define g1								4096
#define g_const						9.80665

#define PREV_ANGLE_WEIGHT_TOTAL			5
#define PREV_ANGLE_WEIGHT_PREVIOUS	4
#define	PREV_ANGLE_WEIGHT_CURRENT		(PREV_ANGLE_WEIGHT_TOTAL - PREV_ANGLE_WEIGHT_PREVIOUS)

#define ACCELERATOR_WRITE	(0x1D << 1)
#define ACCELERATOR_READ	((0x1D << 1) | 1)

#define acos_to_millidegrees(x) (acosf(aval) * 114594.938724) // 360000.0 / 3.1415 = 114594.938724

// coordinates for the center; we update these, then update our individual corner vectors later.
int 	CENTER[3] = {0,0,0};
// these values store the ARCCOS of the angle. We do this because the cosine used in final
// calculations cancels out the ARCCOS anyway, makes our math easier. The angle is that between the
// plane orthogonal to the axis relative to the Earth to the plane orthogonal to the axis relative
// to the board; so, the Z angle (ANGLE[2]) is the arccos of the angle between the XY plane of the
// KL05Z board and the XY plane of the Earth.
float ANGLE[3] = {0.0,0.0,0.0};
// We store the raw accelerometer value every time we check the angle, for calculation when we're
// handling distance.
short int ANGLE_OFFSET_RAW[3] = {0,0,0};
unsigned int COUNTER = 0;
unsigned int COUNTER_2 = 0;

void Init_and_Cal_ADC0(){
	// Enable ADC0 module clock
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
	// Set ADC0 power and timing
	ADC0->CFG1 = ADC0_CFG1_VAL;
	// Select channel A and set timing
	ADC0->CFG2 = ADC0_CFG2_VAL;
	ADC0->SC1[0] = ADC0_SC1_VAL;
	// Select SW trigger and VDDA reference
	ADC0->SC2 = ADC0_SC2_VAL;
	// Start calibration
	do { 
		ADC0->SC3 = ADC0_SC3_VAL;
		// Wait for calibration to complete
		while (ADC0->SC3 & ADC_SC3_CAL_MASK);
	// Check for calibration failure
	} while (ADC0->SC3 & ADC_SC3_CALF_MASK);
	// Compute and store plus-side calibration value
	ADC0->PG = (((UInt16) (ADC0->CLPS + ADC0->CLP4 + ADC0->CLP3 + ADC0->CLP2 +ADC0->CLP1 + ADC0->CLP0) >> 1) | (Int16) 0x8000);
	// Select single conversion
	ADC0->SC3 = ADC0_SC3_SINGLE;
}

unsigned char read_byte_accelerometer(unsigned char address){
		unsigned char val;
		__asm("CPSID   I"); 
		Start_I2C();
		Write_I2C_Data(ACCELERATOR_WRITE);
		Write_I2C_Data((address));
		Repeat_Start_I2C();
		Write_I2C_Data(ACCELERATOR_READ);
		Read_I2C_Data(1); 									// dummy read; need this here for some reason.
		val = Read_I2C_Data(0);
		Stop_I2C();
		__asm("CPSIE   I");  	
		return val;
}

void write_byte_accelerometer(unsigned char address, unsigned char value){
		__asm("CPSID   I"); 
		Start_I2C();
		Write_I2C_Data(ACCELERATOR_WRITE);
		Write_I2C_Data(address);
		Write_I2C_Data(value);
		Stop_I2C();
		__asm("CPSIE   I");  	
}

signed short int convert_raw_to_position(int x){
	signed short int tmp;
	// the accelerometer reads in the format MSB [7:0], LSB [7:2]. Format accordingly.
	tmp = read_byte_accelerometer(x) << 6 | read_byte_accelerometer(x+1) >> 2;
	if(tmp & 0x2000){		// if we're signed at 14 bits, we need to extend that to the top 2
			tmp |= 0xE000;	// sign extend
	}
	return tmp;
}

void Init_Accelerometer(void){
	write_byte_accelerometer(0x2A,0x01);
}

void Setup_GPIO_Pin(unsigned char pin_num, unsigned char io_type){
	PORTB->PCR[pin_num] = 0x100;
	if(io_type == 0){
		Set_GPIO_Input(pin_num);
	} else {
		Set_GPIO_Output(pin_num);
		Write_GPIO(pin_num, 0);
	}
}

void Init_GPIO(void){
	int iii;
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	for(iii = 5; iii < 8; iii++){
		PORTB->PCR[iii] = 0x100;
		Set_GPIO_Output(iii);
	}
}

void Init_GPIO_LED(void){
	int iii;
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	for(iii = 8; iii < 11; iii++){
		PORTB->PCR[iii] = 0x100;
		Set_GPIO_Output(iii);
	}
}

/********************************************************************
* Bakhshali method of calculating a square root.
* https://en.wikipedia.org/wiki/Methods_of_computing_square_roots
********************************************************************/
float sqrt_berkali(int val){
	int iii;
	float guess = (float) (val >> 1);
	float a, b;
	for(iii = 0; iii < 10; iii++){
		a = (val - (guess * guess)) / (2.0 * guess);
		b = guess + a;
		guess = b - ((a * a) / (2 * b));
	}
	return guess;
}

void Angle_Function(float magnitude, short int vals[3]){
	int iii;
	// in time, I'd like to clear this piece out and use more computationally efficient
	// methods, but this makes it so much easier to understand, so we're doing this first.
	//magnitude /= (9.81 / 4096.0);
	for(iii = 0; iii < 3; iii++){
		ANGLE_OFFSET_RAW[iii] = vals[iii];
		ANGLE[iii] = ANGLE_OFFSET_RAW[iii] / magnitude;
	}
}

// this function is deprecated but is kept here for archiving purposes, and in case I ever
// end up needing to use it again.
void Acceleration_Function(float magnitude, short int vals[3]){
	int iii;
	short int alts[3];
	float	tmp_magnitude_coords[3];
	// if we're moving, we want to firstly find our new rotation, but since we don't have access to our
		// rotation data, only raw accelerometer data, we'll do this in a bit of a funky way.
		
		// When we're moving, we can't tell the difference between motional and angular acceleration. So, we
		// find out what our angular motion is if we weigh total acceleration to be equal to gravity. Since if
		// we use ONLY that data we get info that's completely wrong, we weigh it with the previous angle we
		// have on record. If the math is right, we'll have some slight angular acceleration, but mostly motional.
		for(iii = 0; iii < 3; iii++){
			// This will give us the gravitational multiple of the current value, times its weight
			tmp_magnitude_coords[iii] = ((g1 * vals[iii]) / magnitude) * PREV_ANGLE_WEIGHT_CURRENT;
			// This will give us the angle we had before we started moving, times its weight.
			tmp_magnitude_coords[iii] += ANGLE_OFFSET_RAW[iii] * PREV_ANGLE_WEIGHT_PREVIOUS;
			// This will take the average of the two.
			tmp_magnitude_coords[iii] /= PREV_ANGLE_WEIGHT_TOTAL;
			// we'll use this for the angle function later
			alts[iii] = (short int) tmp_magnitude_coords[iii];
		}
		// let's try and find the new angle based on this info.
		Angle_Function(g1, alts);
		// we've already calculated our angular acceleration, so we remove it from our calculations so that we
		// don't accelerate more than we expect to.
		for(iii = 0; iii < 3; iii++){
			vals[iii] -= alts[iii];
			//vals[iii] -= ANGLE_OFFSET_RAW[iii];
		}
		// https://www.cs.helsinki.fi/group/goa/mallinnus/3dtransf/3drot.html
		#define Z_ROT_ANG 	1
		#define X_ROT_ANG 	2
		#define Y_ROT_ANG		0
		// rotate about Z axis
		CENTER[0] += ((vals[0] * ANGLE[Z_ROT_ANG]) - (vals[1] * sinf(acosf(ANGLE[Z_ROT_ANG]))));
		CENTER[1] += ((vals[0] * sinf(acosf(ANGLE[Z_ROT_ANG]))) + (vals[1] * ANGLE[Z_ROT_ANG]));
		// X
		CENTER[1] += ((vals[1] * ANGLE[X_ROT_ANG]) - (vals[2] * sinf(acosf(ANGLE[X_ROT_ANG]))));
		CENTER[2] += ((vals[1] * sinf(acosf(ANGLE[X_ROT_ANG]))) + (vals[2] * ANGLE[X_ROT_ANG]));
		// Y
		CENTER[2] += ((vals[2] * ANGLE[Y_ROT_ANG]) - (vals[0] * sinf(acosf(ANGLE[Y_ROT_ANG]))));
		CENTER[0] += ((vals[2] * sinf(acosf(ANGLE[Y_ROT_ANG]))) + (vals[0] * ANGLE[Y_ROT_ANG]));
}

/********************************************************************
* We want to read the position in on some constant timeframe, to make
* math easier. That value should be ~5x a second at minimum, aiming
* for as much as we can. No inputs or outputs here, but writes to
* our global variables.
* CURRENTLY: Runs ~20x a second.
********************************************************************/
void PIT_IRQHandler(void){
	//short int vals[3];	// X = 0, Y = 1, Z = 2
	//			int sqrs[3];
	//int iii;
	//float magnitude = 0.0;
	__asm("CPSID   I");  
	if(COUNTER_2 > 10){
		COUNTER_2 = 0;
		Write_GPIO(8,0);
		Write_GPIO(9,0);
	}
	if(COUNTER < 1000){
		COUNTER++;
	} else {
		COUNTER = 0;
		COUNTER_2++;
		CENTER[0] = (int) convert_raw_to_position(X_POS);
		CENTER[1] = (int) convert_raw_to_position(Y_POS);
		CENTER[2] = (int) convert_raw_to_position(Z_POS);
		/*
		vals[0] = (convert_raw_to_position(X_POS));
		vals[1] = (convert_raw_to_position(Y_POS));
		vals[2] = (convert_raw_to_position(Z_POS));
		for(iii = 0; iii < 3; iii++){
			sqrs[iii] = vals[iii] * vals[iii];
		}
		magnitude = sqrtf(sqrs[0] + sqrs[1] + sqrs[2]); // * (9.81 / 4096.0);
		// if we have a magnitude of roughly 9.81, we can be SOMEWHAT sure that we aren't actively
		// moving; so, let's update our angular position. The only way we could have an error here
		// is if we are moving towards gravity in one direction, and also moving in some other direction
		// (IE: Z-axis is 0.8g and Y-axis is 0.2g, because we're moving down on the Z and across on the Y).
		// But even this can only happen if we've got a magnitude within bounds, which is rare.
		if(magnitude < (10.5 * g1 / g_const) && magnitude > (9.1 * g1 / g_const)){
			// We used to call the Angle_Function func, but since it's been simplified so much, it's more
			// of a pain to deal with the branch than we need to have.
			// Angle_Function(magnitude, vals);
			for(iii = 0; iii < 3; iii++){
				ANGLE_OFFSET_RAW[iii] = vals[iii];
				ANGLE[iii] = ANGLE_OFFSET_RAW[iii] / magnitude;
			}
		} else {
			// wait fuck it doesn't work
			// Acceleration_Function(magnitude, vals);
		}
		*/
	}
	Clear_PIT_Interrupt_Flag();
	__asm("CPSIE   I"); 
}

/********************************************************************
* FRDM-KL05 BOARDS:
* - Command 'g'; asks for stats. Sent in the format:
* 	- int (4 bytes)	-- title number
* 	- int (4 bytes)	-- division count
* - Command 'v': asks for voltage:
* 	- short 	-- voltage output value. (ADC)
* - Command 'a': asks for accelerometer data:
* 	- int	(4 bytes)	-- X position
* 	- int	(4 bytes)	-- Y position
* 	- int	(4 bytes)	-- Z position
********************************************************************/
int main(void){
	char input_val;
	__asm("CPSID   I");  																			// mask interrupts
  Init_UART0_IRQ();																					// Initialize the UART for text I/O
	Init_and_Cal_ADC0();																			// Initialize the ADC for voltage I/O
	Init_I2C();																								// Initialize the I2C for usage with the Accelerometer
	Init_Accelerometer();																			// Enable the Accelerometer
	Init_PIT_IRQ();																						// Enable the Periodic Interrupt Timer
	Init_GPIO_LED();
	__asm("CPSIE   I");  																			// unmask interrupts
	
	while(1){																									// Do this forever
		input_val = GetChar();																	// Get input from the UART.
		__asm("CPSID   I");
		if(input_val == 'g'){																		// If 'g' is the input, send the stats.
			Write_Num(TITLE_NUMBER, TITLE_NUM_SIZE);							// Write designated output.
			Write_Num(DIVISION_COUNT, DIVISION_CNT_SIZE);					// Write designated output.
		} else if(input_val == 'v'){														// If 'v' is the input, send the voltage.
			Write_GPIO(8,1);
			//Write_Num(Voltage_Get(VOLTAGE_PORT), VOLTAGE_SIZE);		// Read the voltage, send it.
			Write_Num(0, VOLTAGE_SIZE);
		} else if(input_val == 'a') {														// If 'a' is the input, send position data.
			Write_GPIO(9,1);
			Write_Num(CENTER[0], 4);															// X position
			Write_Num(CENTER[1], 4);															// Y position
			Write_Num(CENTER[2], 4);															// Z position
		}
		__asm("CPSIE   I"); 
	}
}

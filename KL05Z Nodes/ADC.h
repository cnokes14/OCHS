/********************************************************************

********************************************************************/

#define EXERCISE_11_C (1)
// this version of the function enables (or keeps enabled) interrupts
// see Voltage_Get for details.
#define Voltage_Get_Interrupt(x) Voltage_Get(0x40 | (x))


typedef int Int32;
typedef short int Int16;
typedef char Int8;
typedef unsigned int UInt32;
typedef unsigned short int UInt16;
typedef unsigned char UInt8;

// assembly language subroutines
char GetChar(void);
void GetStringSB(char String[], int StringBufferCapacity);
void Init_UART0_IRQ(void);
void PutChar(char Character);
void PutNumHex(UInt32);
void PutNumUB(UInt8);
void PutNumU(UInt32);
void PutStringSB (char String[], int StringBufferCapacity);

/******************************************************************************
* FUNCTION: 		Voltage_Get
* DESCRIPTION:	Given an input port, get the voltage read by the
*								analog-to-digital converter (ADC0) on channel A
* 							via software polling. Returns a 16-bit value.
*
* 							The number is defined as:
*													   /      V(ADC) \
*				 				((2^N)-1) * |  1 -  ------ |  = OUTPUT
*														 \      V(OUT) /
*								Where V(ADC) is equal to the voltage across a
*								100-Ohm resistor in series with the circuit
*								attached, V(OUT) is the voltage being output by
*								the given port, and N is the number of bits in
*								the output (either 12, 10, or 8) determined
*								during setup.
*
*								This function assumes that interrupts are disabled
*								and that input has already been checked to be less
*								than 32. Using this function with input greater than
*								32 or while interrupts are enabled can have unforeseeable
*								consequences! Values higher than 23 will not read basic
*								ports; see documentation for further details.
* INPUTS:				int point_to_check -- port to check; see documentation
* 																		for physical input equivalency list.
* OUTPUTS:			UInt16 V_ADC -- the output, defined by the equation above.
******************************************************************************/
UInt16 Voltage_Get(int point_to_check);

void Write_Num(unsigned int num, unsigned int count);

void Init_I2C(void);

void Start_I2C(void);

void Repeat_Start_I2C(void);

void Stop_I2C(void);

void Write_I2C_Data(unsigned char data);

unsigned char Read_I2C_Data(unsigned char acknowledge);

void Init_PIT_IRQ(void);

void Clear_PIT_Interrupt_Flag(void);

void PIT_IRQHandler(void);

void Write_GPIO(unsigned char pin, unsigned char val);

void Set_GPIO_Output(unsigned char pin);

void Set_GPIO_Input(unsigned char pin);

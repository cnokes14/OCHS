            TTL Voltage Functions
;****************************************************************
; Author: 		  	  Christopher Nokes
; Supporting Authors: Ethan Adkins	 -- Project aide.
;					  Veran Stanek	 	 -- Project aide.
;					  Dr. Robert Kremens -- Faculty advisor.
; Description:		  Functions for the ADC0.
;****************************************************************
;Assembler directives
            THUMB
            GBLL  MIXED_ASM_C
MIXED_ASM_C SETL  {TRUE}
            OPT   64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL05Z4.s
            OPT  1          ;Turn on listing
;****************************************************************
;EQUates
ADC_POS 				EQU 	0x4003B000
ADC_SC1A_SCR_OFFSET		EQU		0x0
ADC_SC1A_SCR			EQU		ADC_POS
ADC_SC1B_SCR_OFFSET		EQU		0x4
ADC_SC1B_SCR			EQU		ADC_POS + 0x4
ADC_SC1A_READ_OFFSET	EQU		0x10
ADC_SC1A_READ			EQU		ADC_POS + 0x10
ADC_SC1B_READ_OFFSET	EQU		0x14
ADC_SC1B_READ			EQU		ADC_SC1A_READ + 0x4
ADC_RESET_READ			EQU		0xFFFFFFE0
	
;****************************************************************
SIM_SCGC4_I2C_MASK		EQU		(1 << 6)

I2C_POS					EQU		0x40066000
I2C_FDR_OFFSET			EQU		0x01
I2C_CR1_OFFSET			EQU		0x02
I2C_STATUS_OFFSET		EQU		0x03
I2C_DATA_OFFSET			EQU		0x04
I2C_ENABLE_MASK			EQU		(1 << 7)
I2C_MASTER_MASK			EQU		(1 << 5)
I2C_TX_MASK				EQU		(1 << 4)
I2C_TXAK_MASK			EQU		(1 << 3)
I2C_REPEAT_MASK			EQU		(1 << 2)
I2C_INTERRUPT_MASK		EQU		(1 << 1)
I2C_RXAK_MASK			EQU		1
I2C_INIT_MASK			EQU		(I2C_ENABLE_MASK | I2C_MASTER_MASK | I2C_TX_MASK)
	
I2C_TCF_MASK			EQU		(1 << 7)
I2C_BUSY_MASK			EQU		(1 << 5)
I2C_IICIF_MASK			EQU		(1 << 1)

ACC_ADDR_WRITE			EQU		(0x1C << 1)
ACC_ADDR_READ			EQU		((0x1C << 1) | 1)
ACC_OUT_X_MSB_ADDR		EQU		0x01
ACC_OUT_X_LSB_ADDR		EQU		0x02
ACC_OUT_Y_MSB_ADDR		EQU		0x03
ACC_OUT_Y_LSB_ADDR		EQU		0x04
ACC_OUT_Z_MSB_ADDR		EQU		0x05
ACC_OUT_Z_LSB_ADDR		EQU		0x06
	
PIT_POS					EQU		0x40037000
PIT_LOAD_OFFSET			EQU		0x100
PIT_CTRL_OFFSET			EQU		0x108
PIT_FLAG_OFFSET			EQU		0x10C
PIT_DISABLE_MASK		EQU		(1 << 30)
PIT_INTERRUPT_MASK		EQU		(1 << 1)
PIT_ENABLE_MASK			EQU		(1 << 0)
PIT_INIT_MASK			EQU		(PIT_ENABLE_MASK | PIT_INTERRUPT_MASK)
PIT_LOAD				EQU		(PIT_POS + PIT_LOAD_OFFSET)
PIT_CTRL				EQU		(PIT_POS + PIT_CTRL_OFFSET)
PIT_FLAG				EQU		(PIT_POS + PIT_FLAG_OFFSET)
	
GPIO_POS				EQU		0x400FF040
	
;***************************************************************
;Program
            AREA    MyCode,CODE,READONLY
			EXPORT	Clear_PIT_Interrupt_Flag
			EXPORT	Voltage_Get
			EXPORT	Write_Num
			EXPORT	Init_I2C
			EXPORT	Start_I2C
			EXPORT	Write_I2C_Data
			EXPORT	Read_I2C_Data
			EXPORT	Repeat_Start_I2C
			EXPORT	Stop_I2C
			EXPORT	Set_GPIO_Output
			EXPORT	Set_GPIO_Input
			EXPORT	Write_GPIO
			IMPORT	PutChar
;>>>>> begin subroutine code <<<<<
;***************************************************************
; 
; INPUT: 	R0 - Pin #
; 			R1 - Value (0 = 0, anything else = 1)
; OUTPUT:	
;***************************************************************
Write_GPIO	PROC	{R0-R15}
;***************************************************************
			PUSH	{R0-R2}
			MOVS	R2,#1
			LSLS	R2,R2,R0
			LDR		R0,=GPIO_POS
			CMP		R1,#0
			BEQ		Write_GPIO_Clear
			STR		R2,[R0,#GPIO_PSOR_OFFSET]
			B		Write_GPIO_End
Write_GPIO_Clear
			STR		R2,[R0,#GPIO_PCOR_OFFSET]
Write_GPIO_End
			POP		{R0-R2}
			BX		LR
			ENDP
;***************************************************************
; 
; INPUT: 	R0 - Pin #
; OUTPUT:	R0 - Val (0 or 1)
;***************************************************************
Read_GPIO	PROC	{R0-R15}
;***************************************************************
			PUSH	{R0-R3}
			
			POP		{R0-R3}
			BX		LR
			ENDP
;***************************************************************
; 
; INPUT: 	R0 - Pin
; OUTPUT:	
;***************************************************************
Set_GPIO_Output	PROC	{R0-R15}
;***************************************************************
			PUSH	{R0-R3}
			LDR		R1,=GPIO_POS
			LDR		R2,[R1,#GPIO_PDDR_OFFSET]
			MOVS	R3,#1
			LSLS	R3,R3,R0
			ORRS	R2,R2,R3
			STR		R2,[R1,#GPIO_PDDR_OFFSET]
			POP		{R0-R3}
			BX		LR
			ENDP
;***************************************************************
; 
; INPUT: 	R0 - Pin
; OUTPUT:	N/A, but sets a GPIO pin to be an input device.
;***************************************************************
Set_GPIO_Input	PROC	{R0-R15}
;***************************************************************
			PUSH	{R1-R3}
			LDR		R1,=GPIO_POS
			LDR		R2,[R1,#GPIO_PDDR_OFFSET]
			MOVS	R3,#1
			LSLS	R3,R3,R0
			BICS	R2,R2,R3
			STR		R2,[R1,#GPIO_PDDR_OFFSET]
			POP		{R1-R3}
			BX		LR
			ENDP
;***************************************************************
; 
; INPUT: 	N/A
; OUTPUT:	N/A, but clears PIT interrupt flag
;***************************************************************
Clear_PIT_Interrupt_Flag	PROC	{R0-R15}
;***************************************************************
			PUSH	{R0-R1}
			LDR		R0,=PIT_FLAG
			LDR		R1,=PIT_TFLG_TIF_MASK
			STR		R1,[R0,#0]
			POP		{R0-R1}
			BX		LR
			ENDP
;***************************************************************
; https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf
; https://mycourses.rit.edu/d2l/le/content/992824/viewContent/8347260/View
; https://mycourses.rit.edu/d2l/le/content/992824/viewContent/8347259/View
; INPUT: 	
; OUTPUT:	
;***************************************************************
Init_I2C 	PROC	{R0-R15}
;***************************************************************
			PUSH	{R0-R2}						; Save used values to the stack.
			LDR		R0,=SIM_SCGC5
			LDR		R1,[R0,#0]
			LDR		R2,=SIM_SCGC5_PORTA_MASK
			ORRS	R1,R1,R2
			STR		R1,[R0,#0]
			
			LDR		R0,=PORTB_BASE
			LDR		R2,=PORT_PCR_MUX_SELECT_2_MASK
			STR		R2,[R0,#PORTB_PCR3_OFFSET]
			STR		R2,[R0,#PORTB_PCR4_OFFSET]
			
			LDR		R0,=SIM_SCGC4
			LDR		R1,[R0,#0]
			MOVS	R2,#SIM_SCGC4_I2C_MASK
			ORRS	R1,R1,R2
			STR		R1,[R0,#0]					; Enable I2C clock unit connection.
			
			LDR		R0,=I2C_POS
			MOVS	R1,#0x14
			STRB	R1,[R0,#I2C_FDR_OFFSET]		; Set clock divider.
			MOVS	R1,#0x0
			STRB	R1,[R0,#I2C_CR1_OFFSET]		; Clear CR1
			MOVS	R1,#I2C_INIT_MASK
			STRB	R1,[R0,#I2C_CR1_OFFSET]		; Enable the unit.
			POP		{R0-R2}
			BX		LR							; Return
			ENDP
;***************************************************************
; 
; INPUT: 	
; OUTPUT:	
;***************************************************************
Start_I2C	PROC	{R0-R15}
;***************************************************************
			PUSH	{R0-R2}						; Save used values to the stack.
			LDR		R0,=I2C_POS
			LDRB	R1,[R0,#I2C_CR1_OFFSET]
			MOVS	R2,#I2C_TX_MASK
			ORRS	R1,R1,R2
			STRB	R1,[R0,#I2C_CR1_OFFSET]
			MOVS	R2,#I2C_MASTER_MASK
			ORRS	R1,R1,R2
			STRB	R1,[R0,#I2C_CR1_OFFSET]
			MOVS	R2,#I2C_BUSY_MASK
Start_I2C_Loop
			LDRB	R1,[R0,#I2C_STATUS_OFFSET]
			TST		R1,R2
			BEQ		Start_I2C_Loop
			POP		{R0-R2}						; Restore values from the stack.
			BX		LR							; Return
			ENDP
;***************************************************************
; 
; INPUT: 	
; OUTPUT:	
;***************************************************************
Repeat_Start_I2C	PROC	{R0-R15}
;***************************************************************
			PUSH	{R0-R2}						; Save used values to the stack.
			LDR		R0,=I2C_POS
			LDRB	R1,[R0,#I2C_CR1_OFFSET]
			MOVS	R2,#I2C_REPEAT_MASK
			ORRS	R1,R1,R2
			STRB	R1,[R0,#I2C_CR1_OFFSET]
			MOVS	R2,#I2C_BUSY_MASK
Repeat_Start_I2C_Loop
			LDRB	R1,[R0,#I2C_STATUS_OFFSET]
			TST		R1,R2
			BEQ		Repeat_Start_I2C_Loop
			POP		{R0-R2}						; Restore values from the stack.
			BX		LR							; Return
			ENDP
;***************************************************************
; 
; INPUT: 	
; OUTPUT:	
;***************************************************************
Stop_I2C	PROC	{R0-R15}
;***************************************************************
			PUSH	{R0-R3}						; Save used values to the stack.
			LDR		R0,=I2C_POS
			LDRB	R1,[R0,#I2C_CR1_OFFSET]
			MOVS	R2,#I2C_MASTER_MASK
			BICS	R1,R1,R2
			STRB	R1,[R0,#I2C_CR1_OFFSET]
			MOVS	R2,#I2C_BUSY_MASK
			LDR		R3,=0xFFFF
Stop_I2C_Loop
			SUBS	R3,R3,#0x01
			BEQ		Stop_I2C_Timeout
			LDRB	R1,[R0,#I2C_STATUS_OFFSET]
			TST		R1,R2
			BNE		Stop_I2C_Loop
Stop_I2C_Timeout
			POP		{R0-R3}						; Restore values from the stack.
			BX		LR							; Return
			ENDP
;***************************************************************
; 
; INPUT: 	R0 -- Unsigned char, data.
; OUTPUT:	N/A
;***************************************************************
Write_I2C_Data	PROC	{R0-R15}
;***************************************************************
			PUSH	{R1-R4}						; Save used values to the stack.
			LDR		R1,=I2C_POS
			LDR		R2,=I2C_TCF_MASK
			LDR		R4,=0x0100
Write_I2C_WaitForReady
			SUBS	R4,R4,#0x01
			BEQ		Write_I2C_Timeout
			LDRB	R3,[R1,#I2C_STATUS_OFFSET]
			TST		R3,R2
			BEQ		Write_I2C_WaitForReady
			
			LDRB	R3,[R1,#I2C_CR1_OFFSET]
			MOVS	R2,#I2C_TX_MASK
			ORRS	R3,R3,R2
			STRB	R3,[R1,#I2C_CR1_OFFSET]
			
			STRB	R0,[R1,#I2C_DATA_OFFSET]
			MOVS	R2,#I2C_IICIF_MASK
Write_I2C_WaitForSent				
			SUBS	R4,R4,#0x01
			BEQ		Write_I2C_Timeout		
			LDRB	R3,[R1,#I2C_STATUS_OFFSET]
			TST		R3,R2
			BEQ		Write_I2C_WaitForSent
Write_I2C_Timeout
			ORRS	R3,R3,R2
			STRB	R3,[R1,#I2C_STATUS_OFFSET]
			
			POP		{R1-R4}						; Restore values from the stack.
			BX		LR							; Return
			ENDP			
;***************************************************************
; 
; INPUT: 	R0 -- Unsigned char--0 for no acknowledge, anything else acknowledge
; OUTPUT:	R0 -- Output data
;***************************************************************
Read_I2C_Data	PROC	{R1-R15}
;***************************************************************
			PUSH	{R1-R4}						; Save used values to the stack.
			LDR		R1,=I2C_POS
			LDR		R4,=0x0100
			
			LDRB	R2,[R1,#I2C_CR1_OFFSET]
			MOVS	R3,#I2C_TX_MASK
			BICS	R2,R2,R3
			STRB	R2,[R1,#I2C_CR1_OFFSET]
			MOVS	R3,#I2C_TXAK_MASK
			ORRS	R2,R2,R3
			STRB	R2,[R1,#I2C_CR1_OFFSET]
			
			LDR		R2,=I2C_TCF_MASK
Read_I2C_WaitForReady
			SUBS	R4,R4,#0x01
			BEQ		Read_I2C_Timeout
			LDRB	R3,[R1,#I2C_STATUS_OFFSET]
			TST		R3,R2
			BEQ		Write_I2C_WaitForReady
			
			LDRB	R3,[R1,#I2C_CR1_OFFSET]
			MOVS	R2,#I2C_TX_MASK
			BICS	R3,R3,R2
			STRB	R3,[R1,#I2C_CR1_OFFSET]
			
			MOVS	R2,#I2C_TXAK_MASK
			BEQ		Read_I2C_NoAcknowledge
			ORRS	R3,R3,R2
			B		Read_I2C_PostAcknowledge
Read_I2C_NoAcknowledge
			BICS	R3,R3,R2
Read_I2C_PostAcknowledge
			STRB	R3,[R1,#I2C_CR1_OFFSET]
			
			LDRB	R0,[R1,#I2C_DATA_OFFSET]
			MOVS	R2,#I2C_IICIF_MASK
Read_I2C_WaitForGot			
			SUBS	R4,R4,#0x01
			BEQ		Read_I2C_Timeout
			LDRB	R3,[R1,#I2C_STATUS_OFFSET]
			TST		R3,R2
			BEQ		Read_I2C_WaitForGot
Read_I2C_Timeout
			ORRS	R3,R3,R2
			STRB	R3,[R1,#I2C_STATUS_OFFSET]
			
			POP		{R1-R4}						; Restore values from the stack.
			BX		LR							; Return
			ENDP
;***************************************************************
; Reads from the A channel of the analog-to-digital converter (ADC0).
; INPUT: 	R0 - Port # to read from. USE DOCUMENTATION TO DETERMINE PORTS.
; OUTPUT:	R0 - Value read from ADC0, 16 bits, but 4 to 8 of those bits will
;				 be leftmost padded 0s, depending on initialization.
;***************************************************************
Voltage_Get	PROC	{R1-R15}
;***************************************************************
			PUSH	{R1, R2}						; Save used values to the stack.
			LDR		R1,=ADC_POS						; R1 = ADC0's address	
			MOVS	R2,#0x80						; R2 = SCR mask; checks for completed conversion.
			STR		R0,[R1, #ADC_SC1A_SCR_OFFSET]	; Store the SCR; asks the ADC to give it a new value.
Voltage_Get_Test
			LDR		R0,[R1, #ADC_SC1A_SCR_OFFSET]	; Check the SCR
			TST		R0,R2							; If the SCR's conversion is done, this will equal 0x80
			BEQ		Voltage_Get_Test				; If it's equal to 0, loop back and keep waiting.
			LDR		R0,[R1, #ADC_SC1A_READ_OFFSET]	; R0 = Finished, converted ADC value.
			POP		{R1, R2}						; Restore values from the stack.
			BX		LR								; Return
			ENDP					
;**********************************************************************
; ???
; INPUT: 	R0 - Number to write to UART.
;			R1 - Number of bytes to write.
; OUTPUT:	None, but writes the number to the UART.
;***************************************************************
Write_Num	PROC	{R0-R15}
;***************************************************************
			PUSH	{R0-R4, LR}						; Save used values to the stack.
			MOVS	R2,R0
			LDR		R3,=0xFF
Write_Num_Loop
			MOVS	R0,R2
			ANDS	R0,R0,R3
			BL		PutChar
			LSRS	R2,R2,#0x08
			SUBS	R1,R1,#0x01
			BNE		Write_Num_Loop
			POP		{R0-R4, PC}						; We're done; pop and return.
			ENDP
;**********************************************************************
			END
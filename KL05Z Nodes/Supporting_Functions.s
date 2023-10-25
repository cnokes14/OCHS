            TTL Supporting Functions
;****************************************************************
; Author: 		  	  Christopher Nokes
; Supporting Authors: Dr. Roy Melton     -- File outline, CMPE-250
; 					  Ethan Adkins	     -- Project aide.
;					  Veran Stanek	 	 -- Project aide.
;					  Dr. Robert Kremens -- Faculty advisor.
; Description:		  Supporting functions for the FRDM-KL05 board.
;					  Originally used for CMPE-250 Lab Exercise 11
;					  in Fall of 2022; updated and shortened for
;					  use in this project.
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
;---------------------------------------------------------------
;Characters
BS          EQU  0x08
CR          EQU  0x0D
DEL         EQU  0x7F
ESC         EQU  0x1B
LF          EQU  0x0A
NULL        EQU  0x00
; Queue management record field offsets
IN_PTR      EQU   0
OUT_PTR     EQU   4
BUF_STRT    EQU   8
BUF_PAST    EQU   12
BUF_SIZE    EQU   16
NUM_ENQD    EQU   17
; Queue structure sizes
TRBUF_SZ	EQU	  80  ;Transmit queue size
RCBUF_SZ	EQU	  80  ;Receive queue size
Q_REC_SZ    EQU   18  ;Queue management record

; UART INITIALIZATION EQUATES
; (these were included in the original outline written by Dr. Melton,
; they are black magic and shall not be questioned.)
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
PORT_PCR_SET_PTB2_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTB1_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
SIM_SOPT2_UART0SRC_MCGFLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
UART0_BDH_9600  EQU  0x01
UART0_BDL_9600  EQU  0x38
UART0_C1_8N1  EQU  0x00
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
UART0_C3_NO_TXINV  EQU  0x00
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
		
PIT_IRQ_PRIORITY EQU 0
NVIC_ICPR_PIT_MASK EQU PIT_IRQ_MASK
NVIC_IPR_PIT_MASK EQU (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0 EQU (PIT_IRQ_PRIORITY << PIT_PRI_POS)
PIT_TCTRL_CH_IE EQU (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
PIT_MCR_EN_FRZ EQU PIT_MCR_FRZ_MASK
PIT_LDVAL_VAL EQU 0x0A0 ; 150000 times a second; 150kHZ
NVIC_ISER_PIT_MASK EQU PIT_IRQ_MASK
;---------------------------------------------------------------
;****************************************************************
;MACROs
;***************************************************************
			MACRO
			ClearC
;**************************************************
; Clears the C value in the PSR
;**************************************************
			PUSH 	{R0,R1}
			MRS		R0, APSR			; Loads the APSR into R0
			LDR		R1,=APSR_C_MASK		; Loads a mask to set / clear only C
			BICS	R0,R0,R1			; Masks to clear C
			MSR		APSR, R0			; Puts R0 back into the APSR
			POP		{R0,R1}
			MEND
;***************************************************************
			MACRO
			SetC
;**************************************************
; Sets the C value in the PSR
;**************************************************
			PUSH 	{R0,R1}
			MRS		R0, APSR			; Loads the APSR into R0
			LDR		R1,=APSR_C_MASK		; Loads a mask to set / clear only C
			ORRS	R0,R0,R1			; Masks to set C
			MSR		APSR, R0			; Puts R0 back into the APSR
			POP		{R0,R1}
			MEND
;****************************************************************
;Program
;C file will contain main(); only subroutines in the assembly files
            AREA    MyCode,CODE,READONLY
			EXPORT	Init_PIT_IRQ
			EXPORT	Init_UART0_IRQ
			EXPORT	UART0_IRQHandler
			EXPORT	PutStringSB
			EXPORT	GetStringSB
			EXPORT	PutChar
			EXPORT	GetChar
			EXPORT	PutNumHex
			EXPORT	PutNumUB
			EXPORT	PutNumU
;>>>>> begin subroutine code <<<<<
;***************************************************************
; Initializes PIT for periodic interrupts
; No input or output for this subroutine.
;***************************************************************
Init_PIT_IRQ	PROC	{R0-R15}
;***************************************************************
			PUSH	{R0-R3}
			; Enable clock for PIT module
			LDR		R0,=SIM_SCGC6
			LDR		R1,=SIM_SCGC6_PIT_MASK
			LDR		R2,[R0,#0]
			ORRS	R2,R2,R1
			STR		R2,[R0,#0]
			; Disable PIT timer 0
			LDR		R0,=PIT_CH0_BASE
			LDR		R1,=PIT_TCTRL_TEN_MASK
			LDR		R2,[R0,#PIT_TCTRL_OFFSET]
			BICS	R2,R2,R1
			STR		R2,[R0,#PIT_TCTRL_OFFSET]
			; Set PIT priority
			LDR		R0,=PIT_IPR
			LDR		R1,=NVIC_IPR_PIT_MASK
			LDR		R2,[R0,#0]
			BICS	R2,R2,R1
			STR		R2,[R0,#0]
			; Clear pending requests
			LDR		R0,=NVIC_ICPR
			LDR		R1,=NVIC_ICPR_PIT_MASK
			STR		R1,[R0,#0]
			; Unmask PIT interrupts
			LDR		R0,=NVIC_ISER
			LDR 	R1,=NVIC_ISER_PIT_MASK
			STR		R1,[R0,#0]
			; enable PIT module
			LDR		R0,=PIT_BASE
			LDR		R1,=PIT_MCR_EN_FRZ
			STR		R1,[R0,#PIT_MCR_OFFSET]
			; Set PIT timer 0 period for 0.01s
			LDR		R0,=PIT_CH0_BASE
			LDR		R1,=PIT_LDVAL_VAL
			STR		R1,[R0,#PIT_LDVAL_OFFSET]
			; enable PIT timer 0 interrupt
			LDR		R1,=PIT_TCTRL_CH_IE
			STR		R1,[R0,#PIT_TCTRL_OFFSET]
			POP		{R0-R3}
			BX		LR
			ENDP
;****************************************************************
Init_UART0_IRQ		PROC	{R0-R15}
;***************************************************************
; Initializes UART0 for polling
; No input or output for this subroutine.
;***************************************************************
			PUSH	{R0, R1, R2, LR}
			;-------------------------------
			; Initialize queue structures.
			;-------------------------------
			; Initialize Receive Queue
			LDR		R0,=ReceiveQueueBuffer
			LDR		R1,=ReceiveQueueRecord
			MOVS	R2,#RCBUF_SZ
			BL		InitQueue
			; Initialize Transmit Queue
			LDR		R0,=TransmitQueueBuffer
			LDR		R1,=TransmitQueueRecord
			MOVS	R2,#TRBUF_SZ
			BL		InitQueue
			;-------------------------------
			; Start actual initialization
			;-------------------------------
			;Select MCGFLLCLK as UART0 clock source
			LDR		R0,=SIM_SOPT2
			LDR		R1,=SIM_SOPT2_UART0SRC_MASK
			LDR		R2,[R0,#0]
			BICS	R2,R2,R1
			LDR		R1,=SIM_SOPT2_UART0SRC_MCGFLLCLK
			ORRS	R2,R2,R1
			STR		R2,[R0,#0]
			;Set UART0 for external connection
			LDR		R0,=SIM_SOPT5
			LDR		R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
			LDR		R2,[R0,#0]
			BICS	R2,R2,R1
			STR		R2,[R0,#0]
			;Enable UART0 module clock
			LDR		R0,=SIM_SCGC4
			LDR		R1,=SIM_SCGC4_UART0_MASK
			LDR		R2,[R0,#0]
			ORRS	R2,R2,R1
			STR		R2,[R0,#0]
			;Enable PORT B module clock
			LDR		R0,=SIM_SCGC5
			LDR		R1,=SIM_SCGC5_PORTB_MASK
			LDR		R2,[R0,#0]
			ORRS	R2,R2,R1
			STR		R2,[R0,#0]
			;Select PORT B Pin 2 (D0) for UART0 RX (J8 Pin 01)
			LDR		R0,=PORTB_PCR2
			LDR		R1,=PORT_PCR_SET_PTB2_UART0_RX
			STR		R1,[R0,#0]
			;Select PORT B Pin 2 (D1) for UART0 TX (J8 Pin 02)
			LDR		R0,=PORTB_PCR1
			LDR		R1,=PORT_PCR_SET_PTB1_UART0_TX
			STR		R1,[R0,#0]
			;Disable UART0 reciever and transmitter
			LDR		R0,=UART0_BASE
			MOVS	R1,#UART0_C2_T_R
			LDRB	R2,[R0,#UART0_C2_OFFSET]
			BICS	R2,R2,R1
			STRB	R2,[R0,#UART0_C2_OFFSET]
			;Initialize NVIC for UART0 interrupts
			LDR		R0,=UART0_IPR
			LDR		R2,=NVIC_IPR_UART0_PRI_3
			LDR		R3,[R0,#0]
			ORRS	R3,R3,R2
			STR		R3,[R0,#0]
			LDR		R0,=NVIC_ICPR
			LDR		R1,=NVIC_ICPR_UART0_MASK
			STR		R1,[R0,#0]
			LDR		R0,=NVIC_ISER
			LDR		R1,=NVIC_ISER_UART0_MASK
			STR		R1,[R0,#0]			
			;Set UART0 for 9600 baud, 8N1 protocol
			LDR		R0,=UART0_BASE
			MOVS	R1,#UART0_BDH_9600
			STRB	R1,[R0,#UART0_BDH_OFFSET]
			MOVS	R1,#UART0_BDL_9600
			STRB	R1,[R0,#UART0_BDL_OFFSET]
			MOVS	R1,#UART0_C1_8N1
			STRB	R1,[R0,#UART0_C1_OFFSET]
			MOVS	R1,#UART0_C3_NO_TXINV
			STRB	R1,[R0,#UART0_C3_OFFSET]
			MOVS	R1,#UART0_C4_NO_MATCH_OSR_16
			STRB	R1,[R0,#UART0_C4_OFFSET]
			MOVS	R1,#UART0_C5_NO_DMA_SSR_SYNC
			STRB	R1,[R0,#UART0_C5_OFFSET]
			MOVS	R1,#UART0_S1_CLEAR_FLAGS
			STRB	R1,[R0,#UART0_S1_OFFSET]
			MOVS	R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
			STRB	R1,[R0,#UART0_S2_OFFSET]
			;Enable receive interrupt.
			MOVS	R1,#UART0_C2_TI_RI
			STRB	R1,[R0,#UART0_C2_OFFSET]					
			;-------------------------------
			; End initialization
			;-------------------------------
			POP		{R0, R1, R2, PC}
			ENDP
;***************************************************************
GetChar		PROC	{R1-R15}
;***************************************************************
; Waits for the terminal to send a character, then stores it in R0.
; OUTPUT: R0, CHARACTER BYTE (ASCII)
;***************************************************************
			PUSH	{R1, LR}					; Push R1, LR
			LDR		R1,=ReceiveQueueRecord		; R1 = ReceiveQueueRecord
GetCharWait										; Loop to wait for a character
			CPSID	I							; Mask interrupts
			BL		Dequeue						; Attempt to dequeue a character
			CPSIE	I							; Unmask interrupts
			BCS		GetCharWait					; If dequeuing failed, keep waiting.
			POP		{R1, PC}					; Pop R1 and return.
			ENDP
;***************************************************************
PutChar		PROC	{R0-R15}
;***************************************************************
; Waits for the terminal to be ready to accept a character, then sends it.
; INPUT: R0 - Character, ASCII byte
;***************************************************************
			PUSH	{R1,R2,R4, LR}				; Push R1, LR
			LDR		R1,=TransmitQueueRecord		; R1 = ReceiveQueueRecord
PutCharWait										; Loop to wait for a character
			CPSID	I							; Mask interrupts
			BL		Enqueue						; Attempt to dequeue a character
			CPSIE	I							; Unmask interrupts
			BCS		PutCharWait					; If dequeuing failed, keep waiting.
			LDR		R4,=UART0_BASE				; Load first UART0 address to R4
			MOVS	R2,#UART0_C2_TI_RI			; Load the C2 interrupt enable mask to R2
			STRB	R2,[R4,#UART0_C2_OFFSET]	; Store C2 back with TIE enabled.
			POP		{R1,R2,R4, PC}				; Pop R1 and return.
			ENDP
;***************************************************************
GetStringSB	PROC	{R0-R15}
;***************************************************************
; Gets a string of characters from a terminal until a termination key
; is pressed, or the user hits the character limit.
; INPUT: 	R0 - Memory address to write the strings to.
;			R1 - Length of the string buffer.
; VALUES:	R0 - (After being pushed) input character
;			R2 - Index
;			R3 - Start memory address
;***************************************************************
			PUSH	{R0-R3, LR}			; Put the LR at the bottom of the stack; pop at the end of the subroutine.
			MOVS	R3, R0				; Put the memory starting address so that R0 can be used to read characters
			SUBS	R1,R1,#1			; Get R1-1 to find last character add point of buffer.
			MOVS	R2,#0				; index is 0
GetStringSB_Loop						
			BL		GetChar				; get the character, store it in register 0
			CMP		R0,#'\r'			; check if the character is the newline (enter key)
			BEQ		GetStringSB_EndLoop	; if it is, end the loop
			CMP		R0,#'\b'			; if the character is a backspace, do something special
			BEQ		GetStringSB_Backspace	; branch to the backspace special case
			CMP		R0,#0x20			; if the character is a control character (besides backspace and newline)
			BLO		GetStringSB_Loop	; the character is invalid, go back
			CMP		R2,R1				; If the index is at R1, then the limit characters have been entered.
			BEQ		GetStringSB_Loop	; If the limit has been reached and the character isn't a newline, loop until it is a newline.
			BL		PutChar				; Put the character on the terminal
			STRB	R0,[R3,R2]			; Load R0 (the character) into R3 (Starting memory address) + R2 (index)
			ADDS	R2,R2,#1			; Increment the index by 1.
			B		GetStringSB_Loop	; Start again
			
GetStringSB_Backspace					; special case if backspace is hit
			CMP		R2,#0				; make sure it doesn't try to erase a character that isn't there
			BEQ		GetStringSB_Loop	; loop back if there's no valid character
			MOVS	R0,#DEL				; Moves a delete character into the register
			BL		PutChar				; Deletes the character at the previous space
			SUBS	R2,R2,#1			; Move 1 space back in memory
			B		GetStringSB_Loop	; start again
			
GetStringSB_EndLoop
			MOVS	R0,#0				; Load the null character into R0
			STRB	R0,[R3,R2]			; Store the null character into the last spot in the array
			MOVS	R0,#'\r'			; Load the line feed
			BL		PutChar				; put the line feed
			MOVS	R0,#'\n'			; Load the new line
			BL		PutChar				; Put the new line
			POP		{R0-R3, PC}			; Pop registers, end subroutine
			ENDP
;***************************************************************
PutStringSB	PROC	{R0-R15}
;***************************************************************
; Puts a string of characters from a terminal until a null character
; is reached, or the program hits the character limit.
; INPUT: 	R0 - Memory address to read the strings from.
;			R1 - Capacity size of the terminal.
; OUTPUT:	R0 - Memory address for the next 
; VALUES:	R0 - (After being pushed): character from memory
;			R2 - Index
;			R3 - Memory start index
;***************************************************************
			PUSH	{R0-R3, LR}			; Push changed values onto stack
			MOVS	R3, R0				; Copy memory address into R3 so R0 can store characters
			MOVS	R2, #0				; Index = 0
PutStringSB_Loop
			LDRB	R0,[R3,R2]			; Load the printing character into R0
			CMP		R0, #0				; If the character is the null character
			BEQ		PutStringSB_EndLoop	; Stop printing
			BL		PutChar				; Otherwise, print the character
			ADDS	R2,R2,#1			; Increment the index
			CMP		R2,R1				; Compare the index with the limit
			BEQ		PutStringSB_EndLoop	; If the index is at the limit, end the loop.
			B		PutStringSB_Loop	; Start again
PutStringSB_EndLoop
			POP		{R0-R3, PC}	
			ENDP
;***************************************************************
PutNumU		PROC	{R0-R15}
;***************************************************************
; Prints a word value number in decimal base from register 0.
; INPUT: 	R0 - Unsigned word value to print.
; VALYES:	R0 - (during convertloop) Divisor and Quotient
;			R0 - (during printloop) Character to print
;			R1 - (during convertloop) Dividend and Remainder
;			R2 - Index / length of the 'string' of numbers
;***************************************************************
			PUSH	{R0-R2, LR}
			MOVS	R1, R0				; Copy the number into R1
			MOVS	R2, #0				; Index
PutNumU_ConvertLoop
			MOVS	R0, #10				; Divisor will always be 10
			BL		DIVU				; Perform R1 / R0
			ADDS	R1, R1, #'0'		; Convert the remainder to a character version of itself
			PUSH	{R1}				; Push the remainder onto the stack
			ADDS	R2, #1				; Add 1 to the index
			CMP		R0, #0				; If the quotient is 0, end the loop
			BEQ		PutNumU_BetweenLoops
			MOVS	R1, R0				; Set R1 (the new dividend) to R0 (the previous quotient)
			B		PutNumU_ConvertLoop	; start again
PutNumU_BetweenLoops
			POP		{R0}				; Pop the top of the stack to R0
			BL		PutChar				; Put the character from R0 onto the terminal
			SUBS	R2,R2,#1			; Decrement the index
			BEQ		PutNumU_EndPrintLoop	; End the loop if the index is 0
			B		PutNumU_BetweenLoops	; otherwise, loop again
PutNumU_EndPrintLoop
			POP		{R0-R2, PC}			; pop pushed values and end the subroutine
			ENDP
;**************************************************
PutNumUB	PROC	{R0-R14}
;**************************************************
; Print a byte value in R0.
; INPUT: 	R0 	-	Value to print
; OUTPUT:	N/A
; MODIFY:	PSR
;**************************************************
			PUSH	{R0-R1, LR}	; Pushes values used
			MOVS	R1,#0xFF	; Sets R1 to be a mask for only the last byte of a number
			ANDS	R0,R0,R1	; Masks R0 with R1
			BL		PutNumU		; Posts the byte value now stored by R0
			POP		{R0-R1, PC}	; Returns values
			ENDP				; Ends subroutine
;***************************************************************
DIVU	PROC	{R2-R14}
;***************************************************************
; INPUTS: R0, R1
; OUTPUTS: R0 - Quotient, R1 - Remainder
; R0 = DIVISOR; R1 = DIVIDEND, DECREASING EACH LOOP // FINAL VALUE is REMAINDER
; R2 = QUOTIENT; R3 = PSR; R4 = PSR MASK
; Notes: Performs R1 / R0 and returns R0 remainder R1
;***************************************************************
			PUSH	{R2, R3, R4} 		; REGISTERS USED FOR ARITHMETIC
			MRS		R3, APSR			; R3 = PSR
			LDR		R4,=APSR_C_MASK		; R4 = Mask to set or clear C value in the PSR.
			CMP		R0, #0				; IF R0 != 0
			BNE		DIVU_SKIP			; SKIP FORWARD, ELSE
			ORRS	R3, R3, R4			; Change R3's C value to be 1 if it isn't already, leaves other values unchanged.
			B		DIVU_ESC			; GOTO PROGRAM END
DIVU_SKIP	MOVS	R2, #0				; R2 = 0, R2 is the quotient
DIVU_LOOP	CMP		R0, R1				; IF R0 > R1
			BHI		DIVU_END			; END LOOP, ELSE
			ADDS	R2, R2, #1			; Increment R2 by 1
			SUBS	R1, R1, R0			; Decrement R1 by the divisor
			B 		DIVU_LOOP			; Loop
DIVU_END	MOVS	R0, R2				; R0 (Return quotient register) = R2 (Looped quotient increment)
			BICS	R3, R3, R4			; Change R3's C value to be 0 if it isn't already, leaves other values unchanged.
DIVU_ESC	MSR		APSR, R3			; SET THE PSR TO R3
			POP		{R2, R3, R4} 		; RETURNS REGISTERS TO OLD VALUES
			BX		LR
			ENDP
;**************************************************
InitQueue	PROC	{R0-R14}
;**************************************************
; Initialize a queue for use.
; INPUT: 	R0 	-	Queue address.
;			R1	-	Queue record address.
;			R2	-	Queue size.
; OUTPUT:	NONE
; MODIFY:	PSR
;**************************************************
			PUSH	{R2}				; Push registers changed
			STR		R0,[R1,#BUF_STRT]	; Store the start address in the record
			STR		R0,[R1,#IN_PTR]		; Store the in address as the start in the record
			STR		R0,[R1,#OUT_PTR]	; Store the out address as the start in the record
			STRB	R2,[R1,#BUF_SIZE]	; Store the size in the record
			ADDS	R2,R2,R0			; Add the size to the start address
			STR		R2,[R1,#BUF_PAST]	; Store the size+start as the first address past in the record
			MOVS	R2, #0				; R2 = 0
			STRB	R2,[R1,#NUM_ENQD]	; Store the number enqueued as 0
			POP		{R2}				; Pop values changed
			BX		LR					; Return to caller code
			ENDP						; End subroutine
;**************************************************
Enqueue		PROC	{R0-R14}
;**************************************************
; Enqueue a character if it can be done, prep the queue
; for the next input. Stores a byte value.
; INPUT: 	R0 	-	Character to store.
;			R1	-	Queue record address
; OUTPUT:	PSR-C - 0 if success, 1 if failure
; MODIFY:	PSR
; VALUES:	R2 (Until C clear)	-	Number of items in the queue
;			R2 (After Clear)	-	First address past buffer end
;			R2 (In Enqueue_Loop)-	First address of the buffer
;			R3 (Fail check)		-	Buffer size
;			R3 (Check passes)	-	Queue in-pointer
;**************************************************
			PUSH	{R2-R3}				; Push used registers
			LDRB	R2,[R1,#NUM_ENQD]	; R2 = Number of items in the queue
			LDRB	R3,[R1,#BUF_SIZE]	; R3 = Maximum number of items for a queue
			CMP		R2, R3				; If the number is already the max
			BEQ		Enqueue_Fail		; then another character cannot be stored; break
			LDR		R3,[R1,#IN_PTR]		; R3 now points to the memory slot for the next byte
			STRB	R0,[R3,#0]			; Stores the input character in its memory slot
			ADDS	R2, R2, #1			; Increments the local copy of the number enqueued
			STRB	R2,[R1,#NUM_ENQD]	; Stores the local number enqueued to memory
			ADDS	R3,R3,#1			; Increments the in pointer by one
			LDR		R2,[R1,#BUF_PAST]	; R2 = first address past the buffer
			CMP		R2,R3				; If the in pointer is now past the buffer
			BEQ		Enqueue_Loop		; loop back to the start, otherwise
			STR		R3,[R1,#IN_PTR]		; Store the new pointer copy
			ClearC
			B		Enqueue_End			; Go to the end
Enqueue_Loop							; Loop the in pointer back to the start
			LDR		R2,[R1,#BUF_STRT]	; Load the start
			STR		R2,[R1,#IN_PTR]		; Store the start as the in pointer
			ClearC
			B		Enqueue_End			; Go to the end
Enqueue_Fail							; If the queue didn't work
			SetC						; Set the C value, handled by a macro.	
Enqueue_End								; When the function is over, one way or another.
			POP		{R2-R3}				; return registers to their original values
			BX		LR					; return to whatever called this subroutine
			ENDP						; end subroutine
;**************************************************
Dequeue		PROC	{R1-R14}
;**************************************************
; Dequeue a character if it can be done, prep the queue
; for the next output. Returns a character in R0.
; INPUT: 	R1 	-	Queue record address
; OUTPUT:	R0  - 	Dequeued character
;			PSR-C - 0 if success, 1 if failure
; MODIFY:	R0, PSR
; VALUES:	R0 (Pre-check)	-	Number of items enqueued.
;			R0 (Post-check)	-	Output value
;			R1				-	Record address
;			R2				-	Out pointer
;			R3				-	Buffer past address
;**************************************************
			PUSH 		{R1-R3}				; push values used
			LDRB		R0,[R1,#NUM_ENQD]	; load number of elements in queue
			CMP			R0,#0				; if there's nothing, 
			BEQ			Dequeue_Fail		; break the queue, otherwise
			SUBS		R0,R0,#1			; decrement the queue size
			STRB		R0,[R1,#NUM_ENQD]	; store the new queue size
			LDR			R2,[R1,#OUT_PTR]	; Load the new character's memory address
			LDRB		R0,[R2,#0]			; Loads the value at the out pointer to be the output
			ADDS		R2,R2,#1			; Increments the out pointer value by one to point to whatever's next in the list.
			LDR			R3,[R1,#BUF_PAST]	; R3 = first address past the buffer
			CMP			R2,R3				; If the out pointer is now past the buffer
			BEQ			Dequeue_Loop		; loop back to the start, otherwise
			STR			R2,[R1,#OUT_PTR]	; Store the new pointer copy
			ClearC							; Clear C to show success.
			B			Dequeue_End			; Go to the end

Dequeue_Loop								; if the pointer needs to loop back to the start
			LDR			R2,[R1,#BUF_STRT]	; Load the start
			STR			R2,[R1,#OUT_PTR]	; Store the start as the in pointer
			ClearC							; Clear C to show success.
			B			Dequeue_End			; Go to the end

Dequeue_Fail								; if there was nothing to dequeue
			SetC							; Set C
			
Dequeue_End									; Run at the end of the subroutine
			POP			{R1-R3}				; Return pushed values
			BX 			LR					; Return to whatever called the subroutine
			ENDP							; End subroutine
;**************************************************
PutNumHex	PROC	{R0-R14}
;**************************************************
; Print the hex representation of a 32 bit number in R0
; INPUT: 	R0 	-	Value to print
; OUTPUT:	N/A
; MODIFY:	PSR
;**************************************************
			PUSH	{R0-R3, LR}			; Changed values
			MOVS	R1,#0				; Loop counter
			MOVS	R2,#0xF				; Mask to get only the last 4 digits
PutNumHex_GetLoop
			CMP		R1,#8				; If the loop counter is the maximum number of hex digits in a word
			BEQ		PutNumHex_BetweenLoops	; break, else
			ADDS	R1,R1,#1			; increment the counter
			MOVS	R3,R0				; Copy R0 into R3
			ANDS	R3,R3,R2			; Get the back 4 digits of R3
			CMP		R3,#10				; Check for hex values
			BHS		PutNumHex_HexCheck	; Store a hex value
			ADDS	R3,R3,#'0'			; Change R3 to its ascii equivalent
			PUSH	{R3}				; push R3
			LSRS	R0,R0,#4			; shift the number over by 4
			B		PutNumHex_GetLoop	; Start again
PutNumHex_HexCheck			
			ADDS	R3,R3,#('A'-10)		; Change R3 to its ascii equivalent
			PUSH	{R3}				; push R3
			LSRS	R0,R0,#4			; shift the number over by 4
			B		PutNumHex_GetLoop	; start again
PutNumHex_BetweenLoops
			POP		{R0}				; remove the top of the stack
			BL		PutChar				; print it
			SUBS	R1,R1,#1			; decrement the counter
			BNE		PutNumHex_BetweenLoops	; if the counter is not 0, do it again

			POP		{R0-R3, PC}			; pop the values given initially, exit the loop
			ENDP						; end subroutine
;**************************************************
UART0_IRQHandler	PROC	{R0-R14}
;**************************************************
; Handles interrupts from the UART0.
; INPUT: 	N/A
; OUTPUT:	N/A
; MODIFY:	PSR
;**************************************************
			CPSID 	I							; Mask interrupts
			PUSH	{R0-R5,LR}					; ISR has nested subroutines; push LR
			LDR		R1,=UART0_BASE				; Load first UART0 address to R1
			MOVS	R2,#UART0_C2_TIE_MASK		; Load TIE mask into R2
			LDRB	R3,[R1,#UART0_S1_OFFSET]	; Load UART0 S1 into R3
			LDRB	R4,[R1,#UART0_C2_OFFSET]	; Load UART0 C2 into R4
			TST		R4,R2						; Check if Transmit Interrupts are on.
			BEQ		UART0_ISR_Back				; If they aren't, skip transmit.
			MOVS	R2,#UART0_S1_TDRE_MASK		; If they are, load the TDRE mask
			TST		R3,R2						; Check for TDRE
			BNE		UART0_ISR_Transmit			; And transmit if there's a character to send.
UART0_ISR_Back									; Come here after transmit check.
			MOVS	R2,#UART0_S1_RDRF_MASK		; Load RDRF mask into R2.
			TST		R3,R2						; Check for RDRF
			BNE		UART0_ISR_Receive			; And receive if there's a new character input
UART0_ISR_End									; Come here after receive check.
			CPSIE	I							; Unmask interrupts
			POP		{R0-R5,PC}					; Return
			
UART0_ISR_Transmit								; Come here to transmit a character.
			PUSH	{R1}						; Push the UART0 address to the stack.
			LDR		R1,=TransmitQueueRecord		; Load the transmit queue record to R1
			BL		Dequeue						; Dequeue a character to R0.
			POP		{R1}						; Pop the UART0 address back onto R1.
			BCS		UART0_ISR_Transmit_Fail		; If the C bit is set, the transmit failed; handle accordingly.
			STRB	R0,[R1,#UART0_D_OFFSET]		; Otherwise, transmit the given character.
			B		UART0_ISR_Back				; End transmit section
			
UART0_ISR_Receive								; Come here to receive a character
			LDRB	R0,[R1,#UART0_D_OFFSET]		; Load the character into R0
			LDR		R1,=ReceiveQueueRecord		; Load the receive queue record address into R1
			BL		Enqueue						; Enqueue the new character; fails if queue is full
			B		UART0_ISR_End				; End receive section

UART0_ISR_Transmit_Fail							; come here to handle failed transmission
			MOVS	R0,#UART0_C2_T_RI			; Load the transmit interrupt disabling mask into R0
			STRB	R0,[R1,#UART0_C2_OFFSET]	; Store the altered UART0_C2 register.
			B		UART0_ISR_Back				; End transmit section
			
			ENDP
;>>>>>   end subroutine code <<<<<
;**********************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
			ALIGN
TransmitQueueBuffer	SPACE	TRBUF_SZ
TransmitQueueRecord	SPACE	Q_REC_SZ
			ALIGN
ReceiveQueueBuffer	SPACE	RCBUF_SZ
ReceiveQueueRecord	SPACE	Q_REC_SZ
			ALIGN
;>>>>>   end variables here <<<<<
            END

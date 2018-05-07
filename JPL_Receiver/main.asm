;-----------------------------------------------------------------------------
; Assembly main line
;-----------------------------------------------------------------------------

include "m8c.inc"       ; part specific constants and macros
include "memory.inc"    ; Constants & macros for SMM/LMM and Compiler
include "PSoCAPI.inc"   ; PSoC API definitions for all User Modules
include "nrf.inc"		; Symbolic names for nRF's commands and registers addresses

export _main
export rxrdy_flag

;; GPIO macros
macro SetCE_High
	or		[Port_0_Data_SHADE], 2h				; OR CE pin with shadow reg
	mov		A, [Port_0_Data_SHADE]				; Load shadow reg in A
	mov		reg[CE_Data_ADDR], A				; Store data in port 0
endm

macro SetCE_Low
	or		[Port_0_Data_SHADE], ~2h			; OR not-CE with shadow reg
	mov		A, [Port_0_Data_SHADE]				; Load shadow reg in A
	mov		reg[CE_Data_ADDR], A				; Store data in port 0
endm

macro SetCSN_High
	or		[Port_0_Data_SHADE], 4h				; OR CSN pin with shadow reg
	mov		A, [Port_0_Data_SHADE]				; Load shadow reg in A
	mov		reg[CSN_Data_ADDR], A				; Store data in port 0
endm

macro SetCSN_Low
	and		[Port_0_Data_SHADE], ~4h			; OR not-CSN with shadow reg
	mov		A, [Port_0_Data_SHADE]				; Load shadow reg in A
	mov		reg[CSN_Data_ADDR], A				; Store data in port 0
endm

;; RAM variables 
area bss(ram, rel)

rxrdy_flag: blk 1	; interrupt flag

pipe0_data: blk 10	; Allocate 10 bytes for each transmitter 
pipe1_data: blk 10	
pipe2_data: blk 10
pipe3_data: blk 10
pipe4_data: blk 10
pipe5_data: blk 10

pointer: blk 1		; Byte used as a pointer for Source/Destination Indirect Post Increment addressing
					
row1Index: blk 1	; indexes to alternate values on LCD
row2Index: blk 1	; 

temp: blk 1			; Temporary storage

;; ROM constants
area table(rom,rel)
pipes_table:
	DB pipe0_data, pipe1_data, pipe2_data, pipe3_data, pipe4_data, pipe5_data	; Pointers to pipes data
area text(rom, rel)
.LITERAL 
default_string:
asciz "N/A"
.ENDLITERAL 

;############################## MAIN PROGRAM ###############################
_main:
;-------------------------------- RAM INIT ---------------------------------
	
	mov [row1Index], 0	; init display indexes to the first 2 elements
	mov [row2Index], 1
	mov [rxrdy_flag], 0	; init rxrdy flag to 0

; Init the 6 pipes buffer to a default value ("NA")

	mov A, 0	; Push pipe buffer index on the stack (init to 0)
	push A
.init_buffers:
	pop X				; Pop pipe index to X
	mov A, X			; Transfer it to A
	cmp A, 0x6			; End when the default string is copied to all 6 buffers
	jz .end_init_buff
	index pipes_table	; Get next pipe buffer address
	mov [pointer], A	; Store it to the pointer
	inc X				; Increase pipe index
	push X				; Preserve it on the stack
	mov X, 0			; Init x to 0, now the character index
.init_pipe_buff:
	mov A, X				; Transfer character index to A
	index default_string	; Get current character from the default string
	mvi [pointer], A		; Store it to the buffer, increasing the pointer
	jz .init_buffers		; End on null character and init next buffer
	inc X					; Increase character index
	jmp .init_pipe_buff		; write next character
.end_init_buff:

	
;------------------------------ HARDWARE SETUP-----------------------------
   
; Interrupts 
	or reg[PRT0IE], 0x08 						; Enable int on IRQ pin
	M8C_EnableIntMask INT_MSK0, INT_MSK0_GPIO 	; Enable GPIO int
	M8C_EnableGInt 								; Enable Global Interrupts

; SPI init 
	mov A,  SPIM_SPIM_MODE_0 | SPIM_SPIM_MSB_FIRST	; Leading edge latch, MSB first
	lcall SPIM_Start								; Start SPI module

; LCD 
	lcall LCD_Start 			; Init and start the LCD module

; Sleep Timer 
	lcall SleepTimer_EnableInt	; Enable Sleep Timer interrupt
	lcall SleepTimer_Start		; Start Sleep Timer
	mov A, 0x10
	lcall SleepTimer_SetTimer	; Set it to count 2 secs
	
; Nrf Module init 
	mov A, 0x0A					; Give 100 ms to the nRF module for power on reset
	lcall Delay10msTimes
	call InitNrfRx				; Configure and start the nRF in RX mode
	
	jmp .update_display			; start the program with display update, then go through main loop
	

;------------------------------- MAIN LOOP -----------------------------------	

.main_loop:
	mov A, [rxrdy_flag] 		; Check RX Ready status
	jnz .get_rx_data			; Go get new data if available
	lcall SleepTimer_bGetTimer	; Get the Sleep Timer count
	cmp A, 0	    			; Check if it expired
	jnz .main_loop				; If not, jump back to beginning of main loop

	;----- Increase indexes to alternate values on LCD -----
.change_display_idx:
	inc [row1Index]				; Increment row index 1
	cmp [row1Index], 6			; check if out of bound (>5)
	jnz .inc_rowI2				; Skip to 2nd row index if ok
	mov [row1Index], 0			; Otherwise reset it to 0
.inc_rowI2:
	inc [row2Index]				; Increment row index 2
	cmp [row2Index], 6			; check if out of bound (>5)
	jnz .reload_timer			; Skip to reload timer if ok
	mov [row2Index], 0			; Otherwise reset it to 0
.reload_timer:
	mov A, 0x10
	lcall SleepTimer_SetTimer	; Set timer to count 2 secs
	jmp .update_display			; Jump to update display

		;----- Get data from the nRF FIFO -----
.get_rx_data:	
	mov [rxrdy_flag], 0			; Clear rx ready flag
.loop_rx_data:
	mov A, STATUS 				; Get status register content
	call NrfGetReg				; 
	and A, 0x0e					; Mask data pipe number
	and F, 0xfb 				; Clear carry 
	rrc A						; Rotate right -> get pipe number in A
	cmp A, 0x07					; Check if FIFO empty
	jz .clear_buffer			; End read payload if empty
	push A						; Store pipe number on stack
	mov A, R_RX_PL_WID			; Get payload width in bytes
	call NrfGetReg
	mov X, A				    ; Transfer payload width in X
	pop A						; Pop the pipe number back in A
	call ReadPayload			; Read the payload from nRF
	mov A, W_REGISTER | STATUS 	; Write 0x40 to config register to ACK interrupt
	mov X, MASK_RX_DR
	call NrfSetReg				; Ack interrupt
	jmp .loop_rx_data			; check if more payloads are on FIFO	
.clear_buffer:	
	call WaitBufferFree	
	SetCSN_Low
	mov A, FLUSH_RX
	lcall SPIM_SendTxData	; Send value over SPI
	call WaitBufferFree		; Wait until tx buffer is empty
	SetCSN_High				; Set chip select high (inactive)
	call Read_All_Regs
;----- Refresh the display if timer expired or if data has changed -----
.update_display:			
	mov A, 0				
	mov X, 0
	lcall LCD_Position
	mov A, [row1Index]
	call Write_To_LCD
	mov A, 1
	mov X, 0
	lcall LCD_Position
	mov A, [row2Index]
	call Write_To_LCD
    
	jmp .main_loop
	
;-------------------------------- END MAIN -----------------------------------	

;-----------------------------------------------------------------------------
;  FUNCTION NAME: InitNrfRx
;
;  DESCRIPTION:
;     Initialize and start the nRF module as a receiver
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:
;     none
;
;  RETURNS:
;	  none
;-----------------------------------------------------------------------------
InitNrfRx:
	SetCSN_High			; Set CSN pin high
	mov A, 20			; load accumulator with delay counter
	call Delay50uTimes	; Wait 1 ms 
	
	;-- Configuration registers --;
	mov A, W_REGISTER | CONFIG 	; Set write to config register
	mov X, 0x39				   	; Set register value ( Rx ready int, CRC, Rx mode)
	call NrfSetReg				; Write to SPI
	
	mov A, W_REGISTER | EN_AA		; Enable auto ack
	mov X, 0x3f						; Enable AA on all pipes
	call NrfSetReg
	
	
	mov A, W_REGISTER | EN_RXADDR	; Set write to enabled rx address register
	mov X, 0x3f						; Enable all 6 registers
	call NrfSetReg					; Write to SPI
	
	mov A, W_REGISTER | SETUP_AW	; Set write to address width register
	mov X, 0x03						; Set width to 5
	call NrfSetReg					; Write to SPI
	
	mov A, W_REGISTER | SETUP_RETR	; Set write to retransmit register
	mov X, 0x00						; disable retransmit
	call NrfSetReg					; Write to SPI
	
	mov A, W_REGISTER | RF_CH		; Set write to rf channel register
	mov X, 0x05						; Set channel to 5
	call NrfSetReg					; Write to SPI
	
	mov A, W_REGISTER | RF_SETUP	; Set write to rf setup register
	mov X, 0x20						; RF data rate to 250kbps, output power to 0dBm
	call NrfSetReg					; Write to SPI
	
	mov A, W_REGISTER | DYNPD		; Set write to dynamic payload register
	mov X, 0x3f						; Enable dynamic payload on all channels
	call NrfSetReg					; Write to SPI
	
	mov A, W_REGISTER | FEATURE		; Set write to feature register
	mov X, 0x04						; Enable dynamic payload length & ack
	call NrfSetReg					; Write to SPI
	
	;-------- Set Rx Address --------;
	mov X, SP
	mov [pointer], X 	; store sp (stack frame address) in the pointer
	mov A, 0x00			; push the 5 address bytes on stack
	push A
	mov A, 0x18
	push A
	mov A, 0xef
	push A
	mov A, 0xcd
	push A
	mov A, 0xab
	push A
	
	push X		 					; push the initial sp to repeat below without unwinding
	
	mov A, W_REGISTER | RX_ADDR_P0	; Load the write to pipe 0 rx address command in A
	mov X, 5						; load X with the number of values to send
	call SendBytes					; Send command and address over SPI
	
	pop X				; Pop sp (bottom of stack frame address) back in X
	inc [X]				; Increment the element at the bottom (nrf address LSByte)
	mov [pointer], X	; Store the bottom of stack frame address in pointer
	
	mov X, 5						; Load X with the number of values to send
	mov A, W_REGISTER | RX_ADDR_P1 	; Load the write to pipe 1 rx address command in A
	call SendBytes 					; Send command and address over SPI
	add SP, 0xFB					; substract 5 to stack pointer / remove 5 bytes address
	
	mov A, W_REGISTER | RX_ADDR_P2	; Load the write to pipe 2 rx address command in A
	mov X, 0x02						; Set value to 0x02
	call NrfSetReg					; Write to nrf register
	
	mov A, W_REGISTER | RX_ADDR_P3	; Load the write to pipe 3 rx address command in A
	mov X, 0x03						; Set value to 0x03
	call NrfSetReg					; Write to nrf register
	
	mov A, W_REGISTER | RX_ADDR_P4	; Load the write to pipe 4 rx address command in A
	mov X, 0x04						; Set value to 0x04
	call NrfSetReg					; Write to nrf register
	
	mov A, W_REGISTER | RX_ADDR_P5	; Load the write to pipe 5 rx address command in A
	mov X, 0x05						; Set value to 0x05
	call NrfSetReg					; Write to nrf register
	
	;-------- Power on --------;
	mov A, W_REGISTER | CONFIG
	mov X, 0x3b						; RX Ready int enabled, CRC enabled, power on, rx mode
	call NrfSetReg	
	
	SetCE_High						; Set CE high - Waiting for rx	
	
	; Testing - Read registers:
	call Read_All_Regs
	ret 							; End configs
	
;-----------------------------------------------------------------------------
;  FUNCTION NAME: NrfSetReg
;
;  DESCRIPTION:
;     Write a byte to the specified nRF module register 
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:
;     Register address / command in A
;	  Value in X
;
;  RETURNS:
;	  none
;-----------------------------------------------------------------------------
NrfSetReg:
	push X					; Preserve registers
	push A
	SetCSN_Low				; Set chip select low (active)
	call WaitBufferFree		; Wait until tx buffer is empty
	pop A					; Restore command in A
	lcall SPIM_SendTxData	; Send value over SPI
	call WaitBufferFree		; Wait until tx buffer is empty
	pop A					; Restore nRF register value in A
	lcall SPIM_SendTxData	; Send value over SPI
	call WaitBufferFree		; Wait until tx buffer is empty
	SetCSN_High				; Set chip select high (inactive)
	ret
;-----------------------------------------------------------------------------
;  FUNCTION NAME: NrfGetReg
;
;  DESCRIPTION:
;     Get a one byte register value from the nRF 
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:
;     Register adress in A
;
;  RETURNS:
;	  Register value in A
;-----------------------------------------------------------------------------	
NrfGetReg:
	push A
	SetCSN_Low
	call WaitBufferFree		; Wait until tx buffer is empty
	pop A
	lcall SPIM_SendTxData	; Send value over SPI
	call WaitBufferFree		; Wait until tx buffer is empty
	mov A, 0
	lcall SPIM_SendTxData	; Send dummy byte / Receive reg value
	call WaitBufferFree		; Wait until tx buffer is empty
.wait_rx:	
	lcall  SPIM_bReadStatus	; Read SPI status
	and A, SPIM_SPIM_SPI_COMPLETE | SPIM_SPIM_RX_BUFFER_FULL ; Check if rx is completed
	jz .wait_rx				; Loop until rx buffer is full
	lcall SPIM_bReadRxData	; Store content of rx buffer in A
	push A					; Store A on stack
	SetCSN_High				; Set Chip select inactive 
	pop A					; Restore RX value in A
	ret					
	
;-----------------------------------------------------------------------------
;  FUNCTION NAME: SendBytes
;
;  DESCRIPTION:
;     Send multiple bytes to the nRF
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:
;     -Destination Command/Register in A
;	  -Number of bytes to send in X (excluding command)
;	  -Pointer to the first element in ram[pointer]
;  RETURNS:
;	  none	
;-----------------------------------------------------------------------------
SendBytes:
	mov [temp], X			; Store bytes count in temp
	push A					; Store command on stack
	SetCSN_Low				; Set nrf chip select signal low (active)
	call WaitBufferFree		; Wait until TX buffer is empty
	pop A					; Restore Command in Acc
	lcall SPIM_SendTxData	; Send the Command/register address
.byteToSend:
	call WaitBufferFree		; Wait until TX buffer is empty
	mvi A, [pointer]		; Load next value (on the stack), increment pointer
	lcall SPIM_SendTxData	; Send byte over SPI
	dec [temp]				; Decrement counter
	jnz .byteToSend			; If counter == 0, done
	SetCSN_High				; Set nrf chip select signal high (inactive)
	ret
	
;-----------------------------------------------------------------------------
;  FUNCTION NAME: ReadPayload
;
;  DESCRIPTION:
;     - Get the payload and store it in buffer for the specified pipe
;	  - If the length of the data received is greater than buffer, 
;		it discards the rest.
;	  - Get the first 9 bytes and append a null char
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:
;	  Pipe index/number in A, Length in X
;  RETURNS:
;	  none	/ Store string in corresponding buffer
;-----------------------------------------------------------------------------
ReadPayload:
	mov [temp], X		; Store data length in temp
	cmp [temp], 10		; Check if data length is greater than buffer - 1 ( CF set length < 10 )
	jnc .fix_length		; If greater than buffer, fix length
	mov X, 0			; Otherwise set number of bytes to send to 0
	jmp .length_ok		; And skip adjustments
.fix_length: 
	swap A, X			; Preserve A val, get data length in A
	sub A, 9			; Calculate the number of bytes to flush out of the FIFO after filling buffer
	swap A, X			; Swap values back, now X has the extra bytes to clear
	mov [temp], 9		; Set to read 9 bytes in the buffer
.length_ok:
	push X				; Push amount of bytes to clear at the end (or 0)
	index pipes_table	; Get target pipe buffer starting address
	mov [pointer], A	; Store buffer addr. in pointer
	call WaitBufferFree	; Make sure SPI is ready to send
	SetCSN_Low			; Set nrF Chip Select low/active
	mov A, R_RX_PAYLOAD	; Send receive payload command to nRF
	lcall SPIM_SendTxData
	call WaitBufferFree	; Wait until SPI is complete
.get_pl_byte:
	mov A, 0x00				; Load dummy data in A
	lcall SPIM_SendTxData	; Clock SPI to receive a payload byte	
.wait_pl_rx:	
	lcall  SPIM_bReadStatus	; Read SPI status
	and A, SPIM_SPIM_SPI_COMPLETE | SPIM_SPIM_RX_BUFFER_FULL ; Check if SPI Rx is completed
	jz .wait_pl_rx			; Wait until Rx is done
	lcall SPIM_bReadRxData	; Read data in A
	mvi [pointer], A		; Store byte in pipe buffer, increment index
	dec [temp] 				; Decrement count
	jnz .get_pl_byte		; Get other bytes until count is 0.
	pop A					; Pop number of bytes to clear in A
	jz .end_pl_rx			; End if extra bytes = 0
	mov [temp], A			; Store in temp
.discard_bytes:
	mov A, 0x00				; Load dummy data in A
	lcall SPIM_SendTxData	; Clock SPI to receive a payload byte	
	call WaitBufferFree		; Wait until tx is done
	dec [temp]				; decrease count
	jnz .discard_bytes		; repeat until count is zero
.end_pl_rx:
	SetCSN_High				; Set nRF Chip Select high
	mov A, 0				; Load A with null char
	mvi [pointer], A		; Store it at the end of the buffer
	ret
;-----------------------------------------------------------------------------
;  FUNCTION NAME: WaitBufferFree
;
;  DESCRIPTION:
;     Wait until the SPI buffer is clear to send
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:
;	  none
;  RETURNS:
;	  none	
;-----------------------------------------------------------------------------
WaitBufferFree:
	lcall SPIM_bReadStatus				; Fetch SPI Status
	and A, SPIM_SPIM_TX_BUFFER_EMPTY 	; Check if buffer is empty
	jz WaitBufferFree					; Loop until it is
	ret									; Return from subroutine
	
;-----------------------------------------------------------------------------
;  FUNCTION NAME: Write_To_LCD
;
;  DESCRIPTION:
;     Write a string with a static format on LCD
;     "Tx<pipe index>: <temperature> C"
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:
;	  Pipe Index in A
;  RETURNS:
;	  none
;-----------------------------------------------------------------------------	
Write_To_LCD:
	mov [temp], A 		; copy the pipe index to temp
	mov X, sp			; store current sp to X
	mov A, 'T'			; Push the string on the stack
	push A
	mov A, 'x'
	push A
	mov A, [temp]		; Load A with pipe index
	add A, 0x30			; Add 0x30 to convert to ascii
	push A
	mov A, ':'
	push A
	mov A, ' '
	push A	
	mov A, [temp]		; Load A with pipe index
	index pipes_table	; Load the address of the buffer in A
	mov [pointer], A	; copy the address to pointer 
.copy_to_stack:
	mvi A, [pointer]	; Load A with indexed character, increase index
	jz .end_copy		; stop at null character, do not push it on stack
	push A				; push character on the stack
	jmp .copy_to_stack	; repeat until null character
.end_copy:
	mov A, ' '			; Push the rest of the string on stack (" C    ")
	push A
	mov A, 'C'
	push A
	mov A, ' '			; Adding spaces to avoid "ghost" characters from previous prints \
	push A				; if the string is smaller than the last
	mov A, ' '
	push A
	mov A, ' '
	push A
	mov A, 0			; Push null character on stack
	push A
	push X 				; Preserve original SP
	
	mov A, 0			; Load A with str pointer MSByte (256 bytes ram...)
	lcall LCD_PrString	; Call LCD PrString, X already has str pointer LSByte (original SP)
	
	pop A		; pop SP back in A
	swap A, sp	; Restore SP / get rid of string
	ret
	
Read_All_Regs:	; Get a register dump on the logic analyzer
	mov A, 0
.read_loop:	
	push a
	call NrfGetReg
	pop A
	inc A
	cmp A, 0x18
	jnz .read_loop
	ret
	
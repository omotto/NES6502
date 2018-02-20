#include <stdio.h>
#include <stdint.h>

// ----------------------------------------------------------------------------------------------
// APU (Audio Processor Unit)
// ----------------------------------------------------------------------------------------------

// -- PULSE 1
#define APU_PULSE1_CONTROL_REGISTER			0x4000 // DDLC VVVV	Duty (D), envelope loop / length counter halt (L), constant volume (C), volume/envelope (V)
#define APU_PULSE1_RAMP_CONTROL_REGISTER	0x4001 // EPPP NSSS	Sweep unit: enabled (E), period (P), negate (N), shift (S)
#define APU_PULSE1_FINE_TUNE_REGISTER		0x4002 // TTTT TTTT	Timer low (T)
#define APU_PULSE1_COARSE_TUNE_REGISTER		0x4003 // LLLL LTTT	Length counter load (L), timer high (T)
// -- PULSE 2
#define APU_PULSE2_CONTROL_REGISTER			0x4004 // DDLC VVVV	Duty (D), envelope loop / length counter halt (L), constant volume (C), volume/envelope (V)
#define APU_PULSE2_RAMP_CONTROL_REGISTER	0x4005 // EPPP NSSS	Sweep unit: enabled (E), period (P), negate (N), shift (S)
#define APU_PULSE2_FINE_TUNE_REGISTER		0x4006 // TTTT TTTT	Timer low (T)
#define APU_PULSE2_COARSE_TUNE_REGISTER		0x4007 // LLLL LTTT	Length counter load (L), timer high (T)
// -- TRIANGLE
#define APU_TRIANGLE_CONTROL_REGISTER1		0x4008 // CRRR RRRR	Length counter halt / linear counter control (C), linear counter load (R)
#define APU_TRIANGLE_UNUSED					0x4009 // ---- ----	Unused
#define APU_TRIANGLE_FREQUENCY_REGISTER1	0x400A // TTTT TTTT	Timer low (T)
#define APU_TRIANGLE_FREQUENCY_REGISTER2	0x400B // LLLL LTTT	Length counter load (L), timer high (T)
// -- NOISE
#define APU_NOISE_CONTROL_REGISTER 			0x400C // --LC VVVV	Envelope loop / length counter halt (L), constant volume (C), volume/envelope (V)
#define APU_NOISE_UNUSED					0x400D // ---- ----	Unused
#define APU_NOISE_FREQUENCY_REGISTER1		0x400E // L--- PPPP	Loop noise (L), noise period (P)
#define APU_NOISE_FREQUENCY_REGISTER2		0x400F // LLLL L---	Length counter load (L)
// -- DELTA MODULATION CONTROL
#define APU_DMC_CONTROL_REGISTER			0x4010 // IL-- RRRR	IRQ enable (I), loop (L), frequency (R)
#define APU_DMC_DA_REGISTER					0x4011 // -DDD DDDD	Load counter (D)
#define APU_DMC_ADDRESS_REGISTER			0x4012 // AAAA AAAA	Sample address (A)
#define APU_DMC_DATA_LENGTH_REGISTER		0x4013 // LLLL LLLL	Sample length (L)
// -- DMA
#define APU_DMA_SPRITE_REGISTER				0x4014
// -- APU Sound
#define APU_RD_SOUND						0x4015 // IF-D NT21	DMC interrupt (I), frame interrupt (F), DMC active (D), length counter > 0 (N/T/2/1)
#define APU_WR_SOUND						0x4015 // ---D NT21	Enable DMC (D), noise (N), triangle (T), and pulse channels (2/1)
// -- Frame Counter
#define APU_FRAME_COUNTER					0x4017 // MI-- ----	Mode (M, 0 = 4-step, 1 = 5-step), IRQ inhibit flag (I)

#define APU_PULSE_DUTY						0xC0
#define APU_PULSE_LOOP_ENVELOPE				0x20
#define APU_PULSE_LENGTH_ENABLE				0x20
#define APU_PULSE_CONSTANT_VOLUME			0x10
#define APU_PULSE_ENVELOPE_PERIOD			0x0F
#define APU_PULSE_ENVELOPE_VOLUME			0x0F
#define APU_PULSE_SWEEP_UNIT_ENABLE			0x80
#define APU_PULSE_PERIOD					0x70
#define APU_PULSE_NEGATIVE					0x08
#define APU_PULSE_SHIFT_COUNT				0x07
#define APU_PULSE_TIMER_LOW					0xFF
#define APU_PULSE_LENGTH_COUNTER			0xF8
#define APU_PULSE_TIMRE_HIGH				0x07


// ----------------------------------------------------------------------------------------------
/*
NES CPU (NTSC)
RP2A03G / RP2A03H
               _____   _____
              |     \_/     |
       SND1 - | 01       40 | - +5V (Vcc)
       SND2 - | 02       39 | - STROBE (joypad)
       /RST - | 03       38 | - EXT 44
         A0 - | 04       37 | - EXT 45
         A1 - | 05       36 | - /OE (joypad 0)
         A2 - | 06       35 | - /OE (joypad 1)
         A3 - | 07       34 | - R/W
         A4 - | 08       33 | - /NMI
         A5 - | 09       32 | - /IRQ
         A6 - | 10       31 | - M2
         A7 - | 11       30 | - TST
         A8 - | 12       29 | - CLK
         A9 - | 13       28 | - D0
        A10 - | 14       27 | - D1
        A11 - | 15       26 | - D2
        A12 - | 16       25 | - D3
        A13 - | 17       24 | - D4
        A14 - | 18       23 | - D5
        A15 - | 19       22 | - D6
        GND - | 20       21 | - D7
              |_____________|

Considerations:
---------------
- CLK : 21.47727 MHz (NTSC) or 26.6017 MHz (PAL) clock input. Internally, this clock is divided by 12 (NTSC 2A03) or 16 (PAL 2A07) 
  to feed the 6502's clock input f0, which is in turn inverted to form f1, which is then inverted to form f2. f1 is high during the 
  first phase (half-cycle) of each CPU cycle, while f2 is high during the second phase.
- SND1 : Audio out pin (is the output for square waves 1 & 2).
- SND2 : Audio out pin (is the output for triangle, noise, and DMC).
- Axx and Dx : Address and data bus, respectively. Axx holds the target address during the entire read/write cycle. For reads, the 
  value is read from Dx during f2. For writes, the value appears on Dx during f2 (and no sooner).
- OUT0..OUT2 : Output pins used by the controllers ($4016 output latch bits 0-2). These 3 pins are connected to either the NES or 
  Famicom's expansion port, and OUT0 is additionally used as the "strobe" signal (OUT) on both controller ports.
- /OE1 and /OE2 : Controller ports (for controller #1 and #2 respectively). Each enable the output of their respective controller, if 
  present.
- R/W : Read/write signal, which is used to indicate operations of the same names. Low is write. R/W stays high/low during the entire 
  read/write cycle.
- /NMI : Non-maskable interrupt pin. See the 6502 manual and CPU interrupts for more details.
- /IRQ : Interrupt pin. See the 6502 manual and CPU interrupts for more details.
- M2 : Can be considered as a "signals ready" pin. It is a modified version the 6502's f2 (which roughly corresponds to the CPU input 
  clock f0) that allows for slower ROMs. CPU cycles begin at the point where M2 goes low. M2 is clocked at 1/2 the NTSC colorburst.  
  This signal is divided (by 12) internally by the CPU and coincides with the CPU's clock speed of 1.7897725 MHz.
- In the NTSC 2A03, M2 has a duty cycle of 5/8th, or 350ns/559ns. Equivalently, a CPU read (which happens during the second, high phase 
  of M2) takes 1 and 7/8th PPU cycles. The internal f2 duty cycle is exactly 1/2 (one half).
- In the PAL 2A07, the duty cycle is not known, but suspected to be 19/32.
- TST : (tentative name) Pin 30 is special: normally it is grounded in the NES, Famicom, PC10/VS. NES and other Nintendo Arcade Boards
  (Punch-Out!! and Donkey Kong 3). But if it is pulled high on the RP2A03G, extra diagnostic registers to test the sound hardware are 
  enabled from $4018 through $401A, and the joystick ports $4016 and $4017 become open bus. On older revisions of the CPU, pulling pin 
  30 high instead causes the CPU to stop execution.
- there is NO decimal mode on the 2A03
- All memory access occurs when M2 is high (/CE is low on a ROM)
- EXT 44 and EXT 45 connect to the expansion connector pins 44 and 45 respectively.
- STROBE (joypad) is connected to STA $4016 D0
- /OE (joypads) is low during reads
*/
// ----------------------------------------------------------------------------------------------

uint8_t memory[65535];

uint8_t cyclesCPU;

// instructionCycles indicates the number of cycles used by each instruction, not including conditional cycles
uint8_t instructionCycles[256] = {
	7, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 4, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 4, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	6, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 3, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 5, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
	2, 6, 2, 6, 4, 4, 4, 4, 2, 5, 2, 5, 5, 5, 5, 5,
	2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
	2, 5, 2, 5, 4, 4, 4, 4, 2, 4, 2, 4, 4, 4, 4, 4,
	2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
};

// instructionPageCycles indicates the number of cycles used by each instruction when a page is crossed
uint8_t instructionPageCycles[256] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0,
};

// instructionSizes indicates the size of each instruction in bytes
uint8_t instructionSizes[256] = {
	1, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
	3, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
	1, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
	1, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 0, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 0, 3, 0, 0,
	2, 2, 2, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 2, 1, 0, 3, 3, 3, 0,
	2, 2, 0, 0, 2, 2, 2, 0, 1, 3, 1, 0, 3, 3, 3, 0,
};

/* 
	Accumulator (A) - The A register is used for all arithmetic and logic instructions. 
*/
uint8_t A; 

/* 	
	Index Register 1 & 2 (X & Y) - Registers X and Y are used for indirect addressing 
	and also as counters/indexes. X is used by certain instructions to save/restore the 
	value of P using the stack.
*/
uint8_t X, Y; 

/* 
	Stack Pointer (SP) - Stores the least-significant byte of the top of the stack. The 
	6502’s stack is hardwired to occupy $0100 - $01ff with SP initialized to $ff at power-up. 
	If the value of SP is $84 then the top of the stack is located at $0184. The top of the 
	stack moves downward in memory as values are pushed and upward as values are popped. 
*/
uint8_t SP; 

/*
	Program Counter (PC) - The only 16-bit register on the 6502, PC points to the next 
	instruction to execute.
*/
uint16_t PC;

/* 	Processor Status (P) - The bits in P indicate the results of the last arithmetic and 
	logic instructions as well as indicate if a break/interrupt instruction has just been executed.

	Bit 0 - Carry Flag (C)
	Bit 1 - Zero Flag (Z)
	Bit 2 - Interrupt Disable (I)
	Bit 3 - Decimal Mode (D)
	Bit 4 - Break Command (B)
	Bit 5 - -UNUSED-
	Bit 6 - Overflow Flag (O)
	Bit 7 - Negative Flag (N)	
*/

#define CARRY		0
#define ZERO		1
#define INTERRUPT	2
#define DECIMAL		3
#define BREAK		4
#define UNUSED		5
#define OVERFLOW	6
#define NEGATIVE	7

uint8_t P[8];

// Memory Addressing Modes

#define accumulator					 0x00	// The Accumulator is implied as the operand, so no address needs to be specified.
#define implied						 0x01	// The operand is implied, so it does not need to be specified.
#define	immediate					 0x02	// The operand is used directly to perform the computation.
#define absolute					 0x03	// A full 16-bit address is specified and the byte at that address is used to perform the computation.
#define zero_page					 0x04	// A single byte specifies an address in the first page of memory ($00xx), also known as the zero page, and the byte at that address is used to perform the computation.
#define relative					 0x05	// The offset specified is added to the current address stored in the Program Counter (PC). Offsets can range from -128 to +127.
#define absolute_indexed_x			 0x06	// The value in X is added to the specified address for a sum address. The value at the sum address is used to perform the computation.
#define absolute_indexed_y			 0x07	// The value in Y is added to the specified address for a sum address. The value at the sum address is used to perform the computation.
#define zero_page_indexed_x 		 0x08	// The value in X is added to the specified zero page address for a sum address. The value at the sum address is used to perform the computation.
#define zero_page_indexed_y 		 0x09	// The value in Y is added to the specified zero page address for a sum address. The value at the sum address is used to perform the computation.
#define zero_page_indexed_indirect	 0x0A	// The value in X is added to the specified zero page address for a sum address. The little-endian address stored at the two-byte pair of sum address (LSB) and sum address plus one (MSB) is loaded and the value at that address is used to perform the computation.
#define zero_page_indirect_indexed_y 0x0B	// The value in Y is added to the address at the little-endian address stored at the two-byte pair of the specified address (LSB) and the specified address plus one (MSB). The value at the sum address is used to perform the computation. Indeed addressing mode actually repeats exactly the accumulator register's digits.
#define indirect					 0x0C	// 

	
// RAM Access [0x0000 ... 0x1FFF]
// PPU Access [0x2000 ... 0x3FFF]
// APU Access [0x4000 ... 0x4017]
// Cartridge  [0x4018 ... 0xFFFF]

uint8_t readByte(uint16_t address) {
	return memory[address];
}

void writeByte(uint16_t address, uint8_t value) {
	memory[address] = value;
}

uint16_t readWord(uint16_t address) {
	uint16_t lo = (uint16_t)(readByte(address));
	uint16_t hi = (uint16_t)(readByte(address + 1));
	return (hi <<8 | lo);
}	

uint16_t readWordBug(uint16_t address) {
	uint16_t lo = (uint16_t)(readByte(address));
	uint16_t hi = (uint16_t)(readByte((address & 0xFF00) | (uint16_t)(address + 1)));
	return (hi <<8 | lo);
}

uint16_t getMemoryAddress(uint8_t mem_acc_mode, uint8_t opcode) {
	uint16_t offset;
	uint16_t address;
	uint8_t pageCrossed = 0;
	switch (mem_acc_mode) {
		case absolute:						
			printf("absolute\r\n");
			address = readWord(PC + 1);			
		break;
		case absolute_indexed_x:
			printf("absolute_indexed_x\r\n");
			address = readWord(PC + 1) + (uint16_t)(X);
			pageCrossed = ((address-(uint16_t)(X))&0xFF00 != (address)&0xFF00);
		break;
		case absolute_indexed_y:
			printf("absolute_indexed_y\r\n");
			address = readWord(PC + 1) + (uint16_t)(Y);
			pageCrossed = ((address-(uint16_t)(Y))&0xFF00 != (address)&0xFF00);
		break;	
		case immediate:
			printf("immediate\r\n");
			address = PC + 1;
		break;
		case zero_page:
			printf("zero_page\r\n");
			address = (uint16_t)(readByte(PC + 1));
		break;
		case zero_page_indexed_x:
			printf("zero_page_indexed_x\r\n");			
			address = (uint16_t)(readByte(PC+1) + X);
		break;
		case zero_page_indexed_y:
			printf("zero_page_indexed_y\r\n");						
			address = (uint16_t)(readByte(PC+1) + Y);
		break;
		case relative:
			printf("relative\r\n");						
			offset = (uint16_t)(readByte(PC + 1));
			if (offset < 0x80) address = PC + 2 + offset; else address = PC + 2 + offset - 0x100;
		break;
		case zero_page_indexed_indirect:
			printf("zero_page_indexed_indirect\r\n");			
			address = readWordBug((uint16_t)(readByte(PC+1) + X));
		break;
		case indirect:
			printf("indirect\r\n");						
			address = readWordBug(readWord(PC + 1));
		break;
		case zero_page_indirect_indexed_y:
			printf("zero_page_indirect_indexed_y\r\n");	
			address = readWordBug((uint16_t)(readByte(PC+1))) + Y;
			pageCrossed = ((address-(uint16_t)(Y))&0xFF00 != (address)&0xFF00); 
		break;
		default:
			address = 0;
		break;
	}
	// Execution Cycles
	cyclesCPU = instructionCycles[opcode];
	if (pageCrossed != 0) cyclesCPU += instructionPageCycles[opcode];
	// Program Counter Increment
	PC += (uint16_t)(instructionSizes[opcode]);	
	// --
	printf("Address: %d\r\n", address);
	return address;
}

void LDA(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("LDA\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	A = readByte(address);
	// Update Flags
	if (A == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (A & 0x80) >> 7;
}	

void LDX(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("LDX\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	X = readByte(address);
	// Update Flags
	if (X == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (X & 0x80) >> 7;
}

void LDY(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("LDY\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	Y = readByte(address);
	// Update Flags
	if (Y == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (Y & 0x80) >> 7;
}

void STA(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("STA\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	writeByte(address, A); 
}

void STX(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("STX\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	writeByte(address, X); 
}

void STY(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("STY\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	writeByte(address, Y); 
}

void ADC(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("ADC\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	uint8_t value = readByte(address);
	uint8_t a = A;
	A = A + value + P[CARRY];
	// Update Flags
	if (A == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (A & 0x80) >> 7;
	if ((int16_t)(a) + (int16_t)(value) + (int16_t)(P[CARRY]) > 0x00FF) P[CARRY] = 1; else P[CARRY] = 0;
	if (((a ^ value) & 0x80 == 0) && ((a ^ A) & 0x80 != 0)) P[OVERFLOW] = 1; else P[OVERFLOW] = 0;
}

void SBC(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("SBC\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	uint8_t value = readByte(address);
	uint8_t a = A;
	A = A - value -  (1 - P[CARRY]);
	// Update Flags
	if (A == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (A & 0x80) >> 7;	
	if ((int16_t)(a) - (int16_t)(value) - (int16_t)(1 - P[CARRY]) >= 0) P[CARRY] = 1; else P[CARRY] = 0;
	if (((a ^ value) & 0x80 != 0) && ((a ^ A) & 0x80 != 0)) P[OVERFLOW] = 1; else P[OVERFLOW] = 0;
}

void INC(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("INC\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	uint8_t value = readByte(address) + 1;
	writeByte(address, value);
	// Update Flags
	if (value == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (value & 0x80) >> 7;
}

void INX(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("INX\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	X++;
	// Update Flags	
	if (X == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (X & 0x80) >> 7;	
}

void INY(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("INY\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	Y++;
	// Update Flags	
	if (Y == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (Y & 0x80) >> 7;	
}

void DEC(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("DEC\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	uint8_t value = readByte(address) - 1;
	writeByte(address, value);
	// Update Flags
	if (value == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (value & 0x80) >> 7;	
}

void DEX(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("DEX\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	X--;
	// Update Flags	
	if (X == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (X & 0x80) >> 7;	
}

void DEY(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("DEY\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	Y--;
	// Update Flags	
	if (Y == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (Y & 0x80) >> 7;		
}

void ASL(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("ASL\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	uint8_t c = P[CARRY];
	uint8_t value;	
	if (mem_acc_mode == accumulator) {
		P[CARRY] = (A >> 7) & 0x01;
		A = A << 1;
		// Update Flags		
		if (A == 0) P[ZERO] = 1; else P[ZERO] = 0;
		P[NEGATIVE] = (A & 0x80) >> 7;		
	} else {
		value = readByte(address);	
		P[CARRY] = (value >> 7) & 0x01;
		value = value << 1;
		writeByte(address, value);
		// Update Flags		
		if (value == 0) P[ZERO] = 1; else P[ZERO] = 0;
		P[NEGATIVE] = (value & 0x80) >> 7;			
	}	
}

void LSR(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("LSR\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	uint8_t c = P[CARRY];
	uint8_t value;	
	if (mem_acc_mode == accumulator) {
		P[CARRY] = A & 0x01;
		A = A >> 1;
		// Update Flags		
		if (A == 0) P[ZERO] = 1; else P[ZERO] = 0;
		P[NEGATIVE] = (A & 0x80) >> 7;			
		
	} else {
		value = readByte(address);	
		P[CARRY] = value & 0x01;
		value = value >> 1;
		writeByte(address, value);
		// Update Flags		
		if (value == 0) P[ZERO] = 1; else P[ZERO] = 0;
		P[NEGATIVE] = (value & 0x80) >> 7;			
	}
}

void ROL(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("ROL\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	uint8_t c = P[CARRY];
	uint8_t value;
	if (mem_acc_mode == accumulator) {
		P[CARRY] = (A >> 7) & 0x01;
		A = (A << 1) | c;
		// Update Flags		
		if (A == 0) P[ZERO] = 1; else P[ZERO] = 0;
		P[NEGATIVE] = (A & 0x80) >> 7;			
	} else {
		value = readByte(address);	
		P[CARRY] = (value >> 7) & 0x01;
		value = (value << 1) | c;
		writeByte(address, value);
		// Update Flags		
		if (value == 0) P[ZERO] = 1; else P[ZERO] = 0;
		P[NEGATIVE] = (value & 0x80) >> 7;			
	}
}
		
void ROR(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("ROR\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	uint8_t c = P[CARRY];
	uint8_t value;	
	if (mem_acc_mode == accumulator) {
		P[CARRY] = A & 0x01;
		A = (A >> 1) | (c << 7);
		// Update Flags		
		if (A == 0) P[ZERO] = 1; else P[ZERO] = 0;
		P[NEGATIVE] = (A & 0x80) >> 7;			
	} else {
		value = readByte(address);	
		P[CARRY] = value & 0x01;
		value = (value >> 1) | (c << 7);
		writeByte(address, value);
		// Update Flags		
		if (value == 0) P[ZERO] = 1; else P[ZERO] = 0;
		P[NEGATIVE] = (value & 0x80) >> 7;			
	}
}

void AND(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("AND\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	A = A & readByte(address);
	// Update Flags		
	if (A == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (A & 0x80) >> 7;			
}

void ORA(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("ORA\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	A = A | readByte(address);
	// Update Flags		
	if (A == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (A & 0x80) >> 7;			
}
				
void EOR(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("EOR\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	A = A ^ readByte(address);
	// Update Flags		
	if (A == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (A & 0x80) >> 7;			
}

void CMP(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("CMP\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	uint8_t value = readByte(address);
	// Update Flags	
	if (A >= value) P[CARRY] = 1; else P[CARRY] = 0; 
	if ((A-value) == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = ((A-value) & 0x80) >> 7;
}

void CPX(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("CPX\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	uint8_t value = readByte(address);
	// Update Flags	
	if (X >= value) P[CARRY] = 1; else P[CARRY] = 0; 
	if ((X-value) == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = ((X-value) & 0x80) >> 7;
}
		
void CPY(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("CPY\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	uint8_t value = readByte(address);
	// Update Flags	
	if (Y >= value) P[CARRY] = 1; else P[CARRY] = 0; 
	if ((Y-value) == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = ((Y-value) & 0x80) >> 7;
}

void BIT(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("BIT\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	uint8_t value = readByte(address);
	// Update Flags	
	if ((A & value) == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (value & 0x80) >> 7;
	P[OVERFLOW] = (value & 0x40) >> 6;
}

void BCC(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("BCC\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	if (P[CARRY] == 0) PC = address;	
}

void BCS(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("BCS\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	if (P[CARRY] != 0) PC = address;
}

void BEQ(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("BEQ\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	if (P[ZERO] != 0) PC = address;
}

void BNE(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("BNE\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	if (P[ZERO] == 0) PC = address;
}

void BMI(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("BMI\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	if (P[NEGATIVE] != 0) PC = address;
}

void BLP(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("BLP\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	if (P[NEGATIVE] == 0) PC = address;
}

void BVC(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("BVC\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	if (P[OVERFLOW] == 0) PC = address;
}

void BVS(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("BVS\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	if (P[OVERFLOW] != 0) PC = address;
}

void TAX(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("TAX\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	X = A;
	// Update Flags		
	if (X == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (X & 0x80) >> 7;		
}

void TXA(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("TXA\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	A = X;
	// Update Flags		
	if (A == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (A & 0x80) >> 7;		
}

void TAY(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("TAY\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	Y = A;
	// Update Flags		
	if (Y == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (Y & 0x80) >> 7;		
}

void TYA(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("TYA\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	A = Y;
	// Update Flags		
	if (A == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (A & 0x80) >> 7;		
}

void TSX(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("TSX\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	X = SP;
	// Update Flags		
	if (X == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (X & 0x80) >> 7;		
}

void TXS(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("TXS\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	SP = X;
}

void PHA(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("PHA\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	writeByte(0x100 | (uint16_t)(SP), A);
	SP--;
}

void PLA(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("PLA\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	SP++;
	A = readByte(0x100 | (uint16_t)(SP));
	// Update Flags		
	if (A == 0) P[ZERO] = 1; else P[ZERO] = 0;
	P[NEGATIVE] = (A & 0x80) >> 7;	
}

void PHP(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("PHP\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	uint8_t p = 0;
	p = P[CARRY] | P[ZERO]<<ZERO | P[INTERRUPT]<<INTERRUPT | P[DECIMAL]<<DECIMAL | P[BREAK]<<BREAK | P[OVERFLOW]<<OVERFLOW | P[NEGATIVE]<<NEGATIVE;
	writeByte(0x100 | (uint16_t)(SP), p);
	SP--;
}

void PLP(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("PLP\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	uint8_t p;
	SP++;
	p = readByte(0x100 | (uint16_t)(SP));
	P[CARRY] 	= p & 0x01;
	P[ZERO]		= p & 0x02;
	P[INTERRUPT]= p & 0x04;
	P[DECIMAL]	= p & 0x08;
	P[BREAK]	= p & 0x10;
	P[OVERFLOW] = p & 0x40;
	P[NEGATIVE] = p & 0x80;
}

void JMP(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("JMP\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	PC = address;
}

void JSR(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("JSR\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	uint16_t value = PC - 1;
	uint8_t hi = (uint8_t)(value >> 8);
	uint8_t lo = (uint8_t)(value & 0xFF);
	writeByte(0x100 | (uint16_t)(SP), hi);
	SP--;
	writeByte(0x100 | (uint16_t)(SP), lo);
	SP--;
	PC = address;
}

void RTS(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("RTS\r\n");	
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	SP++;
	uint16_t lo = readByte(0x100 | (uint16_t)(SP));
	SP++;
	uint16_t hi = readByte(0x100 | (uint16_t)(SP));
	uint16_t value =  (hi << 8) | lo;
	PC = value + 1;
}

void RTI(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("RTI\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	SP++;
	uint8_t p = readByte(0x100 | (uint16_t)(SP));
	P[CARRY] 	= p & 0x01;
	P[ZERO]		= p & 0x02;
	P[INTERRUPT]= p & 0x04;
	P[DECIMAL]	= p & 0x08;
	P[BREAK]	= p & 0x10;
	P[OVERFLOW] = p & 0x40;
	P[NEGATIVE] = p & 0x80;
	SP++;
	uint16_t lo = readByte(0x100 | (uint16_t)(SP));
	SP++;
	uint16_t hi = readByte(0x100 | (uint16_t)(SP));
	uint16_t value =  (hi << 8) | lo;
	PC = value;
}

void SEC(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("SEC\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	P[CARRY] = 1;
}

void SED(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("SED\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	P[DECIMAL] = 1;
}

void SEI(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("SEI\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	P[INTERRUPT] = 1;
}

void CLC(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("CLC\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	P[CARRY] = 0;
}

void CLD(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("CLD\r\n");	
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	P[DECIMAL] = 0;
}

void CLI(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("CLI\r\n");	
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	P[INTERRUPT] = 0;
}

void CLV(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("CLV\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	P[OVERFLOW] = 0;
}

void NOP(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("NOP\r\n");	
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
}

void BRK(uint8_t mem_acc_mode, uint8_t opcode) {
	printf("BRK\r\n");
	uint16_t address = getMemoryAddress(mem_acc_mode, opcode);	
	uint16_t value = PC;
	uint8_t hi = (uint8_t)(value >> 8);
	uint8_t lo = (uint8_t)(value & 0xFF);
	writeByte(0x100 | (uint16_t)(SP), hi);
	SP--;
	writeByte(0x100 | (uint16_t)(SP), lo);
	SP--;
	uint8_t p = 0;
	p = P[CARRY] | P[ZERO]<<ZERO | P[INTERRUPT]<<INTERRUPT | P[DECIMAL]<<DECIMAL | P[BREAK]<<BREAK | P[OVERFLOW]<<OVERFLOW | P[NEGATIVE]<<NEGATIVE;
	writeByte(0x100 | (uint16_t)(SP), p);
	SP--;
	PC = readWord(0xFFFE);
	P[INTERRUPT] = 1;
}

void NMI(void) {
	printf("NMI\r\n");
	uint16_t value = PC;
	uint8_t hi = (uint8_t)(value >> 8);
	uint8_t lo = (uint8_t)(value & 0xFF);
	writeByte(0x100 | (uint16_t)(SP), hi);
	SP--;
	writeByte(0x100 | (uint16_t)(SP), lo);
	SP--;
	uint8_t p = 0;
	p = P[CARRY] | P[ZERO]<<ZERO | P[INTERRUPT]<<INTERRUPT | P[DECIMAL]<<DECIMAL | P[BREAK]<<BREAK | P[OVERFLOW]<<OVERFLOW | P[NEGATIVE]<<NEGATIVE;
	writeByte(0x100 | (uint16_t)(SP), p);
	SP--;
	PC = readWord(0xFFFA);
	P[INTERRUPT] = 1;
	cyclesCPU = 7;
}

void IRQ(void) {
	printf("IRQ\r\n");
	uint16_t value = PC;
	uint8_t hi = (uint8_t)(value >> 8);
	uint8_t lo = (uint8_t)(value & 0xFF);
	writeByte(0x100 | (uint16_t)(SP), hi);
	SP--;
	writeByte(0x100 | (uint16_t)(SP), lo);
	SP--;
	uint8_t p = 0;
	p = P[CARRY] | P[ZERO]<<ZERO | P[INTERRUPT]<<INTERRUPT | P[DECIMAL]<<DECIMAL | P[BREAK]<<BREAK | P[OVERFLOW]<<OVERFLOW | P[NEGATIVE]<<NEGATIVE;
	writeByte(0x100 | (uint16_t)(SP), p);
	SP--;
	PC = readWord(0xFFFE);
	P[INTERRUPT] = 1;
	cyclesCPU = 7;
}

void execute(uint8_t opcode) {
	switch (opcode) {
		// -- Load and Store

		// Load Accumulator with Memory: LDA
		case 0xAD: LDA(absolute, opcode);						break;
		case 0xBD: LDA(absolute_indexed_x, opcode);				break;
		case 0xB9: LDA(absolute_indexed_y, opcode);				break;
		case 0xA9: LDA(immediate, opcode);						break;
		case 0xA5: LDA(zero_page, opcode);						break;
		case 0xA1: LDA(zero_page_indexed_indirect, opcode);		break;
		case 0xB5: LDA(zero_page_indexed_x, opcode);			break;
		case 0xB1: LDA(zero_page_indirect_indexed_y, opcode);	break;		
		
		// Load Index X with Memory: LDX
		case 0xAE: LDX(absolute, opcode);						break;
		case 0xBE: LDX(absolute_indexed_y, opcode);				break;
		case 0xA2: LDX(immediate, opcode);						break;
		case 0xA6: LDX(zero_page, opcode);						break;	
		case 0xB6: LDX(zero_page_indexed_y, opcode);			break;	

		// Load Index Y with Memory: LDY
		case 0xAC: LDY(absolute, opcode);						break;
		case 0xBC: LDY(absolute_indexed_x, opcode);				break;
		case 0xA0: LDY(immediate, opcode);						break;
		case 0xA4: LDY(zero_page, opcode);						break;	
		case 0xB4: LDY(zero_page_indexed_x, opcode);			break;	

		// Store Accumulator in Memory: STA
		case 0x8D: STA(absolute, opcode);						break;
		case 0x9D: STA(absolute_indexed_x, opcode);				break;
		case 0x99: STA(absolute_indexed_y, opcode);				break;
		case 0x85: STA(zero_page, opcode);						break;
		case 0x81: STA(zero_page_indexed_indirect, opcode);		break;
		case 0x95: STA(zero_page_indexed_x, opcode);			break;
		case 0x91: STA(zero_page_indirect_indexed_y, opcode);	break;		

		// Store Index X in Memory: STX
		case 0x8E: STX(absolute, opcode);						break;
		case 0x86: STX(zero_page, opcode);						break;
		case 0x96: STX(zero_page_indexed_y, opcode);			break;
		
		// Store Index Y in Memory: STY
		case 0x8C: STY(absolute, opcode);						break;
		case 0x84: STY(zero_page, opcode);						break;
		case 0x94: STY(zero_page_indexed_x, opcode);			break;
		
		// -- Arithmetic

		// Add Memory to Accumulator with Carry: ADC
		case 0x6D: ADC(absolute, opcode);						break;
		case 0x7D: ADC(absolute_indexed_x, opcode);				break;
		case 0x79: ADC(absolute_indexed_y, opcode);				break;
		case 0x69: ADC(immediate, opcode);						break;
		case 0x65: ADC(zero_page, opcode);						break;
		case 0x61: ADC(zero_page_indexed_indirect, opcode);		break;
		case 0x75: ADC(zero_page_indexed_x, opcode);			break;
		case 0x71: ADC(zero_page_indirect_indexed_y, opcode);	break;	

		// Subtract Memory from Accumulator with Borrow: SBC
		case 0xED: SBC(absolute, opcode);						break;
		case 0xFD: SBC(absolute_indexed_x, opcode);				break;
		case 0xF9: SBC(absolute_indexed_y, opcode);				break;
		case 0xE9: SBC(immediate, opcode);						break;
		case 0xE5: SBC(zero_page, opcode);						break;
		case 0xE1: SBC(zero_page_indexed_indirect, opcode);		break;
		case 0xF5: SBC(zero_page_indexed_x, opcode);			break;
		case 0xF1: SBC(zero_page_indirect_indexed_y, opcode);	break;	
		
		// -- Increment and Decrement

		// Increment Memory by One: INC
		case 0xEE: INC(absolute, opcode);						break;
		case 0xFE: INC(absolute_indexed_x, opcode);				break;
		case 0xE6: INC(zero_page, opcode);						break;
		case 0xF6: INC(zero_page_indexed_x, opcode);			break;	

		// Increment Index X by One: INX
		case 0xE8: INX(implied, opcode);						break;		
		
		// Increment Index Y by One: INY
		case 0xC8: INY(implied, opcode);						break;		

		// Decrement Memory by One: DEC
		case 0xCE: DEC(absolute, opcode);						break;
		case 0xDE: DEC(absolute_indexed_x, opcode);				break;
		case 0xC6: DEC(zero_page, opcode);						break;
		case 0xD6: DEC(zero_page_indexed_x, opcode);			break;	
		
		// Decrement Index X by One: DEX
		case 0xCA: DEX(implied, opcode);						break;		
		
		// Decrement Index Y by One: DEY
		case 0x88: DEY(implied, opcode);						break;	

		// -- Shift and Rotate
		
		// Arithmetic Shift Left One Bit: ASL
		case 0x0E: ASL(absolute, opcode);						break;
		case 0x1E: ASL(absolute_indexed_x, opcode);				break;
		case 0x0A: ASL(accumulator, opcode);					break;
		case 0x06: ASL(zero_page, opcode);						break;	
		case 0x16: ASL(zero_page_indexed_x, opcode);			break;		
		
		// Logical Shift Right One Bit: LSR
		case 0x4E: LSR(absolute, opcode);						break;
		case 0x5E: LSR(absolute_indexed_x, opcode);				break;
		case 0x4A: LSR(accumulator, opcode);					break;
		case 0x46: LSR(zero_page, opcode);						break;	
		case 0x56: LSR(zero_page_indexed_x, opcode);			break;			
		
		// Rotate Left One Bit: ROL
		case 0x2E: ROL(absolute, opcode);						break;
		case 0x3E: ROL(absolute_indexed_x, opcode);				break;
		case 0x2A: ROL(accumulator, opcode);					break;
		case 0x26: ROL(zero_page, opcode);						break;	
		case 0x36: ROL(zero_page_indexed_x, opcode);			break;		

		// Rotate Right One Bit: ROR
		case 0x6E: ROR(absolute, opcode);						break;
		case 0x7E: ROR(absolute_indexed_x, opcode);				break;
		case 0x6A: ROR(accumulator, opcode);					break;
		case 0x66: ROR(zero_page, opcode);						break;	
		case 0x76: ROR(zero_page_indexed_x, opcode);			break;			
		
		// -- Logic

		// AND Memory with Accumulator: AND
		case 0x2D: AND(absolute, opcode);						break;
		case 0x3D: AND(absolute_indexed_x, opcode);				break;
		case 0x39: AND(absolute_indexed_y, opcode);				break;
		case 0x29: AND(immediate, opcode);						break;
		case 0x25: AND(zero_page, opcode);						break;
		case 0x21: AND(zero_page_indexed_indirect, opcode);		break;
		case 0x35: AND(zero_page_indexed_x, opcode);			break;
		case 0x31: AND(zero_page_indirect_indexed_y, opcode);	break;		
				
		// OR Memory with Accumulator: ORA
		case 0x0D: ORA(absolute, opcode);						break;
		case 0x1D: ORA(absolute_indexed_x, opcode);				break;
		case 0x19: ORA(absolute_indexed_y, opcode);				break;
		case 0x09: ORA(immediate, opcode);						break;
		case 0x05: ORA(zero_page, opcode);						break;
		case 0x01: ORA(zero_page_indexed_indirect, opcode);		break;
		case 0x15: ORA(zero_page_indexed_x, opcode);			break;
		case 0x11: ORA(zero_page_indirect_indexed_y, opcode);	break;	
		
		// Exclusive-OR Memory with Accumulator: EOR
		case 0x4D: EOR(absolute, opcode);						break;
		case 0x5D: EOR(absolute_indexed_x, opcode);				break;
		case 0x59: EOR(absolute_indexed_y, opcode);				break;
		case 0x49: EOR(immediate, opcode);						break;
		case 0x45: EOR(zero_page, opcode);						break;
		case 0x41: EOR(zero_page_indexed_indirect, opcode);		break;
		case 0x55: EOR(zero_page_indexed_x, opcode);			break;
		case 0x51: EOR(zero_page_indirect_indexed_y, opcode);	break;			
		
		/*
			Compare and Test Bit
			For all Compare instructions:
			Condition			N	Z	C
			Register < Memory	1	0	0
			Register = Memory	0	1	1
			Register > Memory	0	0	1
		*/
				
		// Compare Memory and Accumulator: CMP
		case 0xCD: CMP(absolute, opcode);						break;
		case 0xDD: CMP(absolute_indexed_x, opcode);				break;
		case 0xD9: CMP(absolute_indexed_y, opcode);				break;
		case 0xC9: CMP(immediate, opcode);						break;
		case 0xC5: CMP(zero_page, opcode);						break;
		case 0xC1: CMP(zero_page_indexed_indirect, opcode);		break;
		case 0xD5: CMP(zero_page_indexed_x, opcode);			break;
		case 0xD1: CMP(zero_page_indirect_indexed_y, opcode);	break;	
		
		// Compare Memory and Index X: CPX
		case 0xEC: CPX(absolute, opcode);						break;
		case 0xE0: CPX(immediate, opcode);						break;
		case 0xE4: CPX(zero_page, opcode);						break;		
		
		// Compare Memory with Index Y: CPY
		case 0xCC: CPY(absolute, opcode);						break;
		case 0xC0: CPY(immediate, opcode);						break;
		case 0xC4: CPY(zero_page, opcode);						break;
		
		// Test Bits in Memory with Accumulator: BIT
		case 0x2C: BIT(absolute, opcode);						break;
		case 0x89: BIT(immediate, opcode);						break;
		case 0x24: BIT(zero_page, opcode);						break;

		// -- Branch
		
		// Branch on Carry Clear: BCC
		case 0x90: BCC(relative, opcode);						break;	
		
		// Branch on Carry Set: BCS
		case 0xB0: BCS(relative, opcode);						break;

		// Branch on Result Zero: BEQ
		case 0xF0: BEQ(relative, opcode);						break;			
				
		// Branch on Result Minus: BMI
		case 0x30: BMI(relative, opcode);						break;		
		
		// Branch on Result not Zero: BNE
		case 0xD0: BNE(relative, opcode);						break;	

		// Branch on Result Plus: BPL
		case 0x10: BLP(relative, opcode);						break;		
		
		// Branch on Overflow Clear: BVC
		case 0x50: BVC(relative, opcode);						break;		

		// Branch on Overflow Set: BVS
		case 0x70: BVS(relative, opcode);						break;	
		
		// -- Transfer

		// Transfer Accumulator to Index X: TAX
		case 0xAA: TAX(implied, opcode);						break;	

		// Transfer Index X to Accumulator: TXA
		case 0x8A: TXA(implied, opcode);						break;
		
		// Transfer Accumulator to Index Y: TAY
		case 0xA8: TAY(implied, opcode);						break;	
		
		// Transfer Index Y to Accumulator: TYA
		case 0x98: TYA(implied, opcode);						break;	
	
		// Transfer Stack Pointer to Index X: TSX
		case 0xBA: TSX(implied, opcode);						break;	
		
		// Transfer Index X to Stack Pointer: TXS
		case 0x9A: TXS(implied, opcode);						break;	

		// -- Stack
		
		// Push Accumulator on Stack: PHA
		case 0x48: PHA(implied, opcode);						break;		
		
		// Pull Accumulator from Stack: PLA
		case 0x68: PLA(implied, opcode);						break;	

		// Push Processor Status on Stack: PHP
		case 0x08: PHP(implied, opcode);						break;
		
		// Pull Processor Status from Stack: PLP
		case 0x28: PLP(implied, opcode);						break;
		
		// -- Subroutines and Jump
		
		// Jump to New Location: JMP
		case 0x4C: JMP(absolute, opcode);						break;	
		case 0x6C: JMP(indirect, opcode);						break;	

		// Jump to New Location Saving Return Address: JSR
		case 0x20: JSR(absolute, opcode);						break;

		// Return from Subroutine: RTS
		case 0x60: RTS(implied, opcode);						break;	
		
		// Return from Interrupt: RTI
		case 0x40: RTI(implied, opcode);						break;		
		
		// -- Set and Clear
		
		// Set Carry Flag: SEC
		case 0x38: SEC(implied, opcode);						break;
		
		// Set Decimal Mode: SED
		case 0xF8: SED(implied, opcode);						break;			
		
		// Set Interrupt Disable Status: SEI
		case 0x78: SEI(implied, opcode);						break;	
		
		// Clear Carry Flag: CLC
		case 0x18: CLC(implied, opcode);						break;		

		// Clear Decimal Mode: CLD
		case 0xD8: CLD(implied, opcode);						break;		
		
		// Clear Interrupt Disable Status: CLI
		case 0x58: CLI(implied, opcode);						break;	

		// Clear Overflow Flag: CLV	
		case 0xB8: CLV(implied, opcode);						break;

		// -- Miscellaneous

		// No Operation: NOP
		case 0xEA: NOP(implied, opcode);						break;		
			
		// Break: BRK
		case 0x00: BRK(implied, opcode);						break;

		default: break;
	}

	printf("jump: %d\r\n", instructionSizes[opcode]);
	printf("PC: %d - A: %d - X: %d - Y: %d - SP: %d - CYCLES: %d \r\n", PC, A, X, Y, SP, cyclesCPU);
	printf("C: %d - Z: %d - I: %d - D: %d - B: %d - V: %d - N: %d \r\n", P[CARRY], P[ZERO], P[INTERRUPT], P[DECIMAL], P[BREAK], P[OVERFLOW], P[NEGATIVE]);
}

int main(void) {
	SP = 0xFD;

	A = 0;
    X = 0;
    Y = 0;

    P[CARRY] = 0;
	P[ZERO] = 0;
	P[INTERRUPT] = 0;
	P[DECIMAL] = 0;
	P[BREAK] = 0;
	P[OVERFLOW] = 0;
	P[NEGATIVE] = 0;

	FILE *fp = fopen("mos6502.bin","rb");
	fread(memory, sizeof(uint8_t), 65535, fp);
	fclose(fp);
	
	PC = readWord(0xFFFC);
	
	PC = 0;
	
	
	printf("PC: %d\r\n", PC);
	
	while(1) {
		execute(readByte(PC));
		getch();
	}
		
	return 0;
}

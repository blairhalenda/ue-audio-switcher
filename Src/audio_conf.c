#include "audio_conf.h"
#include "dwt_stm32_delay.h"

void setDigitalPotWiper(uint8_t Address, uint8_t Register, int Value){

	uint8_t firstCommandByte = 0x00;  // Prep the firstCommandByte with 00b for writes
	uint8_t secondCommandByte = 0x00; // Empty data byte
	uint32_t tempWord = Value;
	uint8_t i2c_command[2];

	// Prep the command bytes to write to the digital potentiometer
	Register *= 16;                		// Shift the value of Register to the left by four bits
	firstCommandByte |= Register;  		// Load the register address into the firstCommandByte
	tempWord &= 0x0100;            		// Clear the top 7 bits and the lower byte of the input value to pick up the two data bits
	tempWord /= 256;               		// Shift the top byte of the input value to the right by one byte
	uint8_t tempByte = tempWord;     	// Store the top byte of the input value in a byte sized variable
	firstCommandByte |= tempByte;  		// Load the two top input data bits into the firstCommandByte
	tempWord = Value;              		// Load the input value into the tempWord
	tempWord &= 0x00FF;              	// Clear the top byte
	secondCommandByte = tempWord;  		// Store the lower byte of the input value in the secondCommandByte

	i2c_command[0] = firstCommandByte;
	i2c_command[1] = secondCommandByte;

	// Write to the digital potentiometer
	HAL_I2C_Master_Transmit(&hi2c1,Address<<1,i2c_command,2,100);
}


void fadeOut(uint8_t Address, uint8_t Register1, uint8_t Register2){
	/*
	int i = 0;
	for (i=511;i>=0;i--){
		setDigitalPotWiper(Address, Register1, i);
		setDigitalPotWiper(Address, Register2, i);
		DWT_Delay_us(i*2);
	}
	*/
	setDigitalPotWiper(Address, Register1, 0);
	setDigitalPotWiper(Address, Register2, 0);
}

void fadeIn(uint8_t Address, uint8_t Register1, uint8_t Register2){
	/*
	int i;
	for (i=0;i<=511;i++){
		setDigitalPotWiper(Address, Register1, i);
		setDigitalPotWiper(Address, Register2, i);
		DWT_Delay_us(i*2);
	}
	*/
	setDigitalPotWiper(Address, Register1, 511);
	setDigitalPotWiper(Address, Register2, 511);

}
void mutePorts(){
	setDigitalPotWiper(audio1, volatileWiper0, 0);
	setDigitalPotWiper(audio1, volatileWiper1, 0);

	setDigitalPotWiper(audio2, volatileWiper0, 0);
	setDigitalPotWiper(audio2, volatileWiper1, 0);

	setDigitalPotWiper(audio3, volatileWiper0, 0);
	setDigitalPotWiper(audio3, volatileWiper1, 0);

	setDigitalPotWiper(audio4, volatileWiper0, 0);
	setDigitalPotWiper(audio4, volatileWiper1, 0);

	setDigitalPotWiper(audio5, volatileWiper0, 0);
	setDigitalPotWiper(audio5, volatileWiper1, 0);

	setDigitalPotWiper(audio6, volatileWiper0, 0);
	setDigitalPotWiper(audio6, volatileWiper1, 0);
}

uint8_t switchPort(uint8_t currentPort, uint8_t Address){
	if(currentPort != 0x00){
		// Fade out that port
		fadeOut(currentPort,volatileWiper0,volatileWiper1);
	}
	mutePorts();
	currentPort = Address;
	// Fade in desired port
	fadeIn(currentPort,volatileWiper0,volatileWiper1);

	return(currentPort);

}

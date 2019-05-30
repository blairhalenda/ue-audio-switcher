#include "events.h"

// Button Variables
int buttonState = 0;
int currentState = -1;
int buttonPressed = 0;
int lastButtonPressed = 0;


int i;

void getButtonEvent(){
 // START OF SWITCH
	buttonPressed = readButtons(btnVal);
	buttonState = getButtonState();
	// Map buttons to events
	if(currentState != buttonState){
		currentState = buttonState;
		for (i=0; i<10; i++){
			performAction(systemLogic[currentState][i]);
		}
	}
}

void getSerialEvent(uint8_t port, uint8_t uart_command[10]){
	uint8_t command[uart_command_length+1];
	memset(command,0, uart_command_length+1);
	command[0] = port;
	strcat(command, uart_command);
	for(int i=0; i < 100; i++){
		if(strncmp(command, serialRX[i],strlen(command)) == 0){
			for (int j=0; j<10; j++){
				performAction(systemLogic[i + 100][j]);
			}
		}
	}
}

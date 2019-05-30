#include "button_conf.h"

int buttonConfidence = 0;
int buttonThreshold = 200;

int readButtons(uint32_t btnVal[3]){
	if(btnVal[0] >= 0xF0){
		return(1);
	}
	else if(btnVal[0] < 0xEC && btnVal[0] > 0xE0){
		return(2);
	}
	else if(btnVal[0] < 0xD7 && btnVal[0] > 0xCF){
		return(3);
	}
	else if(btnVal[0] < 0xC6 && btnVal[0] > 0xBF){
		return(4);
	}
	else if(btnVal[0] < 0xB8 && btnVal[0] > 0xAF){
		return(5);
	}
	else if(btnVal[0] < 0xAC && btnVal[0] > 0xA0){
		return(6);
	}

	//Button B pressed
	// FF E8 D5 C4 B6 AA
	else if(btnVal[1] >= 0xFD){
		return(7);
	}
	else if(btnVal[1] < 0xEC && btnVal[1] > 0xE0){
		return(8);
	}
	else if(btnVal[1] < 0xD7 && btnVal[1] > 0xCF){
		return(9);
	}
	else if(btnVal[1] < 0xC6 && btnVal[1] > 0xBF){
		return(10);
	}
	else if(btnVal[1] < 0xB8 && btnVal[1] > 0xAF){
		return(11);
	}
	else if(btnVal[1] < 0xAC && btnVal[1] > 0xA0){
		return(12);
	}

	//Button C pressed
	// FF E8 D5 C4 B6 AA
	else if(btnVal[2] >= 0xFD){
		return(13);
	}
	else if(btnVal[2] < 0xEC && btnVal[2] > 0xE0){
		return(14);
	}
	else if(btnVal[2] < 0xD7 && btnVal[2] > 0xCF){
		return(15);
	}
	else if(btnVal[2] < 0xC6 && btnVal[2] > 0xBF){
		return(16);
	}
	else if(btnVal[2] < 0xB8 && btnVal[2] > 0xAF){
		return(17);
	}
	else if(btnVal[2] < 0xAC && btnVal[2] > 0xA0){
		return(18);
	}

	else{
		return(0);
	}
}


int getButtonState(){
	if(buttonPressed == buttonState){
		buttonConfidence = 0;
	}
	else{
		if(buttonPressed == lastButtonPressed){
			buttonConfidence++;
			if(buttonConfidence >= buttonThreshold){
				buttonState = buttonPressed;
			}
		}
		else{
			buttonConfidence = 0;
			lastButtonPressed = buttonPressed;
		}
	}
	return(buttonState);
}

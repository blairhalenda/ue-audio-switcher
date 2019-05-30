#include "actions.h"


void performAction(int action){
	if(action < 1000){
		switch(action){
			//--------------LED COMMANDS----------------
			case 0:
				//Do nothing
				break;
			case 1:
				//SET LED BLANK
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);		//IND-Red
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);		//IND-Green
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);		//IND-Blue
				break;
			case 2:
				//SET LED RED
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);	//IND-Red
				break;
			case 3:
				//SET LED GREEN
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);		//IND-Green
				break;
			case 4:
				//SET LED BLUE
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);		//IND-Blue
				break;

			//--------------GPIO COMMANDS----------------
			case 5:
				//SET D1-D12 LOW
				HAL_GPIO_WritePin(D1_GPIO_Port,D1_Pin,GPIO_PIN_RESET);		//D1
				HAL_GPIO_WritePin(D2_GPIO_Port,D2_Pin,GPIO_PIN_RESET);		//D2
				HAL_GPIO_WritePin(D3_GPIO_Port,D3_Pin,GPIO_PIN_RESET);		//D3
				HAL_GPIO_WritePin(D4_GPIO_Port,D4_Pin,GPIO_PIN_RESET);		//D4
				HAL_GPIO_WritePin(D5_GPIO_Port,D5_Pin,GPIO_PIN_RESET);		//D5
				HAL_GPIO_WritePin(D6_GPIO_Port,D6_Pin,GPIO_PIN_RESET);		//D6
				HAL_GPIO_WritePin(D7_GPIO_Port,D7_Pin,GPIO_PIN_RESET);		//D7
				HAL_GPIO_WritePin(D8_GPIO_Port,D8_Pin,GPIO_PIN_RESET);		//D8
				HAL_GPIO_WritePin(D9_GPIO_Port,D9_Pin,GPIO_PIN_RESET);		//D9
				HAL_GPIO_WritePin(D10_GPIO_Port,D10_Pin,GPIO_PIN_RESET);	//D10
				HAL_GPIO_WritePin(D11_GPIO_Port,D11_Pin,GPIO_PIN_RESET);	//D11
				HAL_GPIO_WritePin(D12_GPIO_Port,D12_Pin,GPIO_PIN_RESET);	//D12
				break;
			case 6:
				//SET D1-D3 HIGH
				HAL_GPIO_WritePin(D1_GPIO_Port,D1_Pin,GPIO_PIN_SET);	//D1
				HAL_GPIO_WritePin(D2_GPIO_Port,D2_Pin,GPIO_PIN_SET);	//D2
				HAL_GPIO_WritePin(D3_GPIO_Port,D3_Pin,GPIO_PIN_SET);	//D3
				break;
			case 7:
				//SET D4-D6 HIGH
				HAL_GPIO_WritePin(D4_GPIO_Port,D4_Pin,GPIO_PIN_SET);	//D4
				HAL_GPIO_WritePin(D5_GPIO_Port,D5_Pin,GPIO_PIN_SET);	//D5
				HAL_GPIO_WritePin(D6_GPIO_Port,D6_Pin,GPIO_PIN_SET);	//D6
				break;
			case 8:
				HAL_GPIO_WritePin(D7_GPIO_Port,D7_Pin,GPIO_PIN_SET);	//D7
				HAL_GPIO_WritePin(D8_GPIO_Port,D8_Pin,GPIO_PIN_SET);	//D8
				HAL_GPIO_WritePin(D9_GPIO_Port,D9_Pin,GPIO_PIN_SET);	//D9

				break;
			case 9:
				HAL_GPIO_WritePin(D10_GPIO_Port,D10_Pin,GPIO_PIN_SET);	//D10
				HAL_GPIO_WritePin(D11_GPIO_Port,D11_Pin,GPIO_PIN_SET);	//D11
				HAL_GPIO_WritePin(D12_GPIO_Port,D12_Pin,GPIO_PIN_SET);	//D12
				break;

			// AUDIO COMMANDS
			// Mute all ports
			case 10:
				mutePorts();
				break;
			// Set to audio 1
			case 11:
				switchPort(currentPort, audio1);
				break;
			// Set to audio 2
			case 12:
				switchPort(currentPort, audio2);
				break;
			// Set to audio 3
			case 13:
				switchPort(currentPort, audio3);
				break;
			// Set to audio 4
			case 14:
				switchPort(currentPort, audio4);
				break;
			// Set to audio 5
			case 15:
				switchPort(currentPort, audio5);
				break;
			// Set to audio 6
			case 16:
				switchPort(currentPort, audio6);
				break;
			case 17:
				HAL_GPIO_WritePin(RLY_GPIO_Port,RLY_Pin,RESET);
				break;
			case 18:
				HAL_GPIO_WritePin(RLY_GPIO_Port,RLY_Pin,SET);
				break;

			default:
				break;

		}
	}
	else if(action < 1100){
		if(forwardSerial(serialTX[action-1000]) == 0){
			//HAL_Delay(100);
			//CDC_Transmit_FS("ACTION", 7);
		}
	}

}

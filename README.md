# ue-audio-switcher
Speculative electronics system built for Ultimate Ears to connect their simple inline displays to the cloud.

## Display
![Display](/images/display.jpg)

## Electronics
![Electronics](/images/inside.jpg)

## Main Circuit Board
I designed this circuit board from scratch using Eagle from Autodesk. It uses an STM32F205RBT6 as the central microcontroller. The audio player consists of an ATmega328p connected to an SD card slot and a VS1063b MP3 decoder to convert the file into an analog signal on the output jacks. The output jacks have digital potentiometers so you can choose which output to play from without the clicking sound of a mechanical relay.

![Circuit Board](/images/circuit.jpg)


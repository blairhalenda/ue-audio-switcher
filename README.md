# ue-audio-switcher
Speculative electronics system built for Ultimate Ears to connect their simple inline displays to the cloud. The objective of this project was to detect in store how many times buttons were being pressed to hear audio from the speakers, without any dependancy on the store WiFi.

## Display
The display consisted of two speakers and buttons to hear sample music from either one. Underneath each speaker was an acrylic piece lit with RGB lighting which simulated a water ripple effect.
![Display](/images/display.jpg)

## Electronics
Inside the display is a modular electronics system which could be connected in different ways just by using the white connector cables. The data was transmitted from the display using a SIM800L cellular module and a sim card from Hologram.io.
![Electronics](/images/inside.jpg)

## Main Circuit Board
I designed this circuit board from scratch using Eagle from Autodesk. It uses an STM32F205RBT6 as the central microcontroller. The audio player consists of an ATmega328p connected to an SD card slot and a VS1063 MP3 decoder to convert the file into an analog signal on the output jacks. The output jacks have digital potentiometers so you can choose which output to play from without the clicking sound of a mechanical relay.

![Circuit Board](/images/circuit.jpg)


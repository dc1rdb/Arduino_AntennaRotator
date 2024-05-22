# Arduino_AntennaRotator
Az/El antenna rotator controller for Yaesu G-5500 DC

For EME activities, I am using a Yaesu G-5500 DC antenna rotator.
I was not not happy with the original controller, because the accuracy of the analog gauges leaves a lot to be desired and a USB interface to interact with tracking programs has to be purchased separately.
During research for alternatives, I came across this project by Viorel, YO3RAK: https://www.hackster.io/viorelracoviteanu/antenna-rotator-controller-compatible-with-tracking-software-48f9cd . Thanks Viorel, much appreciated!
I used it as a basis for my project and adapted it to my specific needs.
My antenna rotator controller consists of the following:
- Arduino Uno
- L298N H-bridge motor driver board
- 2 rotary encoders
- a two row I2C LCD
- a 24 V power supply for the rotator motors

The controller can be used in manual mode to set Az and El, or in conjunction with EasyCom compatible tracking software. I am using MoonSked by David Anderson, GM4JJJ. Thanks David!



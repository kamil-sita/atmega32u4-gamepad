## About project

This repository contains hardware, software and programmable file for my gamepad project based on atmega32u-4.

The gamepad simulates behaviour of a HID keyboard. Joystick always simulates being WASD keys, with pulsating frequency somewhat proportional to the joystick's tilt.
Basic keys simulation can work in 4 working modes:
* mode 0 - JIKL keys
* mode 1 - arrow keys
* mode 2 - Q, E, Left Shift, Left CTRL
* mode 3 - C, G, V, space

You can switch between modes by pressing HWB button. You can reset gamepad by pressing reset button. When holding both of them, the device will enter programmable mode.

If the device is programmed correctly, it will start blinking once resetted/provided with power and after blinking it will register itself as HID keyboard.

### Configuration 

Most of the changes are in Keyboard.c . It should be pretty easy to modify which key strokes are send to the device. You can also modify Descriptors.c to change ManufacturerString and ProductString.

## Device programming

I recommend using dfu-programmer, but FLIP probably also can be used.

## Licence

LUFA [(1)](https://github.com/abcminiuser/lufa), [(2)](http://www.fourwalledcubicle.com/LUFA.php) is under MIT License.

Software for the gamepad is heavily based on LUFA library, and most of the changes are contained inside Keyboard.c and Descriptors.c .
All of my contributions (changes to the library + hardware) are also under MIT License.


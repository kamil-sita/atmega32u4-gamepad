/*
             LUFA Library
     Copyright (C) Dean Camera, 2017.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2017  Dean Camera (dean [at] fourwalledcubicle [dot] com)
  Copyright 2010  Denver Gingerich (denver [at] ossguy [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the Keyboard demo. This file contains the main tasks of the demo and
 *  is responsible for the initial application hardware configuration.
 */

#include "Keyboard.h"

/** Indicates what report mode the host has requested, true for normal HID reporting mode, \c false for special boot
 *  protocol reporting mode.
 */
static bool UsingReportProtocol = true;

/** Current Idle period. This is set by the host via a Set Idle HID class request to silence the device's reports
 *  for either the entire idle duration, or until the report status changes (e.g. the user presses a key).
 */
static uint16_t IdleCount = 500;

/** Current Idle period remaining. When the IdleCount value is set, this tracks the remaining number of idle
 *  milliseconds. This is separate to the IdleCount timer and is incremented and compared as the host may request
 *  the current idle period via a Get Idle HID class request, thus its value must be preserved.
 */
static uint16_t IdleMSRemaining = 0;


/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Device_Connect(void)
{
	/* Indicate USB enumerating */
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);

	/* Default to report protocol on connect */
	UsingReportProtocol = true;
}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs.
 */
void EVENT_USB_Device_Disconnect(void)
{
	led_blink_enable();
	/* Indicate USB not ready */
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host sets the current configuration
 *  of the USB device after enumeration, and configures the keyboard device endpoints.
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	/* Setup HID Report Endpoints */
	ConfigSuccess &= Endpoint_ConfigureEndpoint(KEYBOARD_IN_EPADDR, EP_TYPE_INTERRUPT, KEYBOARD_EPSIZE, 1);
	ConfigSuccess &= Endpoint_ConfigureEndpoint(KEYBOARD_OUT_EPADDR, EP_TYPE_INTERRUPT, KEYBOARD_EPSIZE, 1);

	/* Turn on Start-of-Frame events for tracking HID report period expiry */
	USB_Device_EnableSOFEvents();

	/* Indicate endpoint configuration success or failure */
	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{
	/* Handle HID Class specific requests */
	switch (USB_ControlRequest.bRequest)
	{
		case HID_REQ_GetReport:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				USB_KeyboardReport_Data_t KeyboardReportData;

				/* Create the next keyboard report for transmission to the host */
				CreateKeyboardReport(&KeyboardReportData);

				Endpoint_ClearSETUP();

				/* Write the report data to the control endpoint */
				Endpoint_Write_Control_Stream_LE(&KeyboardReportData, sizeof(KeyboardReportData));
				Endpoint_ClearOUT();
			}

			break;
		case HID_REQ_SetReport:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Wait until the LED report has been sent by the host */
				while (!(Endpoint_IsOUTReceived()))
				{
					if (USB_DeviceState == DEVICE_STATE_Unattached)
					  return;
				}

				/* Read in the LED report from the host */
				uint8_t LEDStatus = Endpoint_Read_8();

				Endpoint_ClearOUT();
				Endpoint_ClearStatusStage();

				/* Process the incoming LED report */
				ProcessLEDReport(LEDStatus);
			}

			break;
		case HID_REQ_GetProtocol:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Write the current protocol flag to the host */
				Endpoint_Write_8(UsingReportProtocol);

				Endpoint_ClearIN();
				Endpoint_ClearStatusStage();
			}

			break;
		case HID_REQ_SetProtocol:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();

				/* Set or clear the flag depending on what the host indicates that the current Protocol should be */
				UsingReportProtocol = (USB_ControlRequest.wValue != 0);
			}

			break;
		case HID_REQ_SetIdle:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();

				/* Get idle period in MSB, IdleCount must be multiplied by 4 to get number of milliseconds */
				IdleCount = ((USB_ControlRequest.wValue & 0xFF00) >> 6);
			}

			break;
		case HID_REQ_GetIdle:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Write the current idle duration to the host, must be divided by 4 before sent to host */
				Endpoint_Write_8(IdleCount >> 2);

				Endpoint_ClearIN();
				Endpoint_ClearStatusStage();
			}

			break;
	}
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	/* One millisecond has elapsed, decrement the idle time remaining counter if it has not already elapsed */
	if (IdleMSRemaining)
	  IdleMSRemaining--;
}


bool exclusive = false;


/// enables flashing of the LED diode if exclusive is disabled
void led_blink_enable() {
	if (exclusive) return;
	PORTC |= (1 << 7);
}

/// disables flashing of the LED diode if exclusive is disabled
void led_blink_disable() {
	if (exclusive) return;
	PORTC &= ~(1 << 7);
}


/// blinks LED diode count times (0.5s on, 0.5s off). Works only if not exclusive.
void fastblink(int count) {
	for (int i = 0; i < count; i++) {
		led_blink_enable();
		_delay_ms(500);
		led_blink_disable();
		_delay_ms(500);
	}
}

/// does not blink LED diode for the same amount of time as blink function does, count times.
void noblink(int count) {
	for (int i = 0; i < count; i++) {
		_delay_ms(1000);
	}
}

/// performs count fast blinks with LED diode (0.2s on, 0.2s off)
void veryfastblink(int count) {
	for (int i = 0; i < count; i++) {
		led_blink_enable();
		_delay_ms(200);
		led_blink_disable();
		_delay_ms(200);
	}
}

/** Main program entry point. This routine configures the hardware required by the application, then
 *  enters a loop to run the application tasks in sequence.
 */
int main(void)
{
	SetupHardware();

	fastblink(1);
	veryfastblink(4);

	GlobalInterruptEnable();

	for (;;)
	{
		HID_Task();
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

	//gamepad hardware intiialization

	//configuration of LED diode port
	DDRC |= (1 << 7);

	//configuration of keys ports
	DDRD &= ~(1<<PD0);
	DDRD &= ~(1<<PD1);
	DDRD &= ~(1<<PD2);
	DDRD &= ~(1<<PD3);

	//adc configuration
	ADCSRA = (1 << ADPS2)|(1 << ADPS1) | (1 << ADPS0); //16MHZ / 128 = 125khz, which is in ADC allowed range of work
	ADMUX &= ~(1 << REFS1); //ref = AVCC
	ADMUX |= (1 << REFS0); //ref = AVCC
	ADCSRA |= (1 << ADEN); // ADC activation


	TCCR1B |= (1 << CS10); //prescaler CLK/1
	TIMSK1 |= (1 << TOIE1); //overflow vector

	led_blink_enable();
	USB_Init();
	led_blink_disable();
}


/// contains current milisecond, updated by timer interruption. Max 6000. Precision of 8ms.
uint16_t timer_in_ms = 0;

/// interrupt that updates current milisecond
ISR(TIMER1_OVF_vect) {
	timer_in_ms+=8;
	if (timer_in_ms >= 6000) {
		timer_in_ms = 0;
	}
}


/// reads value from selected ADC
/// probably not well optimized, but I guess it works
uint16_t read_adc(uint8_t channel) {
	ADCSRA &= ~(1 << ADEN); //turn ADC off
	channel = channel & 0b00000111; //only 7 channels
	ADMUX &= 0b11111000; //clearing previous channel
	ADMUX |= channel; //channel selection
	ADCSRA |= (1 << ADEN); //turn ADC on

	ADCSRA |= (1 << ADSC); //start waiting for ADC measurement
	while (!(ADCSRA & (1<<ADIF))) { //await
		; 
	}
	ADCSRA |= (1<<ADIF); //receive first result
	
	//first result is ignored! some sources claim that first result might be wrong, and this is not real time
	//tool, neither are we lacking on processing power

	//secondary result - preferably, this one is more correct than the last one
	ADCSRA |= (1 << ADSC); 
	while (!(ADCSRA & (1<<ADIF))) { 
		; 
	}
	ADCSRA |= (1<<ADIF);

	return ADC;
}


/// weighted average horizontal position of joystick, in the last few iterations
uint16_t avg_horizontal = 512;

/// weighted average vertical position of joystick, in the last few iterations
uint16_t avg_vertical = 512;

/// curent working mode
int mode = 0;

/// simulates schmitt game behaviour to minimize any issues with mode switching
bool schmittButton = false;

/// counts how long the button was held down (in order to eliminate short circuits or any other anomalies)
uint8_t button_held_down = 0;

/** Fills the given HID report data structure with the next HID report to send to the host.
 *
 *  \param[out] ReportData  Pointer to a HID report data structure to be filled
 */
void CreateKeyboardReport(USB_KeyboardReport_Data_t* const ReportData)
{
	//number of used keys
	uint8_t UsedKeyCodes      = 0;

	//report creation
	memset(ReportData, 0, sizeof(USB_KeyboardReport_Data_t));


	//changing working mode
	if (bit_is_clear(PINE, PE2)) {
		button_held_down++;
		if (button_held_down > 16) {
			button_held_down = 16;
			if (!schmittButton) {
				led_blink_enable();
				exclusive = true;
				mode++;
				if (mode > 3) {
					mode = 0;
				}
				schmittButton = true;
			}
		}
	} else {
		if (button_held_down > 0) {
			button_held_down--;
			if (button_held_down == 0) {
				exclusive = false;
				led_blink_disable();
				schmittButton = false;
			}
		}
	}


	//checking basic modes
	if (mode == 0) { //mode 0 (JIKL)


		if (bit_is_set(PIND, PD2)) {
			ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_J;
		}
		if (bit_is_set(PIND, PD3)) {
			ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_I;
		}
		if (bit_is_set(PIND, PD1)) {
			ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_K;
		}
		if (bit_is_set(PIND, PD0)) {
			ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_L;
		}


	} else if (mode == 1) { //mode 1 (arrow keys)

		if (timer_in_ms < 1000) {
			led_blink_enable();
		} else {
			led_blink_disable();
		}

		if (bit_is_set(PIND, PD2)) {
			ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_LEFT_ARROW;
		}
		if (bit_is_set(PIND, PD3)) {
			ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_UP_ARROW;
		}
		if (bit_is_set(PIND, PD1)) {
			ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_DOWN_ARROW;
		}
		if (bit_is_set(PIND, PD0)) {
			ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_RIGHT_ARROW;
		}

	} else if (mode == 2) { // mode 2 (Q, E, Left Shift, Left CTRL)

		if (timer_in_ms < 1000) {
			led_blink_enable();
		} else if (timer_in_ms > 3000 && timer_in_ms < 4000) {
			led_blink_enable();
		} else {
			led_blink_disable();
		}

		if (bit_is_set(PIND, PD2)) {
			ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_Q;
		}
		if (bit_is_set(PIND, PD3)) {
			ReportData->Modifier |= HID_KEYBOARD_MODIFIER_LEFTSHIFT;
		}
		if (bit_is_set(PIND, PD1)) {
			ReportData->Modifier |= HID_KEYBOARD_MODIFIER_LEFTCTRL;
		}
		if (bit_is_set(PIND, PD0)) {
			ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_E;
		}
	}else if (mode == 3) { // mode 3 (C, G, V, Space)

		if (timer_in_ms < 3000) {
			led_blink_enable();
		} else {
			led_blink_disable();
		}

		if (bit_is_set(PIND, PD2)) {
			ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_C;
		}
		if (bit_is_set(PIND, PD3)) {
			ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_G;
		}
		if (bit_is_set(PIND, PD1)) {
			ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_SPACE;
		}
		if (bit_is_set(PIND, PD0)) {
			ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_V;
		}
	}

	// horizontal and vertical gamepad tilt reading from ADC
	uint16_t horizontal = read_adc(6);
	uint16_t vertical = read_adc(7);
	
	///weighted average
	avg_horizontal /= 2;
	avg_horizontal += horizontal/2;
	avg_vertical /= 2;
	avg_vertical += vertical/2;


	//joystick output binarization
	uint8_t divided_timer = (timer_in_ms/125)%8+1; //generating number from 1 to 8 - in order to create pulsing effect on the output
	bool pulse = divided_timer % 2 == 0;

	if (avg_horizontal < 128) {
		ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_D;
	} else if (avg_horizontal < 320 && pulse) {
		ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_D;
	}
	
	if (avg_horizontal > 896) {
		ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_A;
	} else if (avg_horizontal > 704 && pulse) {
		ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_A;
	}

	if (avg_vertical < 128) {
		ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_W;
	} else if (avg_vertical < 320 && pulse) {
		ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_W;
	}
	
	if (avg_vertical > 896) {
		ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_S;
	} else if (avg_vertical > 704 && pulse) {
		ReportData->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_S;
	}


}

/** Processes a received LED report, and updates the board LEDs states to match.
 *
 *  \param[in] LEDReport  LED status report from the host
 */
void ProcessLEDReport(const uint8_t LEDReport)
{

}

/** Sends the next HID report to the host, via the keyboard data endpoint. */
void SendNextReport(void)
{
	static USB_KeyboardReport_Data_t PrevKeyboardReportData;
	USB_KeyboardReport_Data_t        KeyboardReportData;
	bool                             SendReport = false;

	/* Create the next keyboard report for transmission to the host */
	CreateKeyboardReport(&KeyboardReportData);

	/* Check if the idle period is set and has elapsed */
	if (IdleCount && (!(IdleMSRemaining)))
	{
		/* Reset the idle time remaining counter */
		IdleMSRemaining = IdleCount;

		/* Idle period is set and has elapsed, must send a report to the host */
		SendReport = true;
	}
	else
	{
		/* Check to see if the report data has changed - if so a report MUST be sent */
		SendReport = (memcmp(&PrevKeyboardReportData, &KeyboardReportData, sizeof(USB_KeyboardReport_Data_t)) != 0);
	}

	/* Select the Keyboard Report Endpoint */
	Endpoint_SelectEndpoint(KEYBOARD_IN_EPADDR);

	/* Check if Keyboard Endpoint Ready for Read/Write and if we should send a new report */
	if (Endpoint_IsReadWriteAllowed() && SendReport)
	{
		/* Save the current report data for later comparison to check for changes */
		PrevKeyboardReportData = KeyboardReportData;

		/* Write Keyboard Report Data */
		Endpoint_Write_Stream_LE(&KeyboardReportData, sizeof(KeyboardReportData), NULL);

		/* Finalize the stream transfer to send the last packet */
		Endpoint_ClearIN();
	}
}

/** Reads the next LED status report from the host from the LED data endpoint, if one has been sent. */
void ReceiveNextReport(void)
{
	/* Select the Keyboard LED Report Endpoint */
	Endpoint_SelectEndpoint(KEYBOARD_OUT_EPADDR);

	/* Check if Keyboard LED Endpoint contains a packet */
	if (Endpoint_IsOUTReceived())
	{
		

		/* Handshake the OUT Endpoint - clear endpoint and ready for next report */
		Endpoint_ClearOUT();
	}
}

/** Function to manage HID report generation and transmission to the host, when in report mode. */
void HID_Task(void)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	/* Send the next keypress report to the host */
	SendNextReport();

	/* Process the LED report sent from the host */
	ReceiveNextReport();
}


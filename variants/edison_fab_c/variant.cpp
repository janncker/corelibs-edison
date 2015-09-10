/*
  Copyright (c) 2011 Arduino.  All right reserved.
  Copyright (c) 2014 Intel Corporation.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <Arduino.h>
#include <interrupt.h>
#include <Mux.h>
#include <sysfs.h>
#include <trace.h>
#include "variant.h"

//Bindings to Arduino
#include "RingBuffer.h"
#include "TTYUART.h"
#include "fast_gpio_pci.h"

#define MY_TRACE_PREFIX "variant"

#ifdef __cplusplus
extern "C" {
#endif

#define GP12_PWM0			12
#define GP13_PWM1			13
#define GP182_PWM2			182
#define GP183_PWM3			183
#define any_equal(a,b,c,d)		(a==b||a==c||a==d||b==c||b==d||c==d)
#define all_pwms(a,b,c,d)		(digitalPinHasPWM(a)&&digitalPinHasPWM(b)&&\
					digitalPinHasPWM(c)&&digitalPinHasPWM(d))

struct pwm_muxing {
	uint8_t gpioid;
	mux_sel_t *muxing;
};

const int mux_sel_analog[NUM_ANALOG_INPUTS] = {
	MUX_SEL_AD7298_VIN0,
	MUX_SEL_AD7298_VIN1,
	MUX_SEL_AD7298_VIN2,
	MUX_SEL_AD7298_VIN3,
};

const int mux_sel_uart[NUM_UARTS][MUX_DEPTH_UART] = {
	/* This is auto-indexed (board pinout) */
	{MUX_SEL_NONE, MUX_SEL_NONE},				// ttyGS0 - USB not muxed
	{MUX_SEL_UART0_RXD,	MUX_SEL_UART0_TXD},		// ttyS0 - muxed
};

const int  mux_sel_spi[NUM_SPI][MUX_DEPTH_SPI] = {
	{
		MUX_SEL_NONE,
		MUX_SEL_NONE,
		MUX_SEL_NONE
	},
	{
		MUX_SEL_SPI1_MOSI,
		MUX_SEL_SPI1_MISO,
		MUX_SEL_SPI1_SCK
	},
};

const int  mux_sel_i2c[NUM_I2C][MUX_DEPTH_I2C] = {
	{
		MUX_SEL_I2C_SCL,
		MUX_SEL_I2C_SDA,
	},
};

  /*
	{ 130, GPIO_REG_OFFSET(47) }, //IO0
	{ 131, GPIO_REG_OFFSET(48) }, //IO1
	{ 49, GPIO_REG_OFFSET(49) }, //IO2
	{ 48, GPIO_REG_OFFSET(15) },  //IO3
	{ 47, GPIO_REG_OFFSET(165) }, //IO4
	{ 46, GPIO_REG_OFFSET(12) },  //IO5
	{ 13, GPIO_REG_OFFSET(13) }, //IO6
	{ 45, GPIO_REG_OFFSET(48) },  //IO7
	{ 44, GPIO_REG_OFFSET(49) },  //IO8
	{ 12, GPIO_REG_OFFSET(183) }, //IO9
	{182, GPIO_REG_OFFSET(41) },  //IO10
	{183, GPIO_REG_OFFSET(43) },  //IO11
	{  15, GPIO_REG_OFFSET(42) },  //IO12
	{  14, GPIO_REG_OFFSET(40) },  //IO13
	{  28, GPIO_REG_OFFSET(44) },  //IO14
	{  27, GPIO_REG_OFFSET(45) },  //IO15

  */
mux_sel_t MuxDesc0[] = {
	//gpio, value, type
        { 130, PIN_MODE_0, FN_GPIO | FN_UART }, 
};

mux_sel_t MuxDesc1[] = {
	//gpio, value, type
        { 131, PIN_MODE_0, FN_GPIO|FN_UART }, 
};

mux_sel_t MuxDesc2[] = {
	//gpio, value, type
	{ 49, PIN_MODE_0, FN_GPIO }, // GPIO mode
};

mux_sel_t MuxDesc3[] = {
	//gpio, value, type
	{  48, PIN_MODE_0, FN_GPIO }, // GPIO mode
};

mux_sel_t MuxDesc4[] = {
//	//gpio, value, type
	{ 47, PIN_MODE_0, FN_GPIO }, // GPIO mode
};

mux_sel_t MuxDesc5[] = {
//	//gpio, value, type
	{  46, PIN_MODE_0, FN_GPIO }, // GPIO mode
};

mux_sel_t MuxDesc6[] = {
	{ 13, PIN_MODE_0, FN_GPIO|FN_PWM }, // GPIO mode
};

mux_sel_t MuxDesc7[] = {
	//gpio, value, type
        {  45, PIN_MODE_0, FN_GPIO }, // GPIO mode
};

mux_sel_t MuxDesc8[] = {
	//gpio, value, type
	{  44, PIN_MODE_0, FN_GPIO }, // GPIO mode
};

mux_sel_t MuxDesc9[] = {
	//gpio, value, type
	{ 12, PIN_MODE_0, FN_GPIO|FN_PWM }, // GPIO mode
};

mux_sel_t MuxDesc10[] = {
	//gpio, value, type
	{ 182, PIN_MODE_0, FN_GPIO|FN_PWM }, // GPIO mode
};

mux_sel_t MuxDesc11[] = {
	//gpio, value, type
	{ 183, PIN_MODE_0, FN_GPIO|FN_PWM }, // GPIO mode
};

mux_sel_t MuxDesc12[] = {
	//gpio, value, type
	{  15, PIN_MODE_0, FN_GPIO }, // GPIO mode
};

mux_sel_t MuxDesc13[] = {
	//gpio, value, type
	{ 14, PIN_MODE_0, FN_GPIO }, // PinMux mode
};

mux_sel_t MuxDesc14[] = {
	//gpio, value, type
	{  28, PIN_MODE_1, FN_I2C }, 
};

mux_sel_t MuxDesc15[] = {
	//gpio, value, type
	{  27, PIN_MODE_1,  FN_I2C }, 
};

mux_sel_t MuxDesc20[] = {
  //MISO
  { 114, PIN_MODE_1, FN_SPI|FN_GPIO },
};

mux_sel_t MuxDesc21[] = {
  //FS0
  { 110, PIN_MODE_0, FN_GPIO },
};

mux_sel_t MuxDesc22[] = {
  //MOSI
  { 115, PIN_MODE_1, FN_SPI|FN_GPIO },
};

mux_sel_t MuxDesc23[] = {
  //FS1
  { 111, PIN_MODE_1, FN_SPI },
};

mux_sel_t MuxDesc24[] = {
  //CLK
  { 109, PIN_MODE_1, FN_SPI|FN_GPIO },
};

  
// Sorted by Linux GPIO ID
PinDescription g_APinDescription[] =
{
//	gpiolib	alias	fastinf	ardid	Initial			FixdSt	ptMuxDesc,		MuxCount		type		Handle	extPU	iAlt	pAlt
	{ 49,	NONE,	2,	2,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc2,	MUX_SIZE(MuxDesc2),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO2
        { 48,	NONE,	3,	3,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc3,	MUX_SIZE(MuxDesc3),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO3
	{ 47,	NONE,	4,	4,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc4,	MUX_SIZE(MuxDesc4),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO4
	{ 46,	NONE,	5,	5,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc5,	MUX_SIZE(MuxDesc5),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO5
	{ 13,	NONE,	6,	6,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc6,	MUX_SIZE(MuxDesc6),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO6
        { 45,	NONE,	7,	7,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc7,	MUX_SIZE(MuxDesc7),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO7
        { 44,	NONE,	8,	8,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc8,	MUX_SIZE(MuxDesc8),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO8
	{ 12,	NONE,	9,	9,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc9,	MUX_SIZE(MuxDesc9),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO9
	{ 182,	NONE,	10,	10,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc10,	MUX_SIZE(MuxDesc10),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO10        
	{ 183,	NONE,	11,	11,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc11,	MUX_SIZE(MuxDesc11),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO11
	{ 15,	NONE,	12,	12,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc12,	MUX_SIZE(MuxDesc12),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO12
	{ 14,	NONE,	13,	13,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc13,	MUX_SIZE(MuxDesc13),	FN_GPIO,	-1,	1,	0,	NULL },	// Arduino IO13
	{ 28,	NONE,	14,	14,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc14,	MUX_SIZE(MuxDesc14),	FN_I2C,	-1,	1,	0,	NULL },	// Arduino IO14 (AIN0)
	{ 27,	NONE,	15,	15,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc15,	MUX_SIZE(MuxDesc15),	FN_I2C,	-1,	1,	0,	NULL },	// Arduino IO15 (AIN1)
	{ 255,	NONE,	16,	16,	FN_GPIO_INPUT_HIZ,	NONE,	 NULL,	0,	-1,	-1,	1,	0,	NULL },	
	{ 255,	NONE,	17,	17,	FN_GPIO_INPUT_HIZ,	NONE,	 NULL,	0,	-1,	-1,	1,	0,	NULL },
        { 255,	NONE,	18,	18,	FN_GPIO_INPUT_HIZ,	NONE,	 NULL,	0,	-1,	-1,	1,	0,	NULL },
        { 255,	NONE,	19,	19,	FN_GPIO_INPUT_HIZ,	NONE,	 NULL,	0,	-1,	-1,	1,	0,	NULL },	
	{ 114,	NONE,	20,	20,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc20,	MUX_SIZE(MuxDesc20),	FN_SPI,	-1, 1,	0,   NULL },
	{ 110,	NONE,	NONE,	21,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc21,	MUX_SIZE(MuxDesc21),	FN_GPIO,	-1, 1,	0,   NULL },
        { 115,	NONE,	22,	22,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc22,	MUX_SIZE(MuxDesc22),	FN_SPI,	-1, 1,	0,   NULL },
        { 111,	NONE,	NONE,     23,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc23,	MUX_SIZE(MuxDesc23),	FN_SPI,	-1, 1,	0,   NULL },
        { 109,	NONE,	24,	24,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc24,	MUX_SIZE(MuxDesc24),	FN_SPI,	-1, 1,	0,   NULL },
};



uint32_t sizeof_g_APinDescription;

uint32_t ardPin2DescIdx[GPIO_TOTAL];

// Sorted by Linux PWM ID
PwmDescription g_APwmDescription[] = {
	{ 0,	9,	-1,	-1,	GP12_PWM0,	-1 },
	{ 1,	6,	-1,	-1,	GP13_PWM1,	-1 },
	{ 2,	10,	-1,	-1,	GP182_PWM2,	-1 },
	{ 3,	11,	-1,	-1,	GP183_PWM3,	-1 },
};
uint32_t sizeof_g_APwmDescription;

AdcDescription g_AdcDescription[] = {
	{ 0,	-1 },
	{ 1,	-1 },
	{ 2,	-1 },
	{ 3,	-1 },
	{ 4,	-1 },
	{ 5,	-1 },
};
uint32_t sizeof_g_AdcDescription;

// Sorted Arduino Pin ID
PinState g_APinState[]=
{
	/* uCurrentPwm	uPwmEnabled	uCurrentInput	uCurrentAdc		*/
	{ 0,		0,		1,		0},	/* 0		*/
	{ 0,		0,		1,		0},	/* 1		*/
	{ 0,		0,		1,		0},	/* 2		*/
	{ 0,		0,		1,		0},	/* 3  - PWM	*/
	{ 0,		0,		1,		0},	/* 4 		*/
	{ 0,		0,		1,		0},	/* 5  - PWM 	*/
	{ 0,		0,		1,		0},	/* 6  - PWM	*/
	{ 0,		0,		1,		0},	/* 7 		*/
	{ 0,		0,		1,		0},	/* 8 		*/
	{ 0,		0,		1,		0},	/* 9  - PWM	*/
	{ 0,		0,		1,		0},	/* 10 - PWM	*/
	{ 0,		0,		1,		0},	/* 11 - PMW	*/
	{ 0,		0,		1,		0},	/* 12		*/
	{ 0,		0,		1,		0},	/* 13		*/
	{ 0,		0,		1,		0},	/* 14 - ADC	*/
	{ 0,		0,		1,		0},	/* 15 - ADC	*/
	{ 0,		0,		1,		0},	/* 16 - ADC	*/
	{ 0,		0,		1,		0},	/* 17 - ADC	*/
	{ 0,		0,		1,		0},	/* 18 - ADC	*/
	{ 0,		0,		1,		0},	/* 19 - ADC	*/
	{ 0,		0,		1,		0},	/* 20 - ADC	*/
        { 0,		0,		1,		0},	/* 21 - ADC	*/
        { 0,		0,		1,		0},	/* 22 - ADC	*/
        { 0,		0,		1,		0},	/* 23 - ADC	*/
        { 0,		0,		1,		0},	/* 24 - ADC	*/

};
uint32_t sizeof_g_APinState;

#ifdef __cplusplus
}
#endif


RingBuffer rx_buffer1;
RingBuffer rx_buffer2;
RingBuffer rx_buffer3;

TTYUARTClass Serial(&rx_buffer1, 0, false);	// ttyGS0  (USB serial)
TTYUARTClass Serial1(&rx_buffer2, 1, false);	// ttyMFD1(IO0/1)
TTYUARTClass Serial2(&rx_buffer3, 2, true);	// ttyMFD2 (system console)


// ----------------------------------------------------------------------------

int variantPinMode(uint8_t pin, uint8_t mode)
{
	/*
	 * Standard (sysfs) or fast-mode UIO options are available for some pins
	 *
	 * The pin at this time is set to Fast-mode by default, if available
	 */

	int ret = 0;
	PinDescription *p = NULL;

	/* Search for entry */
	if ((p = pinDescriptionById(pin)) == NULL) {
		trace_error("%s: invalid pin %u\n", __func__, pin);
		return PIN_EINVAL;
	}

	/* Alternate entries for Fast-Mode GPIO: enable by default if available */
	if (p->pAlternate) {
		p->iAlternate = 1;
		trace_debug("%s: enable Fast-Mode SoC GPIO for pin%u",
			    __func__, pin);
	}
	
	return 0;
}

int variantPinModeIRQ(uint8_t pin, uint8_t mode)
{
	return 0;
}

/*
 * Set the pin as used for PWM and do the muxing at SoC level to enable
 * the PWM output.
 */
void turnOnPWM(uint8_t pin)
{
	int i;

	/* Mark PWM enabled on pin */
	g_APinState[pin].uCurrentPwm = 1;
	g_APinState[pin].uCurrentAdc = 0;

	for (i = 0; i < sizeof_g_APwmDescription; i++)
		if (g_APwmDescription[i].ulArduinoId == pin) {
			sysfsGpioSetCurrentPinmux(g_APwmDescription[i].pwmChPinId,
					PIN_MODE_1);
			break;
		}
}

void turnOffPWM(uint8_t pin)
{
	int handle = 0, ret = 0;
	PinDescription *p = NULL;

	// Scan mappings
	if ((p = pinDescriptionById(pin)) == NULL) {
		trace_error("%s: invalid pin %u\n", __func__, pin);
		return;
	}

	pin2alternate(&p);

	if(p->ulArduinoId == pin) {
		handle = pin2pwmhandle_enable(pin);
		if ((int)PIN_EINVAL == handle) {
			trace_error("%s: bad handle for pin%u",
					__func__, pin);
			return;
		}
		if (sysfsPwmDisable(handle)) {
			trace_error("%s: couldn't disable pwm "
					"on pin%u", __func__, pin);
			return;
		}

		/* Mark PWM disabled on pin */
		g_APinState[pin].uCurrentPwm = 0;
		g_APinState[pin].uPwmEnabled = 0;

		return;
	}

	trace_error("%s: unknown pin%u", __func__, pin);
}

void variantEnableFastGpio(int pin)
{
	int entryno = ardPin2DescIdx[pin];
	PinDescription *p = NULL;
	int ret = 0;

	if (entryno >= sizeof_g_APinDescription) {
		trace_error("%s: ardPin2DescIdx[%d] == %d >= "
				"sizeof_g_APinDescription", __func__, pin, entryno);
		return;
	}

	/* Enable alternate to route to SoC */
	p = &g_APinDescription[entryno];
	p->iAlternate = 1;
}

/*
 * Set the PWM description table accordingly following the PWM swizzler present
 * on the board
 *
 * Suppose the function is called like this: setPwmSwizzler(3, 5, 10, 11). That
 * means the swizzler configuration is as follows:
 *
 *   PWM channel/output 0 is connected to pin 3
 *   PWM channel/output 1 is connected to pin 5
 *   PWM channel/output 2 is connected to pin 10
 *   PWM channel/output 3 is connected to pin 11
 *
 * Default config follows swizzler's default config (3, 5, 6, 9). To change it
 * just call the function in your sketch's setup()
 *
 * Making modifications to the swizzler will break standard GPIO functionality
 * on the affected pins.
 */
void setPwmSwizzler(uint8_t pwmout0, uint8_t pwmout1, uint8_t pwmout2,
		uint8_t pwmout3)
{
	int i;

	/* All parameters must be different */
	if (any_equal(pwmout0, pwmout1, pwmout2, pwmout3)) {
		trace_error("%s All pwm outputs should be different. Got: "
				"%u, %u, %u, %u", __func__, pwmout0, pwmout1,
				pwmout2, pwmout3);
		return;
	}

	/* All parameters must be valid PWM pins */
	if (!all_pwms(pwmout0, pwmout1, pwmout2, pwmout3)) {
		trace_error("%s Some pwm outputs are not valid. Got: "
				"%u, %u, %u, %u", __func__, pwmout0, pwmout1,
				pwmout2, pwmout3);
		return;
	}

	/* Fill the description table with the new pin layout */
	for (i = 0; i < sizeof_g_APwmDescription; i++)
		if (g_APwmDescription[i].ulPWMId == 0)
			g_APwmDescription[i].ulArduinoId = pwmout0;
		else if (g_APwmDescription[i].ulPWMId == 1)
			g_APwmDescription[i].ulArduinoId = pwmout1;
		else if (g_APwmDescription[i].ulPWMId == 2)
			g_APwmDescription[i].ulArduinoId = pwmout2;
		else if (g_APwmDescription[i].ulPWMId == 3)
			g_APwmDescription[i].ulArduinoId = pwmout3;
}

void eepromInit(void)
{
	int fd;
	char buf = 0xff;

	/* Do nothing if file exists already */
	if (access(LINUX_EEPROM, F_OK) == 0)
		return;

	if ((fd = open(LINUX_EEPROM, O_RDWR | O_CREAT, 0660)) < 0) {
		trace_error("%s Can't create EEPROM file: %s", __func__,
				strerror(errno));
		return;
	}

	if (lseek(fd, 0, SEEK_SET)) {
		trace_error("%s Can't lseek in EEPROM file: %s", __func__,
						strerror(errno));
		goto err;
	}

	if (write(fd, &buf, LINUX_EEPROM_SIZE) != LINUX_EEPROM_SIZE)
		trace_error("%s Can't write to EEPROM file: %s", __func__,
				strerror(errno));

	trace_debug("%s Created EEPROM file '%s' of size %u bytes", __func__,
			LINUX_EEPROM, LINUX_EEPROM_SIZE);
err:
	close(fd);
}

void init( int argc, char * argv[] )
{
	if(argc > 1)
		if(Serial.init_tty(argv[1]) != 0)
			return;

	if(Serial1.init_tty(LINUX_SERIAL1_TTY) != 0)
		return;
	if(Serial2.init_tty(LINUX_SERIAL2_TTY) != 0)
		return;

	sizeof_g_APinDescription = sizeof(g_APinDescription)/sizeof(struct _PinDescription);
	sizeof_g_APinState = sizeof(g_APinState)/sizeof(struct _PinState);
	pinInit();

	/* Initialize fast path to GPIO */
	if (fastGpioPciInit())
		trace_error("Unable to initialize fast GPIO mode!");

	sizeof_g_APwmDescription = sizeof(g_APwmDescription)/sizeof(struct _PwmDescription);
	pwmInit();

	sizeof_g_AdcDescription = sizeof(g_AdcDescription)/sizeof(struct _AdcDescription);
	adcInit();

	eepromInit();
}


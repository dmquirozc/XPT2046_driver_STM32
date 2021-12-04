# XPT2046_driver_STM32
A driver for the XPT2046 touchScreen driver for STM32.
## Limitations
This driver was made on the STM32F407VET6 Black Board, tested and verified with an ILI9341 Touch Screen.
It was tested on a 240x320 pixels wide screen, so by default it has configured with this width and height, if want to change it you can use the next function
```c 
void xpt2046_set_size(uint16_t w, uint16_t h);
```

In the testing process, my screen showed an offset of 10 pixels on the X-axis, this value is reflected in the following configuration.
```c
#define XPT2046_X_OFFSET 		10
#define XPT2046_Y_OFFSET		0
```
## Setup
By default this library imports the stm32f4xx files. If you are using another boards, change the next imports to the required libraries. 
```c
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_gpio.h
```
Also, this library assume that you have defined the CS and IRQ pins as follows.
```c
#define XPT2046_CS_Port 		TOUCH_CS_GPIO_Port
#define XPT2046_CS_Pin			TOUCH_CS_Pin
#define XPT2046_PENIRQ_Port 	TC_PEN_GPIO_Port
#define XPT2046_PENIRQ_Pin 		TC_PEN_Pin
```

If you have any problem with the definitions you can also set the pins and ports with the next functions
```c
void xpt2046_cs(GPIO_TypeDef* cs_port, uint16_t cs_pin);
void xpt2046_penirq(GPIO_TypeDef* penirq_port, uint16_t penirq_pin);
```
## Usage
First you have to init the library to setup the default ports, pins and orientation. By default it assumes the orientation XPT2046_ORIENTATION_LANDSCAPE.

```c
void xpt2046_init();
```

Before start using the library you need to define the SPI port to use and the orientation of the screen. 
The definition of the orientations are compatible with the definitions of ILI934 screens. 
```c
void xpt2046_spi(SPI_HandleTypeDef* spi);
void xpt2046_orientation(TouchScreen_OrientationTypeDef orientation_);
```
### Read data
The reading of X and Y screen points is made by the function
```c
void xpt2046_read_position(uint16_t* x, uint16_t* y);
```

This function reads if the touch screen is pressed and puts the x and y values on the pointers, if the screen isn't pressed, this returns  (0,0) value.






#include "xpt2046.h"

/**
 * Init the moduel
 */
void xpt2046_init()
{
	ts_ControlByte.bitMode = XPT2046_12BIT_MODE;
	ts_ControlByte.powerMode = XPT2046_REFERENCE_OFF_ADC_ON;
	ts_ControlByte.reference = XPT2046_DFR_MODE;
	ts_ControlByte.channel = XPT2046_DFR_X; //by default
	ts_ControlByte.startBit = XPT2046_START;
	ts_Orientation = XPT2046_ORIENTATION_LANDSCAPE;
	ts_Size.width = XPT2046_WIDTH;
	ts_Size.height = XPT2046_HEIGHT;
	ts_Cs_Pin = XPT2046_CS_Pin;
	ts_Cs_Port = XPT2046_CS_Port;
	ts_Penirq_Pin = XPT2046_PENIRQ_Pin;
	ts_Penirq_Port = XPT2046_PENIRQ_Port;
	command = ts_ControlByte.bitMode |  ts_ControlByte.powerMode |ts_ControlByte.reference |ts_ControlByte.channel |ts_ControlByte.startBit;
	xpt2046_unselect();
}


void xpt2046_orientation(TouchScreen_OrientationTypeDef orientation_)
{
	ts_Orientation = orientation_;
}

void xpt2046_bit_mode(uint8_t b)
{
	ts_ControlByte.bitMode = b;
}
void xpt2046_set_size(uint16_t w, uint16_t h)
{
	ts_Size.width = w;
	ts_Size.height = h;
}

void xpt2046_cs(GPIO_TypeDef* cs_port, uint16_t cs_pin)
{
	ts_Cs_Port = cs_port;
	ts_Cs_Pin = cs_pin;
}

void xpt2046_penirq(GPIO_TypeDef* penirq_port, uint16_t penirq_pin)
{
	ts_Penirq_Port = penirq_port;
	ts_Penirq_Pin = penirq_pin;
}

uint16_t xpt2046_zthreshold()
{
	if(ts_ControlByte.bitMode == XPT2046_8BIT_MODE)
	{
		return Z_THRESHOLD_08BIT;
	}else
	{
		return Z_THRESHOLD_12BIT;
	}
}
void xpt2046_select()
{
	HAL_GPIO_WritePin(ts_Cs_Port,ts_Cs_Pin,GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(XPT2046_CS_Port,XPT2046_CS_Pin,GPIO_PIN_RESET);
}

void xpt2046_unselect()
{
	HAL_GPIO_WritePin(ts_Cs_Port,ts_Cs_Pin,GPIO_PIN_SET);
	//HAL_GPIO_WritePin(XPT2046_CS_Port,XPT2046_CS_Pin,GPIO_PIN_SET);
}

uint8_t xpt2046_interrupt()
{
	return HAL_GPIO_ReadPin(ts_Penirq_Port,ts_Penirq_Pin) == GPIO_PIN_RESET; //Means touchScreen was pressed
}

uint8_t xpt2046_pressed()
{
	if(xpt2046_interruptions_activated())
	{
		return xpt2046_interrupt();
	}else
	{
		uint16_t zt =  xpt2046_zthreshold();
		return (ts_CoordinatesRaw.z1 > zt );
	}
}

uint8_t xpt2046_interruptions_activated()
{
	return (ts_ControlByte.powerMode == XPT2046_POWER_DOWN || ts_ControlByte.powerMode == XPT2046_REFERENCE_ON_ADC_OFF);
}
/**
 * Generate control byte
 */
void xpt2046_control_byte_update()
{
	command =  ts_ControlByte.startBit |  ts_ControlByte.channel | ts_ControlByte.bitMode | ts_ControlByte.reference | ts_ControlByte.powerMode  ;
}


/**
 * Return max value depending on bit mode.
 */
uint16_t xpt2046_max_measurement(){
	if(ts_ControlByte.bitMode == XPT2046_8BIT_MODE)
	{
		return 120; //This must be calibrated
	}else
	{
		return 1900; // this value must be calibrated on your on touch screen
	}
}
/**
 * Comapre Coordinates using integers and TouchScreen_Coordinates
 */
uint8_t xpt2046_compare_cords(uint16_t x, uint16_t y , uint16_t z, TouchScreen_Coordinates tsc)
{
	return 0;
}

/**
 * Comapre two TouchScreen_Coordinates
 */
uint8_t xtp_compare_tsc(TouchScreen_Coordinates c, TouchScreen_Coordinates tcs)
{
	return 0;
}
/**
 *  Set internal SPI to user defined SPI port
 */
void xpt2046_spi(SPI_HandleTypeDef* spi)
{
	spiPort =  spi;
}


/**
 * Set Size of Touch screen
 */

void xtp2046_set_size(uint16_t w, uint16_t h)
{
	ts_Size.width = w;
	ts_Size.height = h;
}


/**
 * Update X,Y,Z values of touchScreen
 */
void xpt2046_update(){
	xpt2046_select();

	if(xpt2046_interruptions_activated())
	{
		/* this means Interrupt pin was enabled*/
		if(!xpt2046_interrupt())
		{
			return;
		}


	}

	uint8_t zeros[2] = { 0x00,0x00};
	uint32_t x_avg = 0, y_avg = 0,z1_avg=0,z2_avg=0;

	for(int i = 0; i < AVERAGE_POINTS; i++)
	{
		ts_ControlByte.channel = XPT2046_DFR_X;
		xpt2046_control_byte_update();

		uint8_t receiveByteX[2] = {0x00,0x00};

		HAL_SPI_Transmit(spiPort,&command,sizeof(command),HAL_MAX_DELAY);
		HAL_SPI_TransmitReceive(spiPort,(uint8_t*)zeros,receiveByteX,sizeof(receiveByteX),HAL_MAX_DELAY);


		ts_ControlByte.channel = XPT2046_DFR_Y;
		xpt2046_control_byte_update();

		uint8_t receiveByteY[2] = {0x00,0x00};
		HAL_SPI_Transmit(spiPort,&command,sizeof(command),HAL_MAX_DELAY);
		HAL_SPI_TransmitReceive(spiPort,(uint8_t*)zeros,receiveByteY,sizeof(receiveByteY),HAL_MAX_DELAY);


		ts_ControlByte.channel = XPT2046_DFR_Z1;
		xpt2046_control_byte_update();
		uint8_t receiveByteZ1[2] = {0x00,0x00};

		HAL_SPI_Transmit(spiPort,&command,sizeof(command),HAL_MAX_DELAY);
		HAL_SPI_TransmitReceive(spiPort,(uint8_t*)zeros,receiveByteZ1,sizeof(receiveByteZ1),HAL_MAX_DELAY);


		ts_ControlByte.channel = XPT2046_DFR_Z2;
		xpt2046_control_byte_update();
		uint8_t receiveByteZ2[2] = {0x00,0x00};

		HAL_SPI_Transmit(spiPort,&command,sizeof(command),HAL_MAX_DELAY);
		HAL_SPI_TransmitReceive(spiPort,(uint8_t*)zeros,receiveByteZ2,sizeof(receiveByteZ2),HAL_MAX_DELAY);
		if(ts_ControlByte.bitMode == XPT2046_12BIT_MODE)
		{
			x_avg+= 	(receiveByteX[1] 	<< 4) | (receiveByteX[0] 	>> 4);
			y_avg+= 	(receiveByteY[1] 	<< 4) | (receiveByteY[0] 	>> 4);
			z1_avg+= 	(receiveByteZ1[1] 	<< 4) | (receiveByteZ1[0] 	>> 4);
			z2_avg+= 	(receiveByteZ2[1] 	<< 4) | (receiveByteZ2[0] 	>> 4);
		}else
		{
			x_avg+= 	(receiveByteX[1]);
			y_avg+= 	(receiveByteY[1]) ;
			z1_avg+= 	(receiveByteZ1[1]);
			z2_avg+= 	(receiveByteZ2[1]);
		}

	}

	ts_CoordinatesRaw.x = x_avg/AVERAGE_POINTS;
	ts_CoordinatesRaw.y = y_avg/AVERAGE_POINTS;
	ts_CoordinatesRaw.z1 = z1_avg/AVERAGE_POINTS;
	ts_CoordinatesRaw.z2 = z2_avg/AVERAGE_POINTS;
	uint16_t max = xpt2046_max_measurement();
	/* Consider (0,0) as the top left point of the screen */
	switch(ts_Orientation)
	{
		case XPT2046_ORIENTATION_PORTRAIT:
		{
			ts_Coordinates.x = (ts_Size.width * (max - ts_CoordinatesRaw.x))/max - XPT2046_X_OFFSET;
			ts_Coordinates.y = (ts_Size.height * (ts_CoordinatesRaw.y))/max - XPT2046_Y_OFFSET;
			ts_Coordinates.z = ts_CoordinatesRaw.z1;
			break;
		}
		case XPT2046_ORIENTATION_LANDSCAPE:
		{
			ts_Coordinates.x = (ts_Size.width * (max - ts_CoordinatesRaw.y))/max - XPT2046_Y_OFFSET;
			ts_Coordinates.y = (ts_Size.height * (ts_CoordinatesRaw.x))/max - XPT2046_X_OFFSET;
			ts_Coordinates.z = ts_CoordinatesRaw.z1;
			break;
		}
		case XPT2046_ORIENTATION_PORTRAIT_MIRROR:
		{
			ts_Coordinates.x = (ts_Size.width * (ts_CoordinatesRaw.x))/max - XPT2046_X_OFFSET;
			ts_Coordinates.y = (ts_Size.height * (max - ts_CoordinatesRaw.y))/max - XPT2046_Y_OFFSET;
			ts_Coordinates.z = ts_CoordinatesRaw.z1;
			break;
		}
		case XPT2046_ORIENTATION_LANDSCAPE_MIRROR:
		{
			ts_Coordinates.x = (ts_Size.width *  (ts_CoordinatesRaw.y))/max - XPT2046_Y_OFFSET;
			ts_Coordinates.y = (ts_Size.height * (max- ts_CoordinatesRaw.x))/max - XPT2046_X_OFFSET;
			ts_Coordinates.z = ts_CoordinatesRaw.z1;
			break;
		}
		default:
		{
			break;
		}
	}


	xpt2046_unselect();
}

/* by default return 0,0 if the screen isn't pressed*/
void xpt2046_read_position(uint16_t* x, uint16_t* y)
{
	xpt2046_update();
	if(xpt2046_pressed())
	{
		*x = ts_Coordinates.x;
		*y = ts_Coordinates.y;
	}else
	{
		*x = 0;
		*y = 0;
	}
}

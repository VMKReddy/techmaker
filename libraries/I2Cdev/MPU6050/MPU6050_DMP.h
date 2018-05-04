#ifndef _MPU6050_DMP_H_
#define _MPU6050_DMP_H_


#include "MPU6050_6Axis_MotionApps20.h"


#define DMP_INT_PIN 	GPIO_PIN_9
#define DMP_INT_PORT 	GPIOC
#define __DMP_INT_PORT_CLK_ENABLE __GPIOC_CLK_ENABLE
#define DMP_INT_IRQn 	EXTI9_5_IRQn



// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

void MPU6050_calibrate() {
	//Calibration variables
	int16_t gx_off = 0, gy_off = 0, gz_off = 0;
	int16_t az_off = 1350;

	const int samples = 50;
	int16_t gx_raw_tmp, gy_raw_tmp, gz_raw_tmp;
	int i;
	for (i = 0; i < samples; i++) {
		MPU6050_getRotation(&gx_raw_tmp, &gy_raw_tmp, &gz_raw_tmp);
		gx_off += gx_raw_tmp;
		gy_off += gy_raw_tmp;
		gz_off += gz_raw_tmp;
		HAL_Delay(20);
	}
	MPU6050_setXGyroOffset(gx_off / samples);
	MPU6050_setYGyroOffset(gy_off / samples);
	MPU6050_setZGyroOffset(gz_off / samples);
	MPU6050_setZAccelOffset(az_off);
}

uint8_t MPU6050_dmpInit() {
	// initialize sensor
	MPU6050_initialize();
	uint8_t state = MPU6050_getDeviceID();
	LCD_Printf("%x", state);
	// test connection
	while(!MPU6050_testConnection());
	// initialize DMP
	devStatus = MPU6050_dmpInitialize();

	// calibrate if needed
	//	MPU6050_calibrate();

	if (devStatus == 0) {
		MPU6050_setDMPEnabled(true);

		/* MPU6050 GPIO configuration
			    PB8     ------> SCL
			    PB9     ------> SDA
				PC8     ------> INT
		*/
		GPIO_InitTypeDef GPIO_InitStruct;
		__DMP_INT_PORT_CLK_ENABLE();
		GPIO_InitStruct.Pin   = DMP_INT_PIN;
		GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
		GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
		HAL_GPIO_Init(DMP_INT_PORT, &GPIO_InitStruct);
		// turn on Interrupts to obtain DMP data
		HAL_NVIC_SetPriority(DMP_INT_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMP_INT_IRQn);

		mpuIntStatus = MPU6050_getIntStatus();
		packetSize = MPU6050_dmpGetFIFOPacketSize();
		LCD_Printf("DMP Initialization successful!");
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		LCD_Printf("%s %d %s", "DMP Initialization failed (code ", devStatus, ")");
	}
	return devStatus;
}

uint8_t* MPU6050_dmpGetData() {
	// wait for MPU interrupt or extra packet(s) available
	// get INT_STATUS byte
	mpuIntStatus = MPU6050_getIntStatus();
	// get current FIFO count
    fifoCount = MPU6050_getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
        MPU6050_resetFIFO();
        LCD_Printf("FIFO overflow!");
	// otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x03) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = MPU6050_getFIFOCount();
		// read a packet from FIFO
		MPU6050_getFIFOBytes(fifoBuffer, packetSize);
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;
		return fifoBuffer;
    }
    return 0;
}

#endif /* _MPU6050_DMP_H_ */

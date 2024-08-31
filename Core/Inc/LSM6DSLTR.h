/*
 * LSM6DSLTR.h
 *
 *  Created on: Feb 19, 2024
 *      Author: oguzk
 */

#ifndef INC_LSM6DSLTR_H_
#define INC_LSM6DSLTR_H_

#include "stm32f4xx.h"
#include "stdint.h"
#include "stdio.h"
#include "math.h"

#define LSM6DSL_Write_Address	0xD4	// D6
#define LSM6DSL_Read_Address 	0xD5	// D7   sdo hattına bağlı
#define FUNC_CFG_ACCESS 0x01

#define INT1_CTR	0x0D
#define INT2_CTRL	0x0E
#define CTRL1_XL	0x10
#define CTRL2_G		0x11
#define CTRL3_C		0x12
#define CTRL4_C		0x13
#define CTRL5_C		0x14
#define CTRL6_C		0x15
#define CTRL7_G		0x16
#define CTRL8_XL	0x17
#define CTRL9_XL	0x18
#define CTRL10_C	0x19

#define STATUS_REG	0x1E

#define OUTX_L_G	0x22
#define OUTX_H_G	0x23
#define OUTY_L_G	0x24
#define OUTY_H_G	0x25
#define OUTZ_L_G	0x26
#define OUTZ_H_G	0x27

#define OUTX_L_XL	0x28
#define OUTX_H_XL	0x29
#define OUTY_L_XL	0x2A
#define OUTY_H_XL	0x2B
#define OUTZ_L_XL	0x2C
#define OUTZ_H_XL	0x2D


#define ALPHA 0.98f // Filtre katsayısı


typedef struct
{
	volatile float Accel_X;
	volatile float Accel_Y;
	volatile float Accel_Z;
	volatile float Gyro_X;
	volatile float Gyro_Y;
	volatile float Gyro_Z;
	volatile float Roll;
	volatile float Pitch;
	volatile float Yaw;
	volatile float Normal;

}LSM6DSLTR;



void LSM6DSLTR_Init();
void LSM6DSLTR_Read_Accel_Data(LSM6DSLTR* Lsm_Sensor);
void LSM6DSLTR_Read_Gyro_Data(LSM6DSLTR* Lsm_Sensor);
void calculate_roll_pitch(LSM6DSLTR *Lsm_Sensor);
void update_angles(LSM6DSLTR *Lsm_Sensor);
int IS_MPU_READY();
void update_angles_and_gravity(LSM6DSLTR *Lsm_Sensor);

#endif /* INC_LSM6DSLTR_H_ */

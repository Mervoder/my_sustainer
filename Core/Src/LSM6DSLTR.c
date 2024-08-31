/*
 * LSM6DSLTR.c
 *
 *  Created on: Feb 19, 2024
 *      Author: oguzk
 */


#include "LSM6DSLTR.h"

#define HP_FILTER_EN  0x40  // High-pass filter enable
#define HP_FILTER_CUT_OFF 0x00 // Cutoff frequency for high-pass filter (e.g., 0x00 for 16 Hz)
#define LP_FILTER_EN  0x40  // Low-pass filter enable
#define LP_FILTER_CUT_OFF 0x02 // Cutoff frequency for low-pass filter (e.g., 0x00 for 400 Hz)



float gyro_constant=0.01750;

uint32_t prev_time = 0;

//extern LSM6DSLTR;
extern I2C_HandleTypeDef hi2c1;

void LSM6DSLTR_Init()
{
	uint8_t data1;

	// Gyro ve Accel interrupt pin 1 aktif
	data1 = 0x54; //A4 16G 6.66khz
	HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_Write_Address, CTRL1_XL, 1, &data1,  1, 1);

	data1 = 0x54; // A4 500 dps 6.6khz
	HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_Write_Address,CTRL2_G, 1, &data1, 1, 1);

	data1= 0x00;
	HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_Write_Address, CTRL3_C, 1, &data1, 1, 1);

	data1= 0x08;
	HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_Write_Address, CTRL4_C, 1, &data1, 1, 1);

	data1 = 0x38;
	HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_Write_Address, CTRL10_C, 1, &data1, 1, 1);




}


void LSM6DSLTR_Read_Accel_Data(LSM6DSLTR* Lsm_Sensor)
{
	uint8_t data;
	uint8_t s;
	int16_t accel;


	HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_Read_Address, OUTZ_L_XL, 1, &data, 1, 1);

	HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_Read_Address, OUTZ_H_XL, 1, &s, 1, 1);

	accel = (int16_t) ( (s << 8 ) | (data));

	Lsm_Sensor->Accel_Z = (float)accel* 0.000488*9.81; // 16g mg/LSB 0.488



	HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_Read_Address, OUTX_L_XL, 1, &data, 1, 1);

	HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_Read_Address, OUTX_H_XL, 1, &s, 1, 1);

	accel = (int16_t) ( (s << 8 ) | (data));

	Lsm_Sensor->Accel_X= (float)accel* 0.000488*9.81; // 16g mg/LSB 0.488



	HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_Read_Address, OUTY_L_XL, 1, &data, 1, 1);

	HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_Read_Address, OUTY_H_XL, 1, &s, 1, 1);

	accel = (int16_t) ( (s << 8 ) | (data));

	Lsm_Sensor->Accel_Y = (float)accel* 0.000488*9.81; // 16g mg/LSB 0.488


}

void LSM6DSLTR_Read_Gyro_Data(LSM6DSLTR* Lsm_Sensor){

     	uint8_t data;
		uint8_t s;
		int16_t gyro;


		HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_Read_Address, OUTX_L_G, 1, &data, 1, 1);

		HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_Read_Address, OUTX_H_G, 1, &s, 1, 1);

		gyro = (int16_t) ( (s << 8 ) | (data));

		Lsm_Sensor->Gyro_X = (float)gyro*gyro_constant;



		HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_Read_Address, OUTY_L_G, 1, &data, 1, 1);

		HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_Read_Address, OUTY_H_G, 1, &s, 1, 1);

		gyro = (int16_t) ( (s << 8 ) | (data));

		Lsm_Sensor->Gyro_Y = (float)gyro*gyro_constant;


		HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_Read_Address, OUTZ_L_G, 1, &data, 1, 1);

		HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_Read_Address, OUTZ_H_G, 1, &s, 1, 1);

		gyro = (int16_t) ( (s << 8 ) | (data));

		Lsm_Sensor->Gyro_Z = (float)gyro*gyro_constant;

}

int IS_MPU_READY(){

	return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);

}


void calculate_roll_pitch(LSM6DSLTR *Lsm_Sensor) {
    Lsm_Sensor->Roll = atan2f(Lsm_Sensor->Accel_Y, sqrtf(Lsm_Sensor->Accel_X * Lsm_Sensor->Accel_X + Lsm_Sensor->Accel_Z * Lsm_Sensor->Accel_Z)) * 180.0f / 3.14;
    Lsm_Sensor->Pitch = atan2f(-Lsm_Sensor->Accel_X, sqrtf(Lsm_Sensor->Accel_Y * Lsm_Sensor->Accel_Y + Lsm_Sensor->Accel_Z * Lsm_Sensor->Accel_Z)) * 180.0f / 3.14;
}

void update_angles(LSM6DSLTR *Lsm_Sensor) {
    uint32_t current_time = HAL_GetTick(); // Şu anki zamanı al

    // Zaman farkını hesapla (saniye cinsinden)
    float dt = (current_time - prev_time) / 1000.0f;

    // Roll ve pitch açılarını güncelle (tamamlayıcı filtre)
    Lsm_Sensor->Roll = ALPHA * (Lsm_Sensor->Roll + Lsm_Sensor->Gyro_X * dt) + (1 - ALPHA) * Lsm_Sensor->Roll;
    Lsm_Sensor->Pitch = ALPHA * (Lsm_Sensor->Pitch + Lsm_Sensor->Gyro_Y * dt) + (1 - ALPHA) * Lsm_Sensor->Pitch;

    // Yaw açısını jiroskop verileriyle güncelle (basit zamanla entegrasyon)
  //  Lsm_Sensor->Yaw += Lsm_Sensor->Gyro_Z * dt;

    // Önceki zamanı güncelle
    prev_time = current_time;
}


void calculate_gravity_normal(LSM6DSLTR *Lsm_Sensor) {
    // İvmeölçer verilerinden roll ve pitch açılarını hesapla
    float roll = atan2f(Lsm_Sensor->Accel_Y, sqrtf(Lsm_Sensor->Accel_X * Lsm_Sensor->Accel_X + Lsm_Sensor->Accel_Z * Lsm_Sensor->Accel_Z)) * 180.0f / M_PI;
    float pitch = atan2f(-Lsm_Sensor->Accel_X, sqrtf(Lsm_Sensor->Accel_Y * Lsm_Sensor->Accel_Y + Lsm_Sensor->Accel_Z * Lsm_Sensor->Accel_Z)) * 180.0f / M_PI;

    // Açıları güncelle
    Lsm_Sensor->Roll = roll;
    Lsm_Sensor->Pitch = pitch;

    // Yerçekimi normali açısını hesapla (Roll ve Pitch açıları kullanılarak)
    // Normalde bu, roll ve pitch açılarının kombinasyonu ile hesaplanır
    // Örneğin:
    // Yerçekimi normali açısını hesaplamak için roll ve pitch açılarını kullanabiliriz
    float gravity_normal_angle = sqrtf(roll * roll + pitch * pitch);

    // Yerçekimi normali açısını kaydet
    Lsm_Sensor->Normal = gravity_normal_angle;
}



void update_angles_and_gravity(LSM6DSLTR *Lsm_Sensor) {
    // İvmeölçer ve jiroskop verilerini oku
    LSM6DSLTR_Read_Accel_Data(Lsm_Sensor);
    LSM6DSLTR_Read_Gyro_Data(Lsm_Sensor);

    // Roll ve pitch açılarını güncelle
    update_angles(Lsm_Sensor);

    // Yerçekimi normali açısını hesapla
    calculate_gravity_normal(Lsm_Sensor);
}

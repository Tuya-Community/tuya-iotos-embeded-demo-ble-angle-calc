/**
 * @file tuya_app_angle_calc.c
 * @author lifan
 * @brief attitude angles calculation application source file
 * @version 1.0.0
 * @date 2022-01-07
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */

#include "tuya_app_angle_calc.h"
#include "tuya_svc_angle_calc.h"
#include "tuya_mpu6050.h"
#include "tuya_ble_log.h"
#include "tuya_gpio.h"
#include "ty_uart.h"
#include "tuya_ble_utils.h"

/***********************************************************
************************micro define************************
***********************************************************/
#define ANGLE_CALC_BY_QUAT  0
#define DELTA_T             0.005f
#define DAQ_TIME_MS         5

#define MPU_CT_POW_PIN      TY_GPIO_16
#define MPU_INT_PIN         TY_GPIO_2

/***********************************************************
***********************typedef define***********************
***********************************************************/
typedef BYTE_T DBG_DATA_TYPE_E;
#define DBG_GYRO            0x00
#define DBG_ACCEL           0x01
#define DBG_ANGLE           0x02

/***********************************************************
***********************variable define**********************
***********************************************************/
STATIC BOOL_T sg_new_data_ready = CLR;
STATIC FLOAT_T sg_gyro[3] = {0.0f, 0.0f, 0.0f};
STATIC FLOAT_T sg_accel[3] = {0.0f, 0.0f, 0.0f};
STATIC FLOAT_T sg_angle[3] = {0.0f, 0.0f, 0.0f};

/***********************************************************
***********************function define**********************
***********************************************************/
/**
 * @brief send data to virtual oscilloscope
 * @param[in] data: data to be sent
 * @return none
 */
STATIC VOID_T __send_data_to_vi(_IN SHORT_T data1, _IN SHORT_T data2, _IN SHORT_T data3, _IN SHORT_T data4)
{
    USHORT_T crc16 = 0xFFFF;
    UCHAR_T send_buf[10];

    send_buf[0] = (UCHAR_T)((UINT_T)((INT_T)data1) % 256);
    send_buf[1] = (UCHAR_T)((UINT_T)((INT_T)data1) / 256);
    send_buf[2] = (UCHAR_T)((UINT_T)((INT_T)data2) % 256);
    send_buf[3] = (UCHAR_T)((UINT_T)((INT_T)data2) / 256);
    send_buf[4] = (UCHAR_T)((UINT_T)((INT_T)data3) % 256);
    send_buf[5] = (UCHAR_T)((UINT_T)((INT_T)data3) / 256);
    send_buf[6] = (UCHAR_T)((UINT_T)((INT_T)data4) % 256);
    send_buf[7] = (UCHAR_T)((UINT_T)((INT_T)data4) / 256);
    crc16 = tuya_ble_crc16_compute(send_buf, 8, NULL);
    send_buf[8] = (UCHAR_T)(crc16 % 256);
    send_buf[9] = (UCHAR_T)(crc16 / 256);
    ty_uart_send(send_buf, 10);
}

/**
 * @brief debug
 * @param[in] type: data type
 * @return none
 */
STATIC VOID_T __debug_angle_data(_IN CONST DBG_DATA_TYPE_E type)
{
    switch (type) {
    case DBG_GYRO:
        __send_data_to_vi((SHORT_T)sg_gyro[0], (SHORT_T)sg_gyro[1], (SHORT_T)sg_gyro[2], 0);
        break;
    case DBG_ACCEL:
        __send_data_to_vi((SHORT_T)sg_accel[0], (SHORT_T)sg_accel[1], (SHORT_T)sg_accel[2], 0);
        break;
    case DBG_ANGLE:
        __send_data_to_vi((SHORT_T)sg_angle[0], (SHORT_T)sg_angle[1], (SHORT_T)sg_angle[2], 0);
        break;
    default:
        break;
    }
}

/**
 * @brief get IMU data
 * @param[in] gyro: gyro data
 * @param[in] accel: accel data
 * @return TRUE - success, FALSE - fail
 */
VOID_T __get_imu_data(FLOAT_T *gyro, FLOAT_T *accel)
{
    tuya_mpu6050_read_gyro_spec_unit(gyro+1, gyro, gyro+2, MPU_GDT_DPS);
    tuya_mpu6050_read_accel_spec_unit(accel+1, accel, accel+2, MPU_ADT_MPS2);
    gyro[1] = -gyro[1];
    accel[1] = -accel[1];
}

/**
 * @brief sensor new data ready callback
 * @param[in] none
 * @return none
 */
STATIC VOID_T __new_data_ready_cb(VOID_T)
{
    sg_new_data_ready = SET;
}

/**
 * @brief IMU DAQ module init
 * @param[in] none
 * @return none
 */
VOID_T __imu_daq_init(VOID_T)
{
    tuya_mpu6050_power_on(MPU_CT_POW_PIN, TRUE);
    MPU_RET ret = tuya_mpu6050_init(MPU_CLK_PLL_XGYRO, MPU_GYRO_FS_2000, MPU_ACCEL_FS_16, 1000/DAQ_TIME_MS, MPU_INT_PIN, TY_GPIO_IRQ_FALLING, __new_data_ready_cb);
    if (MPU_OK != ret) {
        TUYA_APP_LOG_ERROR("tuya_mpu6050_init error: %d.", ret);
        return;
    }
}

/**
 * @brief angles calculation init
 * @param[in] none
 * @return none
 */
VOID_T tuya_app_angle_calc_init(VOID_T)
{
    __imu_daq_init();
}

/**
 * @brief angles calculation loop
 * @param[in] none
 * @return none
 */
VOID_T tuya_app_angle_calc_loop(VOID_T)
{
    if (sg_new_data_ready) {
        sg_new_data_ready = CLR;

        __get_imu_data(sg_gyro, sg_accel);

#if (ANGLE_CALC_BY_QUAT == 0)
        tuya_calc_angles(DELTA_T, TRUE,
                         sg_gyro[0], sg_gyro[1], sg_gyro[2],
                         sg_accel[0], sg_accel[1], sg_accel[2],
                         &sg_angle[0], &sg_angle[1], &sg_angle[2]);
#else
        tuya_calc_angles_quat(DELTA_T, TRUE,
                              sg_gyro[0], sg_gyro[1], sg_gyro[2],
                              sg_accel[0], sg_accel[1], sg_accel[2],
                              &sg_angle[0], &sg_angle[1], &sg_angle[2]);
#endif
        __debug_angle_data(DBG_ANGLE);
    }
}

/**
 * @file tuya_app_angle_calc.h
 * @author lifan
 * @brief attitude angles calculation application header file
 * @version 1.0.0
 * @date 2022-01-07
 *
 * @copyright Copyright (c) tuya.inc 2022
 *
 */

#ifndef __TUYA_APP_ANGLE_CALC_H__
#define __TUYA_APP_ANGLE_CALC_H__

#include "tuya_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************
************************micro define************************
***********************************************************/

/***********************************************************
***********************typedef define***********************
***********************************************************/

/***********************************************************
***********************variable define**********************
***********************************************************/

/***********************************************************
***********************function define**********************
***********************************************************/
/**
 * @brief angles calculation init
 * @param[in] none
 * @return none
 */
VOID_T tuya_app_angle_calc_init(VOID_T);

/**
 * @brief angles calculation loop
 * @param[in] none
 * @return none
 */
VOID_T tuya_app_angle_calc_loop(VOID_T);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __TUYA_APP_ANGLE_CALC_H__ */

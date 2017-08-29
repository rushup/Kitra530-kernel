#ifndef LSM6DSL_EVENT_H
#define LSM6DSL_EVENT_H

#include "LSM6DSL_ACC_GYRO_driver.h"

#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_2G   0.061  /**< Sensitivity value for 2 g full scale [mg/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_4G   0.122  /**< Sensitivity value for 4 g full scale [mg/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_8G   0.244  /**< Sensitivity value for 8 g full scale [mg/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_16G  0.488  /**< Sensitivity value for 16 g full scale [mg/LSB] */

/**
 * @}
 */

/** @addtogroup LSM6DSL_GYRO_SENSITIVITY Gyro sensitivity values based on selected full scale
 * @{
 */

#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_125DPS   04.375  /**< Sensitivity value for 125 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_245DPS   08.750  /**< Sensitivity value for 245 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_500DPS   17.500  /**< Sensitivity value for 500 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_1000DPS  35.000  /**< Sensitivity value for 1000 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_2000DPS  70.000  /**< Sensitivity value for 2000 dps full scale [mdps/LSB] */

/**
 * @}
 */

/** @addtogroup LSM6DSL_PEDOMETER_THRESHOLD Pedometer threshold values
 * @{
 */

#define LSM6DSL_PEDOMETER_THRESHOLD_LOW       0x00  /**< Lowest  value of pedometer threshold */
#define LSM6DSL_PEDOMETER_THRESHOLD_MID_LOW   0x07
#define LSM6DSL_PEDOMETER_THRESHOLD_MID       0x0F
#define LSM6DSL_PEDOMETER_THRESHOLD_MID_HIGH  0x17
#define LSM6DSL_PEDOMETER_THRESHOLD_HIGH      0x1F  /**< Highest value of pedometer threshold */

/**
 * @}
 */

/** @addtogroup LSM6DSL_WAKE_UP_THRESHOLD Wake up threshold values
 * @{
 */

#define LSM6DSL_WAKE_UP_THRESHOLD_LOW       0x01  /**< Lowest  value of wake up threshold */
#define LSM6DSL_WAKE_UP_THRESHOLD_MID_LOW   0x0F
#define LSM6DSL_WAKE_UP_THRESHOLD_MID       0x1F
#define LSM6DSL_WAKE_UP_THRESHOLD_MID_HIGH  0x2F
#define LSM6DSL_WAKE_UP_THRESHOLD_HIGH      0x3F  /**< Highest value of wake up threshold */

/**
 * @}
 */

/** @addtogroup LSM6DSL_TAP_THRESHOLD Tap threshold values
 * @{
 */

#define LSM6DSL_TAP_THRESHOLD_LOW       0x01  /**< Lowest  value of wake up threshold */
#define LSM6DSL_TAP_THRESHOLD_MID_LOW   0x08
#define LSM6DSL_TAP_THRESHOLD_MID       0x10
#define LSM6DSL_TAP_THRESHOLD_MID_HIGH  0x18
#define LSM6DSL_TAP_THRESHOLD_HIGH      0x1F  /**< Highest value of wake up threshold */

/**
 * @}
 */

/** @addtogroup LSM6DSL_TAP_SHOCK_TIME Tap shock time window values
 * @{
 */

#define LSM6DSL_TAP_SHOCK_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
#define LSM6DSL_TAP_SHOCK_TIME_MID_LOW   0x01
#define LSM6DSL_TAP_SHOCK_TIME_MID_HIGH  0x02
#define LSM6DSL_TAP_SHOCK_TIME_HIGH      0x03  /**< Highest value of wake up threshold */

/**
 * @}
 */

/** @addtogroup LSM6DSL_TAP_QUIET_TIME Tap quiet time window values
 * @{
 */

#define LSM6DSL_TAP_QUIET_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
#define LSM6DSL_TAP_QUIET_TIME_MID_LOW   0x01
#define LSM6DSL_TAP_QUIET_TIME_MID_HIGH  0x02
#define LSM6DSL_TAP_QUIET_TIME_HIGH      0x03  /**< Highest value of wake up threshold */

/**
 * @}
 */

/** @addtogroup LSM6DSL_TAP_DURATION_TIME Tap duration time window values
 * @{
 */

#define LSM6DSL_TAP_DURATION_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
#define LSM6DSL_TAP_DURATION_TIME_MID_LOW   0x04
#define LSM6DSL_TAP_DURATION_TIME_MID       0x08
#define LSM6DSL_TAP_DURATION_TIME_MID_HIGH  0x0C
#define LSM6DSL_TAP_DURATION_TIME_HIGH      0x0F  /**< Highest value of wake up threshold */

static int lsm6dsl_get_wakeup_status( void *handle, u8_t *status )
{

  LSM6DSL_ACC_GYRO_WU_EV_STATUS_t wake_up_status;

  if ( LSM6DSL_ACC_GYRO_R_WU_EV_STATUS( (void *)handle, &wake_up_status ) == MEMS_ERROR )
  {
    return -1;
  }

  switch( wake_up_status )
  {
    case LSM6DSL_ACC_GYRO_WU_EV_STATUS_DETECTED:
      *status = 1;
      break;
    case LSM6DSL_ACC_GYRO_WU_EV_STATUS_NOT_DETECTED:
      *status = 0;
      break;
    default:
      return -1;
  }

  return 0;
}

static int lsm6dsl_get_tilt_status( void *handle, u8_t *status )
{

  LSM6DSL_ACC_GYRO_TILT_EV_STATUS_t tilt_status;

  if ( LSM6DSL_ACC_GYRO_R_TILT_EV_STATUS( (void *)handle, &tilt_status ) == MEMS_ERROR )
  {
    return -1;
  }

  switch( tilt_status )
  {
    case LSM6DSL_ACC_GYRO_TILT_EV_STATUS_DETECTED:
      *status = 1;
      break;
    case LSM6DSL_ACC_GYRO_TILT_EV_STATUS_NOT_DETECTED:
      *status = 0;
      break;
    default:
      return -1;
  }

  return 0;
}

static int lsm6dsl_get_pedometer_step_count( void *handle, u16_t *step_count )
{

  if ( LSM6DSL_ACC_GYRO_Get_GetStepCounter( (void *)handle, ( u8_t* )step_count ) == MEMS_ERROR )
  {
    return -1;
  }

  return 0;
}


static int lsm6dsl_pedometer_enable_step_reset( void *handle )
{

  if ( LSM6DSL_ACC_GYRO_W_PedoStepReset( (void *)handle, LSM6DSL_ACC_GYRO_PEDO_RST_STEP_ENABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  return 0;
}


static int lsm6dsl_pedometer_disable_step_reset( void *handle )
{

  if ( LSM6DSL_ACC_GYRO_W_PedoStepReset( (void *)handle, LSM6DSL_ACC_GYRO_PEDO_RST_STEP_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  return 0;
}

static int lsm6dsl_get_pedometer_status( void *handle, u8_t *status )
{

  LSM6DSL_ACC_GYRO_PEDO_EV_STATUS_t pedometer_status;

  if ( LSM6DSL_ACC_GYRO_R_PEDO_EV_STATUS( (void *)handle, &pedometer_status ) == MEMS_ERROR )
  {
    return -1;
  }

  switch( pedometer_status )
  {
    case LSM6DSL_ACC_GYRO_PEDO_EV_STATUS_DETECTED:
      *status = 1;
      break;
    case LSM6DSL_ACC_GYRO_PEDO_EV_STATUS_NOT_DETECTED:
      *status = 0;
      break;
    default:
      return -1;
  }

  return 0;
}

static int lsm6dsl_get_freefall_status( void *handle, u8_t *status )
{

  LSM6DSL_ACC_GYRO_FF_EV_STATUS_t free_fall_status;

  if ( LSM6DSL_ACC_GYRO_R_FF_EV_STATUS( (void *)handle, &free_fall_status ) == MEMS_ERROR )
  {
    return -1;
  }

  switch( free_fall_status )
  {
    case LSM6DSL_ACC_GYRO_FF_EV_STATUS_DETECTED:
      *status = 1;
      break;
    case LSM6DSL_ACC_GYRO_FF_EV_STATUS_NOT_DETECTED:
      *status = 0;
      break;
    default:
      return -1;
  }

  return 0;
}



static int lsm6dsl_get_tap_and_dtap_status( void *handle, u8_t *status_tap, u8_t* status_dtap )
{
	u8_t value = 0;
	if( !LSM6DSL_ACC_GYRO_ReadReg(handle, LSM6DSL_ACC_GYRO_TAP_SRC, (u8_t *)&value, 1) )
		return -1;

	if(value & LSM6DSL_ACC_GYRO_DOUBLE_TAP_EV_STATUS_MASK)
		*status_dtap = 1;
	else
		*status_dtap = 0;

	if(value & LSM6DSL_ACC_GYRO_SINGLE_TAP_EV_STATUS_MASK)
		*status_tap = 1;
	else
		*status_tap = 0;

	return 0;
}

static int LSM6DSL_X_Set_Tap_Quiet_Time( void *handle, u8_t time )
{

  if ( LSM6DSL_ACC_GYRO_W_QUIET_Duration( (void *)handle, time ) == MEMS_ERROR )
  {
    return -1;
  }

  return 0;
}

static int LSM6DSL_X_Set_Tap_Shock_Time( void* handle, u8_t time )
{

  if ( LSM6DSL_ACC_GYRO_W_SHOCK_Duration( (void *)handle, time ) == MEMS_ERROR )
  {
    return -1;
  }

  return 0;
}

static int LSM6DSL_X_Set_Tap_Threshold( void* handle, u8_t thr )
{

  if ( LSM6DSL_ACC_GYRO_W_TAP_THS( (void *)handle, thr ) == MEMS_ERROR )
  {
    return -1;
  }

  return 0;
}

static int LSM6DSL_X_Set_Tap_Duration_Time( void *handle, u8_t time )
{

  if ( LSM6DSL_ACC_GYRO_W_DUR( (void *)handle, time ) == MEMS_ERROR )
  {
    return -1;
  }

  return 0;
}

static int LSM6DSL_X_Set_Pedometer_Threshold( void *handle, u8_t thr )
{

  if ( LSM6DSL_ACC_GYRO_W_PedoThreshold( (void *)handle, thr ) == MEMS_ERROR )
  {
    return -1;
  }

  return 0;
}

static int lsm6dsl_disable_tap( void *handle )
{

  /* Disable single tap interrupt on INT1 pin. */
  if ( LSM6DSL_ACC_GYRO_W_SingleTapOnInt1( (void *)handle, LSM6DSL_ACC_GYRO_INT1_SINGLE_TAP_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable single tap interrupt on INT2 pin. */
  if ( LSM6DSL_ACC_GYRO_W_SingleTapOnInt2( (void *)handle, LSM6DSL_ACC_GYRO_INT2_SINGLE_TAP_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable basic Interrupts */
  if ( LSM6DSL_ACC_GYRO_W_BASIC_INT( (void *)handle, LSM6DSL_ACC_GYRO_BASIC_INT_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Reset tap threshold. */
  if ( LSM6DSL_X_Set_Tap_Threshold( handle, 0x0 ) == -1 )
  {
    return -1;
  }

  /* Reset tap shock time window. */
  if ( LSM6DSL_X_Set_Tap_Shock_Time( handle, 0x0 ) == -1 )
  {
    return -1;
  }

  /* Reset tap quiet time window. */
  if ( LSM6DSL_X_Set_Tap_Quiet_Time( handle, 0x0 ) == -1 )
  {
    return -1;
  }

  /* _NOTE_: Tap duration time window - don't care for single tap. */

  /* _NOTE_: Single/Double Tap event - don't care of this flag for single tap. */

  /* Disable Z direction in tap recognition. */
  if ( LSM6DSL_ACC_GYRO_W_TAP_Z_EN( (void *)handle, LSM6DSL_ACC_GYRO_TAP_Z_EN_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable Y direction in tap recognition. */
  if ( LSM6DSL_ACC_GYRO_W_TAP_Y_EN( (void *)handle, LSM6DSL_ACC_GYRO_TAP_Y_EN_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable X direction in tap recognition. */
  if ( LSM6DSL_ACC_GYRO_W_TAP_X_EN( (void *)handle, LSM6DSL_ACC_GYRO_TAP_X_EN_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  return 0;
}



static int lsm6dsl_enable_tap( void* handle, u8_t int_pin )
{

/*
  // Output Data Rate selection
  if(LSM6DSL_X_Set_ODR_Value(handle, 416.0f) == -1)
  {
    return -1;
  }

  // Full scale selection 
  if ( LSM6DSL_ACC_GYRO_W_FS_XL( (void *)handle, LSM6DSL_ACC_GYRO_FS_XL_2g ) == MEMS_ERROR )
  {
    return -1;
  }
*/
  /* Enable X direction in tap recognition. */
  if ( LSM6DSL_ACC_GYRO_W_TAP_X_EN( (void *)handle, LSM6DSL_ACC_GYRO_TAP_X_EN_ENABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Enable Y direction in tap recognition. */
  if ( LSM6DSL_ACC_GYRO_W_TAP_Y_EN( (void *)handle, LSM6DSL_ACC_GYRO_TAP_Y_EN_ENABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Enable Z direction in tap recognition. */
  if ( LSM6DSL_ACC_GYRO_W_TAP_Z_EN( (void *)handle, LSM6DSL_ACC_GYRO_TAP_Z_EN_ENABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Set tap threshold. */
  if ( LSM6DSL_X_Set_Tap_Threshold( handle, LSM6DSL_TAP_THRESHOLD_MID_LOW ) == -1 )
  {
    return -1;
  }

  /* Set tap shock time window. */
  if ( LSM6DSL_X_Set_Tap_Shock_Time( handle, LSM6DSL_TAP_SHOCK_TIME_MID_HIGH ) == -1 )
  {
    return -1;
  }

  /* Set tap quiet time window. */
  if ( LSM6DSL_X_Set_Tap_Quiet_Time( handle, LSM6DSL_TAP_QUIET_TIME_MID_LOW ) == -1 )
  {
    return -1;
  }

  /* _NOTE_: Tap duration time window - don't care for single tap. */

  /* _NOTE_: Single/Double Tap event - don't care of this flag for single tap. */

  /* Enable basic Interrupts */
  if ( LSM6DSL_ACC_GYRO_W_BASIC_INT( (void *)handle, LSM6DSL_ACC_GYRO_BASIC_INT_ENABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Enable single tap on either INT1 or INT2 pin */
  switch (int_pin)
  {
  case 1:
    if ( LSM6DSL_ACC_GYRO_W_SingleTapOnInt1( (void *)handle, LSM6DSL_ACC_GYRO_INT1_SINGLE_TAP_ENABLED ) == MEMS_ERROR )
    {
      return -1;
    }
    break;

  case 2:
    if ( LSM6DSL_ACC_GYRO_W_SingleTapOnInt2( (void *)handle, LSM6DSL_ACC_GYRO_INT2_SINGLE_TAP_ENABLED ) == MEMS_ERROR )
    {
      return -1;
    }
    break;

  default:
    break;
  }

  return 0;
}


static int lsm6dsl_disable_dtap( void *handle )
{

  /* Disable double tap interrupt on INT1 pin. */
  if ( LSM6DSL_ACC_GYRO_W_TapEvOnInt1( (void *)handle, LSM6DSL_ACC_GYRO_INT1_TAP_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable double tap interrupt on INT2 pin. */
  if ( LSM6DSL_ACC_GYRO_W_TapEvOnInt2( (void *)handle, LSM6DSL_ACC_GYRO_INT2_TAP_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable basic Interrupts */
  if ( LSM6DSL_ACC_GYRO_W_BASIC_INT( (void *)handle, LSM6DSL_ACC_GYRO_BASIC_INT_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Reset tap threshold. */
  if ( LSM6DSL_X_Set_Tap_Threshold( handle, 0x0 ) == -1 )
  {
    return -1;
  }

  /* Reset tap shock time window. */
  if ( LSM6DSL_X_Set_Tap_Shock_Time( handle, 0x0 ) == -1 )
  {
    return -1;
  }

  /* Reset tap quiet time window. */
  if ( LSM6DSL_X_Set_Tap_Quiet_Time( handle, 0x0 ) == -1 )
  {
    return -1;
  }

  /* Reset tap duration time window. */
  if ( LSM6DSL_X_Set_Tap_Duration_Time( handle, 0x0 ) == -1 )
  {
    return -1;
  }

  /* Only single tap enabled. */
  if ( LSM6DSL_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV( (void *)handle,
       LSM6DSL_ACC_GYRO_SINGLE_DOUBLE_TAP_SINGLE_TAP ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable Z direction in tap recognition. */
  if ( LSM6DSL_ACC_GYRO_W_TAP_Z_EN( (void *)handle, LSM6DSL_ACC_GYRO_TAP_Z_EN_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable Y direction in tap recognition. */
  if ( LSM6DSL_ACC_GYRO_W_TAP_Y_EN( (void *)handle, LSM6DSL_ACC_GYRO_TAP_Y_EN_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable X direction in tap recognition. */
  if ( LSM6DSL_ACC_GYRO_W_TAP_X_EN( (void *)handle, LSM6DSL_ACC_GYRO_TAP_X_EN_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  return 0;
}



static int lsm6dsl_enable_dtap( void *handle, u8_t int_pin )
{

/*  //Output Data Rate selection
  if(LSM6DSL_X_Set_ODR_Value(handle, 416.0f) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

   //Full scale selection
  if ( LSM6DSL_ACC_GYRO_W_FS_XL( (void *)handle, LSM6DSL_ACC_GYRO_FS_XL_2g ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
*/

  /* Enable X direction in tap recognition. */
  if ( LSM6DSL_ACC_GYRO_W_TAP_X_EN( (void *)handle, LSM6DSL_ACC_GYRO_TAP_X_EN_ENABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Enable Y direction in tap recognition. */
  if ( LSM6DSL_ACC_GYRO_W_TAP_Y_EN( (void *)handle, LSM6DSL_ACC_GYRO_TAP_Y_EN_ENABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Enable Z direction in tap recognition. */
  if ( LSM6DSL_ACC_GYRO_W_TAP_Z_EN( (void *)handle, LSM6DSL_ACC_GYRO_TAP_Z_EN_ENABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Set tap threshold. */
  if ( LSM6DSL_X_Set_Tap_Threshold( handle, LSM6DSL_TAP_THRESHOLD_MID_LOW ) == -1 )
  {
    return -1;
  }

  /* Set tap shock time window. */
  if ( LSM6DSL_X_Set_Tap_Shock_Time( handle, LSM6DSL_TAP_SHOCK_TIME_HIGH ) == -1 )
  {
    return -1;
  }

  /* Set tap quiet time window. */
  if ( LSM6DSL_X_Set_Tap_Quiet_Time( handle, LSM6DSL_TAP_QUIET_TIME_HIGH ) == -1 )
  {
    return -1;
  }

  /* Set tap duration time window. */
  if ( LSM6DSL_X_Set_Tap_Duration_Time( handle, LSM6DSL_TAP_DURATION_TIME_MID ) == -1 )
  {
    return -1;
  }

  /* Single and double tap enabled. */
  if ( LSM6DSL_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV( (void *)handle,
       LSM6DSL_ACC_GYRO_SINGLE_DOUBLE_TAP_DOUBLE_TAP ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Enable basic Interrupts */
  if ( LSM6DSL_ACC_GYRO_W_BASIC_INT( (void *)handle, LSM6DSL_ACC_GYRO_BASIC_INT_ENABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Enable double tap on either INT1 or INT2 pin */
  switch (int_pin)
  {
  case 1:
    if ( LSM6DSL_ACC_GYRO_W_TapEvOnInt1( (void *)handle, LSM6DSL_ACC_GYRO_INT1_TAP_ENABLED ) == MEMS_ERROR )
    {
      return -1;
    }
    break;

  case 2:
    if ( LSM6DSL_ACC_GYRO_W_TapEvOnInt2( (void *)handle, LSM6DSL_ACC_GYRO_INT2_TAP_ENABLED ) == MEMS_ERROR )
    {
      return -1;
    }
    break;

  default:
    break;
  }

  return 0;
}

static int lsm6dsl_disable_tilt( void * handle )
{

  /* Disable tilt event on INT1. */
  if ( LSM6DSL_ACC_GYRO_W_TiltEvOnInt1( (void *)handle, LSM6DSL_ACC_GYRO_INT1_TILT_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable tilt event on INT2. */
  if ( LSM6DSL_ACC_GYRO_W_TiltEvOnInt2( (void *)handle, LSM6DSL_ACC_GYRO_INT2_TILT_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable tilt calculation. */
  if ( LSM6DSL_ACC_GYRO_W_TILT( (void *)handle, LSM6DSL_ACC_GYRO_TILT_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable embedded functionalities */
  if ( LSM6DSL_ACC_GYRO_W_FUNC_EN( (void *)handle, LSM6DSL_ACC_GYRO_FUNC_EN_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  return 0;
}

static int lsm6dsl_enable_tilt( void *handle, u8_t int_pin )
{
/*
  // Output Data Rate selection
  if(LSM6DSL_X_Set_ODR_Value(handle, 26.0f) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  // Full scale selection
  if ( LSM6DSL_ACC_GYRO_W_FS_XL( (void *)handle, LSM6DSL_ACC_GYRO_FS_XL_2g ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
*/
  /* Enable embedded functionalities */
  if ( LSM6DSL_ACC_GYRO_W_FUNC_EN( (void *)handle, LSM6DSL_ACC_GYRO_FUNC_EN_ENABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Enable tilt calculation. */
  if ( LSM6DSL_ACC_GYRO_W_TILT( (void *)handle, LSM6DSL_ACC_GYRO_TILT_ENABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Enable tilt detection on either INT1 or INT2 pin */
  switch (int_pin)
  {
  case 1:
    if ( LSM6DSL_ACC_GYRO_W_TiltEvOnInt1( (void *)handle, LSM6DSL_ACC_GYRO_INT1_TILT_ENABLED ) == MEMS_ERROR )
    {
      return -1;
    }
    break;

  case 2:
    if ( LSM6DSL_ACC_GYRO_W_TiltEvOnInt2( (void *)handle, LSM6DSL_ACC_GYRO_INT2_TILT_ENABLED ) == MEMS_ERROR )
    {
      return -1;
    }
    break;

  default:
    break;
  }

  return 0;
}

static int lsm6dsl_disable_pedometer( void *handle )
{

  /* Disable pedometer on INT1. */
  if ( LSM6DSL_ACC_GYRO_W_STEP_DET_on_INT1( (void *)handle, LSM6DSL_ACC_GYRO_INT1_PEDO_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable pedometer algorithm. */
  if ( LSM6DSL_ACC_GYRO_W_PEDO( (void *)handle, LSM6DSL_ACC_GYRO_PEDO_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable embedded functionalities. */
  if ( LSM6DSL_ACC_GYRO_W_FUNC_EN( (void *)handle, LSM6DSL_ACC_GYRO_FUNC_EN_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Reset pedometer threshold. */
  if ( LSM6DSL_X_Set_Pedometer_Threshold( handle, 0x0 ) == -1 )
  {
    return -1;
  }

  return 0;
}

static int lsm6dsl_enable_pedometer( void *handle, u8_t int_pin )
{

/*
  // Output Data Rate selection
  if(LSM6DSL_X_Set_ODR_Value(handle, 26.0f) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  // Full scale selection.
  if ( LSM6DSL_ACC_GYRO_W_FS_XL( (void *)handle, LSM6DSL_ACC_GYRO_FS_XL_2g ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
*/

  /* Set pedometer threshold. */
  if ( LSM6DSL_X_Set_Pedometer_Threshold( handle, LSM6DSL_PEDOMETER_THRESHOLD_MID_HIGH ) == -1 )
  {
    return -1;
  }

  /* Enable embedded functionalities. */
  if ( LSM6DSL_ACC_GYRO_W_FUNC_EN( (void *)handle, LSM6DSL_ACC_GYRO_FUNC_EN_ENABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Enable pedometer algorithm. */
  if ( LSM6DSL_ACC_GYRO_W_PEDO( (void *)handle, LSM6DSL_ACC_GYRO_PEDO_ENABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  if(int_pin == 1) {
    /* Enable pedometer on INT1 pin */
    if ( LSM6DSL_ACC_GYRO_W_STEP_DET_on_INT1( (void *)handle, LSM6DSL_ACC_GYRO_INT1_PEDO_ENABLED ) == MEMS_ERROR )
    {
      return -1;
    }
  }

  return 0;
}

static int lsm6dsl_enable_freefall( void *handle, u8_t int_pin )
{

/*
  // Output Data Rate selection
  if(LSM6DSL_X_Set_ODR_Value(handle, 416.0f) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  // Full scale selection
  if ( LSM6DSL_ACC_GYRO_W_FS_XL( (void *)handle, LSM6DSL_ACC_GYRO_FS_XL_2g ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
*/
  /* FF_DUR setting */
  if ( LSM6DSL_ACC_GYRO_W_FF_Duration( (void *)handle, 0x06 ) == MEMS_ERROR )
  {
    return -1;
  }

  /* WAKE_DUR setting */
  if ( LSM6DSL_ACC_GYRO_W_WAKE_DUR( (void *)handle, 0x00 ) == MEMS_ERROR )
  {
    return -1;
  }

  /* TIMER_HR setting */
  if ( LSM6DSL_ACC_GYRO_W_TIMER_HR( (void *)handle, LSM6DSL_ACC_GYRO_TIMER_HR_6_4ms ) == MEMS_ERROR )
  {
    return -1;
  }

  /* SLEEP_DUR setting */
  if ( LSM6DSL_ACC_GYRO_W_SLEEP_DUR( (void *)handle, 0x00 ) == MEMS_ERROR )
  {
    return -1;
  }

  /* FF_THS setting */
  if ( LSM6DSL_ACC_GYRO_W_FF_THS( (void *) handle, LSM6DSL_ACC_GYRO_FF_THS_312mg ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Enable basic Interrupts */
  if ( LSM6DSL_ACC_GYRO_W_BASIC_INT( (void *)handle, LSM6DSL_ACC_GYRO_BASIC_INT_ENABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Enable free fall event on either INT1 or INT2 pin */
  switch (int_pin)
  {
  case 1:
    if ( LSM6DSL_ACC_GYRO_W_FFEvOnInt1( (void *)handle, LSM6DSL_ACC_GYRO_INT1_FF_ENABLED ) == MEMS_ERROR )
    {
      return -1;
    }
    break;

  case 2:
    if ( LSM6DSL_ACC_GYRO_W_FFEvOnInt2( (void *)handle, LSM6DSL_ACC_GYRO_INT2_FF_ENABLED ) == MEMS_ERROR )
    {
      return -1;
    }
    break;

  default:
    break;
  }

  return 0;
}

static int lsm6dsl_disable_freefall( void *handle )
{

  /* Disable free fall event on INT1 pin */
  if ( LSM6DSL_ACC_GYRO_W_FFEvOnInt1( (void *)handle, LSM6DSL_ACC_GYRO_INT1_FF_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable free fall event on INT2 pin */
  if ( LSM6DSL_ACC_GYRO_W_FFEvOnInt2( (void *)handle, LSM6DSL_ACC_GYRO_INT2_FF_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable basic Interrupts */
  if ( LSM6DSL_ACC_GYRO_W_BASIC_INT( (void *)handle, LSM6DSL_ACC_GYRO_BASIC_INT_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* FF_DUR setting */
  if ( LSM6DSL_ACC_GYRO_W_FF_Duration( (void *)handle, 0x00 ) == MEMS_ERROR )
  {
    return -1;
  }

  /* FF_THS setting */
  if ( LSM6DSL_ACC_GYRO_W_FF_THS( (void *)handle, LSM6DSL_ACC_GYRO_FF_THS_156mg ) == MEMS_ERROR )
  {
    return -1;
  }

  return 0;
}

static int lsm6dsl_enable_motion( void *handle, u8_t int_pin )
{

/*
  // Output Data Rate selection
  if(LSM6DSL_X_Set_ODR_Value(handle, 416.0f) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  // Full scale selection
  if ( LSM6DSL_ACC_GYRO_W_FS_XL( (void *)handle, LSM6DSL_ACC_GYRO_FS_XL_2g ) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
*/
  /* WAKE_DUR setting */
  if ( LSM6DSL_ACC_GYRO_W_WAKE_DUR( (void *)handle, 0x00 ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Set wake up threshold. */
  if ( LSM6DSL_ACC_GYRO_W_WK_THS( (void *)handle, 0x02 ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Enable basic Interrupts */
  if ( LSM6DSL_ACC_GYRO_W_BASIC_INT( (void *)handle, LSM6DSL_ACC_GYRO_BASIC_INT_ENABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Enable wake up detection on either INT1 or INT2 pin */
  switch (int_pin)
  {
  case 1:
    if ( LSM6DSL_ACC_GYRO_W_WUEvOnInt1( (void *)handle, LSM6DSL_ACC_GYRO_INT1_WU_ENABLED ) == MEMS_ERROR )
    {
      return -1;
    }
    break;

  case 2:
    if ( LSM6DSL_ACC_GYRO_W_WUEvOnInt2( (void *)handle, LSM6DSL_ACC_GYRO_INT2_WU_ENABLED ) == MEMS_ERROR )
    {
      return -1;
    }
    break;

  default:
    break;
  }

  return 0;
}

static int lsm6dsl_disable_motion( void *handle )
{

  /* Disable wake up event on INT1. */
  if ( LSM6DSL_ACC_GYRO_W_WUEvOnInt1( (void *)handle, LSM6DSL_ACC_GYRO_INT1_WU_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable wake up event on INT2. */
  if ( LSM6DSL_ACC_GYRO_W_WUEvOnInt2( (void *)handle, LSM6DSL_ACC_GYRO_INT2_WU_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* Disable basic Interrupts */
  if ( LSM6DSL_ACC_GYRO_W_BASIC_INT( (void *)handle, LSM6DSL_ACC_GYRO_BASIC_INT_DISABLED ) == MEMS_ERROR )
  {
    return -1;
  }

  /* WU_DUR setting */
  if ( LSM6DSL_ACC_GYRO_W_WAKE_DUR( (void *)handle, 0x00 ) == MEMS_ERROR )
  {
    return -1;
  }

  /* WU_THS setting */
  if ( LSM6DSL_ACC_GYRO_W_WK_THS( (void *)handle, 0x00 ) == MEMS_ERROR )
  {
    return -1;
  }

  return 0;
}

#endif

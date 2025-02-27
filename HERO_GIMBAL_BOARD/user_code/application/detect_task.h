/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       detect_task.c/h
  * @brief      detect error task, judged by receiving data time. provide detect
                hook function, error exist function.
  *             ���������� ͨ����������ʱ�����ж�.�ṩ ��⹳�Ӻ���,������ں���.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add oled, gyro accel and mag sensors
  *
  @verbatim
  ==============================================================================
    
    ���Ҫ����һ�����豸
    1.��һ����detect_task.h�������豸������errorList�������
    enum errorList
    {
        ...
        XXX_TOE,    //���豸
        ERROR_LIST_LENGHT,
    };
    2.��detect_init����,����offlineTime, onlinetime, priority����
        uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };
    3.�����data_is_error_fun ,solve_lost_fun,solve_data_error_fun��������ֵ������ָ��
    4.��XXX_TOE�豸��������ʱ��, ���Ӻ���detect_hook(XXX_TOE).
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
  
#ifndef DETECT_TASK_H
#define DETECT_TASK_H
#include "struct_typedef.h"

#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 10

//�������Լ���Ӧ�豸˳��
enum errorList
{
  DBUS_TOE = 0,
  SHOOT_LEFT_FRIC_MOTOR_ID,
  SHOOT_RIGHT_FRIC_MOTOR_ID,
  SHOOT_TRIGGER_MOTOR_TOE,
  SHOOT_COVER_MOTOR_TOE,
  // CHASSIS_MOTIVE_FR_MOTOR_TOE,
  // CHASSIS_MOTIVE_FL_MOTOR_TOE,
  // CHASSIS_MOTIVE_BL_MOTOR_TOE,
  // CHASSIS_MOTIVE_BR_MOTOR_TOE,
  CHASSIS_RUDDER_FR_MOTOR_TOE,
  CHASSIS_RUDDER_FL_MOTOR_TOE,
  CHASSIS_RUDDER_BL_MOTOR_TOE,
  CHASSIS_RUDDER_BR_MOTOR_TOE,
  GIMBAL_YAW_MOTOR_TOE,
  GIMBAL_PITCH_MOTOR_TOE,
  BOARD_GYRO_TOE,
  BOARD_ACCEL_TOE,
  BOARD_MAG_TOE,
  RM_IMU_TOE,
  REFEREE_TOE,
  SUPER_CAP_TOE,
  OLED_TOE,
  VISION_TOE,
  BOARD_COM,
  ERROR_LIST_LENGHT,
};

typedef __packed struct
{
    uint32_t new_time;
    uint32_t last_time;
    uint32_t lost_time;
    uint32_t work_time;
    uint16_t set_offline_time : 12;
    uint16_t set_online_time : 12;
    uint8_t enable : 1;
    uint8_t priority : 4;
    uint8_t error_exist : 1;
    uint8_t is_lost : 1;
    uint8_t data_is_error : 1;

    fp32 frequency;
    bool_t (*data_is_error_fun)(void);
    void (*solve_lost_fun)(void);
    void (*solve_data_error_fun)(void);
} error_t;


/**
  * @brief          �������
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void detect_task(void *pvParameters);


/**
  * @brief          ��ȡ�豸��Ӧ�Ĵ���״̬
  * @param[in]      toe:�豸Ŀ¼
  * @retval         true(����) ����false(û����)
  */
extern bool_t toe_is_error(uint8_t err);


/**
  * @brief          ��¼ʱ��
  * @param[in]      toe:�豸Ŀ¼
  * @retval         none
  */
extern void detect_hook(uint16_t toe);

/**
  * @brief          �õ������б�
  * @param[in]      none
  * @retval         error_list��ָ��
  */
extern const error_t *get_error_list_point(void);

#endif

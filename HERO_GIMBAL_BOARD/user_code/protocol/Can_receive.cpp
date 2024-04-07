#include "Can_receive.h"

#include "cmsis_os.h"
#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "bsp_delay.h"

#ifdef __cplusplus
}
#endif
#include "bsp_can.h"
#include "can.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void Can_receive::init()
{
    can_filter_init();
}

void Can_receive::get_gimbal_motor_measure(uint8_t num, uint8_t data[8])
{
    gimbal_motor[num].last_ecd = gimbal_motor[num].ecd;
    gimbal_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    gimbal_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    gimbal_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    gimbal_motor[num].temperate = data[6];
}

/**
 * @brief          发送Yaw电机电流
 * @param[in]      yaw yaw轴电流
 */
void Can_receive::can_cmd_yaw_motor(int16_t yaw)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = yaw >> 8;
    can_send_data[1] = yaw;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&GIMBAL_YAW_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

/**
 * @brief          发送Pitch电机电流
 * @param[in]      pitch pitch轴电流
 */
void Can_receive::can_cmd_pitch_motor(int16_t pitch)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = 0;
    can_send_data[1] = 0;
    can_send_data[2] = pitch >> 8;
    can_send_data[3] = pitch;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&GIMBAL_PITCH_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

/**
 * @brief           发送望远镜小云台pitch电机电流
 * 
 * @param telescopes 望远镜小云台pitch电机电流
 */
void Can_receive::can_cmd_telescopes_motor(int16_t telescopes)
{
    // uint32_t send_mail_box;
    // can_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    // can_tx_message.IDE = CAN_ID_STD;
    // can_tx_message.RTR = CAN_RTR_DATA;
    // can_tx_message.DLC = 0x08;
    // can_send_data[0] = 0;
    // can_send_data[1] = 0;
    // can_send_data[2] = 0;
    // can_send_data[3] = 0;
    // can_send_data[4] = 0;
    // can_send_data[5] = 0;
    // can_send_data[6] = telescopes >> 8;
    // can_send_data[7] = telescopes;
    // //调试暂时用can1电调ID3，最终用can2电调ID4
    // HAL_CAN_AddTxMessage(&TELESCOPES_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

/**
 * @brief          返回云台电机 6020电机数据指针
 * @param[in]      i: 电机编号,范围[0,1]
 * @retval         电机数据指针
 */
const motor_measure_t *Can_receive::get_gimbal_motor_measure_point(uint8_t i)
{
    return &gimbal_motor[i];
}

void Can_receive::get_shoot_motor_measure(uint8_t num, uint8_t data[8])
{
    shoot_motor[num].last_ecd = shoot_motor[num].ecd;
    shoot_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    shoot_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    shoot_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    shoot_motor[num].temperate = data[6];
    if (shoot_motor[num].ecd - shoot_motor[num].last_ecd > 4096)
    {
        shoot_motor[num].round_cnt--;
    }
    else if (shoot_motor[num].ecd - shoot_motor[num].last_ecd < -4096)
    {
        shoot_motor[num].round_cnt++;
    }
}


/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      left_fric: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      right_fric: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      trigger: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      cover: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void Can_receive::can_cmd_shoot_motor(int16_t left_fric, int16_t right_fric, int16_t trigger, int16_t telescopes)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_SHOOT_ALL_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = left_fric >> 8;
    can_send_data[1] = left_fric;
    can_send_data[2] = right_fric >> 8;
    can_send_data[3] = right_fric;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = telescopes >> 8;
    can_send_data[7] = telescopes;

    HAL_CAN_AddTxMessage(&SHOOT_CAN, &can_tx_message, can_send_data, &send_mail_box);
}
/**
 * @brief          发送Trigger电机电流
 * @param[in]      trigger轴电流 trigger轴电流
 */
void Can_receive::can_cmd_trigger_motor(int16_t trigger)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_SHOOT_ALL_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;

    can_send_data[0] = 0;
    can_send_data[1] = 0;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = trigger >> 8;
    can_send_data[5] = trigger;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&TRIGGER_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
void Can_receive::can_cmd_shoot_motor_reset_ID(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x700;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = 0;
    can_send_data[1] = 0;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&SHOOT_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

/**
 * @brief          返回云台电机 6020电机数据指针
 * @param[in]      i: 电机编号,范围[0,1]
 * @retval         电机数据指针
 */
const motor_measure_t *Can_receive::get_shoot_motor_measure_point(uint8_t i)
{
    return &shoot_motor[i];
}

void Can_receive::receive_cooling_and_id_board_com(uint8_t data[8])
{
    gimbal_receive.shoot_cooling_limit = (uint16_t)(data[0] << 8 | data[1]);
    gimbal_receive.shoot_cooling_rate = (uint16_t)(data[2] << 8 | data[3]);
    gimbal_receive.shoot_cooling_heat = (uint16_t)(data[4] << 8 | data[5]);
    gimbal_receive.color = (data[6]);
    gimbal_receive.robot_id = (data[7]);
}

void Can_receive::receive_shoot_speed_and_mode_board_com(uint8_t data[8])
{
    gimbal_receive.shoot_speed_limit = 16;
    gimbal_receive.bullet_speed = (uint16_t)(data[2] << 8 | data[3]);
    gimbal_receive.chassis_behaviour = data[4];
    gimbal_receive.game_progress = data[5];

}

void Can_receive::send_rc_board_com(int16_t ch_0, int16_t ch_2, int16_t ch_3, uint16_t v)
{
    //数据填充
    gimbal_send.ch_0 = ch_0;
    gimbal_send.ch_2 = ch_2;
    gimbal_send.ch_3 = ch_3;
    gimbal_send.v = v;

    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_RC_BOARM_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = ch_0 >> 8;
    can_send_data[1] = ch_0;
    can_send_data[2] = ch_2 >> 8;
    can_send_data[3] = ch_2;
    can_send_data[4] = ch_3 >> 8;
    can_send_data[5] = ch_3;
    can_send_data[6] = v >> 8;
    can_send_data[7] = v;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

void Can_receive::send_gimbal_board_com(uint8_t s0, uint8_t gimbal_behaviour, fp32 gimbal_yaw_angle)
{
    int32_t temp_gimbal_yaw_angle = (int32_t)(gimbal_yaw_angle * 1000);

    //数据填充
    gimbal_send.s0 = s0;
    gimbal_send.gimbal_behaviour = gimbal_behaviour;
    gimbal_send.gimbal_yaw_angle = gimbal_yaw_angle;

    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_GIMBAL_BOARD_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = s0;
    can_send_data[1] = gimbal_behaviour;
    can_send_data[2] = (uint8_t)((int32_t)temp_gimbal_yaw_angle >> 24);
    can_send_data[3] = (uint8_t)((int32_t)temp_gimbal_yaw_angle >> 16);
    can_send_data[4] = (uint8_t)((int32_t)temp_gimbal_yaw_angle >> 8);
    can_send_data[5] = (uint8_t)((int32_t)temp_gimbal_yaw_angle);
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
}


void Can_receive::send_gimbal_navi_com(fp32 navi_x, fp32 navi_y)
{
    int32_t temp_navi_x = (int32_t)(navi_x * 1000);
    int32_t temp_navi_y = (int32_t)(navi_y * 1000);
    
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_NAVI_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = (uint8_t)((int32_t)temp_navi_x >> 24);
    can_send_data[1] = (uint8_t)((int32_t)temp_navi_x >> 16);
    can_send_data[2] = (uint8_t)((int32_t)temp_navi_x >> 8);
    can_send_data[3] = (uint8_t)((int32_t)temp_navi_x );
    can_send_data[4] = (uint8_t)((int32_t)temp_navi_y >> 24);
    can_send_data[5] = (uint8_t)((int32_t)temp_navi_y >> 16);
    can_send_data[6] = (uint8_t)((int32_t)temp_navi_y >> 8);
    can_send_data[7] = (uint8_t)((int32_t)temp_navi_y );

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

void Can_receive::send_gimbal_navi_com_2(fp32 navi_z, uint8_t MODE)
{
    int32_t temp_navi_z = (int32_t)(navi_z * 1000);

    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_NAVI_COM_ID_2;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = (uint8_t)((int32_t)temp_navi_z >> 24);
    can_send_data[1] = (uint8_t)((int32_t)temp_navi_z >> 16);
    can_send_data[2] = (uint8_t)((int32_t)temp_navi_z >> 8);
    can_send_data[3] = (uint8_t)((int32_t)temp_navi_z );
    can_send_data[4] = MODE;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

void Can_receive::send_UI_com(bool_t auto_s, bool_t aim_s, bool_t fric_s, fp32 gimbal_pitch_angle, uint16_t v,uint8_t gimbal_turn_flag)
{
    int16_t temp_gimbal_pitch_angle = (int16_t)(gimbal_pitch_angle * 1000);

    //数据填充
    gimbal_send.auto_state = auto_s;
    gimbal_send.aim_state = aim_s;
    gimbal_send.fric_state = fric_s;
    gimbal_send.gimbal_pitch_angle = gimbal_pitch_angle;
    gimbal_send.v = v;
    gimbal_send.gimbal_turn_flag = gimbal_turn_flag;

    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_UI_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = auto_s;
    can_send_data[1] = aim_s;
    can_send_data[2] = fric_s;
    can_send_data[3] = (uint8_t)((int16_t)temp_gimbal_pitch_angle >> 8);
    can_send_data[4] = (uint8_t)((int16_t)temp_gimbal_pitch_angle);
    can_send_data[5] = v >> 8;
    can_send_data[6] = v;
    can_send_data[7] = gimbal_turn_flag;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

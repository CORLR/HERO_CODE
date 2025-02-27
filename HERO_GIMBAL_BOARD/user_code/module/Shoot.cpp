#include "Shoot.h"
#include "tim.h"
#include "main.h"

#include "bsp_fric.h"
#include "user_lib.h"

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
#include "bsp_laser.h"
}
#endif

#include "detect_task.h"

#include "Gimbal.h"
#include "Communicate.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
#define shoot_fric_off() fric_off()         //关闭两个摩擦轮

#define shoot_laser_on() laser_on()   //激光开启宏定义
#define shoot_laser_off() laser_off() //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

#define POWER_LIMIT 80.0f
#define WARNING_POWER 40.0f
#define WARNING_POWER_BUFF 50.0f

#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f // 16000 * 4,
#define BUFFER_TOTAL_CURRENT_LIMIT 16000.0f
#define POWER_TOTAL_CURRENT_LIMIT 20000.0f
/*17mm
shoot射速上限 15 18 30 m/s
shoot热量上限 50 100 150 280 400
shoot热量冷却 10 20 30 40 60 80
10热量
42mm
shoot射速上限 10 16 m/s
shoot热量上限 100 200 300 350 500
shoot热量冷却 20 40 60 80 100 120
100热量
*/
fp32 fric_refree_para = 1.0f;

fp32 trigger_speed_to_radio = 0.8;

fp32 shoot_n = 25.0f;
//通过读取裁判数据,直接修改射速和射频等级
//射速等级  摩擦电机
fp32 shoot_fric_grade[3] = {0, 1.6 , 3.8};

//射频等级 拨弹电机
fp32 shoot_trigger_grade[6] = {0, 5.0f * trigger_speed_to_radio, 10.0f * trigger_speed_to_radio, 15.0f * trigger_speed_to_radio, 28.0f * trigger_speed_to_radio, 40.0f * trigger_speed_to_radio};

//拨盘等级 摩擦轮等级
uint8_t trigger_speed_grade;
uint8_t fric_speed_grade;

Shoot shoot;

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
void Shoot::init()
{
    //遥控器指针
    shoot_rc = remote_control.get_remote_control_point();
    last_shoot_rc = remote_control.get_last_remote_control_point();

    //记录上一次按键值
    shoot_last_key_v = 0;

    //设置初试模式
    shoot_mode = SHOOT_STOP;

    cover_mode = COVER_CLOSE_DONE;

    //设置望远镜为关
    telescopes_mode = TELESCOPES_CLOSE;

    //摩擦轮电机
    
        //获取电机数据
        left_fric_motor.init(can_receive.get_shoot_motor_measure_point(0));
        //初始化PID
        fp32 left_fric_speed_pid_parm[5] = {LEFT_FRIC_SPEED_PID_KP, LEFT_FRIC_SPEED_PID_KI, LEFT_FRIC_SPEED_PID_KD, LEFT_FRIC_PID_MAX_IOUT, LEFT_FRIC_PID_MAX_OUT};
        left_fric_motor.speed_pid.init(PID_SPEED, left_fric_speed_pid_parm, &left_fric_motor.speed, &left_fric_motor.speed_set, NULL);
        left_fric_motor.speed_pid.pid_clear();
        //设置最大 最小值  左摩擦轮顺时针转 右摩擦轮逆时针转
        left_fric_motor.max_speed = FRIC_MAX_SPEED;
        left_fric_motor.min_speed = -FRIC_MAX_SPEED;
        left_fric_motor.require_speed = -FRIC_MAX_REQUIRE_SPEED;


        //获取电机数据
        right_fric_motor.init(can_receive.get_shoot_motor_measure_point(1));
        //初始化PID
        fp32 right_fric_speed_pid_parm[5] = {RIGHT_FRIC_SPEED_PID_KP, RIGHT_FRIC_SPEED_PID_KI, RIGHT_FRIC_SPEED_PID_KD, RIGHT_FRIC_PID_MAX_IOUT, RIGHT_FRIC_PID_MAX_OUT};
        right_fric_motor.speed_pid.init(PID_SPEED, right_fric_speed_pid_parm, &right_fric_motor.speed, &right_fric_motor.speed_set, NULL);
        right_fric_motor.speed_pid.pid_clear();
        //设置最大 最小值  左摩擦轮顺时针转 右摩擦轮逆时针转
        right_fric_motor.max_speed = FRIC_MAX_SPEED;
        right_fric_motor.min_speed = -FRIC_MAX_SPEED;
        right_fric_motor.require_speed = -FRIC_MAX_REQUIRE_SPEED;
    

    trigger_motor.init(can_receive.get_shoot_motor_measure_point(TRIGGER));
    //初始化拨弹轮速度环PID
    fp32 trigger_speed_pid_parm[5] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD, TRIGGER_SPEED_PID_MAX_IOUT, TRIGGER_SPEED_PID_MAX_OUT};
    trigger_motor.speed_pid.init(PID_SPEED, trigger_speed_pid_parm, &trigger_motor.speed, &trigger_motor.speed_set, NULL);
    trigger_motor.speed_pid.pid_clear();
    //初始化拨弹轮角度环pid
    fp32 trigger_angle_pid_parm[5] = {TRIGGER_ANGLE_PID_KP,TRIGGER_ANGLE_PID_KI,TRIGGER_ANGLE_PID_KD,TRIGGER_ANGLE_PID_MAX_IOUT,TRIGGER_ANGLE_PID_MAX_OUT};
    trigger_motor.angle_pid.init(PID_SPEED, trigger_angle_pid_parm, &trigger_motor.angle, &trigger_motor.angle_set, NULL);
    trigger_motor.angle_pid.pid_clear();


    // //TODO,此处限幅,暂时不设置
    // //设置最大 最小值  左摩擦轮顺时针转 右摩擦轮逆时针转
    // trigger_motor.max_speed = FRIC_MAX_SPEED_RMP;
    // trigger_motor.min_speed = -FRIC_MAX_SPEED_RMP;
    // trigger_motor.require_speed = -FRIC_REQUIRE_SPEED_RMP;

    //摩擦轮 限位舵机状态
    fric_status = FALSE;
    limit_switch_status = FALSE;

    // TODO 此处先添加,后面可能会删去
    trigger_motor.angle = trigger_motor.motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    trigger_motor.ecd_count = 0;
    trigger_motor.current_give = 0;
    trigger_motor.angle_set = trigger_motor.angle;
    trigger_motor.speed = 0.0f;
    trigger_motor.speed_set = 0.0f;

    //弹仓电机初始化
    //cover_motor.init(can_receive.get_shoot_motor_measure_point(COVER));

    //小云台pitch电机初始化
    telescopes_motor.init(can_receive.get_shoot_motor_measure_point(TELESCOPES));

    //初始化PID
    fp32 cover_speed_pid_parm[5] = {COVER_ANGLE_PID_KP, COVER_ANGLE_PID_KI, COVER_ANGLE_PID_KD, COVER_BULLET_PID_MAX_IOUT, COVER_BULLET_PID_MAX_OUT};
    cover_motor.speed_pid.init(PID_SPEED, cover_speed_pid_parm, &cover_motor.speed, &cover_motor.speed_set, NULL);
    cover_motor.angle_pid.pid_clear();

    cover_motor.angle = cover_motor.motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    cover_motor.ecd_count = 0;
    cover_motor.current_give = 0;
    cover_motor.angle_set = cover_motor.angle;
    cover_motor.speed = 0.0f;
    cover_motor.speed_set = 0.0f;

    fp32 telescopes_speed_pid_parm[5] = {TELESCOPES_SPEED_PID_KP, TELESCOPES_SPEED_PID_KI, TELESCOPES_SPEED_PID_KD, TRIGGER_SPEED_PID_MAX_IOUT, TRIGGER_SPEED_PID_MAX_OUT};
    telescopes_motor.speed_pid.init(PID_SPEED, telescopes_speed_pid_parm, &telescopes_motor.speed, &telescopes_motor.speed_set, NULL);
    telescopes_motor.current_give = 0;

    const static fp32 chassis_x_order_filter[1] = {SHOOT_ACCEL_FRIC_LEFT_NUM};
    const static fp32 chassis_y_order_filter[1] = {SHOOT_ACCEL_FRIC_RIGHT_NUM};

    shoot_cmd_slow_fric_left.init(SHOOT_CONTROL_TIME, chassis_x_order_filter);
    shoot_cmd_slow_fric_right.init(SHOOT_CONTROL_TIME, chassis_y_order_filter);


    //更新数据
    feedback_update();

    move_flag = 0;
    cover_move_flag = 0;
    key_time = 0;

    //未防止卡弹, 上电后自动开启摩擦轮,可以手动关闭
    // shoot_mode = SHOOT_READY_FRIC;
    // buzzer_on(5, 10000);
}

/**
 * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */
uint8_t temp_a;
uint8_t temp_b;
uint8_t temp_c;

void Shoot::set_mode()
{
    static int8_t last_s = RC_SW_UP; //记录上一次遥控器按键值

    //上拨判断， 一次开启，再次关闭
    if ((switch_is_up(shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_mode == SHOOT_STOP))
    {
        buzzer_on(5, 10000);
        shoot_mode = SHOOT_READY_FRIC;
    }
    else if ((switch_is_up(shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_mode != SHOOT_STOP))
    {
        buzzer_off();
        shoot_mode = SHOOT_STOP;
    }

    static uint16_t last_fric_key_value = 0;

    //处于中档， 可以使用键盘开启/关闭摩擦轮
    if (switch_is_mid(shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && if_key_singal_pessed(shoot_rc->key.v, last_fric_key_value, KEY_PRESSED_SHOOT_FRIC))
    {
        if (shoot_mode == SHOOT_STOP)
        {
            buzzer_on(5, 10000);
            shoot_mode = SHOOT_READY_FRIC;
        }
        else
        {
            buzzer_off();
            shoot_mode = SHOOT_STOP;
        }
    }

    last_fric_key_value = shoot_rc->key.v;

    //为了便于测试,右按键为下时关闭摩擦轮
    if (switch_is_down(shoot_rc->rc.s[0]))
    {
        shoot_mode = SHOOT_STOP;
    }

    //摩擦轮速度达到一定值,才可开启拨盘  为了便于测试,这里至少需要一个摩擦轮电机达到拨盘启动要求就可以开启拨盘
    if (shoot_mode == SHOOT_READY_FRIC && (abs_int16(left_fric_motor.motor_measure->speed_rpm) > abs_fp32(left_fric_motor.require_speed) || abs_int16(right_fric_motor.motor_measure->speed_rpm) > abs_fp32(right_fric_motor.require_speed)))
    {
        fric_status = TRUE;
        shoot_mode = SHOOT_READY_BULLET;
    }
    else if (shoot_mode == SHOOT_READY_BULLET && key == SWITCH_TRIGGER_ON)
    {
        shoot_mode = SHOOT_READY;
    }
    else if (shoot_mode == SHOOT_READY && key == SWITCH_TRIGGER_OFF)
    {
        shoot_mode = SHOOT_READY_BULLET;
    }
    else if (shoot_mode == SHOOT_READY)
    {
        //下拨一次或者鼠标按下一次，进入射击状态
        if ((switch_is_down(shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (press_l && last_press_l == 0))
        {
            shoot_mode = SHOOT_BULLET;
        }
    }
    else if (shoot_mode == SHOOT_DONE)
    {
        if (key == SWITCH_TRIGGER_OFF)
        {
            key_time++;
            if (key_time > SHOOT_DONE_KEY_OFF_TIME)
            {
                key_time = 0;
                shoot_mode = SHOOT_READY_BULLET;
            }
        }
        else
        {
            key_time = 0;
            shoot_mode = SHOOT_BULLET;
        }
    }

    if (shoot_mode > SHOOT_READY_FRIC)
    {
        //鼠标长按一直进入射击状态 保持连发
        if ((press_l_time == PRESS_L_LONG_TIME) || (rc_s_time == RC_S_LONG_TIME))
        {
            shoot_mode = SHOOT_CONTINUE_BULLET;
        }
        else if (shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_mode = SHOOT_READY_BULLET;
        }
    }

    // //检测两个摩擦轮同时上线，为了便于调试，暂时注释
    // if(!toe_is_error(REFEREE_TOE) && (heat + SHOOT_HEAT_REMAIN_VALUE > heat_limit))
    // {
    //     if(shoot_mode == SHOOT_BULLET || shoot_mode == SHOOT_CONTINUE_BULLET)
    //     {
    //         shoot_mode =SHOOT_READY_BULLET;
    //     }
    // }

    // //如果云台状态是 无力状态，就关闭射击
     if (gimbal_cmd_to_shoot_stop())
     {
         shoot_mode = SHOOT_STOP;
     }

    static uint16_t last_cover_key_value = 0;
    static uint16_t last_telescopes_key_value = 0;

    // if (if_key_singal_pessed(shoot_rc->key.v, last_cover_key_value, KEY_PRESSED_SHOOT_COVER) && cover_mode == COVER_OPEN_DONE) //单击R并且开启完毕
    // {
    //     cover_mode = COVER_CLOSE;
    // }

    // if (press_cover_time == PRESS_COVER_LONG_TIME && cover_mode == COVER_CLOSE_DONE) //长按R并且关闭完毕
    // {
    //     cover_mode = COVER_OPEN;
    // }

    last_cover_key_value = shoot_rc->key.v;

    last_s = shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];

    //切换望远镜
    if (if_key_singal_pessed(shoot_rc->key.v, last_telescopes_key_value, KEY_PRESSE_TELESCOPES)) //单击R切换望远镜开关
    {
        if(telescopes_mode == TELESCOPES_CLOSE)
        {
            telescopes_mode = TELESCOPES_OPEN;
        }
        else if(telescopes_mode == TELESCOPES_OPEN)
        {
            telescopes_mode = TELESCOPES_CLOSE;
        }
    }
    last_telescopes_key_value = shoot_rc->key.v;
}

/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
void Shoot::feedback_update()
{
    last_shoot_rc->key.v = shoot_rc->key.v;
    shoot_last_key_v = shoot_rc->key.v;

    //更新摩擦轮电机速度
    left_fric_motor.speed = -left_fric_motor.motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;
    right_fric_motor.speed = right_fric_motor.motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;

    //更新小云台电机速度，RPM
    telescopes_motor.speed = telescopes_motor.motor_measure->speed_rpm;

    // 拨弹轮电机速度滤波一下
    static fp32 trigger_speed_fliter_1 = 0.0f;
    static fp32 trigger_speed_fliter_2 = 0.0f;
    static fp32 trigger_speed_fliter_3 = 0.0f;

    static const fp32 trigger_fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    //二阶低通滤波
    trigger_speed_fliter_1 = trigger_speed_fliter_2;
    trigger_speed_fliter_2 = trigger_speed_fliter_3;
    trigger_speed_fliter_3 = trigger_speed_fliter_2 * trigger_fliter_num[0] + trigger_speed_fliter_1 * trigger_fliter_num[1] + (trigger_motor.motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * trigger_fliter_num[2];
    trigger_motor.speed = trigger_speed_fliter_3;

    // 拨弹轮电机速度滤波一下
    static fp32 cover_speed_fliter_1 = 0.0f;
    static fp32 cover_speed_fliter_2 = 0.0f;
    static fp32 cover_speed_fliter_3 = 0.0f;

    static const fp32 cover_fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    //二阶低通滤波
    cover_speed_fliter_1 = cover_speed_fliter_2;
    cover_speed_fliter_2 = cover_speed_fliter_3;
    cover_speed_fliter_3 = cover_speed_fliter_2 * cover_fliter_num[0] + cover_speed_fliter_1 * cover_fliter_num[1] + (cover_motor.motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * cover_fliter_num[2];
    cover_motor.speed = cover_speed_fliter_3;

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    // if (trigger_motor.motor_measure->ecd - trigger_motor.motor_measure->last_ecd > HALF_ECD_RANGE)
    // {
    //     trigger_motor.ecd_count--;
    // }
    // else if (trigger_motor.motor_measure->ecd - trigger_motor.motor_measure->last_ecd < -HALF_ECD_RANGE)
    // {
    //     trigger_motor.ecd_count++;
    // }
    //计算拨盘电机输出轴角度
    trigger_motor.angle = (trigger_motor.motor_measure->round_cnt * ECD_RANGE + trigger_motor.motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (cover_motor.motor_measure->ecd - cover_motor.motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        cover_motor.ecd_count--;
    }
    else if (cover_motor.motor_measure->ecd - cover_motor.motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        cover_motor.ecd_count++;
    }

    if (cover_motor.ecd_count == FULL_COUNT)
    {
        cover_motor.ecd_count = -(FULL_COUNT - 1);
    }
    else if (cover_motor.ecd_count == -FULL_COUNT)
    {
        cover_motor.ecd_count = FULL_COUNT - 1;
    }

    //计算输出轴角度
    cover_motor.angle = (cover_motor.ecd_count * ECD_RANGE + cover_motor.motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;

    // TODO 此处没有安装微动开关,暂时把key设置为1
    //微动开关
    key = 0;
    //鼠标按键
    last_press_l = press_l;
    last_press_r = press_r;
    press_l = shoot_rc->mouse.press_l;
    press_r = shoot_rc->mouse.press_r;

    last_press_cover = press_cover;
    press_cover = if_key_pessed(shoot_rc->key.v, KEY_PRESSED_SHOOT_COVER);

    //长按计时
    if (press_l)
    {
        if (press_l_time < PRESS_L_LONG_TIME)
        {
            press_l_time++;
        }
    }
    else
    {
        press_l_time = 0;
    }

    //长按计时
    if (press_cover)
    {
        if (press_cover_time < PRESS_COVER_LONG_TIME)
        {
            press_cover_time++;
        }
    }
    else
    {
        press_cover_time = 0;
    }

    //射击开关下档时间计时
    if (shoot_mode != SHOOT_STOP && switch_is_down(shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (rc_s_time < RC_S_LONG_TIME)
        {
            rc_s_time++;
        }
    }
    else
    {
        rc_s_time = 0;
    }

    //便于调参:

//    //射速等级  摩擦电机
//    shoot_fric_grade[0] = 0;
//    shoot_fric_grade[1] = 15 * fric_refree_para;
//    shoot_fric_grade[2] = 18 * fric_refree_para;
//    shoot_fric_grade[3] = 50 * fric_refree_para;

//    //射频等级 拨弹电机
//    shoot_trigger_grade[0] = 0;
//    shoot_trigger_grade[1] = 5.0f * trigger_speed_to_radio;
//    shoot_trigger_grade[2] = 10.0f * trigger_speed_to_radio;
//    shoot_trigger_grade[3] = 15.0f * trigger_speed_to_radio;
//    shoot_trigger_grade[4] = 28.0f * trigger_speed_to_radio;
//    shoot_trigger_grade[5] = 40.0f * trigger_speed_to_radio;
}

/**
 * @brief          射击循环
 * @param[in]      void
 * @retval         返回can控制值
 */
void Shoot::set_control()
{
    if (shoot_mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
        trigger_motor.speed_set = 0.0f;
    }
    else if (shoot_mode == SHOOT_READY_FRIC)
    {
        //设置拨弹轮的速度
        trigger_motor.speed_set = 0.0f;
    }
    else if (shoot_mode == SHOOT_READY_BULLET)
    {
        if (key == SWITCH_TRIGGER_OFF)
        {
            //设置拨弹轮的拨动速度,并开启堵转反转处理
            trigger_motor.speed_set = shoot_trigger_grade[1] * SHOOT_TRIGGER_DIRECTION;
        }
        else
        {
            trigger_motor.speed_set = 0.0f;
        }
        trigger_motor.speed_pid.data.max_out = TRIGGER_READY_PID_MAX_OUT;
        trigger_motor.speed_pid.data.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_mode == SHOOT_READY)
    {
        //设置拨弹轮的速度
        trigger_motor.speed_set = trigger_motor.angle_pid.pid_calc();
    }
    else if (shoot_mode == SHOOT_BULLET)
    {
        trigger_motor.speed_pid.data.max_out = TRIGGER_SPEED_PID_MAX_OUT;
        trigger_motor.speed_pid.data.max_iout = TRIGGER_SPEED_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    else if (shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        //设置拨弹轮的拨动速度,并开启堵转反转处理
        trigger_motor.speed_set = shoot_trigger_grade[1] * SHOOT_TRIGGER_DIRECTION;
    }
    else if (shoot_mode == SHOOT_DONE)
    {
        trigger_motor.speed_set = trigger_motor.angle_pid.pid_calc();
    }

    if (cover_mode == COVER_OPEN_DONE || cover_mode == COVER_CLOSE_DONE)
    {
        //设置拨弹轮的速度
        cover_motor.speed_set = 0.0f;
    }
    else
    {
        cover_motor.speed_pid.data.max_out = COVER_BULLET_PID_MAX_OUT;
        cover_motor.speed_pid.data.max_iout = COVER_BULLET_PID_MAX_IOUT;
        cover_control();
    }
}

/**
 * @brief          发射机构弹速和热量控制
 * @param[in]      void
 * @retval
 */
void Shoot::cooling_ctrl()
{
    // shoot枪口热量上限, shoot枪口实时热量
    uint16_t shoot_cooling_limit;
    uint16_t shoot_cooling_heat;
    uint16_t shoot_cooling_rate;

    // shoot枪口枪口射速上限,shoot实时射速
    uint16_t shoot_speed_limit;
    static fp32 bullet_speed;
    static fp32 last_bullet_speed;

    //保留被强制降速前的射频
    static uint8_t last_trigger_speed_grade = 1;

    // TODO 暂时认为没有必要
    //      //手动调整射频
    //  #if SHOOT_SET_TRIGGER_SPEED_BY_HAND
    //      if (KEY_SHOOT_TRIGGER_SPEED_UP && trigger_speed_grade < 5)
    //      {
    //          trigger_speed_grade++;
    //      }
    //      else if (KEY_SHOOT_TRIGGER_SPEED_DOWN && trigger_speed_grade > 0)
    //      {
    //          trigger_speed_grade--;
    //      }
    //  #endif

    // //TODO 离线监测暂时没有添加
    // if (toe_is_error(REFEREE_TOE))
    // {
    //     trigger_speed_grade = 2;
    //     fric_speed_grade = 2;
    // }
    // else
    {
        //更新裁判数据
        last_bullet_speed = bullet_speed;

        shoot_cooling_limit = can_receive.gimbal_receive.shoot_cooling_limit;
        shoot_cooling_heat = can_receive.gimbal_receive.shoot_cooling_heat;
        shoot_cooling_rate = can_receive.gimbal_receive.shoot_cooling_rate;

        shoot_speed_limit = can_receive.gimbal_receive.shoot_speed_limit;
        bullet_speed = can_receive.gimbal_receive.bullet_speed;

        //手动调整射频
#if SHOOT_SET_TRIGGER_SPEED_BY_HAND

#else
        //根据热量和射速上限修改等级
        //热量
        if (shoot_cooling_limit <= 50)
            trigger_speed_grade = 1;
        else if (shoot_cooling_limit <= 100)
            trigger_speed_grade = 2;
        else if (shoot_cooling_limit <= 200)
            trigger_speed_grade = 3;
        else if (shoot_cooling_limit <= 280)
            trigger_speed_grade = 4;
        else if (shoot_cooling_limit <= 400)
            trigger_speed_grade = 5;

#endif

        //射速
        if (shoot_speed_limit <= 10)
            fric_speed_grade = 1;
        else if (shoot_speed_limit <= 30)
            fric_speed_grade = 2;

        //根据当前热量和射速修改等级,确保不会因超限扣血
        //热量 当剩余热量低于100,强制制动
        if (shoot_cooling_limit - shoot_cooling_heat <= 100 && trigger_speed_grade != 0)
        {
            last_trigger_speed_grade = trigger_speed_grade;
            trigger_speed_grade = 0;
        }
        else
        {
            trigger_speed_grade = last_trigger_speed_grade;
        }

        //超射速,强制降低摩擦轮转速
        if (bullet_speed > shoot_speed_limit && last_bullet_speed != bullet_speed)
        {
            fric_speed_grade--;
        }
    }

    //连发模式下，对拨盘电机输入控制值
    if (shoot_mode == SHOOT_CONTINUE_BULLET)
        trigger_motor.speed_set = shoot_trigger_grade[trigger_speed_grade] * SHOOT_TRIGGER_DIRECTION;
    //对摩擦轮电机输入控制值
    left_fric_motor.speed_set = shoot_fric_grade[fric_speed_grade];
    right_fric_motor.speed_set = shoot_fric_grade[fric_speed_grade];

    //一阶低通滤波作为摩擦轮输入
    shoot_cmd_slow_fric_left.first_order_filter_cali(left_fric_motor.speed_set);
    shoot_cmd_slow_fric_right.first_order_filter_cali(right_fric_motor.speed_set);

    left_fric_motor.speed_set = shoot_cmd_slow_fric_left.out;
    right_fric_motor.speed_set = shoot_cmd_slow_fric_right.out;
}

/**
 * @brief          PID计算
 * @param[in]      void
 * @retval
 */
void Shoot::solve()
{
    // TODO 此处应该对速度设置值进行操作,而不是电流值,后续进行修改,有可能摩擦轮停止旋转速度慢和这个有关
    if (shoot_mode == SHOOT_STOP)
    {
        shoot_laser_off();
        fric_status = FALSE;

        trigger_motor.speed_set = 0;
        left_fric_motor.speed_set = 0;
        right_fric_motor.speed_set = 0;
    }
    else
    {
#if SHOOT_LASER_OPEN
        shoot_laser_on(); //激光开启
#else
        shoot_laser_off(); //激光关闭
#endif

        //控制shoot发射机构射速和热量控制
        cooling_ctrl();

        if (shoot_mode == SHOOT_READY_BULLET || shoot_mode == SHOOT_CONTINUE_BULLET)
            trigger_motor_turn_back(); //将设置的拨盘旋转角度,转化为速度,且防止卡弹

        //确保摩擦轮未达到最低转速不会转动拨盘
        if (shoot_mode < SHOOT_READY_BULLET)
        {
            trigger_motor.current_set = 0;
        }
    }

    if (left_fric_motor.speed_set > left_fric_motor.max_speed)
        left_fric_motor.speed_set = left_fric_motor.max_speed;
    else if (left_fric_motor.speed_set < left_fric_motor.min_speed)
        left_fric_motor.speed_set = left_fric_motor.min_speed;

    if (right_fric_motor.speed_set > right_fric_motor.max_speed)
        right_fric_motor.speed_set = right_fric_motor.max_speed;
    else if (right_fric_motor.speed_set < right_fric_motor.min_speed)
        right_fric_motor.speed_set = right_fric_motor.min_speed;

    //计算PID
    left_fric_motor.current_set = left_fric_motor.speed_pid.pid_calc();
    right_fric_motor.current_set = right_fric_motor.speed_pid.pid_calc();
    trigger_motor.current_set = trigger_motor.speed_pid.pid_calc();

    cover_motor.current_set = cover_motor.speed_pid.pid_calc();
    telescopes_motor.current_set = telescopes_motor.speed_pid.pid_calc();
}

void Shoot::output()
{
    left_fric_motor.current_give = -(int16_t)(left_fric_motor.current_set);
    right_fric_motor.current_give = (int16_t)(right_fric_motor.current_set);
    telescopes_motor.current_give = (int16_t)(telescopes_motor.current_set);
    trigger_motor.current_give = trigger_motor.current_set;
    //cover_motor.current_give = cover_motor.current_set;

//电流输出控制,通过调整宏定义控制
#if SHOOT_FRIC_MOTOR_HAVE_CURRENT
    ;
#else
    left_fric_motor.current_give = 0;
    right_fric_motor.current_give = 0;
#endif

#if SHOOT_TRIGGER_MOTOR_HAVE_CURRENT
    ;
#else
    trigger_motor.current_give = 0;
#endif
    //发送电流
    can_receive.can_cmd_shoot_motor(left_fric_motor.current_give, right_fric_motor.current_give, trigger_motor.current_give, telescopes_motor.current_give);
    can_receive.can_cmd_trigger_motor(trigger_motor.current_give);
}
/**
 * @brief 望远镜舵机控制
 * 
 */
void Shoot::telescopes()
{
    if(telescopes_mode == TELESCOPES_CLOSE)
    {
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,TELESCOPES_CLOSE_DUTY);
    }
    else if(telescopes_mode == TELESCOPES_OPEN)
    {
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,TELESCOPES_OPEN_DUTY);
    }
}

/**
 * @brief          拨盘电机反转控制
 * @param[in]      void
 * @retval         返回can控制值
 */
void Shoot::trigger_motor_turn_back()
{
    // if (block_time < BLOCK_TIME)
    // {
    //     trigger_motor.speed_set = trigger_motor.speed_set;
    // }
    // else
    // {
    //     trigger_motor.speed_set = -trigger_motor.speed_set;
    // }

    // if (fabs(trigger_motor.speed) < BLOCK_TRIGGER_SPEED && block_time < BLOCK_TIME)
    // {
    //     block_time++;
    //     reverse_time = 0;
    // }
    // else if (block_time == BLOCK_TIME && reverse_time < REVERSE_TIME)
    // {
    //     reverse_time++;
    // }
    // else
    // {
    //     block_time = 0;
    // }
}

/**
 * @brief          射击控制，控制拨弹电机角度，完成一次发射
 * @param[in]      void
 * @retval         void
 */
void Shoot::shoot_bullet_control()
{

    //每次拨动的角度
    if (move_flag == 0)
    {
        trigger_motor.angle_set = trigger_motor.angle + TRIGGER_ONCE;
        move_flag = 1;
    }
    if (key == SWITCH_TRIGGER_OFF)
    {
        shoot_mode = SHOOT_DONE;
    }
    //到达角度判断
    //没到达一直设置旋转速度
    trigger_motor.speed_set = trigger_motor.angle_pid.pid_calc();//shoot_trigger_grade[1] * SHOOT_TRIGGER_DIRECTION;
    trigger_motor_turn_back();
    if (trigger_motor.angle_set - trigger_motor.angle < 0.05f)
    {
        move_flag = 0;
        shoot_mode = SHOOT_READY;
    }

}

/**
 * @brief          弹仓控制，控制弹仓电机运动
 * @param[in]      void
 * @retval         void
 */
void Shoot::cover_control()
{
    if (cover_move_flag == 0)
    {
        if (cover_mode == COVER_OPEN)
        {
            cover_motor.angle_set = rad_format(cover_motor.angle + (fp32)COVER_OPEN_ANGLE);
        }
        else if (cover_mode == COVER_CLOSE)
        {
            cover_motor.angle_set = rad_format(cover_motor.angle - (fp32)COVER_OPEN_ANGLE);
        }
        cover_move_flag = 1;
    }
    if (cover_mode == COVER_OPEN)
    { //到达角度判断
        if (rad_format(cover_motor.angle_set - cover_motor.angle) > 0.05f)
        {
            //没到达一直设置旋转速度
            cover_motor.speed_set = COVER_MOTOR_SPEED;
        }
        else
        {
            cover_mode = COVER_OPEN_DONE;
            cover_move_flag = 0;
        }
    }
    else if (cover_mode == COVER_CLOSE)
    {
        if (rad_format(cover_motor.angle_set - cover_motor.angle) < -0.05f)
        {
            //没到达一直设置旋转速度
            cover_motor.speed_set = -COVER_MOTOR_SPEED;
        }
        else
        {
            cover_mode = COVER_CLOSE_DONE;
            cover_move_flag = 0;
        }
    }
}

void Shoot::telescopes_control()
{
    if(if_key_pessed(shoot_rc->key.v, 'Q'))
    {
        telescopes_motor.speed_set = -TELESCOPES_SPEED;
        telescopes_motor.flag = 1;
    }
    else if(if_key_pessed(shoot_rc->key.v, 'E'))
    {
        telescopes_motor.speed_set = TELESCOPES_SPEED;
        telescopes_motor.flag = 0;
    }
    else
    {
        telescopes_motor.speed_set = 0;
    }
    telescopes_motor.speed_pid.pid_calc();
}

/**
 * @brief          弹仓打开时,云台要停止运动
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
bool_t shoot_cmd_to_gimbal_stop()
{
    if (shoot.cover_mode == COVER_OPEN)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief          摩擦轮刚打开时,云台抬头
 * @param[in]      none
 * @retval         1: no move 0:normal
 */
bool_t shoot_open_fric_cmd_to_gimbal_up()
{
    if (shoot.shoot_mode > SHOOT_READY_FRIC)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

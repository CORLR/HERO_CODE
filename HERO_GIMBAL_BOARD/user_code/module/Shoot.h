/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "Communicate.h"
#include "Motor.h"
#include "user_lib.h"
#include "First_order_filter.h"

#include "Config.h"

#define TRIGGER_CCW 1 //拨盘顺时针
#define TRIGGER_CW -1 //拨盘逆时针

#define SHOOT_TRIGGER_DIRECTION TRIGGER_CCW

//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL 1

//开启摩擦轮的斜坡
#define SHOOT_FRIC_ADD_VALUE 0.1f

#define SHOOT_CONTROL_TIME 0.002f


//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME 15
//鼠标左键长按判断
#define PRESS_L_LONG_TIME 400
#define PRESS_R_LONG_TIME 50
//弹仓按键长按判断
#define PRESS_COVER_LONG_TIME 400
//摩擦轮开启按键延时
#define KEY_FRIC_LONG_TIME 200

//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME 2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

//摩擦轮电机rmp 变化成 旋转速度的比例
#define FRIC_RPM_TO_SPEED 0.000415809748903494517209f

#define FRIC_REQUIRE_SPEED_RMP 500.0f
#define FRIC_MAX_SPEED_RMP 4000.0f

#define FRIC_MAX_SPEED 80.0f
#define FRIC_MAX_REQUIRE_SPEED 30.0f

//拨盘电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED 0.000415809748903494517209f
#define MOTOR_ECD_TO_ANGLE 0.000039940741761990380111011191762043f * SHOOT_TRIGGER_DIRECTION/2  //2pi * 187/8192/3591  (3508减速比为3591:187)
#define FULL_COUNT 9.5f

//拨弹速度
#define TRIGGER_SPEED 10.0f * SHOOT_TRIGGER_DIRECTION          //10
#define CONTINUE_TRIGGER_SPEED 15.0f * SHOOT_TRIGGER_DIRECTION //15
#define READY_TRIGGER_SPEED 5.0f * SHOOT_TRIGGER_DIRECTION     //5

#define KEY_OFF_JUGUE_TIME 500
#define SWITCH_TRIGGER_ON 0
#define SWITCH_TRIGGER_OFF 1

//卡弹时间 以及反转时间
#define BLOCK_TRIGGER_SPEED 1.0f
#define BLOCK_TIME 700
#define REVERSE_TIME 500
#define REVERSE_SPEED_LIMIT 13.0f

#define PI_FOUR 0.78539816339744830961566084581988f
#define PI_TEN 0.314f
/*---------------------------pid----------------------*/
//摩擦轮电机PID
#define LEFT_FRIC_SPEED_PID_KP 3000//2000.0f // 1800
#define LEFT_FRIC_SPEED_PID_KI 0.05f    // 0.5
#define LEFT_FRIC_SPEED_PID_KD 5000.0f    // 2.0
#define LEFT_FRIC_PID_MAX_IOUT 200.0f
#define LEFT_FRIC_PID_MAX_OUT 40000.0f

#define RIGHT_FRIC_SPEED_PID_KP 3500//2000.0f // 1800
#define RIGHT_FRIC_SPEED_PID_KI 0.05f    // 0.5
#define RIGHT_FRIC_SPEED_PID_KD 5000.0f    // 2.0
#define RIGHT_FRIC_PID_MAX_IOUT 200.0f
#define RIGHT_FRIC_PID_MAX_OUT 40000.0f

//拨弹轮角度PID
#define TRIGGER_ANGLE_PID_KP 80.0f //800
#define TRIGGER_ANGLE_PID_KI 0.0f    //0.5
#define TRIGGER_ANGLE_PID_KD 8000.0f
#define TRIGGER_ANGLE_PID_MAX_IOUT 200.0f
#define TRIGGER_ANGLE_PID_MAX_OUT 10000.0f

#define TRIGGER_READY_PID_MAX_IOUT 2000.0f
#define TRIGGER_READY_PID_MAX_OUT 4000.0f
//拨弹轮速度
#define TRIGGER_SPEED_PID_KP 8000.0f //800
#define TRIGGER_SPEED_PID_KI 0.0f    //0.5
#define TRIGGER_SPEED_PID_KD 0.0f
#define TRIGGER_SPEED_PID_MAX_IOUT 200.0f
#define TRIGGER_SPEED_PID_MAX_OUT 10000.0f

//弹仓开合电机PID
#define COVER_ANGLE_PID_KP 1000.0f //800
#define COVER_ANGLE_PID_KI 0.0f    //0.5
#define COVER_ANGLE_PID_KD 0.0f
#define COVER_BULLET_PID_MAX_IOUT 200.0f
#define COVER_BULLET_PID_MAX_OUT 10000.0f

//小云台pitch电机初PID
#define TELESCOPES_SPEED_PID_KP 10.0f
#define TELESCOPES_SPEED_PID_KI 0.2f    
#define TELESCOPES_SPEED_PID_KD 0.01f
#define TELESCOPES_SPEED_PID_MAX_IOUT 200.0f
#define TELESCOPES_SPEED_PID_MAX_OUT 10000.0f

#define TELESCOPES_SPEED 100


#define SHOOT_HEAT_REMAIN_VALUE 80
//拨盘格数
#define TRIGGER_GRID_NUM 12
#define TRIGGER_ONCE 2 * PI / TRIGGER_GRID_NUM

#define COVER_OPEN_ANGLE 0.2 * PI
#define COVER_MOTOR_SPEED 1.0f

//一阶低通滤波参数
#define SHOOT_ACCEL_FRIC_LEFT_NUM 0.2666666667f
#define SHOOT_ACCEL_FRIC_RIGHT_NUM 0.2666666667f



//电机序号
#define LEFT_FRIC 0
#define RIGHT_FRIC 1
#define TRIGGER 2
#define TELESCOPES 3

//望远镜舵机占空比
#define TELESCOPES_CLOSE_DUTY 1000
#define TELESCOPES_OPEN_DUTY 2025


typedef enum
{
  SHOOT_STOP = 0,        //停止发射结构
  SHOOT_READY_FRIC,      //摩擦轮准备中
  SHOOT_READY_BULLET,    //拨盘准备中,摩擦轮已达到转速
  SHOOT_READY,           //整个发射机构准备完成
  SHOOT_BULLET,          //单发
  SHOOT_CONTINUE_BULLET, //连发
  SHOOT_DONE,
} shoot_mode_e;

typedef enum
{
  COVER_OPEN = 0,   //弹仓电机打开
  COVER_CLOSE,      //弹仓电机关闭
  COVER_OPEN_DONE,  //弹仓电机开启完毕
  COVER_CLOSE_DONE, //弹仓电机关闭完毕
} cover_mode_e;

typedef enum
{
  TELESCOPES_CLOSE = 0,
  TELESCOPES_OPEN,
} telescopes_mode_e;

class Shoot
{
public:
  const RC_ctrl_t *shoot_rc;
  RC_ctrl_t *last_shoot_rc;

  uint16_t shoot_last_key_v;

  shoot_mode_e shoot_mode;
  cover_mode_e cover_mode;

  //摩擦轮电机
  Firc_motor left_fric_motor,right_fric_motor;
  //拨弹电机
  Trigger_motor trigger_motor;
  //弹仓开合电机
  Cover_motor cover_motor;
  //望远镜小云台pitch电机电流
  Telescopes_motor telescopes_motor;
  
  First_order_filter shoot_cmd_slow_fric_left; //使用一阶低通滤波减缓设定值
  First_order_filter shoot_cmd_slow_fric_right; //使用一阶低通滤波减缓设定值

  //摩擦轮电机 限位开关 状态
  bool_t fric_status;
  bool_t limit_switch_status;
  //TODO 添加收到开关激光

  //鼠标状态
  bool_t press_l;
  bool_t press_r;
  bool_t last_press_l;
  bool_t last_press_r;
  uint16_t press_l_time;
  uint16_t press_r_time;
  uint16_t rc_s_time;
  //弹仓电机按键状态
  uint16_t press_cover;
  uint16_t last_press_cover;
  uint16_t press_cover_time;

  uint16_t block_time;
  uint16_t reverse_time;
  uint16_t move_flag;
  uint16_t cover_move_flag;

  //望远镜状态
  telescopes_mode_e telescopes_mode;
  //TODO 暂时未安装微动开关
  //微动开关
  bool_t key;
  uint8_t key_time;

  void init();            //云台初始化
  void set_mode();        //设置发射机构控制模式
  void feedback_update(); //发射数据反馈
  void set_control();     //设置发射机构控制量
  void cooling_ctrl();    //发射机构弹速和热量控制
  void solve();           //发射机构控制PID计算
  void output();          //输出电流
  void telescopes();      //云台望远镜控制

  //拨盘旋转相关函数
  void trigger_motor_turn_back(); //拨盘电机回转
  void shoot_bullet_control();

  //弹仓控制相关函数
  void cover_control();

  //小云台pitch电机控制
  void telescopes_control();
};

//发射机构控制云台不动
bool_t shoot_cmd_to_gimbal_stop();
//发射机构控制云台抬头
bool_t shoot_open_fric_cmd_to_gimbal_up();

extern Shoot shoot;

#endif

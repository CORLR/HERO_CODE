#ifndef CONFIG_H
#define CONFIG_H

/*----------------------底盘---------------------------*/
//底盘动力电机无电流输出
#define CHASSIS_MOTIVE_MOTOR_NO_CURRENT 0
/*---------------------通信-----------------------------*/
//底盘遥控器是否开启 1为开启上下板通讯底盘不需要遥控器
#define CHASSIS_REMOTE_OPEN 1
/*---------------------超电---------------------------*/
//底盘功率控制算法 1为启用自研超级电容控制板算法，0为启用雾列超级电容控制板算法
#define SUPER_CAP_WULIE_OR_SELF 1
/*---------------------按键---------------------------*/
//底盘小陀螺 单击F
#define KEY_PRESSED_CHASSIS_TOP     'F'

//底盘摇摆  单击C
#define KEY_PRESSED_CHASSIS_SWING   'C'

//底盘45度角 单击V
#define KEY_PRESSED_CHASSIS_PISA    'V'

//底盘超级电容加速 单击SHIFT !代表shift
#define KEY_PRESSED_CHASSIS_SUPER_CAP    '!'

//底盘前后左右控制按键
#define KEY_PRESSED_CHASSIS_FRONT  'W'
#define KEY_PRESSED_CHASSIS_BACK   'S'
#define KEY_PRESSED_CHASSIS_LEFT   'A'
#define KEY_PRESSED_CHASSIS_RIGHT  'D'
#define KEY_PRESSED_UI_UPDATE      'B'

#endif

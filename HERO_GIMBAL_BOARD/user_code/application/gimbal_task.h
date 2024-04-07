/**
  ****************************(C) COPYRIGHT 2021 *******************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-03-2022     WSJ      1. doing
  *  V2.0.0     JUN-21-2023     NJY      
  *
  @verbatim
  ==============================================================================
 *        へ　　　　　／|
 *   　　/＼7　　　 ∠＿/
 *   　 /　│　　 ／　／
 *   　│　Z ＿,＜　／　　 /`ヽ
 *   　│　　　　　ヽ　　 /　　〉
 *   　 Y　　　　　`　 /　　/
 *   　?●　?　●　　??〈　　/
 *   　()　 へ　　　　|　＼〈
 *   　　>? ?_　 ィ　 │ ／／
 *   　 / へ　　 /　?＜| ＼＼
 *   　 ヽ_?　　(_／　 │／／
 *   　　7　　　　　　　|／
 *   　　＞―r￣￣`?―＿
 *               皮卡丘保佑
 *               代码无BUG!
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 *******************************
  */
#ifndef GIMBAL_BOARD_GIMBAL_TASK_H
#define GIMBAL_BOARD_GIMBAL_TASK_H
#include "cmsis_os.h"
#include "INS.h"
#include "gimbal.h"

//任务开始空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201

#define GIMBAL_CONTROL_TIME_MS 2


extern void gimbal_task(void *pvParameters);

extern bool_t gimbal_imu_open_flag ;

#endif 

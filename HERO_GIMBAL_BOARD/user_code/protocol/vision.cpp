#include "vision.h"
#include "Remote_control.h"
#include "struct_typedef.h"
#include "string.h"
#include "INS.h"
#include "tim.h"
#include "gimbal.h"
#include "SolveTrajectory.h"
#include "../algorithm/SolveTrajectory.h"
#ifdef __cplusplus
extern "C"
{
#endif

#include "CRC8_CRC16.h"

#ifdef __cplusplus
}
#endif

// #include "referee.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t Vision_Buffer[2][VISION_BUFFER_LEN]; //视觉数据暂存
uint16_t get_CRC16_check_sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

extern RC_ctrl_t rc_ctrl;
extern INS imu;
extern Gimbal gimbal;

//角度初始化补偿
float Vision_Comps_Yaw = COMPENSATION_YAW;
float Vision_Comps_Pitch = COMPENSATION_PITCH;           //固定补偿，减小距离的影响
float Vision_Comps_Pitch_Dist = COMPENSATION_PITCH_DIST; //根据距离补偿


VisionSendHeader_t VisionSendHeader; //帧头

VisionActData_t VisionActData; //行动模式结构体

VisionRecvData_t VisionRecvData; //接收数据结构体

VisionSendData_t VisionSendData; //发送数据结构体

VisionSendData_test_t VisionSendData_test;//魏瞄发送结构体（测试用）

VisionRecvData_test_t VisionRecvData_test;//魏瞄接受结构体（测试用）

VisionSendData_test_t VisionSendData_test2[8];

uint32_t flag[8];

uint8_t Attack_Color_Choose = ATTACK_NONE; //默认不识别

//打符是否换装甲了
uint8_t Vision_Armor = FALSE;

//是否识别到装甲板
bool_t if_identify_target = FALSE;

//角度补偿,发送给视觉
float Vision_Comps_Yaw_Send = COMPENSATION_YAW;
float Vision_Comps_Pitch_Send = COMPENSATION_PITCH;
float SB_K_comps = 3.f;

static uint64_t send_cnt = 0;

void vision_init()
{
  usart1_init(Vision_Buffer[0], Vision_Buffer[1], VISION_BUFFER_LEN);

  VisionRecvData.pitch_angle = 0;
  VisionRecvData.yaw_angle = 0;
  VisionRecvData.distance = 0; //距离
  VisionRecvData.centre_lock = 0; //是否瞄准到了中间  0没有  1瞄准到了
  VisionRecvData.identify_target = 0; //视野内是否有目标/是否识别到了目标   0否  1是
  VisionRecvData.identify_buff = 0;   //打符时是否识别到了目标，1是，2识别到切换了装甲，0没识别到

  st.k = 0.038;
  st.bullet_type = BULLET_42;
  st.current_v = 18;
  st.bias_time = 0;
  st.s_bias = 0;
  st.z_bias = 0;
}

/**
  * @brief  读取视觉信息
  * @param  uart1缓存数据
  * @retval void
  * @attention  IRQ执行
  */
uint8_t Vision_Time_Test[2] = {0}; //当前数据和上一次数据
uint8_t Vision_Ping = 0;           //发送时间间隔

/**
  * @brief          定时器周期给视觉发送陀螺仪数据
  * @param[in]      htim:定时器指针
  * @retval         none
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1)
  {
    HAL_GPIO_TogglePin(CRAMA_TRI_GPIO_Port, CRAMA_TRI_Pin);
    if(send_cnt++ % 2 == 0)
    {
      vision_send_data(0x02);
    }
  }
}

void vision_read_data(uint8_t *ReadFormUart)
{
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;

 //判断帧头数据是否为0xA5
 if (ReadFormUart[0] == VISION_BEGIN)
 {
   //判断帧头数据是否为0xff
  //  if(ReadFormUart[17] == VISION_END)
   {

     //接收数据拷贝
     //memcpy(&VisionRecvData, ReadFormUart, VISION_READ_LEN_PACKED);

      //  if (VisionRecvData.identify_target == TRUE)
      //    if_identify_target = TRUE; // 识别到装甲板
      //  else
      //    if_identify_target = FALSE; // 未识别到装甲板



     // //帧计算
     // Vision_Time_Test[NOW] = xTaskGetTickCount();
     // Vision_Ping = Vision_Time_Test[NOW] - Vision_Time_Test[LAST];//计算时间间隔
     // Vision_Time_Test[LAST] = Vision_Time_Test[NOW];


     memcpy(&VisionRecvData_test, ReadFormUart, sizeof(VisionRecvData_test));
      if(VisionRecvData_test.tracking == TRUE)
         if_identify_target = TRUE; // 识别到装甲板
      else
         if_identify_target = FALSE; // 未识别到装甲板
      
      st.xw = VisionRecvData_test.x;
      st.yw = VisionRecvData_test.y;
      st.zw = VisionRecvData_test.z;
      st.vxw = VisionRecvData_test.v_x;
      st.vyw = VisionRecvData_test.v_y;
      st.vzw = VisionRecvData_test.v_z;
      st.tar_yaw = VisionRecvData_test.yaw;
      st.v_yaw = VisionRecvData_test.v_yaw;
      st.r1 = VisionRecvData_test.r1;
      st.r2 = VisionRecvData_test.r2;
      st.dz = VisionRecvData_test.dz;

      switch(VisionRecvData_test.id)
      {
        case 0:st.armor_id = ARMOR_OUTPOST;
        break;
        case 1:st.armor_id = ARMOR_HERO;
        break;
        case 2:st.armor_id = ARMOR_ENGINEER;
        break;
        case 3:st.armor_id = ARMOR_INFANTRY3;
        break;
        case 4:st.armor_id = ARMOR_INFANTRY4;
        break;
        case 5:st.armor_id = ARMOR_INFANTRY5;
        break;
        case 6:st.armor_id = ARMOR_GUARD;
        break;
        case 7:st.armor_id = ARMOR_BASE;
        break;
      }

      switch(VisionRecvData_test.armors_num)
      {
        case 2:st.armor_num = ARMOR_NUM_BALANCE;
        break;
        case 3:st.armor_num = ARMOR_NUM_OUTPOST;
        break;
        case 4:st.armor_num = ARMOR_NUM_NORMAL;
        break;
      }


      autoSolveTrajectory(&pitch,&yaw,&aim_x,&aim_y,&aim_z);
      VisionRecvData.pitch_angle = pitch;
      VisionRecvData.yaw_angle = yaw;

   }
 }
}

/**
  * @brief  发送视觉指令
  * @param  CmdID
  * @retval void
  * @attention  按协议打包好数据发送
  *				CmdID   0x00   关闭视觉
  *				CmdID   0x01   识别红色装甲
  *				CmdID   0x02   识别蓝色装甲
  *				CmdID   0x03   小符
  *				CmdID   0x04   大符
  */
uint8_t vision_send_pack[sizeof(VisionSendData_test2)] = {0};
uint8_t CmdID = 0;
void vision_send_data(uint8_t CmdID)
{
  for(int i = 0;i < 8;i++)
  {
    st.current_pitch = imu.INS_angle[2];
    st.current_yaw = imu.INS_angle[0];
    float roll = imu.INS_angle[1];
    float pitch = imu.INS_angle[2];
    float yaw = imu.INS_angle[0];
    float aim_x;
    float aim_y;
    float aim_z;
    autoSolveTrajectory(&pitch,&yaw,&aim_x,&aim_y,&aim_z);
    if(can_receive.gimbal_receive.color == 0)
    {
      CmdID = 1;
    }else
    {
      CmdID = 0;
    }
    // VisionSendData.BEGIN = VISION_BEGIN;
    // VisionSendData.CmdID = CmdID;
    // VisionSendData.speed = 2;
    // VisionSendData.yaw = imu.INS_angle[0];
    // VisionSendData.pitch = imu.INS_angle[2];
    // VisionSendData.roll = imu.INS_angle[1];
    // VisionSendData.END = 0xFF;

    VisionSendData_test2[i].BEGIN = VISION_BEGIN;
    VisionSendData_test2[i].detect_color = CmdID;
    VisionSendData_test2[i].reset_tracker = 0;
    VisionSendData_test2[i].roll = imu.INS_angle[1];
    VisionSendData_test2[i].pitch = imu.INS_angle[2];
    VisionSendData_test2[i].yaw = imu.INS_angle[0];
    VisionSendData_test2[i].aim_x = aim_x;
    VisionSendData_test2[i].aim_y = aim_y;
    VisionSendData_test2[i].aim_z = aim_z;
    // VisionSendData_test2[i].checksum = get_CRC16_check_sum(&VisionSendData_test2[i].BEGIN,sizeof(VisionSendData_test2[i]),VisionSendData_test2[i].checksum);
    append_CRC16_check_sum(&VisionSendData_test2[i].BEGIN,sizeof(VisionSendData_test2[i]));
    flag[i] = verify_CRC16_check_sum(&VisionSendData_test2[i].BEGIN,sizeof(VisionSendData_test2[i]));
  }
  memcpy(vision_send_pack, &VisionSendData_test2, sizeof(VisionSendData_test2));

  //将打包好的数据通过串口移位发送到上位机
  HAL_UART_Transmit(&huart1, vision_send_pack, sizeof(VisionSendData_test2), 0xFFF);

  //memset(vision_send_pack, 0, 50);
}

//调解自瞄的跟随速度
uint16_t yaw_para = 50;
uint16_t pitch_para = 200;

void vision_error_angle(float *yaw_angle_error, float *pitch_angle_error)
{
  *yaw_angle_error = -VisionRecvData.yaw_angle / yaw_para;
  *pitch_angle_error = -VisionRecvData.pitch_angle / pitch_para;

  if (VisionRecvData.yaw_angle == 0)
  {
    *yaw_angle_error = 0;
  }
  if (VisionRecvData.pitch_angle == 0)
  {
    *pitch_angle_error = 0;
  }
}

/**
  * @brief  判断是否识别到装甲板
  * @param  void
  * @retval TRUE识别到   FALSE未识别到
  * @attention  为自瞄做准备
  */
bool_t vision_if_find_target(void)
{
  return if_identify_target;
}

/**
  * @brief  判断换装甲板了吗
  * @param  void
  * @retval TRUE换了   FALSE没换
  * @attention  为自动打符做准备,串口空闲中断每触发一次且通过校验,则Vision_Armor置TRUE
  */
bool_t vision_if_armor(void)
{
  return Vision_Armor;
}

/**
  * @brief  换装甲标志位手动置0(false)
  * @param  void
  * @retval void
  * @attention  记得要清零,在哪清零自己选,调用这个函数就行
  */
void vision_clean_ammorflag(void)
{
  Vision_Armor = FALSE;
}
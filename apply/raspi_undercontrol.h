#ifndef __RASPI_UNDERCONTROL_H
#define __RASPI_UNDERCONTROL_H
#include <stdint.h>
// ----------------------------指令或任务类型--------------------------------
#define Instruction_Number_Max 10
#define Instruction_Base_Address 0xA0

typedef enum
{
    Raspi_Ctrl_Speed_Control = Instruction_Base_Address,           // 被树莓派速度控制
    Raspi_Ctrl_Distance_Control,            // 被树莓派距离控制
    Raspi_Ctrl_Contrarotate_90,             // 被树莓派控制左转
    Raspi_Ctrl_Clockwise_Rotation_90,       // 被树莓派控制右转
    Raspi_Ctrl_OPen_Loop_Output_Pwm,        // 树莓派控制ti开环输出pwm
    Raspi_Ctrl_Send_State_Data,             // ti发送状态数据给树莓派
    Raspi_Ctrl_Send_Instruction_toRaspi,    // ti板发送控制任务给树莓派
    Raspi_Ctrl_Number_Recongition_inbegin_task,     // 树莓派起初数字识别任务
    Raspi_Ctrl_Number_recognition_intrack_task,     // 树莓派赛道数字识别任务
    
}Raspi_Ctrl_Instruction_Type;


// ----------------------------ti板接收树莓派数据---------------------------
typedef struct 
{
    uint8_t instruture;    // 控制指令

    float speed;            // 速度控制的期望速度
    float distance;         // 距离控制的期望距离
    
    uint8_t Instruture_Pending_Bit[Instruction_Number_Max];     // 指令待解决挂起位
    uint8_t Task_Executing_Bit[Instruction_Number_Max];     // 指令正在执行挂起位

    int16_t left_pwm, right_pwm;       // 开环控制

    uint8_t speed_byte_buf[4];
    uint8_t distance_byte_buf[4];

}Raspi_Ctrl_Procedure;
extern Raspi_Ctrl_Procedure raspi_ctrl_procedure;

void Raspi_Ctrl_Procedure_Init(void);
void Raspi_Ctrl_Send_State(void);
void Tidata_Tosend_Raspi(uint8_t mode);
void Raspi_Ctrl_Instruction_Dispatch(void);
void Raspi_Data_Phrase_Prepare_Lite(uint8_t data);
void Raspi_Data_Phrase_Process_Lite(uint8_t *data_buf,uint8_t num);  //树莓派数据解析进程




#endif

#ifndef PC_COMM_H
#define PC_COMM_H

#include "bsp_usb.h"
#include "cmsis_os2.h"

struct PC_Send_Data
{
    uint8_t Start_Of_Frame = 0x5A;
    uint8_t Armor = 0x00;
    uint8_t End_Of_Frame[6] = {0xCD, 0xCC, 0x00, 0x00, 0x00, 0x00};
    uint8_t Yaw[4] = {0};
    uint8_t Pitch[4] = {0};
};

struct PC_Recv_Data
{
    uint8_t Start_Of_Frame = 0xA5;
    uint8_t Yaw[4] = {0};
    uint8_t Pitch[4] = {0};
    uint8_t Fire = 0x00;
    uint8_t CRC16[2] = {0};
};

class PcComm
{
public:
    PC_Send_Data send_data;
    PC_Recv_Data recv_data;
    void Init();
    void Send_Message();
    void RxCpltCallback();
private:

};



#endif //PC_COMM_H

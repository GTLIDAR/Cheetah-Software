//
// Created by ziyi on 5/26/21.
//

#ifndef CHEETAH_SOFTWARE_HIGHCMD_H
#define CHEETAH_SOFTWARE_HIGHCMD_H

#include <stdint.h>
#include "custom_cmd_lcmt.hpp"


struct HighCmdCustom {
    int8_t levelFlag;
    int16_t commVersion;
    int16_t robotID;
    int32_t SN;
    int8_t bandWidth;
    int8_t mode;                      // 0:idle, default stand      1:forced stand     2:walk continuously
    float forwardSpeed;                // speed of move forward or backward, scale: -1~1
    float sideSpeed;                   // speed of move left or right, scale: -1~1
    float rotateSpeed;	               // speed of spin left or right, scale: -1~1
    float bodyHeight;                  // body height, scale: -1~1
    float footRaiseHeight;             // foot up height while walking, scale: 0~1
    float yaw;                         // unit: radian, scale: -1~1
    float pitch;                       // unit: radian, scale: -1~1
    float roll;                        // unit: radian, scale: -1~1
    int8_t wirelessRemote[40];
    int8_t AppRemote[40];
    int32_t reserve;
    int32_t crc;

    void set(const custom_cmd_lcmt* msg){
        levelFlag = msg->levelFlag;
        commVersion = msg->commVersion;
        robotID = msg->robotID;
        SN = msg->SN;
        bandWidth = msg->bandWidth;
        mode = msg->mode;
        forwardSpeed = msg->forwardSpeed;
        sideSpeed = msg->sideSpeed;
        rotateSpeed = msg->rotateSpeed;
        bodyHeight = msg->bodyHeight;
        footRaiseHeight = msg->footRaiseHeight;
        yaw = msg->yaw;
        pitch = msg->pitch;
        roll = msg->roll;
        for(int i = 0; i<40; i++){
            wirelessRemote[i] = msg->wirelessRemote[i];
            AppRemote[i] = msg->AppRemote[i];
        }
        reserve = msg->reserve;
        crc = msg->crc;
    }
};
#endif //CHEETAH_SOFTWARE_HIGHCMD_H

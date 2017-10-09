#ifndef COMMU_DATA_EXCHANGE_
#define COMMU_DATA_EXCHANGE_

#include <stdint.h>
//Mega to due
#pragma pack(1)
typedef struct {
  uint8_t cmd;
  uint16_t vx; //x-axis speed
  uint16_t vy; //y-axis speed
  uint16_t wr; //Rotation speed
}Speed_cmd;

//Due to Mega
#pragma pack()

#endif

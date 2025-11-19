/**
* @Function: IMU message functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/stream/format_imu.h"

#define IMUPREAMB 0xFECB  /* IMU frame preamble */

// UART Byte Markers
// Start of all UART transfers
const unsigned char UART_HEADER = 0x80;
// End of all UART transfers
const unsigned char UART_DELIMITER = 0x0D;
static unsigned char rxByteBuf[256];

int scale_inited = 0;
double acc_encode_factor;
double gyro_encode_factor;

// Epson G370PDF1 Properties; may change for other IMUs
struct EpsonProperties epson_sensor = {
  .model = G370PDF1,
  .product_id = "G370PDF1",
  .feature_flags =
        (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG | HAS_ROT_MATRIX |
         HAS_RANGE_OVER | HAS_RTDIAG | HAS_INITIAL_BACKUP),
  .gyro_sf_dps = (1.0 / 66),
  .accl_sf_mg = (1.0 / 2.5),
  .tempc_sf_degc = (-0.0037918),
  .tempc_25c_offset = (2364),
  .rstcnt_sf_micros = (16000),
  .ang_sf_deg = (0),
  .qtn_sf = (0),
  .dlta0_sf_deg = (1.0 / 66 * 1 / 1000),
  .dltv0_sf_mps = (1.0 / 2.5 * 1 / 1000 * 1 / 1000 * 9.80665),
  .delay_reset_ms = (800),
  .delay_flashtest_ms = (5),
  .delay_flashbackup_ms = (200),
  .delay_selftest_ms = (80),
  .delay_filter_ms = (1),
  .delay_atti_profile_ms = (0),
};

struct EpsonOptions epson_options = {
    .ext_sel = 0,  // 0 = Sample Counter 1=Reset Counter 2=External Trigger
    .ext_pol = 0,
#ifdef SPI
    .drdy_on = 1,
#endif  // SPI
    .drdy_pol = 1,
    .dout_rate = CMD_RATE100,
    .filter_sel = CMD_FLTAP32,
    .flag_out = 0,
    .temp_out = 0,
    .gyro_out = 1,
    .accel_out = 1,
    .gyro_delta_out = 0,
    .accel_delta_out = 0,
    .qtn_out = 0,   // Only valid for devices that support attitude output
    .atti_out = 0,  // Only valid for devices that support attitude output
    .count_out = 0,
    .checksum_out = 1,

    // Set 0=16bit 1=32bit sensor output.
    // These only have effect if above related "_out = 1"
    .temp_bit = 0,
    .gyro_bit = 1,
    .accel_bit = 1,
    .gyro_delta_bit = 0,
    .accel_delta_bit = 0,
    .qtn_bit = 0,
    .atti_bit = 0,

    .dlta_range_ctrl = 8,
    .dltv_range_ctrl = 8,

    // NOTE: The following are only valid when attitude output is enabled
    .atti_mode = 1,    // 0=Inclination mode 1=Euler mode
    .atti_conv = 0,    // Attitude Conversion Mode, must be 0 when quaternion
                       // output is enabled
    .atti_profile = 0  // Attitude Motion Profile 0=modeA 1=modeB 2=modeC
};



/* init scale  --------------------------------------------------------*/
static void init_scale()
{
  acc_encode_factor=1.0/(9.8*24)*pow(2.0,20);
  gyro_encode_factor=R2D/4000.0*pow(2.0,20);
}

/* 8 bit parity  ------------------------------------------------------*/
static uint8_t crc8(const uint8_t *buff, int len)
{
  uint8_t crc=0;
  int i;
  
  trace(4,"crc8: len=%d\n",len);
  
  for (i=0;i<len;i++) crc^=buff[i];

  return crc;
}

// /* decode imu message -------------------------------------------------*/
// static int decode_imu(imu_t *imu)
// {
//   gtime_t time;
//   int i=24,j,acc[3],gyro[3],sec;
  
//   trace(3,"decode_imu: len=%3d\n",imu->len);

//   if (!scale_inited) init_scale();

//   time.time=getbitu(imu->buff,i,32); i+=32;
//   sec    =getbitu(imu->buff,i,32); i+=32;
//   for (j=0;j<3;j++) {
//     acc[j]=getbits(imu->buff,i,20); i+=20;
//   }
//   for (j=0;j<3;j++) {
//     gyro[j]=getbits(imu->buff,i,20); i+=20;
//   }
//   time.sec=(double)sec*1.0e-9;
//   imu->time=time;
//   for (j=0;j<3;j++) {
//     imu->acc[j]=(double)acc[j]/acc_encode_factor;
//     imu->gyro[j]=(double)gyro[j]/gyro_encode_factor;
//   }
  
//   return 1;
// }

/* encode imu message -------------------------------------------------*/
extern int encode_imu(imu_t *imu)
{
  int i=24,j,sec=imu->time.sec*1.0e9;
  int32_t tmp;

  trace(3,"encode_imu\n");

  if (!scale_inited) init_scale();
  
  setbitu(imu->buff,i,32,imu->time.time); i+=32;
  setbitu(imu->buff,i,32,sec       ); i+=32;
  /* acceleration, range: -12g~12g, precision: 4.5e-4 m/s^2 */
  for (j=0;j<3;j++) {
    tmp = imu->acc[j]*acc_encode_factor;
    setbits(imu->buff,i,20,tmp); i+=20;
  }
  /* angular rate, range: -2000~2000 deg/s, precision: 3.8e-3 deg/s */
  for (j=0;j<3;j++) {
    tmp = imu->gyro[j]*gyro_encode_factor;
    setbits(imu->buff,i,20,tmp); i+=20;
  }

  imu->len=i/8;

  return 1;
}


/* input imu message from stream --------------------------------------------
* fetch next imu message and input a message from byte stream
* args   : imu_t *imu     IO  imu control struct
*      uint8_t data   I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input imu data)
* notes  : 
*      imu message format:
*      +----------+-----------+--------------------+---------+
*      | preamble |  length   |    data message    | parity  |
*      +----------+-----------+--------------------+---------+
*      |<-- 16 -->|<--- 8 --->|<--- length x 8 --->|<-- 8 -->|
*      
*-----------------------------------------------------------------------------*/
// extern int input_imu(imu_t *imu, uint8_t data)
// {
//   trace(5,"input_imu: data=%02x\n",data);
  
//   /* synchronize frame */
//   if (imu->nbyte==0) {
//     if (data!=(uint8_t)((IMUPREAMB&0xFF00)>>8)) return 0;
//     imu->buff[imu->nbyte++]=data;
//     return 0;
//   }
//   if (imu->nbyte==1) {
//     if (data!=(uint8_t)(IMUPREAMB&0x00FF)) {
//       imu->nbyte = 0; return 0;
//     }
//     imu->buff[imu->nbyte++]=data;
//     return 0;
//   }
//   imu->buff[imu->nbyte++]=data;
  
//   if (imu->nbyte==3) {
//     imu->len=getbitu(imu->buff,16,8)+3; /* length */
//   }
//   if (imu->nbyte<3||imu->nbyte<imu->len+1) return 0;
//   imu->nbyte=0;

//   /* check parity */
//   if (crc8(imu->buff,imu->len)!=getbitu(imu->buff,imu->len*8,8)) {
//     trace(2,"imu parity error: len=%d\n",imu->len);
//     return 0;
//   }

//   /* decode imu message */
//   return decode_imu(imu);
// }

/*****************************************************************************
** Function name:       sensorDataScaling
** Description:         Retrieves burst data buffer, converts and stores into
**                      sensor data struct based on settings.
**                      based on configuration.
** Parameters:          pointer to struct describing IMU properties.
**                      pointer to struct describing IMU settings.
**                      pointer to struct for converted sensor data.
** Return value:        none
** Notes:
******************************************************************************/
void sensorDataScaling(const struct EpsonProperties* esensor,
                       const struct EpsonOptions* options,
                       struct EpsonData* data) {
  // stores the sensor data array index when parsing out data fields
  int idx = 0;

  // parsing of data fields applying conversion factor if applicable
  if (options->flag_out) {
    unsigned short ndflags = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
    data->ndflags = ndflags;
    idx += 2;
  }

  if (options->temp_out) {
    if (options->temp_bit) {
      // 32-bit calculation
      int temp = (rxByteBuf[idx] << 8 * 3) + (rxByteBuf[idx + 1] << 8 * 2) +
                 (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];

      if ((esensor->model == G330PDG0) || (esensor->model == G366PDG0) ||
          (esensor->model == G370PDG0) || (esensor->model == G370PDT0) ||
          (esensor->model == G570PR20)) {
        // These models do not have a 25degC temperature offset
        data->temperature = temp * esensor->tempc_sf_degc / 65536 + 25;
      } else {
        data->temperature = (temp - esensor->tempc_25c_offset * 65536) *
                              esensor->tempc_sf_degc / 65536 +
                            25;
      }

      idx += 4;
    } else {
      // 16-bit calculation
      short temp = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];

      if ((esensor->model == G330PDG0) || (esensor->model == G366PDG0) ||
          (esensor->model == G370PDG0) || (esensor->model == G370PDT0) ||
          (esensor->model == G570PR20)) {
        // These models do not have a 25degC temperature offset
        data->temperature = temp * esensor->tempc_sf_degc + 25;
      } else {
        data->temperature =
          (temp - esensor->tempc_25c_offset) * esensor->tempc_sf_degc + 25;
      }

      idx += 2;
    }
  }

  if (options->gyro_out) {
    if (options->gyro_bit) {
      // 32-bit calculation
      int gyro_x = (rxByteBuf[idx] << 8 * 3) + (rxByteBuf[idx + 1] << 8 * 2) +
                   (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int gyro_y = (rxByteBuf[idx + 4] << 8 * 3) +
                   (rxByteBuf[idx + 5] << 8 * 2) + (rxByteBuf[idx + 6] << 8) +
                   rxByteBuf[idx + 7];
      int gyro_z = (rxByteBuf[idx + 8] << 8 * 3) +
                   (rxByteBuf[idx + 9] << 8 * 2) + (rxByteBuf[idx + 10] << 8) +
                   rxByteBuf[idx + 11];
      data->gyro_x = (esensor->gyro_sf_dps / 65536) * DEG2RAD * gyro_x;
      data->gyro_y = (esensor->gyro_sf_dps / 65536) * DEG2RAD * gyro_y;
      data->gyro_z = (esensor->gyro_sf_dps / 65536) * DEG2RAD * gyro_z;
      idx += 12;
    } else {
      // 16-bit calculation
      short gyro_x = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short gyro_y = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short gyro_z = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      data->gyro_x = esensor->gyro_sf_dps * DEG2RAD * gyro_x;
      data->gyro_y = esensor->gyro_sf_dps * DEG2RAD * gyro_y;
      data->gyro_z = esensor->gyro_sf_dps * DEG2RAD * gyro_z;
      idx += 6;
    }
  }

  if (options->accel_out) {
    if (options->accel_bit) {
      // 32-bit calculation
      int accel_x = (rxByteBuf[idx] << 8 * 3) + (rxByteBuf[idx + 1] << 8 * 2) +
                    (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int accel_y = (rxByteBuf[idx + 4] << 8 * 3) +
                    (rxByteBuf[idx + 5] << 8 * 2) + (rxByteBuf[idx + 6] << 8) +
                    rxByteBuf[idx + 7];
      int accel_z = (rxByteBuf[idx + 8] << 8 * 3) +
                    (rxByteBuf[idx + 9] << 8 * 2) + (rxByteBuf[idx + 10] << 8) +
                    rxByteBuf[idx + 11];
      data->accel_x = (esensor->accl_sf_mg / 65536) * MG2MPS2 * accel_x;
      data->accel_y = (esensor->accl_sf_mg / 65536) * MG2MPS2 * accel_y;
      data->accel_z = (esensor->accl_sf_mg / 65536) * MG2MPS2 * accel_z;
      idx += 12;
    } else {
      // 16-bit calculation
      short accel_x = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short accel_y = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short accel_z = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      data->accel_x = (esensor->accl_sf_mg) * MG2MPS2 * accel_x;
      data->accel_y = (esensor->accl_sf_mg) * MG2MPS2 * accel_y;
      data->accel_z = (esensor->accl_sf_mg) * MG2MPS2 * accel_z;
      idx += 6;
    }
  }

  if (options->gyro_delta_out) {
    double da_sf =
      esensor->dlta0_sf_deg * (1 << options->dlta_range_ctrl) * DEG2RAD;
    if (options->gyro_delta_bit) {
      // 32-bit calculation
      int gyro_delta_x = (rxByteBuf[idx] << 8 * 3) +
                         (rxByteBuf[idx + 1] << 8 * 2) +
                         (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int gyro_delta_y = (rxByteBuf[idx + 4] << 8 * 3) +
                         (rxByteBuf[idx + 5] << 8 * 2) +
                         (rxByteBuf[idx + 6] << 8) + rxByteBuf[idx + 7];
      int gyro_delta_z = (rxByteBuf[idx + 8] << 8 * 3) +
                         (rxByteBuf[idx + 9] << 8 * 2) +
                         (rxByteBuf[idx + 10] << 8) + rxByteBuf[idx + 11];

      data->gyro_delta_x = gyro_delta_x * (da_sf) / 65536;
      data->gyro_delta_y = gyro_delta_y * (da_sf) / 65536;
      data->gyro_delta_z = gyro_delta_z * (da_sf) / 65536;
      idx += 12;
    } else {
      // 16-bit calculation
      short gyro_delta_x = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short gyro_delta_y = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short gyro_delta_z = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      data->gyro_delta_x = gyro_delta_x * (da_sf);
      data->gyro_delta_y = gyro_delta_y * (da_sf);
      data->gyro_delta_z = gyro_delta_z * (da_sf);
      idx += 6;
    }
  }

  if (options->accel_delta_out) {
    double dv_sf = esensor->dltv0_sf_mps * (1 << options->dltv_range_ctrl);
    if (options->accel_delta_bit) {
      // 32-bit calculation
      int accel_delta_x = (rxByteBuf[idx] << 8 * 3) +
                          (rxByteBuf[idx + 1] << 8 * 2) +
                          (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int accel_delta_y = (rxByteBuf[idx + 4] << 8 * 3) +
                          (rxByteBuf[idx + 5] << 8 * 2) +
                          (rxByteBuf[idx + 6] << 8) + rxByteBuf[idx + 7];
      int accel_delta_z = (rxByteBuf[idx + 8] << 8 * 3) +
                          (rxByteBuf[idx + 9] << 8 * 2) +
                          (rxByteBuf[idx + 10] << 8) + rxByteBuf[idx + 11];
      data->accel_delta_x = accel_delta_x * (dv_sf) / 65536;
      data->accel_delta_y = accel_delta_y * (dv_sf) / 65536;
      data->accel_delta_z = accel_delta_z * (dv_sf) / 65536;
      idx += 12;
    } else {
      // 16-bit calculation
      short accel_delta_x = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short accel_delta_y = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short accel_delta_z = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      data->accel_delta_x = accel_delta_x * (dv_sf);
      data->accel_delta_y = accel_delta_y * (dv_sf);
      data->accel_delta_z = accel_delta_z * (dv_sf);
      idx += 6;
    }
  }

  if (options->qtn_out) {
    if (options->qtn_bit) {
      // 32-bit calculation
      int qtn0 = (rxByteBuf[idx] << 8 * 3) + (rxByteBuf[idx + 1] << 8 * 2) +
                 (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int qtn1 = (rxByteBuf[idx + 4] << 8 * 3) + (rxByteBuf[idx + 5] << 8 * 2) +
                 (rxByteBuf[idx + 6] << 8) + rxByteBuf[idx + 7];
      int qtn2 = (rxByteBuf[idx + 8] << 8 * 3) + (rxByteBuf[idx + 9] << 8 * 2) +
                 (rxByteBuf[idx + 10] << 8) + rxByteBuf[idx + 11];
      int qtn3 = (rxByteBuf[idx + 12] << 8 * 3) +
                 (rxByteBuf[idx + 13] << 8 * 2) + (rxByteBuf[idx + 14] << 8) +
                 rxByteBuf[idx + 15];
      data->qtn0 = (double)qtn0 * esensor->qtn_sf / 65536;
      data->qtn1 = (double)qtn1 * esensor->qtn_sf / 65536;
      data->qtn2 = (double)qtn2 * esensor->qtn_sf / 65536;
      data->qtn3 = (double)qtn3 * esensor->qtn_sf / 65536;
      idx += 16;
    } else {
      // 16-bit calculation
      short qtn0 = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short qtn1 = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short qtn2 = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      short qtn3 = (rxByteBuf[idx + 6] << 8) + rxByteBuf[idx + 7];
      data->qtn0 = (double)qtn0 * esensor->qtn_sf;
      data->qtn1 = (double)qtn1 * esensor->qtn_sf;
      data->qtn2 = (double)qtn2 * esensor->qtn_sf;
      data->qtn3 = (double)qtn3 * esensor->qtn_sf;
      idx += 8;
    }
  }

  if (options->atti_out) {
    if (options->atti_bit) {
      // 32-bit calculation
      int ang1 = (rxByteBuf[idx] << 8 * 3) + (rxByteBuf[idx + 1] << 8 * 2) +
                 (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int ang2 = (rxByteBuf[idx + 4] << 8 * 3) + (rxByteBuf[idx + 5] << 8 * 2) +
                 (rxByteBuf[idx + 6] << 8) + rxByteBuf[idx + 7];
      int ang3 = (rxByteBuf[idx + 8] << 8 * 3) + (rxByteBuf[idx + 9] << 8 * 2) +
                 (rxByteBuf[idx + 10] << 8) + rxByteBuf[idx + 11];
      data->ang1 = (esensor->ang_sf_deg / 65536) * DEG2RAD * ang1;
      data->ang2 = (esensor->ang_sf_deg / 65536) * DEG2RAD * ang2;
      data->ang3 = (esensor->ang_sf_deg / 65536) * DEG2RAD * ang3;
      idx += 12;
    } else {
      // 16-bit calculation
      short ang1 = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short ang2 = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short ang3 = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      data->ang1 = esensor->ang_sf_deg * DEG2RAD * ang1;
      data->ang2 = esensor->ang_sf_deg * DEG2RAD * ang2;
      data->ang3 = esensor->ang_sf_deg * DEG2RAD * ang3;
      idx += 6;
    }
  }

  if (options->gpio_out) {
    unsigned short gpio = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
    data->gpio = gpio;
    idx += 2;
  }

  if (options->count_out) {
    int count = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
    if (options->ext_sel == 1)
      data->count = count * esensor->rstcnt_sf_micros;
    else
      data->count = count;
  }
}

/* decode imu message -------------------------------------------------*/
static int decode_imu(imu_t *imu)
{
  struct EpsonData epson_data; 
  sensorDataScaling(&epson_sensor, &epson_options, &epson_data);
  imu->acc[0] = epson_data.accel_x;
  imu->acc[1] = epson_data.accel_y;
  imu->acc[2] = epson_data.accel_z;
  imu->gyro[0] = epson_data.gyro_x;
  imu->gyro[1] = epson_data.gyro_y;
  imu->gyro[2] = epson_data.gyro_z;

  return 1;
}

/* input imu EPSON message from stream --------------------------------------------
* fetch next imu message and input a message from byte stream
* args   : imu_t *imu     IO  imu control struct
*      uint8_t data   I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input imu data)
* notes  : 
*      imu message format:
*      +----------+-----------+--------------------+---------+
*      | header  |  data message   |    checksum  | delimiter  |
*      +----------+-----------+--------------------+---------+
*      |<-- 8 -->|<--- lengthx8--->|<--- 8 --->|<-- 8 -->|
*      
*-----------------------------------------------------------------------------*/
extern int input_imu(imu_t *imu, uint8_t data)
{
  trace(5,"input_imu: data:%02x\n", data);
  int byte_length = sensorDataByteLength(&epson_sensor, &epson_options);
  int data_length = byte_length - 2;

  /* UART HEADER */ 
  if(data==UART_HEADER){
    imu->nbyte = 0; 
    return 0;
  }

  /* populate imu data */
  if(imu->nbyte != data_length){
    imu->buff[imu->nbyte] = data;
    rxByteBuf[imu->nbyte] = data;
    imu->nbyte++; 
    return 0;
  }

  if(data==UART_DELIMITER){
    
    // checksum validation
    if (epson_options.checksum_out == 1) {
      unsigned short calc_checksum = 0;
      for (int i = 0; i < data_length - 2; i += 2) {
        calc_checksum += (rxByteBuf[i] << 8) + rxByteBuf[i + 1];
      }
      unsigned short epson_checksum =
              (rxByteBuf[data_length - 2] << 8) + rxByteBuf[data_length - 1];

      if (calc_checksum != epson_checksum) {
        printf("checksum failed\n");
        printf("calc_checksum: %d\n", calc_checksum);
        printf("epson_checksum: %d\n", epson_checksum);
        return -1;
      }
      else{
        printf("checksum success\n");
        // decode imu data
        // sensorDataScaling(&epson_sensor, &epson_options, &epson_data);
        // imu->acc[0] = epson_data.accel_x;
        // imu->acc[1] = epson_data.accel_y;
        // imu->acc[2] = epson_data.accel_z;
        // imu->gyro[0] = epson_data.gyro_x;
        // imu->gyro[1] = epson_data.gyro_y;
        // imu->gyro[2] = epson_data.gyro_z;
      }
    }
    // decode imu data
    //sensorDataScaling(&epson_sensor, &epson_options, &epson_data);
    //printf("Successfully decoded data\n"); 
    imu->nbyte = 0;
    return decode_imu(imu);
  }

  

}

/* generate imu message -----------------------------------------------------
* generate imu message
* args   : imu_t *imu     IO  imu control struct
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int gen_imu(imu_t *imu)
{
  int i=0,crc=0;
  
  trace(4,"gen_imu\n");
  
  imu->nbyte=imu->len=0;
  
  /* set preamble and reserved */
  setbitu(imu->buff,i,16,IMUPREAMB); i+=16;
  setbitu(imu->buff,i,8 ,0    ); i+=8;
  
  /* encode imu message body */
  if (!encode_imu(imu)) return 0;

  /* message length without header and parity */
  setbitu(imu->buff,16,8,imu->len-3);
  
  /* crc-8q */
  crc=crc8(imu->buff,imu->len);
  i=imu->len*8;
  setbitu(imu->buff,i,8,crc);
  
  /* length total (bytes) */
  imu->nbyte=imu->len+1;
  
  return 1;
}

/* Initialize imu structure */
extern void init_imu(imu_t *imu)
{
  gtime_t time0 = {0};
  int i;

  imu->time = time0;
  imu->nmax = 65535;
  for (i = 0; i < 3; i++) {
    imu->acc[i] = 0.0;
    imu->gyro[i] = 0.0;
  }
  imu->nbyte = 0;
  imu->len = 0;
  if (!(imu->buff = (uint8_t*)malloc(sizeof(uint8_t)*imu->nmax))) {
    free_imu(imu);
    return;
  }
}

/* Free imu structure */
extern void free_imu(imu_t *imu)
{
  free(imu->buff); imu->buff = NULL;
}
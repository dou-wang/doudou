#include "ICM20948.h"
#include "math.h"
ICM20948_ST_SENSOR_DATA gstGyroOffset ={0,0,0};
IMU_ST_SENSOR_DATA gstMagOffset = {0,0,0};
int16_t TestmagnBuff[9]={0};
uint32_t KeyAndJostickValue;
int16_t magn[3];

mpu20948 angle;


//Relative to the range set by the IMU gyroscope, the range is ±500°, corresponding data range is ±32768
//The gyroscope raw data is converted in radian (rad) units, 1/65.5/57.30=0.00026644
//与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位，1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO   0.00026644f
//Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is ±32768
//Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=1671.84	
//与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACCEl_RATIO 	  1671.84f  


//零点漂移计数
int Deviation_Count;
//Triaxial IMU data
//三轴IMU数据

// Gyro static error, raw data
//陀螺仪静差，原始数据
short Deviation_gyro[3],Original_gyro[3];    
short Deviation_accel[3],Original_accel[3];   
short s16Gyro[3], s16Accel[3], s16Magn[3];

short gyro[3], accel[3],magnet[3];

void ICM20948_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while(1)
	{	
		//This task runs at 100Hz
		//此任务以100Hz的频率运行
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));	
		
		//Read the gyroscope zero before starting
		//开机前，读取陀螺仪零点			
		if(Deviation_Count<CONTROL_DELAY)
		{	 
			Deviation_Count++;
			memcpy(Deviation_gyro,gyro,sizeof(gyro));		
			memcpy(Deviation_accel,accel,sizeof(accel));
		}	
		
		//Get acceleration sensor data 
		MPU_Get_Accel(); //得到加速度传感器数据
		
		//Get gyroscope data 
		MPU_Get_Gyroscope(); //得到陀螺仪数据
		
		
		Quaternion_Solution(gyro[0]*GYROSCOPE_RATIO,gyro[1]*GYROSCOPE_RATIO,gyro[2]*GYROSCOPE_RATIO,accel[0]/ACCEl_RATIO,accel[1]/ACCEl_RATIO,accel[2]/ACCEl_RATIO);
		
		
		
		#if 0 //是否开启采集磁力计数据
		static u8 mag_count=0;
		//Get magnetometer data
		mag_count++;
		if(mag_count>=13)	 //磁力计最大采样速率为8hz，因此每进入这个函数13次才采样一次磁力计信息
		{
			invMSMagRead(&magnet[0], &magnet[1], &magnet[2]); //得到磁力计数据
			mag_count=0;
		}	
		#endif


	}
}  

 bool invmsICM20948Check(void)
{
    bool bRet = false;
    if(REG_VAL_WIA == I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_WIA))
    {
        bRet = true;
    }
    return bRet;
}

void invMSInit()
{
    if(invmsICM20948Check())//检测是否能识别到ICM20498器件
		{
			invmsICM20948Init(); //ICM20498初始化
//			 printf("ICM20948 init_successful");
		}
    return;
}



 void invmsICM20948Init(void)
{
    /* user bank 0 register */
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_PWR_MIGMT_1,  REG_VAL_ALL_RGE_RESET);
    delay_ms(10);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_PWR_MIGMT_1,  REG_VAL_RUN_MODE);  
    
    /* user bank 2 register */
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2);
	
    I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_GYRO_SMPLRT_DIV, 0x07);
    I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_GYRO_CONFIG_1,REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_500DPS | REG_VAL_BIT_GYRO_DLPF);
	
    I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_ACCEL_SMPLRT_DIV_2,  0x07);
    I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_ACCEL_CONFIG,REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF);
    
    /* user bank 0 register */
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);   
    
    delay_ms(100);
    /* offset */
    invmsICM20948GyroOffset();

    invmsICM20948MagCheck();

    invmsICM20948WriteSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_WRITE,REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_100HZ);  
    return;
}

 void invmsICM20948GyroRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
    uint8_t u8Buf[6];
    int16_t s16Buf[3] = {0}; 
    uint8_t i;
    int32_t s32OutBuf[3] = {0};
    static ICM20948_ST_AVG_DATA sstAvgBuf[3];
    static int16_t ss16c = 0;
    ss16c++;

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_XOUT_L); 
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_XOUT_H);
    s16Buf[0]=	(u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_YOUT_L); 
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_YOUT_H);
    s16Buf[1]=	(u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_ZOUT_L); 
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_ZOUT_H);
    s16Buf[2]=	(u8Buf[1]<<8)|u8Buf[0];
    
#if 1
    for(i = 0; i < 3; i ++)	
    {
        invmsICM20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
    }
    *ps16X = s32OutBuf[0] - gstGyroOffset.s16X;
    *ps16Y = s32OutBuf[1] - gstGyroOffset.s16Y;
    *ps16Z = s32OutBuf[2] - gstGyroOffset.s16Z;
#else     
    *ps16X = s16Buf[0];
    *ps16Y = s16Buf[1];
    *ps16Z = s16Buf[2];
#endif    
    return;
}

 void invmsICM20948AccelRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
    uint8_t u8Buf[2];
    int16_t s16Buf[3] = {0}; 
    uint8_t i;
    int32_t s32OutBuf[3] = {0};
    static ICM20948_ST_AVG_DATA sstAvgBuf[3];

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_XOUT_L); 
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_XOUT_H);
    s16Buf[0]=	(u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_YOUT_L); 
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_YOUT_H);
    s16Buf[1]=	(u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_ZOUT_L); 
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_ZOUT_H);
    s16Buf[2]=	(u8Buf[1]<<8)|u8Buf[0];

#if 1
    for(i = 0; i < 3; i ++)	
    {
        invmsICM20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
    }
    *ps16X = s32OutBuf[0];
    *ps16Y = s32OutBuf[1];
    *ps16Z = s32OutBuf[2];

#else     
    *ps16X = s16Buf[0];
    *ps16Y = s16Buf[1];
    *ps16Z = s16Buf[2];
#endif    
    return;

}

 void invmsICM20948MagRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{   
    uint8_t counter = 20;
    uint8_t u8Data[MAG_DATA_LEN];
    int16_t s16Buf[3] = {0}; 
    uint8_t i;
    int32_t s32OutBuf[3] = {0};
    static ICM20948_ST_AVG_DATA sstAvgBuf[3];
    while( counter>0 )
    {
        delay_ms(10);
        invmsICM20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ, 
                                    REG_ADD_MAG_ST2, 1, u8Data);
        
        if ((u8Data[0] & 0x01) != 0)
            break;
        
        counter--;
    }
    
    if(counter != 0)
    {
        invmsICM20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ, 
                                    REG_ADD_MAG_DATA, 
                                    MAG_DATA_LEN,
                                    u8Data);
        s16Buf[0] = ((int16_t)u8Data[1]<<8) | u8Data[0];
        s16Buf[1] = ((int16_t)u8Data[3]<<8) | u8Data[2];
        s16Buf[2] = ((int16_t)u8Data[5]<<8) | u8Data[4];       
    }
    else
    {
        printf("\r\n Mag is bussy \r\n");
    }
#if 1    
    for(i = 0; i < 3; i ++)	
    {
        invmsICM20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
    }
    
    *ps16X =  s32OutBuf[0];
    *ps16Y = -s32OutBuf[1];
    *ps16Z = -s32OutBuf[2];
#else     
    *ps16X = s16Buf[0];
    *ps16Y = -s16Buf[1];
    *ps16Z = -s16Buf[2];
#endif   
    return;
}

bool invmsICM20948MagCheck(void)
{
    bool bRet = false;
    uint8_t u8Ret[2];
    
    invmsICM20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,
                                REG_ADD_MAG_WIA1, 2,u8Ret);
    if( (u8Ret[0] == REG_VAL_MAG_WIA1) && ( u8Ret[1] == REG_VAL_MAG_WIA2) )
    {
        bRet = true;
    }
    
    return bRet;
}

void invmsICM20948ReadSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8Len, uint8_t *pu8data)
{
    uint8_t i;
    uint8_t u8Temp;
    
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3); //swtich bank3
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_ADDR, u8I2CAddr);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_REG,  u8RegAddr);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN|u8Len);

    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0
    
    u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_USER_CTRL);
    u8Temp |= REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);
    delay_ms(5);
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);
    
    for(i=0; i<u8Len; i++)
    {
        *(pu8data+i) = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_EXT_SENS_DATA_00+i);
        
    }
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); //swtich bank3
    
    u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_I2C_SLV0_CTRL);
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL,  u8Temp);
    
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

}

void invmsICM20948WriteSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8data)
{
    uint8_t u8Temp;
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3); //swtich bank3
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_ADDR, u8I2CAddr);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_REG,  u8RegAddr);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_DO,   u8data);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN|1);
    
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0
    
    u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_USER_CTRL);
    u8Temp |= REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);
    delay_ms(5);
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);
    
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); //swtich bank3
    
    u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_I2C_SLV0_CTRL);
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL,  u8Temp);
   
   I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0
    
    return;
}
 
void invmsICM20948CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
{	
	uint8_t i;
	
	*(pAvgBuffer + ((*pIndex) ++)) = InVal;
  	*pIndex &= 0x07;
  	
  	*pOutVal = 0;
	for(i = 0; i < 8; i ++) 
  	{
    	*pOutVal += *(pAvgBuffer + i);
  	}
  	*pOutVal >>= 3;
}

void invmsICM20948GyroOffset(void)
{
	uint8_t i;
    int16_t	s16Gx = 0, s16Gy = 0, s16Gz = 0;
	int32_t	s32TempGx = 0, s32TempGy = 0, s32TempGz = 0;
    for(i = 0; i < 32; i ++)
 	{
        invmsICM20948GyroRead(&s16Gx, &s16Gy, &s16Gz);
        s32TempGx += s16Gx;
		s32TempGy += s16Gy;
		s32TempGz += s16Gz;
        delay_ms(10);
    }
    gstGyroOffset.s16X = s32TempGx >> 5;
	gstGyroOffset.s16Y = s32TempGy >> 5;
	gstGyroOffset.s16Z = s32TempGz >> 5;
    return;
}

void TestMagn(void)
{
	TestmagnBuff[(KeyAndJostickValue-1)*3]=magn[0];
	TestmagnBuff[(KeyAndJostickValue-1)*3+1]=magn[1];
	TestmagnBuff[(KeyAndJostickValue-1)*3+2]=magn[2];
	
	if(KeyAndJostickValue>=3)
	{
			gstMagOffset.s16X = (TestmagnBuff[0]+TestmagnBuff[3])/2;
			gstMagOffset.s16Y = (TestmagnBuff[1]+TestmagnBuff[4])/2;
			gstMagOffset.s16Z = (TestmagnBuff[5]+TestmagnBuff[8])/2;
//			Flag_Check_Magn=true;
	}
}

/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Gx, Gy, Gz: raw readings (plus or minus) of the x,y, and z axes of the gyroscope
Output  : 0: success, others: error code
函数功能：获得陀螺仪值(原始值)
入口参数：gx,gy,gz:陀螺仪x,y,z轴的原始读数(带正负)
返回  值：0:成功, 其他:错误代码
**************************************************************************/
u8 MPU_Get_Gyroscope(void)
{
	u8 res; 
	invMSGyroRead(&gyro[0], &gyro[1], &gyro[2]);
	if(Deviation_Count<CONTROL_DELAY) // 10 seconds before starting //开机前10秒
	{
		Led_Count=1; //LED high frequency flashing //LED高频闪烁	
	}
	else //10 seconds after starting //开机10秒后
	{  
		if(Deviation_Count==CONTROL_DELAY)
		Flag_Stop=0; //The software fails to flag location 0 //软件失能标志位置0
		Led_Count=300; //The LED returns to normal flicker frequency //LED恢复正常闪烁频率	

		//Save the raw data to update zero by clicking the user button
		//保存原始数据用于单击用户按键更新零点
		Original_gyro[0] =gyro[0];  
		Original_gyro[1] =gyro[1];  
		Original_gyro[2]= gyro[2];			

		//Removes zero drift data
		//去除零点漂移的数据
		gyro[0] =Original_gyro[0]-Deviation_gyro[0];  
		gyro[1] =Original_gyro[1]-Deviation_gyro[1];  
		gyro[2]= Original_gyro[2]-Deviation_gyro[2];
	}

	return res;
}


u8 MPU_Get_Accel(void)
{
	u8 res; 
	invMSAccelRead(&accel[0], &accel[1], &accel[2]); //得到加速度传感器数据
	if(Deviation_Count<CONTROL_DELAY) // 10 seconds before starting //开机前10秒
	{
		__nop();
	}
	else //10 seconds after starting //开机10秒后
	{  
		//Save the raw data to update zero by clicking the user button
		//保存原始数据用于单击用户按键更新零点
		Original_accel[0] =accel[0];  
		Original_accel[1] =accel[1];  
		Original_accel[2]= accel[2];			

		//Removes zero drift data
		//去除零点漂移的数据
		accel[0] =Original_accel[0]-Deviation_accel[0];  
		accel[1] =Original_accel[1]-Deviation_accel[1];  
		accel[2]= Original_accel[2]-Deviation_accel[2] + 16384;
	}

	return res;
}

void invMSAccelRead(int16_t* ps16AccelX, int16_t* ps16AccelY, int16_t* ps16AccelZ)
{
		invmsICM20948AccelRead(ps16AccelX, ps16AccelY, ps16AccelZ);
    return;
}

void invMSGyroRead(int16_t* ps16GyroX, int16_t* ps16GyroY, int16_t* ps16GyroZ)
{
		invmsICM20948GyroRead(ps16GyroX, ps16GyroY, ps16GyroZ);
    return;
}

void invMSMagRead(int16_t* ps16MagnX, int16_t* ps16MagnY, int16_t* ps16MagnZ)
{
		invmsICM20948MagRead(ps16MagnX, ps16MagnY, ps16MagnZ);
    return;
}




#define SAMPLING_FREQ 100.0f // 采样频率
/**************************************
Date: May 31, 2020
Function: 平方根倒数 求四元数用到
***************************************/
float InvSqrt(float number)
{
  volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;
    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );

  return y;
}


/**************************************
Date: May 31, 2020
Function: 四元数解算
***************************************/
volatile float twoKp = -0.0f;     // 2 * proportional gain (Kp)
volatile float twoKi = 0.0f;     // 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;          // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
void Quaternion_Solution(float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // 首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
    recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;      
    // 把四元数换算成方向余弦中的第三行的三个元素
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
    //误差是估计的重力方向和测量的重力方向的交叉乘积之和
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    //计算并应用积分反馈（如果启用）
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / SAMPLING_FREQ);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / SAMPLING_FREQ);
      integralFBz += twoKi * halfez * (1.0f / SAMPLING_FREQ);
      gx += integralFBx;        // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f;       // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }
    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / SAMPLING_FREQ));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / SAMPLING_FREQ));
  gz *= (0.5f * (1.0f / SAMPLING_FREQ));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx); 
  // Normalise quaternion
  recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);		//牛顿迭代 速度更快一些
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

   static float yaw_last = 0.0f;
   static volatile float last_q0 = 1.0f, last_q1 = 0.0f, last_q2 = 0.0f, last_q3 = 0.0f; 
   angle.pitch = asin(-2.0f * (q0 * q2 - q1 * q3)) * (180.0f / PI);
   angle.roll = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * (180.0f / PI);
   angle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * (180.0f / PI);
   
   
   
   if(fabs(yaw_last - angle.yaw) < 0.0006f)		//过滤噪声		防止静止时因噪声积分导致yaw角度偏移
   {
	q0 = last_q0;   q1 = last_q1; q2 = last_q2; q3 = last_q3; 	//将上一次的数据 给到本次
   }
   
   last_q0 = q0;last_q1 = q1;last_q2 = q2;last_q3 = q3;		//将本次数据保存 留给下一次用 
   
   yaw_last = angle.yaw;	//将本次数据保存 留给下一次用
}

int8_t mpu6050_rest(void)	//复位陀螺仪
{
	//静止状态下 允许复位
	if(gyro[0] < 50 && gyro[0] > -50 &&
	   gyro[1] < 50 && gyro[1] > -50 &&
	   gyro[2] < 50 && gyro[2] > -50 
		)
	{
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_PWR_MIGMT_1,  REG_VAL_ALL_RGE_RESET);		//陀螺仪复位指令
		delay_ms(20);													//延时20ms  
		memcpy(Deviation_gyro,Original_gyro,sizeof(gyro));							
		memcpy(Deviation_accel,Original_accel,sizeof(accel));
		q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; 								//复位四元素 回到0
		return 1;		//复位成功
	}
	else
		return 0;		//复位失败
}

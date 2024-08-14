#include "MPU6050.h"
#include "I2C.h"
#include "usart.h"
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (200)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f

short gyro[3], accel[3], sensors;
//零点漂移计数
int Deviation_Count;
// Gyro static error, raw data
//陀螺仪静差，原始数据
short Deviation_gyro[3],Original_gyro[3];  
short Deviation_accel[3],Original_accel[3]; 
volatile float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

//陀螺仪角度数据结构定义
mpu6050 angle;
//Relative to the range set by the IMU gyroscope, the range is ±500°, corresponding data range is ±32768
//The gyroscope raw data is converted in radian (rad) units, 1/65.5/57.30=0.00026644
//与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位，1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO   0.00026644f
//Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is ±32768
//Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=1671.84	
//与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACCEl_RATIO 	  16384.0f  


void MPU6050_Task(void)
{
		
			//此任务以100Hz的频率运行
			//开机前，读取陀螺仪零点			
		  if(Deviation_Count<CONTROL_DELAY)
		  {	 
		  	Deviation_Count++;
			  //不断将漂移值给到数组 这样得到的就是去除漂移后的值 经测试基本10s就够了 进程是10ms  所以delay给1000即可
			  memcpy(Deviation_gyro,gyro,sizeof(gyro));		
				memcpy(Deviation_accel,accel,sizeof(accel));
		  }		
		else
		{
			  //四元素解算
			Quaternion_Solution(gyro[0]*GYROSCOPE_RATIO,gyro[1]*GYROSCOPE_RATIO,gyro[2]*GYROSCOPE_RATIO,accel[0]/ACCEl_RATIO,accel[1]/ACCEl_RATIO,accel[2]/ACCEl_RATIO);
		}
		
		 MPU_Get_Gyroscope(); //得到陀螺仪数据
		 MPU_Get_Accelscope(); //获得加速度计值(原始值)
}  

uint8_t buffer[14];

int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;



/**************************************************************************
Function: The new ADC data is updated to FIFO array for filtering
Input   : ax，ay，az：x，y, z-axis acceleration data；gx，gy，gz：x. Y, z-axis angular acceleration data
Output  : none
函数功能：将新的ADC数据更新到 FIFO数组，进行滤波处理
入口参数：ax，ay，az：x，y，z轴加速度数据；gx，gy，gz：x，y，z轴角加速度数据
返回  值：无
**************************************************************************/
void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
	unsigned char i ;
	int32_t sum=0;
	for(i=1;i<10;i++){	//FIFO 操作
	MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
	MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
	MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
	MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
	MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
	MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[0][9]=ax;//将新的数据放置到 数据的最后面
	MPU6050_FIFO[1][9]=ay;
	MPU6050_FIFO[2][9]=az;
	MPU6050_FIFO[3][9]=gx;
	MPU6050_FIFO[4][9]=gy;
	MPU6050_FIFO[5][9]=gz;

	sum=0;
	for(i=0;i<10;i++){	//求当前数组的合，再取平均值
	   sum+=MPU6050_FIFO[0][i];
	}
	MPU6050_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
	   sum+=MPU6050_FIFO[1][i];
	}
	MPU6050_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
	   sum+=MPU6050_FIFO[2][i];
	}
	MPU6050_FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
	   sum+=MPU6050_FIFO[3][i];
	}
	MPU6050_FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
	   sum+=MPU6050_FIFO[4][i];
	}
	MPU6050_FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
	   sum+=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[5][10]=sum/10;
}

/**************************************************************************
Function: Setting the clock source of mpu6050
Input   : source：Clock source number
Output  : none
函数功能：设置  MPU6050 的时钟源
入口参数：source：时钟源编号
返回  值：无
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
**************************************************************************/
void MPU6050_setClockSource(uint8_t source){
    I2C_WriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    I2C_WriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************************************************************
Function: Setting the maximum range of mpu6050 accelerometer
Input   : range：Acceleration maximum range number
Output  : none
函数功能：设置 MPU6050 加速度计的最大量程
入口参数：range：加速度最大量程编号
返回  值：无
**************************************************************************/
//#define MPU6050_ACCEL_FS_2          0x00  		//===最大量程+-2G
//#define MPU6050_ACCEL_FS_4          0x01			//===最大量程+-4G
//#define MPU6050_ACCEL_FS_8          0x02			//===最大量程+-8G
//#define MPU6050_ACCEL_FS_16         0x03			//===最大量程+-16G
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    I2C_WriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************************************************************
Function: Set mpu6050 to sleep mode or not
Input   : enable：1，sleep；0，work；
Output  : none
函数功能：设置 MPU6050 是否进入睡眠模式
入口参数：enable：1，睡觉；0，工作；
返回  值：无
**************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    I2C_WriteOneBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************************************************************
Function: Read identity
Input   : none
Output  : 0x68
函数功能：读取  MPU6050 WHO_AM_I 标识
入口参数：无
返回  值：0x68
**************************************************************************/
uint8_t MPU6050_getDeviceID(void) {

	
	return I2C_ReadOneByte(devAddr,MPU6050_RA_WHO_AM_I);
	
//    IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
//    return buffer[0];
}

/**************************************************************************
Function: Check whether mpu6050 is connected
Input   : none
Output  : 1：Connected；0：Not connected
函数功能：检测MPU6050 是否已经连接
入口参数：无
返回  值：1：已连接；0：未连接
**************************************************************************/
uint8_t MPU6050_testConnection(void) {
   if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
   return 1;
   	else return 0;
}

/**************************************************************************
Function: Setting whether mpu6050 is the host of aux I2C cable
Input   : enable：1，yes；0;not
Output  : none
函数功能：设置 MPU6050 是否为AUX I2C线的主机
入口参数：enable：1，是；0：否
返回  值：无
**************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    I2C_WriteOneBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************************************************************
Function: Setting whether mpu6050 is the host of aux I2C cable
Input   : enable：1，yes；0;not
Output  : none
函数功能：设置 MPU6050 是否为AUX I2C线的主机
入口参数：enable：1，是；0：否
返回  值：无
**************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    I2C_WriteOneBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************************************************************
Function: initialization Mpu6050 to enter the available state
Input   : none
Output  : none
函数功能：初始化	MPU6050 以进入可用状态
入口参数：无
返回  值：无
**************************************************************************/
uint8_t MPU6050_initialize(void) 
	{
		uint8_t res;
	//IIC_Init();  //Initialize the IIC bus //初始化IIC总线
	I2C_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X80);	//Reset MPUrobot_select_init.h //复位MPUrobot_select_init.h
	HAL_Delay(200); //Delay 200 ms //延时200ms
	I2C_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X00);	//Wake mpurobot_select_init.h //唤醒MPUrobot_select_init.h
	
  //MPU6050_Set_Gyro_Fsr(1);  //Gyroscope sensor              //陀螺仪传感器,±500dps=±500°/s ±32768 (gyro/32768*500)*PI/180(rad/s)=gyro/3754.9(rad/s)
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_500);
	//MPU6050_Set_Accel_Fsr(0);	//Acceleration sensor           //加速度传感器,±2g=±2*9.8m/s^2 ±32768 accel/32768*19.6=accel/1671.84
  MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	MPU6050_Set_Rate(50);			//Set the sampling rate to 50Hz //设置采样率50Hz
	
	I2C_WriteOneByte(devAddr,MPU6050_RA_INT_ENABLE,0X00);	  //Turn off all interrupts //关闭所有中断
	I2C_WriteOneByte(devAddr,MPU6050_RA_USER_CTRL,0X00);	//The I2C main mode is off //I2C主模式关闭
	I2C_WriteOneByte(devAddr,MPU6050_RA_FIFO_EN,0X00);	  //Close the FIFO //关闭FIFO
	//The INT pin is low, enabling bypass mode to read the magnetometer directly
	//INT引脚低电平有效，开启bypass模式，可以直接读取磁力计
	I2C_WriteOneByte(devAddr,MPU6050_RA_INT_PIN_CFG,0X80);
	//Read the ID of MPU6050 
	//读取MPU6050的ID	
	res=I2C_ReadOneByte(devAddr,MPU6050_RA_WHO_AM_I);
	if(res==MPU6050_DEFAULT_ADDRESS) //The device ID is correct, The correct device ID depends on the AD pin //器件ID正确, 器件ID的正确取决于AD引脚
	{
		I2C_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X01);	//Set CLKSEL,PLL X axis as reference //设置CLKSEL,PLL X轴为参考
		I2C_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_2,0X00);	//Acceleration and gyroscope both work //加速度与陀螺仪都工作
		MPU6050_Set_Rate(50);	                      //Set the sampling rate to 50Hz //设置采样率为50Hz   
 	}else return 1;
	return 0;

}

/**************************************************************************
Function: Read mpu6050 built-in temperature sensor data
Input   : none
Output  : Centigrade temperature
函数功能：读取MPU6050内置温度传感器数据
入口参数：无
返回  值：摄氏温度
**************************************************************************/
int Read_Temperature(void)
{	   
	  float Temp;
	  Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
		if(Temp>32768) Temp-=65536;	//数据类型转换
		Temp=(36.53f+Temp/340)*10;	  //温度放大十倍存放
	  return (int)Temp;
}

/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : LPF: Digital low-pass filtering frequency (Hz)
Output  : 0: Settings successful, others: Settings failed
函数功能：设置MPUrobot_select_init.h的数字低通滤波器
入口参数：lpf:数字低通滤波频率(Hz)
返回  值：0:设置成功, 其他:设置失败
**************************************************************************/
unsigned char MPU6050_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return I2C_WriteOneByte(devAddr,MPU6050_RA_CONFIG,data); //Set the digital lowpass filter//设置数字低通滤波器  
}
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : rate:4~1000(Hz)
Output  : 0: Settings successful, others: Settings failed
函数功能：设置MPUrobot_select_init.h的采样率(假定Fs=1KHz)
入口参数：rate:4~1000(Hz)
返回  值：0:设置成功, 其他:设置失败
**************************************************************************/
unsigned char MPU6050_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=I2C_WriteOneByte(devAddr,MPU6050_RA_SMPLRT_DIV,data);	//Set the digital lowpass filter//设置数字低通滤波器  
 	return MPU6050_Set_LPF(rate/2);	//Automatically sets LPF to half of the sampling rate //自动设置LPF为采样率的一半
}

/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Gx, Gy, Gz: raw readings (plus or minus) of the x,y, and z axes of the gyroscope
Output  : 0: success, others: error code
函数功能：获得陀螺仪值(原始值)
**************************************************************************/
void MPU_Get_Gyroscope(void)
{
		gyro[0]=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
		gyro[1]=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
		gyro[2]=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
	
	if(Deviation_Count<CONTROL_DELAY) // 10 seconds before starting //开机前10秒
		{

//			Led_Count=1; //LED high frequency flashing //LED高频闪烁
//			Flag_Stop=1; //The software fails to flag location 1 //软件失能标志位置1		
		}
	else //10 seconds after starting //开机10秒后
		{  
			if(Deviation_Count==CONTROL_DELAY)
//				Flag_Stop=0; //The software fails to flag location 0 //软件失能标志位置0
//			Led_Count=300; //The LED returns to normal flicker frequency //LED恢复正常闪烁频率	
			
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
	 	
}
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Gx, Gy, Gz: raw readings (plus or minus) of the x,y, and z axes of the gyroscope
Output  : 0: success, others: error code
函数功能：获得加速度计值(原始值)
**************************************************************************/
void MPU_Get_Accelscope(void)
{
		accel[0]=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
		accel[1]=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
		accel[2]=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
	
		if(Deviation_Count<CONTROL_DELAY) // 10 seconds before starting //开机前10秒
		{
	
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
			accel[2]= Original_accel[2]-Deviation_accel[2]+16384;
		}
}



//角度不准的时候 可以修改下面的值 比如陀螺仪在世界坐标系下转90度，但是yaw数值显示不对的话 可以改这个采样频率 
#define SAMPLING_FREQ 49.0f // 采样频率
/**************************************
Date: May 31, 2020
Function: 平方根倒数 求四元数用到 （牛顿迭代法）
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
volatile float twoKp = 0.0f;     // 2 * proportional gain (Kp)
volatile float twoKi = 0.0f;     // 2 * integral gain (Ki)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;          // quaternion of sensor frame relative to auxiliary frame
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

  
	/*欧拉角转换 滤除噪音*/
   static float yaw_last = 0.0f;		//上一时刻角度值
   static volatile float last_q0 = 1.0f, last_q1 = 0.0f, last_q2 = 0.0f, last_q3 = 0.0f; 	//上一时刻四元数
   
   angle.pitch = asin(-2.0f * (q0 * q2 - q1 * q3)) * (180.0f / PI);
   angle.roll = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * (180.0f / PI);
   angle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * (180.0f / PI);
   
   
   
   if(fabs(yaw_last - angle.yaw) < 0.0008f)		//过滤噪声		防止静止时因噪声积分导致yaw角度偏移
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
		q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; 								//复位四元素 回到0
		return 1;		//复位成功
	}
	else
		return 0;		//复位失败
}


////------------------End of File----------------------------

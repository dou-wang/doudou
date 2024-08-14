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
//���Ư�Ƽ���
int Deviation_Count;
// Gyro static error, raw data
//�����Ǿ��ԭʼ����
short Deviation_gyro[3],Original_gyro[3];  
short Deviation_accel[3],Original_accel[3]; 
volatile float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

//�����ǽǶ����ݽṹ����
mpu6050 angle;
//Relative to the range set by the IMU gyroscope, the range is ��500��, corresponding data range is ��32768
//The gyroscope raw data is converted in radian (rad) units, 1/65.5/57.30=0.00026644
//��IMU���������õ������йأ����̡�500�㣬��Ӧ���ݷ�Χ��32768
//������ԭʼ����ת��λ����(rad)��λ��1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO   0.00026644f
//Relates to the range set by the IMU accelerometer, range is ��2g, corresponding data range is ��32768
//Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=1671.84	
//��IMU���ٶȼ����õ������йأ����̡�2g����Ӧ���ݷ�Χ��32768
//���ٶȼ�ԭʼ����ת��λm/s^2��λ��32768/2g=32768/19.6=1671.84
#define ACCEl_RATIO 	  16384.0f  


void MPU6050_Task(void)
{
		
			//��������100Hz��Ƶ������
			//����ǰ����ȡ���������			
		  if(Deviation_Count<CONTROL_DELAY)
		  {	 
		  	Deviation_Count++;
			  //���Ͻ�Ư��ֵ�������� �����õ��ľ���ȥ��Ư�ƺ��ֵ �����Ի���10s�͹��� ������10ms  ����delay��1000����
			  memcpy(Deviation_gyro,gyro,sizeof(gyro));		
				memcpy(Deviation_accel,accel,sizeof(accel));
		  }		
		else
		{
			  //��Ԫ�ؽ���
			Quaternion_Solution(gyro[0]*GYROSCOPE_RATIO,gyro[1]*GYROSCOPE_RATIO,gyro[2]*GYROSCOPE_RATIO,accel[0]/ACCEl_RATIO,accel[1]/ACCEl_RATIO,accel[2]/ACCEl_RATIO);
		}
		
		 MPU_Get_Gyroscope(); //�õ�����������
		 MPU_Get_Accelscope(); //��ü��ٶȼ�ֵ(ԭʼֵ)
}  

uint8_t buffer[14];

int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;



/**************************************************************************
Function: The new ADC data is updated to FIFO array for filtering
Input   : ax��ay��az��x��y, z-axis acceleration data��gx��gy��gz��x. Y, z-axis angular acceleration data
Output  : none
�������ܣ����µ�ADC���ݸ��µ� FIFO���飬�����˲�����
��ڲ�����ax��ay��az��x��y��z����ٶ����ݣ�gx��gy��gz��x��y��z��Ǽ��ٶ�����
����  ֵ����
**************************************************************************/
void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
	unsigned char i ;
	int32_t sum=0;
	for(i=1;i<10;i++){	//FIFO ����
	MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
	MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
	MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
	MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
	MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
	MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[0][9]=ax;//���µ����ݷ��õ� ���ݵ������
	MPU6050_FIFO[1][9]=ay;
	MPU6050_FIFO[2][9]=az;
	MPU6050_FIFO[3][9]=gx;
	MPU6050_FIFO[4][9]=gy;
	MPU6050_FIFO[5][9]=gz;

	sum=0;
	for(i=0;i<10;i++){	//��ǰ����ĺϣ���ȡƽ��ֵ
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
Input   : source��Clock source number
Output  : none
�������ܣ�����  MPU6050 ��ʱ��Դ
��ڲ�����source��ʱ��Դ���
����  ֵ����
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
Input   : range��Acceleration maximum range number
Output  : none
�������ܣ����� MPU6050 ���ٶȼƵ��������
��ڲ�����range�����ٶ�������̱��
����  ֵ����
**************************************************************************/
//#define MPU6050_ACCEL_FS_2          0x00  		//===�������+-2G
//#define MPU6050_ACCEL_FS_4          0x01			//===�������+-4G
//#define MPU6050_ACCEL_FS_8          0x02			//===�������+-8G
//#define MPU6050_ACCEL_FS_16         0x03			//===�������+-16G
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    I2C_WriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************************************************************
Function: Set mpu6050 to sleep mode or not
Input   : enable��1��sleep��0��work��
Output  : none
�������ܣ����� MPU6050 �Ƿ����˯��ģʽ
��ڲ�����enable��1��˯����0��������
����  ֵ����
**************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    I2C_WriteOneBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************************************************************
Function: Read identity
Input   : none
Output  : 0x68
�������ܣ���ȡ  MPU6050 WHO_AM_I ��ʶ
��ڲ�������
����  ֵ��0x68
**************************************************************************/
uint8_t MPU6050_getDeviceID(void) {

	
	return I2C_ReadOneByte(devAddr,MPU6050_RA_WHO_AM_I);
	
//    IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
//    return buffer[0];
}

/**************************************************************************
Function: Check whether mpu6050 is connected
Input   : none
Output  : 1��Connected��0��Not connected
�������ܣ����MPU6050 �Ƿ��Ѿ�����
��ڲ�������
����  ֵ��1�������ӣ�0��δ����
**************************************************************************/
uint8_t MPU6050_testConnection(void) {
   if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
   return 1;
   	else return 0;
}

/**************************************************************************
Function: Setting whether mpu6050 is the host of aux I2C cable
Input   : enable��1��yes��0;not
Output  : none
�������ܣ����� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
��ڲ�����enable��1���ǣ�0����
����  ֵ����
**************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    I2C_WriteOneBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************************************************************
Function: Setting whether mpu6050 is the host of aux I2C cable
Input   : enable��1��yes��0;not
Output  : none
�������ܣ����� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
��ڲ�����enable��1���ǣ�0����
����  ֵ����
**************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    I2C_WriteOneBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************************************************************
Function: initialization Mpu6050 to enter the available state
Input   : none
Output  : none
�������ܣ���ʼ��	MPU6050 �Խ������״̬
��ڲ�������
����  ֵ����
**************************************************************************/
uint8_t MPU6050_initialize(void) 
	{
		uint8_t res;
	//IIC_Init();  //Initialize the IIC bus //��ʼ��IIC����
	I2C_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X80);	//Reset MPUrobot_select_init.h //��λMPUrobot_select_init.h
	HAL_Delay(200); //Delay 200 ms //��ʱ200ms
	I2C_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X00);	//Wake mpurobot_select_init.h //����MPUrobot_select_init.h
	
  //MPU6050_Set_Gyro_Fsr(1);  //Gyroscope sensor              //�����Ǵ�����,��500dps=��500��/s ��32768 (gyro/32768*500)*PI/180(rad/s)=gyro/3754.9(rad/s)
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_500);
	//MPU6050_Set_Accel_Fsr(0);	//Acceleration sensor           //���ٶȴ�����,��2g=��2*9.8m/s^2 ��32768 accel/32768*19.6=accel/1671.84
  MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	MPU6050_Set_Rate(50);			//Set the sampling rate to 50Hz //���ò�����50Hz
	
	I2C_WriteOneByte(devAddr,MPU6050_RA_INT_ENABLE,0X00);	  //Turn off all interrupts //�ر������ж�
	I2C_WriteOneByte(devAddr,MPU6050_RA_USER_CTRL,0X00);	//The I2C main mode is off //I2C��ģʽ�ر�
	I2C_WriteOneByte(devAddr,MPU6050_RA_FIFO_EN,0X00);	  //Close the FIFO //�ر�FIFO
	//The INT pin is low, enabling bypass mode to read the magnetometer directly
	//INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ�Ӷ�ȡ������
	I2C_WriteOneByte(devAddr,MPU6050_RA_INT_PIN_CFG,0X80);
	//Read the ID of MPU6050 
	//��ȡMPU6050��ID	
	res=I2C_ReadOneByte(devAddr,MPU6050_RA_WHO_AM_I);
	if(res==MPU6050_DEFAULT_ADDRESS) //The device ID is correct, The correct device ID depends on the AD pin //����ID��ȷ, ����ID����ȷȡ����AD����
	{
		I2C_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X01);	//Set CLKSEL,PLL X axis as reference //����CLKSEL,PLL X��Ϊ�ο�
		I2C_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_2,0X00);	//Acceleration and gyroscope both work //���ٶ��������Ƕ�����
		MPU6050_Set_Rate(50);	                      //Set the sampling rate to 50Hz //���ò�����Ϊ50Hz   
 	}else return 1;
	return 0;

}

/**************************************************************************
Function: Read mpu6050 built-in temperature sensor data
Input   : none
Output  : Centigrade temperature
�������ܣ���ȡMPU6050�����¶ȴ���������
��ڲ�������
����  ֵ�������¶�
**************************************************************************/
int Read_Temperature(void)
{	   
	  float Temp;
	  Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
		if(Temp>32768) Temp-=65536;	//��������ת��
		Temp=(36.53f+Temp/340)*10;	  //�¶ȷŴ�ʮ�����
	  return (int)Temp;
}

/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : LPF: Digital low-pass filtering frequency (Hz)
Output  : 0: Settings successful, others: Settings failed
�������ܣ�����MPUrobot_select_init.h�����ֵ�ͨ�˲���
��ڲ�����lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
����  ֵ��0:���óɹ�, ����:����ʧ��
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
	return I2C_WriteOneByte(devAddr,MPU6050_RA_CONFIG,data); //Set the digital lowpass filter//�������ֵ�ͨ�˲���  
}
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : rate:4~1000(Hz)
Output  : 0: Settings successful, others: Settings failed
�������ܣ�����MPUrobot_select_init.h�Ĳ�����(�ٶ�Fs=1KHz)
��ڲ�����rate:4~1000(Hz)
����  ֵ��0:���óɹ�, ����:����ʧ��
**************************************************************************/
unsigned char MPU6050_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=I2C_WriteOneByte(devAddr,MPU6050_RA_SMPLRT_DIV,data);	//Set the digital lowpass filter//�������ֵ�ͨ�˲���  
 	return MPU6050_Set_LPF(rate/2);	//Automatically sets LPF to half of the sampling rate //�Զ�����LPFΪ�����ʵ�һ��
}

/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Gx, Gy, Gz: raw readings (plus or minus) of the x,y, and z axes of the gyroscope
Output  : 0: success, others: error code
�������ܣ����������ֵ(ԭʼֵ)
**************************************************************************/
void MPU_Get_Gyroscope(void)
{
		gyro[0]=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //��ȡX��������
		gyro[1]=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //��ȡY��������
		gyro[2]=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������
	
	if(Deviation_Count<CONTROL_DELAY) // 10 seconds before starting //����ǰ10��
		{

//			Led_Count=1; //LED high frequency flashing //LED��Ƶ��˸
//			Flag_Stop=1; //The software fails to flag location 1 //���ʧ�ܱ�־λ��1		
		}
	else //10 seconds after starting //����10���
		{  
			if(Deviation_Count==CONTROL_DELAY)
//				Flag_Stop=0; //The software fails to flag location 0 //���ʧ�ܱ�־λ��0
//			Led_Count=300; //The LED returns to normal flicker frequency //LED�ָ�������˸Ƶ��	
			
			//Save the raw data to update zero by clicking the user button
			//����ԭʼ�������ڵ����û������������
			Original_gyro[0] =gyro[0];  
			Original_gyro[1] =gyro[1];  
			Original_gyro[2]= gyro[2];			
			
			//Removes zero drift data
			//ȥ�����Ư�Ƶ�����
			gyro[0] =Original_gyro[0]-Deviation_gyro[0];  
			gyro[1] =Original_gyro[1]-Deviation_gyro[1];  
			gyro[2]= Original_gyro[2]-Deviation_gyro[2];
		}
	 	
}
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Gx, Gy, Gz: raw readings (plus or minus) of the x,y, and z axes of the gyroscope
Output  : 0: success, others: error code
�������ܣ���ü��ٶȼ�ֵ(ԭʼֵ)
**************************************************************************/
void MPU_Get_Accelscope(void)
{
		accel[0]=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //��ȡX����ٶȼ�
		accel[1]=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //��ȡX����ٶȼ�
		accel[2]=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�
	
		if(Deviation_Count<CONTROL_DELAY) // 10 seconds before starting //����ǰ10��
		{
	
		}
		else //10 seconds after starting //����10���
		{  		
			//Save the raw data to update zero by clicking the user button
			//����ԭʼ�������ڵ����û������������
			Original_accel[0] =accel[0];  
			Original_accel[1] =accel[1];  
			Original_accel[2]= accel[2];			
			
			//Removes zero drift data
			//ȥ�����Ư�Ƶ�����
			accel[0] =Original_accel[0]-Deviation_accel[0];  
			accel[1] =Original_accel[1]-Deviation_accel[1];  
			accel[2]= Original_accel[2]-Deviation_accel[2]+16384;
		}
}



//�ǶȲ�׼��ʱ�� �����޸������ֵ ��������������������ϵ��ת90�ȣ�����yaw��ֵ��ʾ���ԵĻ� ���Ը��������Ƶ�� 
#define SAMPLING_FREQ 49.0f // ����Ƶ��
/**************************************
Date: May 31, 2020
Function: ƽ�������� ����Ԫ���õ� ��ţ�ٵ�������
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
Function: ��Ԫ������
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
    // ���ȰѼ��ٶȼƲɼ�����ֵ(��ά����)ת��Ϊ��λ����������������ģ
    recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;      
    // ����Ԫ������ɷ��������еĵ����е�����Ԫ��
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
    //����ǹ��Ƶ���������Ͳ�������������Ľ���˻�֮��
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    //���㲢Ӧ�û��ַ�����������ã�
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
  recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);		//ţ�ٵ��� �ٶȸ���һЩ
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  
	/*ŷ����ת�� �˳�����*/
   static float yaw_last = 0.0f;		//��һʱ�̽Ƕ�ֵ
   static volatile float last_q0 = 1.0f, last_q1 = 0.0f, last_q2 = 0.0f, last_q3 = 0.0f; 	//��һʱ����Ԫ��
   
   angle.pitch = asin(-2.0f * (q0 * q2 - q1 * q3)) * (180.0f / PI);
   angle.roll = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * (180.0f / PI);
   angle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * (180.0f / PI);
   
   
   
   if(fabs(yaw_last - angle.yaw) < 0.0008f)		//��������		��ֹ��ֹʱ���������ֵ���yaw�Ƕ�ƫ��
   {
	q0 = last_q0;   q1 = last_q1; q2 = last_q2; q3 = last_q3; 	//����һ�ε����� ��������
   }
   
   last_q0 = q0;last_q1 = q1;last_q2 = q2;last_q3 = q3;		//���������ݱ��� ������һ���� 
   
   yaw_last = angle.yaw;	//���������ݱ��� ������һ����
}

int8_t mpu6050_rest(void)	//��λ������
{
	//��ֹ״̬�� ����λ
	if(gyro[0] < 50 && gyro[0] > -50 &&
	   gyro[1] < 50 && gyro[1] > -50 &&
	   gyro[2] < 50 && gyro[2] > -50 
		)
	{
		q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; 								//��λ��Ԫ�� �ص�0
		return 1;		//��λ�ɹ�
	}
	else
		return 0;		//��λʧ��
}


////------------------End of File----------------------------

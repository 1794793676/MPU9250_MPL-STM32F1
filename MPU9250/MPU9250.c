#include "MPU9250.h"

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

static void I2Cx_Error(uint8_t Addr)
{
    /* 恢复I2C寄存器为默认值 */
    HAL_I2C_DeInit(&MPU6050_I2C_Handle);
    /* 重新初始化I2C外设 */
    MX_I2C2_Init();
}

/**
 * @brief  读寄存器，这是提供给上层的接口
 * @param  slave_addr: 从机地址
 * @param 	reg_addr:寄存器地址
 * @param len：要读取的长度
 *	@param data_ptr:指向要存储数据的指针
 * @retval 正常为0，不正常为非0
 */
int Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,unsigned char *data_ptr)
{

    HAL_StatusTypeDef status = HAL_OK;
    slave_addr <<= 1;
    status = HAL_I2C_Mem_Read(&MPU6050_I2C_Handle, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len,
                              I2Cx_FLAG_TIMEOUT);
    /* 检查通讯状态 */
    if (status != HAL_OK)
    {
        /* 总线出错处理 */
        I2Cx_Error(slave_addr);
    }
    while (HAL_I2C_GetState(&MPU6050_I2C_Handle) != HAL_I2C_STATE_READY)
    {
    }
    /* 检查SENSOR是否就绪进行下一次读写操作 */
    while (HAL_I2C_IsDeviceReady(&MPU6050_I2C_Handle, slave_addr, I2Cx_FLAG_TIMEOUT, I2Cx_FLAG_TIMEOUT) == HAL_TIMEOUT)
        ;
    /* 等待传输结束 */
    while (HAL_I2C_GetState(&MPU6050_I2C_Handle) != HAL_I2C_STATE_READY)
    {
    }
    return status;
}

/**
 * @brief  写寄存器，这是提供给上层的接口
 * @param  slave_addr: 从机地址
 * @param 	reg_addr:寄存器地址
 * @param len：写入的长度
 *	@param data_ptr:指向要写入的数据
 * @retval 正常为0，不正常为非0
 */
int Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr,unsigned short len,unsigned char *data_ptr)
{
    HAL_StatusTypeDef status = HAL_OK;
    slave_addr <<= 1;
    status = HAL_I2C_Mem_Write(&MPU6050_I2C_Handle, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len,
                               I2Cx_FLAG_TIMEOUT);
    /* 检查通讯状态 */
    if (status != HAL_OK)
    {
        /* 总线出错处理 */
        I2Cx_Error(slave_addr);
    }
    while (HAL_I2C_GetState(&MPU6050_I2C_Handle) != HAL_I2C_STATE_READY)
    {
    }
    /* 检查SENSOR是否就绪进行下一次读写操作 */
    while (HAL_I2C_IsDeviceReady(&MPU6050_I2C_Handle, slave_addr, I2Cx_FLAG_TIMEOUT, I2Cx_FLAG_TIMEOUT) == HAL_TIMEOUT)
        ;
    /* 等待传输结束 */
    while (HAL_I2C_GetState(&MPU6050_I2C_Handle) != HAL_I2C_STATE_READY)
    {
    }
    return status;
}

inline void get_ms_user(unsigned long *count) //换壳函数  inline修饰（内联）减少出入栈开销
{
    *count = HAL_GetTick();
}

//q30，q16格式,long转float时的除数.
#define q30  1073741824.0f
#define q16  65536.0f

//陀螺仪方向设置
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};
//磁力计方向设置
static signed char comp_orientation[9] = { 0, 1, 0,
                                           1, 0, 0,
                                           0, 0,-1};

//方向转换
unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

//mpu9250初始化
//返回值:0,正常
//    其他,失败
uint8_t mpu_dmp_init(void)
{
    uint8_t res=0;
    struct int_param_s int_param;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    unsigned short compass_fsr;
    
    if(mpu_init(&int_param)==0)	//初始化MPU9250
    {	 
        res=inv_init_mpl();     //初始化MPL
        if(res)return 1;
        inv_enable_quaternion();
        inv_enable_9x_sensor_fusion();
        inv_enable_fast_nomot();
        inv_enable_gyro_tc();
        inv_enable_vector_compass_cal();
        inv_enable_magnetic_disturbance();
        inv_enable_eMPL_outputs();
        res=inv_start_mpl();    //开启MPL
        if(res)return 1;
        res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL|INV_XYZ_COMPASS);//设置所需要的传感器
        if(res)return 2; 
        res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);   //设置FIFO
        if(res)return 3; 
        res=mpu_set_sample_rate(DEFAULT_MPU_HZ);	            //设置采样率
        if(res)return 4; 
        res=mpu_set_compass_sample_rate(1000/COMPASS_READ_MS);  //设置磁力计采样率
        if(res)return 5;
        mpu_get_sample_rate(&gyro_rate);
        mpu_get_gyro_fsr(&gyro_fsr);
        mpu_get_accel_fsr(&accel_fsr);
        mpu_get_compass_fsr(&compass_fsr);
        inv_set_gyro_sample_rate(1000000L/gyro_rate);
        inv_set_accel_sample_rate(1000000L/gyro_rate);
        inv_set_compass_sample_rate(COMPASS_READ_MS*1000L);
        inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_orientation),(long)gyro_fsr<<15);
        inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_orientation),(long)accel_fsr<<15);
        inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(comp_orientation),(long)compass_fsr<<15);
        res=dmp_load_motion_driver_firmware();		             //加载dmp固件
        if(res)return 6; 		
        res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//设置陀螺仪方向
        if(res)return 7; 
        res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	            //设置dmp功能
            DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
            DMP_FEATURE_GYRO_CAL);
        if(res)return 8; 
        
        res=dmp_set_fifo_rate(DEFAULT_MPU_HZ);	//设置DMP输出速率(最大不超过200Hz)
        if(res)return 9;   
        //res=run_self_test();		//自检
//		if(res)return 10;    
        res=mpu_set_dmp_state(1);	//使能DMP
        if(res)return 11;     
    }
    return 0;
}

//得到dmp处理后的数据(注意,本函数需要比较多堆栈,局部变量有点多)
//pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
//roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
//yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
//返回值:0,正常
//    其他,失败
uint8_t mpu_dmp_get_data(float *pitch,float *roll,float *yaw)
{
    float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4]; 
    if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more))return 1;	 
    /* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
     * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
    **/
    /*if (sensors & INV_XYZ_GYRO )
    send_packet(PACKET_TYPE_GYRO, gyro);
    if (sensors & INV_XYZ_ACCEL)
    send_packet(PACKET_TYPE_ACCEL, accel); */
    /* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
     * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
    **/
    if(sensors&INV_WXYZ_QUAT) 
    {
        q0 = quat[0] / q30;	//q30格式转换为浮点数
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30; 
        //计算得到俯仰角/横滚角/航向角
        *pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
        *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
        *yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
    }else return 2;
    return 0;
}

//得到mpl处理后的数据(注意,本函数需要比较多堆栈,局部变量有点多)
//pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
//roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
//yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
//返回值:0,正常
//    其他,失败
uint8_t mpu_mpl_get_data(float *pitch,float *roll,float *yaw)
{
    unsigned long sensor_timestamp,timestamp;
    short gyro[3], accel_short[3],compass_short[3],sensors;
    unsigned char more;
    long compass[3],accel[3],quat[4],temperature; 
    long data[9];
    int8_t accuracy;
    
//	if(dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors,&more)){
//		printf("%d\r\n",dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors,&more));
//		return 1;	 
//	
//	}
      while(dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors,&more)){};
    if(sensors&INV_XYZ_GYRO)
    {
        inv_build_gyro(gyro,sensor_timestamp);          //把新数据发送给MPL
        mpu_get_temperature(&temperature,&sensor_timestamp);
        inv_build_temp(temperature,sensor_timestamp);   //把温度值发给MPL，只有陀螺仪需要温度值
    }
    
    if(sensors&INV_XYZ_ACCEL)
    {
        accel[0] = (long)accel_short[0];
        accel[1] = (long)accel_short[1];
        accel[2] = (long)accel_short[2];
        inv_build_accel(accel,0,sensor_timestamp);      //把加速度值发给MPL
    }
    
    if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) 
    {
        compass[0]=(long)compass_short[0];
        compass[1]=(long)compass_short[1];
        compass[2]=(long)compass_short[2];
        inv_build_compass(compass,0,sensor_timestamp); //把磁力计值发给MPL
    }
    inv_execute_on_data();
    inv_get_sensor_type_euler(data,&accuracy,&timestamp);
    
    *roll  = (data[0]/q16);
    *pitch = -(data[1]/q16);
    *yaw   = -data[2] / q16;
    return 0;
}

HAL_StatusTypeDef MPU_Write_Byte(uint16_t DevA, uint16_t MemA,uint8_t Data)
{
	uint8_t temp = Data;
	return HAL_I2C_Mem_Write(&hi2c2, (DevA<<1), MemA, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);
}

uint8_t MPU_Read_Byte(uint16_t DevA, uint16_t MemA)
{
	uint8_t temp;
	HAL_I2C_Mem_Read(&hi2c2, (DevA<<1), MemA, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);
	return temp;
}

uint8_t MPU_Read_Len(uint16_t DevA, uint16_t MemA, uint8_t len, uint8_t *buf)
{
	if(HAL_I2C_Mem_Read(&hi2c2, (DevA<<1), MemA, I2C_MEMADD_SIZE_8BIT, buf, len, 100) != HAL_OK)	return 1;
	else	return 0;
}

//初始化MPU9250
//返回值:0,成功
//    其他,错误代码
uint8_t MPU9250_Init(void)
{
	uint8_t res=0;
	MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//复位MPU9250
	HAL_Delay(100);  //延时100ms
	MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//唤醒MPU9250
	MPU_Set_Gyro_Fsr(3);					        	//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					       	 	//加速度传感器,±2g
	MPU_Set_Rate(50);						       	 	//设置采样率50Hz
	MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //关闭所有中断
	MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C主模式关闭
	MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT引脚低电平有效，开启bypass模式，可以直接读取磁力计
	res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //读取MPU6500的ID
	
	
	
    if(res==MPU6500_ID1 || res==MPU6500_ID2) //器件ID正确
    {
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作
		    MPU_Set_Rate(50);						       	//设置采样率为50Hz   
    }else return 1;
 
    res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA);    			//读取AK8963 ID   

	
    if(res==AK8963_ID)
    {
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		//设置AK8963为单次测量模式
    }else return 1;

    return 0;
}

//设置MPU9250陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU9250加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}

//设置MPU9250的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器  
}

//设置MPU9250的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    uint8_t buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((uint16_t)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res; 
	res=MPU_Read_Len(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];  
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

//得到磁力计值(原始值)
//mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Magnetometer(short *mx,short *my,short *mz)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(AK8963_ADDR,MAG_XOUT_L,6,buf);
	if(res==0)
	{
		*mx=((uint16_t)buf[1]<<8)|buf[0];  
		*my=((uint16_t)buf[3]<<8)|buf[2];  
		*mz=((uint16_t)buf[5]<<8)|buf[4];
	} 	
    MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963每次读完以后都需要重新设置为单次测量模式
    return res;;
}


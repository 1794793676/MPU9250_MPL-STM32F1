

# STM32F103+MPU9250 InvenSense Motion Driver 6.12官方库移植进行MPL解算

最近学习无人机的时候需要姿态解算，本项目为使用MPU9250的MPL官方库解算，移植在STM32F103C8T6的主控上，通过KeilAC5编译HAL库开发。

**注：STM32F1系列使用Cortex-M3内核，并不支持DSP指令集，只能使用轻量级MPL（mpllite）。如需使用MPU9250 做高精度姿态解算，则需要搭配具备DSP指令集的MCU，例如STM32F4系列（Cortex-M4内核）。**

**支持 DSP 指令集的 ARM 内核**

- **Cortex-M4/F4**：原生支持 ARMv7E-M DSP 指令集，带硬件除法器和 FPU（可选）。
- **Cortex-M7/F7**：增强型 DSP 指令集，性能更高，支持更复杂的信号处理。
- **Cortex-M33**：可选 DSP 扩展，兼顾安全性和信号处理能力。

## 项目目录

```
MPU9250MPL
|--Application
	|--MDK-ARM
		|--starup_stm32f103xb.s
		|--MPU9250MPL.uvprojx
		|--...
	|--User
		|--Core
			|--main.c
			|--dma.c
			|--...
|--Driver
|--MPU9250
    |--inv_mpu_dmp_motion_driver.c
    |--inv_mpu.c
    |--MPU9250.c
    |--libmlilib.lib
    |--...
```

STM32CubeMX
USART1_TX->PA9	USART1_RX->PA10 	波特率：9600
I2C2_SCL->PB10	I2C2_SDA->PB11	Fast_mode

## 官方库移植（参考官方移植指南）

## 注意事项：

> **闪存和RAM大小**：闪存和RAM大小取决于代码优化，编译器，您要使用的功能以及系统中的其他组件。一般而言，MD6.12要求您需要保留以下数量的Flash和RAM。请记住，这仅适用于运动驱动程序，而不适用于其他可能的功能。

|           | FLASH | RAM  |
| --------- | ----- | ---- |
| 16BIT MCU | 128K  | 12K  |
| 32BIT MCU | 68K   | 10K  |

> **长数学支持**：MPL库需要支持长（64位）数学。您需要确保是否使用MPL库，您的工具链可以支持此功能。通常8051 MCU不支持这种数学计算。如果工具链不支持长时间的数学运算，您仍然可以使用DMP来获得6轴融合。
>
> **中断**：MPU设备可以为低功耗手势识别或数据就绪中断提供各种功能的中断。虽然系统不需要使用MPU中断，但如果您打算使用它，则必须保留具有唤醒功能的GPIO引脚。
>
> **采样率**：传感器融合需要MCU的大量计算能力。这会影响每个样本的处理量并限制采样率。例如，如果**MCU正在进行完整的9轴融合**，则带有运动驱动器的**TI 16位MSP430应限制为100Hz采样率**。任何超过100Hz采样率的MSP430都会开始丢失数据。**如果系统中不需要其他大型计算功能，则高端32位MCU通常可以实现200Hz传感器融合。**如果将处理卸载到DMP上，则可以提高此采样率。

**STM32F103C8T6共有64KB ROM和20KB RAM，在本项目中实际使用了52.2KB ROM和7KB RAM（O2优化），采用100HZ采样率**。

## 移植：

##### 	驱动层

> 用户需要提供以下API以支持I2C读/写功能，系统时钟访问，硬件中断回调以及与要移植MD6.12的平台相对应的日志记录。
>
> **这些函数需要在inv_mpu.c和inv_mpu_dmp_motion_driver.c中定义。**
>
> ```
> inv_mpl.c中API定义
> #if defined EMPL_TARGET_STM32F1
> #include "i2c.h"   
> #include "main.h"
> #include "log.h"
> // #include "board-st_discovery.h"
> #include "MPU9250.h"
>    
> #define i2c_write   Sensors_I2C_WriteRegister
> #define i2c_read    Sensors_I2C_ReadRegister 
> #define delay_ms    HAL_Delay
> #define get_ms      get_ms_user
> #define log_i       MPL_LOGI
> #define log_e       MPL_LOGE
> #define min(a,b) ((a<b)?a:b)
> ```

> i2c_write和i2c_read：这需要链接到i2c驱动程序。此函数将接受4个参数，然后执行i2c事务
> |--i2c_write(unsigned char slave_addr, unsigned char reg_addr,unsigned short len,unsigned char *data_ptr);
> |--i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned short len，unsigned char *data_ptr);
>
> delay_ms：此函数将接受一个无符号长参数，它将作为系统的延迟（以毫秒为单位）
> get_ms：get_ms主要用于获取当前时间戳。时间戳通常是无符号长整数，以毫秒为单位。该功能主要用于罗盘调度器以及传感器融合数据的附加信息。
> log_i和log_e：MPL消息传递系统，可以记录信息或错误消息。当前实现对消息进行分组并通过USB或UART将其发送出去供python客户端接收。日志记录代码位于log_msp430.c或log_stm32l.c文件中。客户可以根据自己的喜好更改传输方法和数据包。

​	提供了上述必须的API后，由于官方库中的代码是基于STM32F4系列实现，我们需要将其替换为STM32F1，并添加宏定义。完整宏定义如下：

```
USE_HAL_DRIVER,STM32F103xB,STM32F10X_MD,EMPL_TARGET_STM32F1,MPU9250,REMOVE_LOGGING,MPL_LOG_NDEBUG=0,USE_DMP,EMPL
```



##### 	通信层

```
MPU9250.c中硬件IIC实现
/**
 * @brief  读寄存器，这是提供给上层的接口
 * @param  slave_addr: 从机地址
 * @param 	reg_addr:寄存器地址
 * @param len：要读取的长度
 *	@param data_ptr:指向要存储数据的指针
 * @retval 正常为0，不正常为非0
 */
int Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
                             unsigned char *data_ptr)
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
int Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
                              unsigned char *data_ptr)
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

static void I2Cx_Error(uint8_t Addr)
{
    /* 恢复I2C寄存器为默认值 */
    HAL_I2C_DeInit(&MPU6050_I2C_Handle);
    /* 重新初始化I2C外设 */
    MX_I2C2_Init();
}
```

##### 对官方库进行下列适配修改：

```
inv_mpu.c	inv_mpu_dmp_motion_driver.c
EMPL_TARGET_STM32F4 -> EMPL_TARGET_STM32F1
#include "board-st_discovery.h" -> #include "MPU9250.h"	修改为我们自己的API接口
__no_operation() -> __NOP()

log_stm32.c
#include "stm32f4xx.h" -> #include "stm32f1xx.h"
#include "uart.h" -> // #include <stdio.h>
将三个包含报错的fputc函数按如下实例修改。
int _MLPrintLog (int priority, const char* tag, const char* fmt, ...)
{
...
FILE *NoUse;
...
fputc(out[i], NoUse);
...
}
```

至此，官方库移植完成。

## MPU9250初始化与获取欧拉角（参考正点原子）

```
MPU9250.c  MPU9250初始化
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
```

注：

1.MPU9250的磁力计数据并非从FIFO寄存器读取，此处需设置MPU9250旁路模式（bypass）直接读取AK8963磁力计数据，再进行MPL计算

> 4.17 FIFO
> MPU-9250 包含一个 512 字节的 FIFO 寄存器，可通过串行接口访问。FIFO 配置寄存器决定哪些数据会写入 FIFO。可选的数据包括陀螺仪数据、加速度计数据、温度读数、辅助传感器读数以及 FSYNC 输入。FIFO 计数器会跟踪 FIFO 中包含的有效数据字节数。FIFO 寄存器支持突发读取。中断功能可用于判断何时有新数据可用				——MPU9250 datasheet 

2.DMP传感器融合仅适用于+ -2000dps和Accel + -2G的陀螺仪

> （DMP sensor fusion works only with gyro at +-2000dps and accel +-2G）

3.DMP采样率最高200HZ



```
MPU9250.c	获取欧拉角
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
```



## 本项目存在的问题

1.设备上电的一分钟内，数据多次跳变

2.数据延迟较大

可能的原因：

IIC读取赶不上FIFO的数据，主控频率太低。数据融合时时间戳不同步等。



解决方式：

上电20s内执行磁力计等校准；判断磁力计数据是否合理，不合理采用六轴融合数据计算欧拉角等。

## 写在最后

本人因为技术有限，有许多问题仍未解决。在使用MPL库解算之前，曾尝试通过卡尔曼滤波和Madgwick滤波进行数据融合获取欧拉角，Madgwick的运算较卡尔曼消耗更少资源。但最终发现官方MPL库数据比自己实现的Madgwick和卡尔曼滤波稳定很多。在实现Madgwick的过程中总是会遇到数据受磁场影响太大的问题，哪怕减少beta系数任然无法解决。且磁场校准与温度的影响比想象中的困难和严重。如果有解决的大佬愿意分享交流，十分欢迎。

# Tuya IoTOS Embedded Mcu Demo Wifi Ble STM32_LCD_WB3S

[English](./README.md) | [中文](./README_zh.md)

## 简介 

本Demo通过涂鸦智能云平台、涂鸦智能APP、BH1750，SHT30，迪文串口屏，IoTOS Embeded MCU SDK实现一款温湿度和光照度数据采集，同时可以通过APP和控制屏控制GPIO口的电平翻转，从而控制相应的负载设备。

已实现功能包括：

+ 光照度和温湿度采集  
+ 控制GPIO电平翻转



## 快速上手 

### 编译与烧录
+ 下载Tuya IoTOS嵌入式代码

+ 执行Project.uvprojx文件

+ 点击软件中的编译，并完成下载


### 文件介绍 

```
├── Core
   ├── Src
       │   ├── main.c
       │   ├── gpio.c
       │   ├── i2c.c
       │   ├── usart.c
       │   ├── stm32l4xx_it.c
       │   ├── stm32l4xx_hal_msp.c
       │   ├── connect_wifi.c
       │   ├── delay.c
       │   ├── lcd.c
       │   ├── BH1750.c
       │   ├── sht3x.c
       │   ├── time.c     
    ├── Inc
       │   ├── main.h
       │   ├── gpio.h
       │   ├── i2c.h
       │   ├── usart.h
       │   ├── stm32l4xx_it.h
       │   ├── stm32l4xx_hal_conf.h
       │   ├── connect_wifi.h
       │   ├── delay.h 
       │   ├── lcd.h 
       │   ├── BH1750.h
       │   ├── sht3x.h
       │   ├── time.h 
├── Drivers
   ├── CMSIS
        ├── Device
           ├──ST
              ├──STM32L4xx
        ├── Include              
   ├── STM32L4xx_HAL_Driver
        ├── Inc
        ├── Src
└── mcu_sdk
    ├── mcu_api.c
    ├── mcu_api.h
    ├── protocol.c
    ├── protocol.h
    ├── system.c
    ├── system.h
    └── wifi.h
```



### Demo入口

入口文件：main.c

重要函数：main()

+ 对mcu的USART，IIC，TIME，BH1750，SHT30传感器等进行初始化配置，所有事件在while(1)中轮询判断。




### DP点相关

+ 上报dp点处理: mcu_dp_value_update()

| 函数名 | unsigned char mcu_dp_value_update(unsigned char dpid,unsigned long value) |
| ------ | ------------------------------------------------------------ |
| dpid   | DP的ID号                                                     |
| value  | DP数据                                                       |
| Return | SUCCESS: 成功  ERROR: 失败                                   |

+ MCU gets the dp value of the bool type:mcu_get_dp_download_bool()

| function name | unsigned char mcu_get_dp_download_bool(const unsigned char value[],unsigned short len) |
| ------------- | ------------------------------------------------------------ |
| value[]       | DP data buffer address                                       |
| len           | DP data length                                               |
| Return        | The current values of dp                                     |

### I/O 列表 

|   LCD    |    BH1750    |     SHT30      |                    TIME                     |  USART1  | USART2  | USART3  |             GPIO             |
| :------: | :----------: | :------------: | :-----------------------------------------: | :------: | :-----: | :-----: | :--------------------------: |
| PA9 TXD  | PB6 I2C1_SCL | PB10 I2C2_SCL  |              Timer 3 interrupt              | PA9 TXD  | PA2 TXD | PC4 TXD |             PC13             |
| PA10 RXD | PB7 I2C1_SDA | PB11  I2C2_SDA | Implement Different Tasks At DifferentTimes | PA10 RXD | PA3 RXD | PC5 RXD | Network Configuration Button |

### 

## 相关文档

涂鸦Demo中心：https://developer.tuya.com/demo



## 技术支持

您可以通过以下方法获得涂鸦的支持:

- 开发者中心：https://developer.tuya.com
- 帮助中心: https://support.tuya.com/help
- 技术支持工单中心: [https://service.console.tuya.com](https://service.console.tuya.com/) 


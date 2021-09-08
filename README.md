Tuya IoTOS Embedded Mcu Demo Wifi Ble STM32_LCD_WB3S

[English](./README.md) | [中文](./README_zh.md)

## Introduction  

This Demo uses the Tuya smart cloud platform, Tuya smart APP, BH1750, SHT30, Devine serial screenand IoTOS Embedded MCU SDK to realize a Illuminance and temperature and humidity acquisition and control of GPIO level reversal to control the corresponding load equipment.

The implemented features include:

+ Illuminance and temperature and humidity acquisition

+ Control GPIO level reversal

  


## Quick start  

### Compile & Burn
+ Download Tuya IoTOS Embeded Code
+ Execute the Project.uvprojx file
+ Click Compile in the software and complete the download


### File introduction 

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



### Demo entry

Entry file：main.c

Important functions：main()

+ Initialize and configure MCU USART,IIC,TIME,BH1750,SHT30sensor, etc. All events are polled and judged in while(1)。




### DataPoint related

+ DP point processing: mcu_dp_value_update()

| function name | unsigned char mcu_dp_value_update(unsigned char dpid,unsigned long value) |
| ------------- | ------------------------------------------------------------ |
| dpid          | DP ID number                                                 |
| value         | DP data                                                      |
| Return        | SUCCESS: Success ERROR: Failure                              |

+ MCU gets the dp value of the bool type:mcu_get_dp_download_bool()

| function name | unsigned char mcu_get_dp_download_bool(const unsigned char value[],unsigned short len) |
| ------------- | ------------------------------------------------------------ |
| value[]       | DP data buffer address                                       |
| len           | DP data length                                               |
| Return        | The current values of dp                                     |

### I/O List  

|   LCD    |    BH1750    |      SHT30       |                    TIME                     |  USART1  | USART2  | USART3  |             GPIO             |
| :------: | :----------: | :--------------: | :-----------------------------------------: | :------: | :-----: | :-----: | :--------------------------: |
| PA9 TXD  | PB6 I2C1_SCL | PB10    I2C2_SCL |              Timer 3 interrupt              | PA9 TXD  | PA2 TXD | PC4 TXD |             PC13             |
| PA10 RXD | PB7 I2C1_SDA |  PB11  I2C2_SDA  | Implement Different Tasks At DifferentTimes | PA10 RXD | PA3 RXD | PC5 RXD | Network Configuration Button |



## Related Documents

  Tuya Demo Center: https://developer.tuya.com/demo



## Technical Support

  You can get support for Tuya by using the following methods:

- Developer Center: https://developer.tuya.com
- Help Center: https://support.tuya.com/help
- Technical Support Work Order Center: [https://service.console.tuya.com](https://service.console.tuya.com/) 


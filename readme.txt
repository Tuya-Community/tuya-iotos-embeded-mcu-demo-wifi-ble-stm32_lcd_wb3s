使用ST的NUCLEO-L476RG开发板，温湿度传感器 （SHT30）,光照度传感器(BH1750)，迪文串口屏，涂鸦云模组WB3S。
控制屏和APP上实时显示温湿度和光照度数据，同时可以使用APP和控制屏进行控制GPIO口的电平，从而控制相应的负载设备。

MCU通过串口1和迪文屏进行通信。
MCU通过串口3和涂鸦云模组WB3S进行通信。
MCU通过串口2打印日志。

通过控制屏和APP控制相应的GPIO口(PC2,PC6,PC7,PC8)。

ST nucleo-L476RG development board, temperature and humidity sensor (SHT30), luminance sensor (BH1750), Diwen serial port panel, tuya cloud module WB3S.  
Temperature, humidity and illumination data can be displayed on the control screen and APP in real time. Meanwhile, 
the level of GPIO port can be controlled by APP and control screen to control corresponding load devices.  
 
MCU communicates with divin screen through serial port 1.  
MCU communicates with tuya cloud module WB3S through serial port 3.  
The MCU generates logs over serial port 2.  
 
Control the corresponding GPIO port (PC2,PC6,PC7,PC8) through the control screen and APP.  
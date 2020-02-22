<div align="center" class="has-mb-6">

![logo][logo]

## :wrench: RM-L-9015 电机系统

[淘宝链接][淘宝链接] |[官网][官网]

</div>

### :book: 简介 / Overview

RM-L-9015 是一款内部集成驱动器的高可靠性直流无刷电机，可广泛应用于机器人比赛、科研教育、自动化设备等领域。高极数设计、稀土材料磁铁及分数槽集中绕组方式确保电机能输出更大的扭矩，可为低转速、大扭矩直接驱动应用提供高性能的解决方案。驱动器采用磁场定向控制（FOC）算法，配合高精度的角度传感器，能实现精确的力矩和位置控制。电机具备异常提示和保护功能；支持多种通信方式，方便控制。

### :gift: 产品特性/ Product Characteristics

•  空心轴结构，用户可根据需求安装滑环等配件
•  电机和驱动器一体化设计，结构紧凑，集成度高
•  支持多种通讯方式（CAN、 PWM）
•  支持使用上位机软件进行调参
•  支持通过拨码开关设置 ID 和选通 CAN 终端电阻
•  具有电源防反接和过压保护等功能

### :construction: 接口说明 / Interface Specification

![RM-L-9015_IMG][电机示意图]  
A. 蓝色电源指示灯。  
B. 红色故障指示灯。  
C. SH1.0-5pin接口，集成TTL接口和PWM接口。  
![SH1.0-5pin接口][SH1.0-5pin接口]  
D. ZH1.5-8pin接口，集成24V电源接口和CAN接口。  
![ZH1.5-8pin接口][ZH1.5-8pin接口]  

##### :one:. CAN 总线接口

&ensp;&ensp; 通过 CAN 信号线连接外部控制设备，支持 CanOpen 总线协议。 CAN 总线比特率为 1Mbps。

##### :two:. 24V电源接口

&ensp;&ensp; 额定供电电压为 24V。（系统含电源防反接保护）

##### :three:. PWM 接口

&ensp;&ensp; 通过 PWM 信号线连接外部控制设备的 PWM 端口，控制电机转速或位置。允许输入的频率为400Hz、脉冲宽度行程为200-2248us的PWM信号。可在上位机软件中切换速度和位置控制模式（默认是位置模式）。
##### 位置模式：
  该模式下电机可以模拟成舵机使用。当脉宽小于200us，电机为0位置；当脉宽大于2248us，电机达到最大位置。脉宽和位置的射影关系如下图所示：  
  ![PWM_position][PWM_position]

##### 速度模式：
  该模式下电机可控制点击双向连续转动。+5v脉冲对应正向速度，-5V脉冲对应负向速度。当脉宽小于200us，速度为0；当脉宽大于1024，速度为物理参数中的最大速度。脉宽和速度的射影关系如下图所示：  
  ![PWM_velocity][PWM_velocity]


##### :four:. TTL 串口

&ensp;&ensp; TTL 串口通过 USB 转 TTL 连接到电脑上位机，用于电机系统的参数调试。配套USB转TTL采用了CH340x方案，如需安装驱动请点击[CH34x驱动][CH34x驱动]下载。

##### :five:. 指示灯:bulb:

&ensp;&ensp; 蓝色指示灯，指示系统的电源，输入电源正常，蓝色 LED 灯常亮。

&ensp;&ensp; 红色指示灯，指示系统故障:x:。详细见下图

| 红色 LED 状态     | 故障类型         |
| :---------------- | :--------------- |
| 不亮              | 系统运行正常     |
| 每隔 1s 闪烁 1 次 | 驱动芯片错误     |
| 每隔 2s 闪烁 2 次 | 编码器故障       |
| 每隔 2s 闪烁 3 次 | 输入电源电压异常 |
| 常亮              | 其它系统故障     |

##### :six:. 拨码开关

通过调节拨码开关设置电机的 ID 和选通 CAN 终端电阻  
![拨码开关][拨码开关]
•  拨码开关的第 1 位用于控制 CAN 终端电阻接入状态，拨至 ON 为接入。
•  拨码开关的第 2,3,4,5 用于控制 CAN ID。开关拨至 ON 为 1，否则为 0。拨码与 ID 的关系如下表所示：
| 拨码[5 4 3 2]位| 0000 | 0001 | 0010 | 0011 | 0100 | 0101 | 0110 | 0111 |
| :---- | :-----:| :-----:| :-----:| :-----:| :-----:| :-----:| :-----:| :-----:|
| CAN ID | 1 | 2| 3 | 4 | 5| 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
| 拨码[5 4 3 2]位| 1000 | 1001 | 1010 | 1011 | 1100 | 1101 | 1110 | 1111 |
| CAN ID | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |

### :hammer: 安装图纸 / Installation And Drawings

请参考电机安装孔尺寸和位置将电机安装到对应设备。  
![安装图][安装图]

### :warning: 注意事项 / Attention

1. 请严格按照本文档内规定使用电机。
2. 避免杂物进入转子内部，否则会导致转子运行异常。
3. 使用前请确保接线正确。
4. 使用前请确保电机安装正确、稳固。
5. 使用时请避免损伤线材，以防电机运行异常。
6. 使用时请勿触摸电机转子部分，避免割伤。
7. 电机大扭矩输出时，会出现发热的情况，请注意避免烫伤。
8. 用户请勿私自拆卸电机，否则会影响电机的控制精度，甚至会导致电机运行异常。

<!-- 以下内容为 Markdown 文档描述中出现的链接所指向的地址，统一在文档末尾进行管理。 -->
<!-- Markdown 超链接管理 -->

[淘宝链接]: https://www.taobao.com ""
[官网]: www.haitaijd.cn
[CH34x驱动]: http://www.wch.cn/downloads/CH341SER_ZIP.html

<!-- Markdown 图片链接管理 -->

[logo]: ./Picture/HaiTai_Logo.png "海泰Logo"
[电机示意图]: ./Picture/RM-L-9015.jpg "RM-L-9015电机示意图"
[拨码开关]: ./Picture/switch.png "拨码开关图"
[安装图]: ./Picture/Installation_diagram.png "安装图"
[SH1.0-5pin接口]: ./Picture/SH1.0-5pin.png "SH1.0-5pin接口"
[ZH1.5-8pin接口]: ./Picture/ZH1.5-8pin.png "ZH1.5-8pin接口"
[PWM_position]: ./Picture/PWM_position.png "PWM位置模式，脉宽与位置的关系"
[PWM_velocity]: ./Picture/PWM_velocity.png "PWM速度模式，脉宽与位置的关系"

# Lower_F407XX
EvolutionBoard STM32F4XX  from Aliexpress BLACK color.
LOWER PCB at mezzanine 
CANOpen at STM32F407

Проект выполнен в среде STM32CubeIDE


Реализовано CAN-устройство на STM32F4X-DISCOVERY from Aliexpress Green color.

CAN1	Activated	CAN1_RX	PD0
CAN1	Activated	CAN1_TX	PD1

TIM4	Internal Clock	
TIM14	Enable_Timer	

USART1	Asynchronous	USART1_RX	PA10
USART1	Asynchronous	USART1_TX	PA9
USART2	Asynchronous	USART2_RX	PA3
USART2	Asynchronous	USART2_TX	PA2
USB_OTG_FS	Device_Only	USB_OTG_FS_DM	PA11
USB_OTG_FS	Device_Only	USB_OTG_FS_DP	PA12

31	PA6	GPIO_Output	LED1
32	PA7	GPIO_Output	LED2

37	PB2	GPIO_Input
2	PE3	GPIO_Input	KEY1
3	PE4	GPIO_Input KEY0

59	PD12	GPIO_Output	LD4 [Green Led]
60	PD13	GPIO_Output	LD3 [Orange Led]
61	PD14	GPIO_Output	LD5 [Red Led]
62	PD15	GPIO_Output	LD6 [Blue Led]

Сеть состоит из устройств из проектов:
 
Disco407_Blue,

Aliex_Disco407green,

EvolutionBoard from Aliexpress BLACK color STM32F4XX LOWER PCB at mezzanine.

EvolutionBoard from Aliexpress BLACK color STM32F4XX UPPER PCB at mezzanine.




Boards_Infantry层是机器人架构中的板级支持包层，目的是为Application_Infantry等层开发提供便利。
该层主要以面向对象的思想编写STM32硬件方面的驱动，比如UART,CAN,IIC,SPI,GPIO,USB，TIM等

为了简化，对于GPIO等简单硬件，不进行面向对象编程。



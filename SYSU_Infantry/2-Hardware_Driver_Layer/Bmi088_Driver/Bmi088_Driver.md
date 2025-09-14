# Bmi088_Driver 文档
请你基于我的Bmi088驱动库，帮我对驱动库进行重构，注意不要修改具体的驱动代码逻辑以及时序接口逻辑

而是帮我封装一个 `Bmi088_device_t` 结构体

要求使用 `Bmi088_device_t *bmi088 = Bmi088_device_init(&bmi088_config1)` 进行初始化
- 我只是想修改成这样的调用形式，具体的初始化实现请你依照原来的驱动库`Bmi088_error_e Bmi088_init(void) `去适配

其中 `bmi088_config1` 是一个 `Bmi088_config_t` 结构体变量
包含一个 `Bmi088` 设备的所有配置参数，把原本是在头文件使用宏定义指定的部分，改成通过配置一个结构体变量来指定
```c
#define BMI088_SPI hspi1
#define BMI088_ACC_GPIOx GPIOA
#define BMI088_ACC_GPIOp GPIO_PIN_4
#define BMI088_GYRO_GPIOx GPIOB
#define BMI088_GYRO_GPIOp GPIO_PIN_0
```

改成
```c
Bmi088_config_t bmi088_config1 = {
    .spi_handle = &hspi1,
    .accel_cs_gpio_port = GPIOA,
    .accel_cs_gpio_pin = GPIO_PIN_4,
    .gyro_cs_gpio_port = GPIOB,
    .gyro_cs_gpio_pin = GPIO_PIN_0,
};
```

对于一些基础函数，只在内部使用的，可以改成 `static` 函数，注意需要修改为支持多结构体的实例化，可以传入 `Bmi088_device_t *bmi088` 作为第一个参数

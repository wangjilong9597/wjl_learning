# c++开源算法学习_PX4固件 _ardupliot

本次学习

一熟悉mavlink消息

二学习px4固件架构

三是学习c++编写知识

## 1.Ardusub

### 1.1 源码程序入口

以ArduSub的程序说明，其他的设备基本一样，程序的main函数被一个宏定义包装。

```cpp
//ArduSub.cpp
AP_HAL_MAIN_CALLBACKS(&sub); 
```

其中sub是在Sub.cpp文件中定义的一个ROV(潜水艇)设备对象，其持有与设备相关的数据或对象和功能函数，如姿态控制器对象attitude_control，位置控制器对象pos_control，电机等，该类型是AP_Vehicle类的子类（AP_Vehicle继承了Callbacks），sub继承了父类持有的板载模块数据和对象，如IMU,GPS,气压计等。对宏转到定义：

```cpp
//AP_HAL_Main.h
#define AP_MAIN main
#define AP_HAL_MAIN_CALLBACKS(CALLBACKS) extern "C" { 
    int AP_MAIN(int argc, char* const argv[]); 
    int AP_MAIN(int argc, char* const argv[]) { 
        hal.run(argc, argv, CALLBACKS); 
        return 0; 
    } 
    }
```

可以看出main函数仅仅调用了hal.run()函数，其主要参数只有上述的sub对象地址。

**注**：这里的参数具体指什么不用管，无非有三个参数：

1.CALLBACKS 

2.argc

3.argv



这里的hal是在Sub.cpp文件中定义的一个**硬件抽象层**对象。hal对象管理着底层的外设和**HAL调度器**，如串口、SPI、IIC。

```cpp
//Sub.cpp
const AP_HAL::HAL& hal = AP_HAL::get_HAL();
```

get_HAL函数声明在AP_HAL_Namespace.h中。

```c++
//HAL_ChibiOS_Class.cpp
const AP_HAL::HAL& AP_HAL::get_HAL() {
 static const HAL_ChibiOS hal_chibios;      //class HAL_ChibiOS : public AP_HAL::HAL
 return hal_chibios;
}
```

`AP_HAL::HAL &AP_HAL::get_HAL()`是利用HAL的子类对父类的一个赋值，
子类用初始化参数进行赋值的成员，赋值给父类的构造函数参数，又通过父类的初始化参数列表给父类成员赋值，而且整个过程运用的是引用，对父类成员的操作能直接影响到子类成员。

Ardupilot项目支持多种飞控板，为了支持不同的OS，该函数在不同的OS中进行了实现。AP_HAL提供了一个飞控总的AP_HAL抽象，通过不同类型的飞控板去实现继承，这样就可以兼容不同的板子，方便ardupliot可以支持不同的硬件。下图是总结图：

![20180606135056378](C:\Users\用户1\Desktop\MD笔记\typora保存的图片\3.ardupliot\20180606135056378.png)

```cpp


```

在HAL_ChibiOS.cpp文件中定义了多种硬件驱动对象和HAL任务调度器对象，并在定义hal对象时在构造函数中引用给了hal对象。

```cpp
//HAL_ChibiOS_Class.cpp
HAL_ChibiOS::HAL_ChibiOS() :
 AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &uartDDriver,
        &uartEDriver,
        &uartFDriver,
        &uartGDriver,
        &uartHDriver,
        &i2cDeviceManager,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        &flashDriver,
        &dspDriver,
 nullptr
        )
{}
```

sub和hal是整个工程最大的两个对象。



## 接下来对hal.run()函数分析

刚才说hal对象定义在HALChibiOS.cpp中，hal.run函数也定义在这里，函数主要内容只有调用了一个main_loop函数。

```cpp
//HAL_ChibiOS_Class.cpp
void HAL_ChibiOS::run(int argc, char * const argv[], Callbacks* callbacks) const
{
 assert(callbacks);
 g_callbacks = callbacks;   //以基类型传递sub对象的地址给指针g_callbacks 

 main_loop();
}
```

main_loop函数可以简单地理解为main函数，它是ChibiOS的主线程，在这里先是对上述的hal和sub(g_callbacks)进行初始化，最后死循环执行g_callbacks->loop()，函数定义如下。

```cpp
//HAL_ChibiOS_Class.cpp
static void main_loop()
{
 daemon_task = chThdGetSelfX();

    /*
      switch to high priority for main loop
     */
 chThdSetPriority(APM_MAIN_PRIORITY);

#ifdef HAL_I2C_CLEAR_BUS
    // Clear all I2C Buses. This can be needed on some boards which
    // can get a stuck I2C peripheral on boot
 ChibiOS::I2CBus::clear_all();
#endif

 ChibiOS::Shared_DMA::init();
 peripheral_power_enable();
//外设初始化
 hal.uartA->begin(115200);

#ifdef HAL_SPI_CHECK_CLOCK_FREQ
    // optional test of SPI clock frequencies
 ChibiOS::SPIDevice::test_clock_freq();
#endif

 hal.uartB->begin(38400);
 hal.uartC->begin(57600);
 hal.analogin->init();
//HAL调度器初始化，开辟5个线程，下边详细分析
 hal.scheduler->init();

    /*
      run setup() at low priority to ensure CLI doesn't hang the
      system, and to allow initial sensor read loops to run
     */
 hal_chibios_set_priority(APM_STARTUP_PRIORITY);

 if (stm32_was_watchdog_reset()) {
        // load saved watchdog data
 stm32_watchdog_load((uint32_t *)&utilInstance.persistent_data, (sizeof(utilInstance.persistent_data)+3)/4);
 utilInstance.last_persistent_data = utilInstance.persistent_data;
    }

 schedulerInstance.hal_initialized();
 //sub设备的初始化，主要是AP调度器的初始化和传感器的初始化
 g_callbacks->setup();

#ifdef IOMCU_FW
 stm32_watchdog_init();
#elif !defined(HAL_BOOTLOADER_BUILD)
    // setup watchdog to reset if main loop stops
 if (AP_BoardConfig::watchdog_enabled()) {
 stm32_watchdog_init();
    }

#ifndef HAL_NO_LOGGING
 if (hal.util->was_watchdog_reset()) {
 INTERNAL_ERROR(AP_InternalError::error_t::watchdog_reset);
    }
#endif // HAL_NO_LOGGING
#endif // IOMCU_FW

 schedulerInstance.watchdog_pat();

 hal.scheduler->system_initialized();

 thread_running = true;
 chRegSetThreadName(SKETCHNAME);

    /*
      switch to high priority for main loop
     */
 chThdSetPriority(APM_MAIN_PRIORITY);

 while (true) {
 g_callbacks->loop();//调用scheduler.loop();即AP调度器在这里循环运行

        /*
          give up 50 microseconds of time if the INS loop hasn't
          called delay_microseconds_boost(), to ensure low priority
          drivers get a chance to run. Calling
          delay_microseconds_boost() means we have already given up
          time from the main loop, so we don't need to do it again
          here
         */
#ifndef HAL_DISABLE_LOOP_DELAY
 if (!schedulerInstance.check_called_boost()) {
 hal.scheduler->delay_microseconds(50);
        }
#endif
 schedulerInstance.watchdog_pat();
    }
 thread_running = false;
} 
```

main_loop函数主要分为三部分，这样的结构也是Arduino编程一贯的风格：

1. hal对象持有的部分底层外设的初始化和HAL调度器的初始化
2. g_callbacks->setup()
3. g_callbacks->loop()

这篇文章先分析第一部分，其余两部分在后边两篇文章分析。

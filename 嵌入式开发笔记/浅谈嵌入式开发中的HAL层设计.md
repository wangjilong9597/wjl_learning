# 浅谈嵌入式开发中的HAL层设计

## 1 关于HAL(Hardware Abstraction Layer)层

硬件抽象层技术最初是由Microsoft公司为确保WindowsNT的稳定性和兼容性而提出的。针对过去Windows系列操作系统经常出现的系统死机或崩溃等现象，Microsoft总结发现，程序设计直接与硬件通信，是造成系统不稳定的主要原因。在得出这个结论的基础上，微软公司在WindowsNT上取消了对硬件的直接访问，首先提出了硬件抽象层(Hardware Abstraction Layer，简称HAL)的概念，硬件抽象层就是：“将硬件差别与操作系统其他层相隔离的一薄层软件，它是通过采用使多种不同硬件在操作系统的其他部分看来是同一种虚拟机的做法来实现的。“后来，这种HAL设计思路被一些嵌入式操作系统参考，其系统内核被分成两层，上层称为“内核(Kernel)”，底层则称为“硬件抽象层”。在EOS中，HAL独立于EOS内核；对于操作系统和应用软件而言，HAL是对底层架构的抽象。综合分析HAL层的代码，可以发现这些代码与底层硬件设备是紧密相关的。因此，可以将硬件抽象层定义为所有依赖于底层硬件的软件。即使有些EOS的HAL在物理上是与系统内核紧密联系的，甚至相互交叉的，但是从功能上可以从分层技术的角度去分析它 。

显然，HAL层的根本设计目的是在于上层软件可以通过常规的通用接口访问MCU的某些资源，而不用去关心“是哪个MCU”在做的，模糊了最底层的函数实现。

在嵌入式方面,ST公司在HAL层的应用方面推广较多，很典型的就是stm32cubemx,这个软件实际是和STM32的HAL库联系起来一起用的。因为HAL库提供了兼容性非常强的函数接口定义，所以代码生成器(stm32cubemx)可以很方便的自动生成代码，而不需要开发者一点一点的翻阅芯片手册，查看寄存器。

因为工作需要，我开始思考“如何设计一个HAL层”的问题，参考了TI的CC26xx系列SDK的封装方法，于是撰写这个文章。

## 2 HAL层的关键设计原则

设计HAL层，应该需要忽略一些硬件实现细节，更加宏观的看待MCU模块提供的功能。 就好比，我们会去想象一个“台灯”，那么用HAL去抽象这个台灯的话，他一定要有“照明”的功能，还可以调节“亮度”。 那么这个台灯就简单的抽象好了，我不会去关心这个台灯用的什么灯泡，只要这个灯泡能亮就行。 于是，我有一个统一的HAL层函数 HAL_OPEN_LIGHT()以及HAL_ADJUST_LIGHT(LIGHT)函数，抽象了这个接口。

## 3 具体的简单实现例子

在C语言中会去这样做

> typedef void (LIGHT_Open_Fxn) (LIGHT_Handle handle);
> typedef void *(*LIGHT_AdjustFxn) (LIGHT_Handle handle , uint16_t light_strength);
> typedef struct LIGHT_FxnTable_ {
> LIGHT_OpenFxn light_open_Fxn;
> LIGHT_AdjustFxn ligh_adjust_Fxn;
> } LIGHT_FxnTable;

上述代码就定义好了，HAL的基本接口。后面如何实现这个接口呢，C中这样去写。

> void Light_open_led(LIGHT_Handle handle){
> //具有特色的实现方式
> } void Light_adjust_led(LIGHT_Handle handle , uint16_t light_strength){
> //特别的实现方式
> }
> LIGHT_FxnTable LIGHT={
> Light_open_led，
> Light_adjust_led，
> }//完成HAL接口的实现

这样一来创建的LIGHT句柄就包含了可以灵活配置实现方法的函数指针列表。

如果我有两家台灯厂家竞争，那么我只需要重写Light_open_led和Light_adjust_led函数即可兼容两个厂家的台灯特点。上层软件调用的始终都是LIGHT->LIGHT_OpenFxn() 和 LIGHT->Light_adjust_led()。

> LIGHT_FxnTable LIGHT={ Light_open_led_A， Light_adjust_led_A， }//HAL接口重新实现 LIGHT_FxnTable LIGHT={ Light_open_led_B， Light_adjust_led_B， } //HAL接口重新实现
> .....

## 3 HAL层的优点

很多时候我们其实希望开发者能够自下而上的构建工程，比如先硬件实现(HW)，然后编写中间层软件(Driver),最后我们再编写最高层软件(Application)。 但是实际情况下，往往不是这样，而是替补，有可能是产品找到了cost-down的低成本主控，或者芯片升级，或者根本就是主控芯片断货了，在这种情况下，APP代码是不可能变动的，只能在APP的要求下，准备中间层软件(Drievr)和硬件实现。

APP代码是一系列的用户操作逻辑，好比开灯关灯的摩斯密码，有自己一套的使用外设逻辑，它只会简单机械的按照之前的套路使用下层提供的外设。而往往绝大多数情况下HW,Driver的组合不能完全兼容APP的调用规范。 所以在这种情境下，HAL层就变的非常重要了，他在HW，Driver 与APP之间提供一个缓冲的作用。

![img](https://pic4.zhimg.com/80/v2-88e7950330e703dc4333304214eb7a6b_720w.webp)

当APP本就是根据HAL设计的时候，需要更换底层的时候，移植HW，Driver以适配HAL 当APP还未设计出来的时候，按照标准的HAL库结构去开发APP，为后期兼容更多处理器做前期工作。

总而言之，HAL层的设计就是为了能兼容更多的处理器型号，兼容更多外设的实现方式。

## 4 最后

写这些分享一下我对HAL的理解，因为工作需要在这方面查找了很多关于HAL的资料，上述的封装思路是借鉴了TI的CC26XX的SDK里面对HAL层的封装，感兴趣的可以下载[SDK](https://link.zhihu.com/?target=http%3A//www.ti.com/tool/download/SIMPLELINK-CC2640R2-SDK/)研究一下，后面可能会更加详细的介绍TI的封装方法。


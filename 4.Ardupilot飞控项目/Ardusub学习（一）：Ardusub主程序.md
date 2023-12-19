# Ardusub学习（一）：Ardusub主程序



## 一、准备工作

在网上查了很多资料，发现关于Ardusub很少，因此此次把自己的研究的一些所得拿出来分享一下，可能写得并不是很周全，希望大家多多包含，如有错误请务必留言指出。在阅读本文内容之前，请仔细阅读Ardupilot的官方手册，建立起一个大概的印象即可，这里便不再讲述一些基础知识。

[Github源码戳这里~](https://github.com/ArduPilot/ardupilot)
[Ardusub官方手册](http://www.ardusub.com/)
[Ardupilot官方手册](https://ardupilot.org/ardupilot/)
[Ardupilot开发者手册](https://ardupilot.org/dev/index.html)

由于源码内容很多，此次我仅挑选 “ardupilot/ArduSub/” 路径下的ArduSub.cpp主文件进行解析，后续如果有时间再慢慢补充其他内容。如果自己下载源码，推荐使用Souce Insight进行阅读。

## 二、Ardusub.cpp解析

源码内容很多，我这里就不全部放出来了，就挑部分进行讲解，讲解思路和官方手册分析Copter的姿态控制的脉络保持一致。

### 2.1 scheduler table

SCHED_TASK_CLASS和SCHED_TASK用来声明产生调度任务表，表中是除了被fast_loop()函数调用任务的其他常规任务。其中_rate_hz表示调用频率，_max_time_micros表示最长等待时间。

```c++
#define SCHED_TASK_CLASS(classname, classptr, func, _rate_hz, _max_time_micros)
```

```c++
#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Sub, &sub, func, rate_hz, max_time_micros)
```


注意到SCHED_TASK_CLASS可以指向不同不同类，而SCHED_TASK固定为Sub类中的函数实现，Sub为车辆类型，在其他文件中科替换为其他类型的车辆（如Copter）。

### 2.2 scheduler

#### get_scheduler_tasks()

如果是master版本的，程序前面应该是get_scheduler_tasks()

```cpp
void Sub::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                 uint8_t &task_count,
                                 uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}
```


该函数主要实现获取调度表其实地址，并且计算出任务数量。

#### setup()和loop()

如果是stable版本的，比如我现在使用的Sub4.0版本的，那么前面应该是setup()和loop()，和官方手册介绍时的一样。

```cpp
void Sub::setup()
{
    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    init_ardupilot();
    
    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), MASK_LOG_PM);

}

void Sub::loop()
{
    scheduler.loop();
    G_Dt = scheduler.get_loop_period_s();
}
```


setup()完成加载默认参数列表值并且初始化APM（`init_ardupilot()`看这里）和任务的工作，`loop()`对任务进行循环并且获得每次循环的间隔时间。

### 2.3 fast_loop()

```c++
// Main loop - 400hz
void Sub::fast_loop()
{
    // update INS immediately to get current gyro data populated
    ins.update();

    //don't run rate controller in manual or motordetection modes
    if (control_mode != MANUAL && control_mode != MOTOR_DETECT) {
        // run low level rate controllers that only require IMU data
        attitude_control.rate_controller_run();
    }
    
    // send outputs to the motors library
    motors_output();
    
    // run EKF state estimator (expensive)
    // --------------------
    read_AHRS();
    
    // Inertial Nav
    // --------------------
    read_inertia();
    
    // check if ekf has reset target heading
    check_ekf_yaw_reset();
    
    // run the attitude controllers
    update_flight_mode();
    
    // update home from EKF if necessary
    update_home_from_EKF();
    
    // check if we've reached the surface or bottom
    update_surface_and_bottom_detector();

#if HAL_MOUNT_ENABLED
    // camera mount's fast update
    camera_mount.update_fast();
#endif

    // log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();
    }
    
    AP_Vehicle::fast_loop();

}
```


该函数为本文件中最重要的函数，以400Hz频率运行。前面的一些调用函数就不细讲了，都是一些传感器和电机的读取或者更新设置函数，大家自己去看源码，很快就能看懂。

需要注意的是这段代码

```cpp
//不要运行在手动或者电机检测下
if (control_mode != MANUAL && control_mode != MOTOR_DETECT) {
    // 实现仅需要IMU数据的低速率控制器
    attitude_control.rate_controller_run();
}
```
这段代码注释了不要在手动或者电机检测模式下运行，然后在代码段内实现了一个仅接受IMU数据的低速率控制器，进入libraries下的AC_Attitude_Control_Sub.cpp文件查看

```cpp
void AC_AttitudeControl_Sub::rate_controller_run()
{
    //将油门与姿态混合移动到所需的位置（从这里调用，因为每次迭代都会方便地调用）
    // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    update_throttle_rpy_mix();

    Vector3f gyro_latest = _ahrs.get_gyro_latest();
    _motors.set_roll(get_rate_roll_pid().update_all(_rate_target_ang_vel.x, gyro_latest.x, _motors.limit.roll));
    _motors.set_pitch(get_rate_pitch_pid().update_all(_rate_target_ang_vel.y, gyro_latest.y, _motors.limit.pitch));
    _motors.set_yaw(get_rate_yaw_pid().update_all(_rate_target_ang_vel.z, gyro_latest.z, _motors.limit.yaw));
    
    control_monitor_update();

}
```


主要是用set_xxx()方法接收roll/pitch/yaw的-1~1的值，这些值表示的是期望尽快往哪个方向运动的意思。具体查看这里。

这里主要讲一下最主要的姿态控制函数update_flight_mode()。

#### 2.3.1 flight_mode.cpp

```c++
// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
void Sub::update_flight_mode()
{
    switch (control_mode) {
    case ACRO:
        acro_run();
        break;

    case STABILIZE:
        stabilize_run();
        break;
    
    case ALT_HOLD:
        althold_run();
        break;
    
    case AUTO:
        auto_run();
        break;
    
    case CIRCLE:
        circle_run();
        break;
    
    case GUIDED:
        guided_run();
        break;
    
    case SURFACE:
        surface_run();
        break;

#if POSHOLD_ENABLED == ENABLED
    case POSHOLD:
        poshold_run();
        break;
#endif

    case MANUAL:
        manual_run();
        break;
    
    case MOTOR_DETECT:
        motordetect_run();
        break;
    
    default:
        break;
    }

}
```


update_flight_mode()定义于Ardusub文件路径下的flight_mode.cpp，该函数通常以100Hz及以上频率被调用。内部的 control_mode 定义于Sub.h中，用来表示不同的飞行状态，共计10种飞行模式，各个模式宏定义于defines.h中。通过代码可以看出是通过了一个swtich()函数接收并且判断进入对应飞行模式的xxx_run()函数中。

在defines.h中定义了各个飞行模式的具体含义内容：

```c++
// Auto Pilot Modes enumeration
enum control_mode_t : uint8_t {
    STABILIZE =     0,  // manual angle with manual depth/throttle
    ACRO =          1,  // manual body-frame angular rate with manual depth/throttle
    ALT_HOLD =      2,  // manual angle with automatic depth/throttle
    AUTO =          3,  // fully automatic waypoint control using mission commands
    GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    CIRCLE =        7,  // automatic circular flight with automatic throttle
    SURFACE =       9,  // automatically return to surface, pilot maintains horizontal control
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    MANUAL =       19,  // Pass-through input with no stabilization
    MOTOR_DETECT = 20   // Automatically detect motors orientation
};
```


update_flight_mode()中对应的xxx_run ()函数则是在同路径下的control_xxx.cpp中定义。下面以stabilize_run()为例进行讲解，因此我们需要进入control_stabilize.cpp中。

#### 2.3.2 control_stabilize.cpp

master版本

注意到内部包含Sub.h头文件，然后仅实现了两个函数，分别为stabilize_init()和stabilize_run()。

```cpp
// stabilize_init - initialise stabilize controller
bool Sub::stabilize_init()
{
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);
    last_pilot_heading = ahrs.yaw_sensor;

    return true;

}
```


其中的init()函数是将这个飞行模式下的控制器进行初始化，如将期望高度归零并且获取最后的机头朝向(yaw角)。run()函数则是实现了具体的控制器功能，该函数应该以100Hz及以上频率被调用。

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Sub::stabilize_run()
{
    uint32_t tnow = AP_HAL::millis();
    float target_roll, target_pitch;
    float target_yaw_rate;

```cpp
// if not armed set throttle to zero and exit immediately
if (!motors.armed()) {
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    attitude_control.set_throttle_out(0,true,g.throttle_filt);
    attitude_control.relax_attitude_controllers();
    last_pilot_heading = ahrs.yaw_sensor;
    return;
}

motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

// convert pilot input to lean angles
// To-Do: convert get_pilot_desired_lean_angles to return angles as floats
get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

// get pilot's desired yaw rate
target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

// call attitude controller
// update attitude controller targets

if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    last_pilot_heading = ahrs.yaw_sensor;
    last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

} else { // hold current heading

    // this check is required to prevent bounce back after very fast yaw maneuvers
    // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
    if (tnow < last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
        target_yaw_rate = 0;  // Stop rotation on yaw axis

        // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

    } else { // call attitude controller holding absolute absolute bearing
        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, last_pilot_heading, true);
    }
}

// output pilot's throttle
attitude_control.set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);

//control_in is range -1000-1000
//radio_in is raw pwm value
motors.set_forward(channel_forward->norm_input());
motors.set_lateral(channel_lateral->norm_input());
```

}

- run()内部首先声明期望的roll和pitch角度，以及yaw的角速度，并获取了自无人机启动之后的当前时间，用tnow存储。


- 程序运行第一步，首先判断电机是否已经准备就绪，如果还没有，就进入if中的代码段

  - 首先是通过set_desired_spool_state()函数设置为期望的轴状态，此处设置为GROUOND_IDLE，即将所有电机闲置（AP_Motors.cpp中声明了3种轴状态：SHUT_DOWN关闭、GROUND_IDLE空闲和THROTTLE_UNLIMITED取消限制）。

  - set_throttle_out()函数用来设置油门输出，共接收3个参数，第一个参数throttle_in表示提供给姿态控制器的油门输入，第二个参数apply_angle_boost表示是否启动角度提升，这将会在set_throttle_out()内部调用get_throttle_boosted()函数用来实现对roll/pitch的补偿并且返回一个0~1的油门量，第三个参数filter_cutoff设置油门的低通滤波器。

  - relax_attitude_controllers()函数确保姿态控制器无误并且松弛速率控制器的输出，内部主要是实现参数的重初始化以及PID等控制器的重启。然后返回退出stabilize_run()函数。

- 如果电机已经准备好了，那么就不进入if()的代码段中，首先设置轴状态为取消限制。void Sub::get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max)函数将获取roll和pitch的输入量然后将其转换为对应的以度单位表示的角度输出传输给在run()最开始设置的target_roll和target_pitch。get_pilot_desired_yaw_rate()函数将获取输入的yaw期望并且将其转换为°/s为单位yaw角速度输出并保存在target_yaw_rate中。

- 然后进入姿态控制器，更新姿态至期望值。分为两种情况：

  - 当期望yaw角速度控制量不为0时，通过调用input_euler_angle_roll_pitch_euler_rate_yaw()函数（后面详解），实现角速度前馈和平滑来控制欧拉角和俯仰角以及欧拉偏航率。然后将当前yaw角保存到上次机头朝向变量中，并保存当前时间为上一次yaw控制量输入时间。

  - 如果期望yaw角速度控制量为0，那么保持当前机头朝向不变，但是为了防止由于无人机的关系使得航向超过设定位置，需要执行一次检查来防止快速偏航操作后的反弹。具体检查内容如下：判断当前时间与上一次YAW角变更操作的时间差，如果在250ms内，将target_yaw_rate置0，然后调用input_euler_angle_roll_pitch_euler_rate_yaw()函数执行姿态变更；但是如果时间差超过250ms，那么改为调用input_euler_angle_roll_pitch_yaw()函数，视角输入yaw控制角位上一次测量获得的yaw角。

- 然后对油门进行配置即可。

- 最后的set_forward()和set_lateral()查看2.3小节前面部分讲述set_xxx()方法的地方。这边是设置前后平移和左右平移的意思。

#### 2.3.3 AC_AttitudeControl.cpp

最后来看一下在libraries库中AC_AttitudeControl.cpp中的input_euler_angle_roll_pitch_euler_rate_yaw()方法，其实大家看源码也能看出大概的意思了。

这边也简单说一下，函数内部先把相应的角度转换为弧度制，然后计算出对于的期望欧拉角，并在欧拉角的roll向上增加侧倾调整以补偿推力，然后判断是否开启了前馈。是的话，还需要进行一系列计算，例如将摇杆期望欧拉角与前馈目标欧拉角进行差值计算以得到前馈角速度，还有将目标欧拉角速度转换为机体角速度等等；如果没有开启前馈，那么就直接将欧拉角输入到目标中，并根据目标欧拉角转换得出目标四元数，然后前馈率归零。最后调用四元数姿态控制器。

总的来说，这个函数的功能就是通过角速度前馈和平滑来控制欧拉角和俯仰角以及欧拉偏航率（原谅我直接翻译了注释…）。对于前面control_stabilize.cpp中所用的input_euler_angle_roll_pitch_yaw()方法则是通过角速度前馈和平滑控制欧拉角，俯仰角和偏航角。如果有兴趣继续研究，详看官方的手册，虽然是以Copter为讲解对象的，但是原理基本一致。

```c++
// Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
void AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds)
{
    // Convert from centidegrees on public interface to radians
    float euler_roll_angle = radians(euler_roll_angle_cd * 0.01f);
    float euler_pitch_angle = radians(euler_pitch_angle_cd * 0.01f);
    float euler_yaw_rate = radians(euler_yaw_rate_cds * 0.01f);

    // calculate the attitude target euler angles
    _attitude_target_quat.to_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

    // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
    euler_roll_angle += get_roll_trim_rad();

    if (_rate_bf_ff_enabled) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        Vector3f euler_accel = euler_accel_limit(_attitude_target_euler_angle, Vector3f(get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()));

        // When acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler
        // angular velocity that will cause the euler angle to smoothly stop at the input angle with limited deceleration
        // and an exponential decay specified by smoothing_gain at the end.
        _attitude_target_euler_rate.x = input_shaping_angle(wrap_PI(euler_roll_angle - _attitude_target_euler_angle.x), _input_tc, euler_accel.x, _attitude_target_euler_rate.x, _dt);
        _attitude_target_euler_rate.y = input_shaping_angle(wrap_PI(euler_pitch_angle - _attitude_target_euler_angle.y), _input_tc, euler_accel.y, _attitude_target_euler_rate.y, _dt);

        // When yaw acceleration limiting is enabled, the yaw input shaper constrains angular acceleration about the yaw axis, slewing
        // the output rate towards the input rate.
        _attitude_target_euler_rate.z = input_shaping_ang_vel(_attitude_target_euler_rate.z, euler_yaw_rate, euler_accel.z, _dt);

        // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
        euler_rate_to_ang_vel(_attitude_target_euler_angle, _attitude_target_euler_rate, _attitude_target_ang_vel);
        // Limit the angular velocity
        ang_vel_limit(_attitude_target_ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
        // Convert body-frame angular velocity into euler angle derivative of desired attitude
        ang_vel_to_euler_rate(_attitude_target_euler_angle, _attitude_target_ang_vel, _attitude_target_euler_rate);
    } else {
        // When feedforward is not enabled, the target euler angle is input into the target and the feedforward rate is zeroed.
        _attitude_target_euler_angle.x = euler_roll_angle;
        _attitude_target_euler_angle.y = euler_pitch_angle;
        _attitude_target_euler_angle.z += euler_yaw_rate * _dt;
        // Compute quaternion target attitude
        _attitude_target_quat.from_euler(_attitude_target_euler_angle.x, _attitude_target_euler_angle.y, _attitude_target_euler_angle.z);

        // Set rate feedforward requests to zero
        _attitude_target_euler_rate = Vector3f(0.0f, 0.0f, 0.0f);
        _attitude_target_ang_vel = Vector3f(0.0f, 0.0f, 0.0f);
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

```

### 2.4 其他

剩余部分如 fifty_hz_loop()，ten_hz_logging_loop() 等等，都是以某一特定频率持续运行的任务，大家就自己看看吧，这里就不多介绍了（后续研究深入之后有时间再来更新）。


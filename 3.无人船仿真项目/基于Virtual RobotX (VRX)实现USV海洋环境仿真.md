# 基于Virtual RobotX (VRX)实现USV海洋环境仿真

关于看到这个项目的来源：

<img src="基于Virtual RobotX (VRX)实现USV海洋环境仿真.assets/image-20231208102730797.png" alt="image-20231208102730797" style="zoom: 80%;" />

根据这个issue的建议，vrx这个仿真环境是更加适合水面仿真。

而且这个项目在一直维护：

<img src="基于Virtual RobotX (VRX)实现USV海洋环境仿真.assets/image-20231208103003271.png" alt="image-20231208103003271" style="zoom:50%;" />

所以打算做做看。

官方的wiki文档：https://github.com/osrf/vrx/wiki/documentation

官方的wiki文档中有关于整个项目的全部介绍。

## 一. 安装VRX

Gazebo Garden 和 ROS humble ，而ROS humble 需要ubuntu22.04才能安装。这些现在是 VRX 的默认先决条件。这是对新用户的推荐配置。

关于安装22.04在此就不介绍了。

### 1.ROS humble安装

直接使用一键安装指令，跟随提示选择需要的ROS进行安装

```bash
wget http://fishros.com/install -O fishros && . fishros
```

### 2.Gazebo Garden安装

为 Ubuntu Focal 和 Jammy 提供了 Garden 二进制文件。Garden 二进制文件托管在packages.osrfoundation.org 存储库中。要安装所有这些，`gz-garden`可以安装元包。

**警告：** `gz-garden`不能与gazebo-classic（例如`gazebo11`）一起安装，因为两者都使用`gz`命令行工具。尝试`gz-garden`在已经从二进制文件安装了gazebo-classic的系统上进行安装将导致gazebo-classic及其依赖项被卸载。目前，解决方法是从源安装或使用 Docker [`gazebo-classic`](https://hub.docker.com/_/gazebo)，这样它们就不会在同一系统上并排安装。

首先安装一些必要的工具：

```bash
sudo apt-get update 
sudo apt-get install lsb-release wget gnupg
```

然后安装Gazebo Garden：

```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg 
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | 
sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null sudo apt-get update sudo apt-get install gz-garden
```

所有库都应该准备好使用，并且`gz sim`应用程序应该准备好执行。

返回[入门](https://gazebosim.org/docs/all/getstarted) 页面开始使用 Gazebo！

如果您已经从二进制文件安装了库，则需要卸载 Gazebo 或切换到基于源的安装，请运行以下命令：

```bash
sudo apt remove gz-garden && sudo apt autoremove
```

### 3.安装VRX项目

安装完这两个后。

然后安装依赖

```bash
sudo apt install python3-sdformat13 ros-humble-ros-gzgarden ros-humble-xacro
```

然后安装项目

Create a colcon workspace and clone the vrx repository

```bash
mkdir -p ~/vrx_ws/src
cd ~/vrx_ws/src
git clone https://github.com/osrf/vrx.git
```

Source your ROS 2 installation.

```bash
source /opt/ros/humble/setup.bash
```

Build the workspace

```bash
cd ~/vrx_ws
colcon build --merge-install
```

Now that you've built the simulation, you will need to source the setup script before you can do anything with it. From the root of your workspace, run:

```bash
. install/setup.bash
```

Note that, in general, you always need to perform this step when working with ROS and ROS 2 workspaces. Forgetting to source the environment is one of the most common mistakes among new users. If you're new to ROS 2 workspaces, you may find [this tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) helpful.

至此就可以运行该项目了，使用下面命令

```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

## 二.根据自己的浮体进行修改


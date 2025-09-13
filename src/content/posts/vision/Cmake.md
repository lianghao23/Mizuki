---
title: 视觉组培训：三阶段-CMake
published: 2025-09-04
tags: [Cmake]
category: 视觉组
draft: false
---

# CMake
## 1. cmake是什么
    CMake是一个开源、跨平台的构建系统生成器。它使用一种相对独立于编译器的配置文件（CMakeLists.txt）来管理整个软件构建过程。
    
    啥意思呢？简单来说就类似于一个清单，他告诉编译器头文件都在哪些地方，如果没有他，每一次编译我们都要用命令去告诉编译器头文件A在a目录下，头文件B在b目录下，这样就很麻烦，但有了cmake之后只需要将这些头文件的地址统一写在cmakelists文件里便可以完成上述操作，简单高效且不同平台都可使用。

没有cmake时：编译一个c++程序需要输入的命令：
```
（编译一个用到 OpenCV 和 Boost 的程序）：
g++ -I./include 
    -I/usr/local/include/opencv4 
    -I/usr/include/boost main.cpp 
    utils.cpp \
    -L/usr/local/lib 
    -lopencv_core 
    -lopencv_imgproc 
    -lopencv_highgui 
    -lboost_system \
    -o my_app
```
    项目越大，命令越长，没人记得住，且无法跨平台使用，其在Windows上完全不能用，换台 Linux 机器路径可能又不一样。于是乎，cmake应运而生。
## 2. cmake的写法
我们有了cmake，便会将头文件等声明写在CMakeLists.txt文件中。camke根据不同的需求会加入不同的命令，但构建cmake所必须的语法有以下四个：
```
#1.声明cmake最低版本为3.10
cmake_minimum_required(VERSION 3.10)

#2.声明项目名
project(MyAwesomeApp)

#3.告诉编译器头文件在./include路径下。
include_directories(./include)

#4.告诉它要生成一个名为my_app的可执行文件，用到了名为main.cpp的源文件
add_executable(my_app main.cpp)
```

没有接触过linux的小伙伴可能不知道路径中的“.”是什么意思，他在linux命令中代表“当前目录”，./include的意思就是你当前目录下的include文件夹，例如我们的视觉相机文件结构：
    
    hnurm_camera
        |——include
        |——launch
        |——lib
        |——params
        |——src
        |——CMakeLists.txt
        |——package.xml
        |——README.md

在CMakeLists中写入“./include”，其实际便是“hnurm_camera/include”。

## 3. cmake的使用
写完了CMakeLists.txt文件后并非万事大吉了，我们得让计算机知道我们这个文件的意义和目的，也就是我们要运行它。

打开终端（terminal），总共是四个步骤：

### (1).创建build文件夹

```
#跳到你的工作目录下
cd MyAwesomeApp

#在此目录下创建名为build的子目录
mkdir build

#跳到此build目录下
cd build
```

其目的是为了整合收放构建过程中所生成的文件，其编译过程会生成CMake目录CMakeFiles/，Cmake缓存文件CMakeCache.txt，构建文件Makefile，可执行文件my_app等，如果不统一放在build文件夹下，其便会散落在源代码目录中，看上去不美观，维护起来也极为麻烦。

### (2).运行cmake配置项目及生成

```
cmake ..
```
的意思是在build目录下运行cmake并指定CMakeLists所在的上级目录。如果一切顺利，你会在build目录下看到生成的文件（如Makefile和CMakeCache.txt）。

### (3)运行构建工具进行编译

```
make
```

其目的是使用上一步生成的构建文件来实际编译你的源代码。运行后，你会看到编译输出。如果成功，就会在build目录下生成一个名为my_app的可执行文件（这是在 add_executable(my_app ...) 中指定的名字）。

### (4)运行生成的可执行文件
```
./my_app
```
这个“my_app”也可以叫其他名字，取决于你在CMakeLists中命名成什么样子。

至此，cmake的基础运用便告一段落。





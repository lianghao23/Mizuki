---
title: Package.xml
published: 2025-09-09
tags: [Markdown, Blogging]
category: 视觉组
draft: true
---


# Package.xml
## 1. package.xml是什么

如果说CMakeLists.txt是告诉计算机如何构建你的代码的“构建清单”，那么package.xml就是告诉人类和其他程序，你的软件包是什么、依赖谁的“软件包说明书”。

它是一个标准的XML格式文件，是ROS生态中必不可少的一部分。每个ROS软件包都必须包含一个 package.xml文件，它位于包的根目录下，和 CMakeLists.txt是邻居。

没有它行不行？ 在ROS里不行。ROS 的工具（如命令colcon build）就依赖这个文件，一用于计算机识别这是一个ROS包，二用于查找该包依赖的其他的ROS包（比如你用了别人的代码，或者用了ROS提供的功能），三用于提供包的元信息（Meta-Information），比如名字、版本、作者、许可证。这些信息对于软件的分发和共享至关重要。听不懂不要紧，以后会接触到。

## 2. package.xml的写法

package.xml的内容虽然看起来有点复杂，但核心就是“声明属性”和“声明依赖”。

一个最基本的package.xml长这样：
```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<!-- 1. 声明包的基本信息 -->
<package format="3">
  <name>my_awesome_package</name>
  <version>0.0.0</version>
  <description>这是一个超级厉害的ROS包，用来做视觉识别。</description>
  <maintainer email="your-email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <!-- 2. 声明构建依赖 -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- 3. 声明运行依赖 -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

  <!-- 4. 导出其他信息，例如需要被发现的插件 -->
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## 3. package.xml 和 CMakeLists.txt 的关系

可以把它们看作一对好兄弟，分工合作：

    package.xml (说明书): 对外。负责社交，告诉外界“我是谁，我需要哪些依赖”。

    CMakeLists.txt (构建清单): 对内。负责干活，指挥编译器“怎么把我自己的代码和外来给的零件组装成产品”。

当你运行 colcon build 命令时，构建系统会：

    1. 先读取所有包的package.xml，解析出一个依赖关系图。搞清楚谁依赖谁，决定构建顺序。

    2. 再按照依赖顺序，依次调用每个包里的CMakeLists.txt来真正执行编译。


## 4. 如何使用（它不需要单独运行）

package.xml 文件不需要你单独去运行它。它的使用是自动的，融合在 ROS 的构建流程中。

    1. 创建包时：当你使用ros2 pkg create ...命令创建一个新包时，ROS 会自动为你生成一个模板 package.xml，你只需要去修改它即可。

    2. 构建时：当你在其工作空间的根目录下运行colcon build时，工具会自动找到所有包的package.xml并处理它们。

    3. 查询信息时：你可以用 ROS 工具命令来查看包的信息，这些信息就来自 package.xml。
抽象成命令就是：
```
# 跳到你的工作空间源码目录下
cd ~/ros2_ws/src

# 创建一个新的包，会自动生成package.xml
ros2 pkg create --build-type ament_cmake my_awesome_package

# 构建后，查看某个包的信息
ros2 pkg list # 列出所有包
ros2 pkg xml my_awesome_package # 输出指定包的package.xml内容
```
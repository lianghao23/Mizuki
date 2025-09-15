---
title: 视觉组培训：二阶段-linux基础命令
published: 2025-09-03
tags: [Linux]
category: 视觉组
draft: false
---

# 一. 什么是Linux？

Linux 是一个开源的、类 Unix 的操作系统内核。由于它常与各种开源软件打包在一起分发，所以通常我们所说的 "Linux" 指的是基于 Linux 内核的完整操作系统，如 Ubuntu、CentOS、Debian 等，我们所用的就是ubuntu。

# 二. 为什么要学linux命令？

在图形化界面（GUI）中，你可以通过点击图标和菜单来操作电脑。而在Linux中，命令行界面（CLI）是一个更强大、更高效的工具。其优点主要体现为：

    高效：一个命令可以完成图形界面需要多次点击才能完成的操作
    
    远程管理：通过 SSH 远程连接服务器时，命令行是唯一的选择
    
    自动化：可以将一系列命令写成脚本，自动完成复杂任务
    
    资源占用少：对系统资源需求极低，特别适合服务器环境

# 三. Linux命令基础结构

类似英语遵循主谓宾结构一样，Linux也遵循以下结构：

```
    command [options] [arguments]
```

    command是要执行的命令本身
    
    options是修改命令行为的选项
    
    arguments是命令操作的对象

听不懂没关系，后面会有很多例子。

# 四. 核心命令

1. 文件系统操作命令

cd：切换目录

```
cd /path/to/directory  # 切换到绝对路径
cd ..                 # 返回上一级目录
cd ~                  # 回到家目录
cd -                  # 返回上一个所在目录
```

pwd：查看目前所在位置

```
pwd
```

ls：查看目录内容

```
ls        # 列出当前目录下的文件和目录
ls -l     # 以详细列表形式显示（显示权限、大小、时间等信息）
ls -a     # 显示所有文件（包括隐藏文件，以.开头的文件）
ls -la    # 详细列表显示所有文件（最常用）
```

mkdir：创建目录

```
mkdir new_folder      # 创建名为new_folder的目录
mkdir -p parent/child # 创建多级目录（如果parent不存在也会被创建）
```

touch：创建文件

```
touch file.cpp # 创建一个名为file.cpp的空文件
```

cp：复制文件/目录

```
cp file1.txt file2.txt    # 复制file1.txt为file2.txt
cp file.txt ~/Documents/  # 复制文件到指定目录
cp -r dir1 dir2           # 递归复制目录（包括子目录和文件）
```

mv：移动/重命名

```
mv old.txt new.txt        # 重命名文件
mv file.txt ~/Documents/  # 移动文件到指定目录
```

rm：删除文件/目录

```
rm file.txt           # 删除文件
rm -r directory       # 递归删除目录及其内容
rm -f file.txt        # 强制删除，不提示确认
# ⚠️ 谨慎使用 rm -rf /* ！这是极其危险的命令！
```

2. 文件内容操作

cat：查看文件内容

```
cat file.txt      # 显示文件的全部内容
```

grep：查找文件内容

```
grep "pattern" file.txt      # 在文件中搜索包含"pattern"的行
grep -r "pattern" directory/ # 递归在目录中搜索
grep -i "pattern" file.txt   # 忽略大小写搜索
```

3. 系统管理命令

ps：进程查看

```
ps aux        # 查看系统所有进程的详细信息
```

kill：结束进程

```
kill 1234              # 终止PID为1234的进程
kill -9 1234           # 强制终止进程
```

df：查看磁盘使用

```
df -h # 以易读格式显示磁盘空间使用情况（GB/MB）
```

du：查看目录大小

```
du -sh directory/      # 查看目录的总大小
du -h --max-depth=1    # 查看当前目录下各子目录的大小
```

top或htop：查看系统资源

```
top           # 动态显示系统进程和资源使用情况（类似任务管理器）
# 在top中：按q退出，按M按内存排序，按P按CPU排序
```

4. 权限管理

chmod：修改文件权限

```
chmod 755 script.sh    # 设置文件权限为rwxr-xr-x
chmod +x script.sh     # 为文件添加可执行权限
```

chown：修改文件所有者

```
chown user:group file.txt    # 修改文件的所有者和所属组
```

5. 压缩与解压

tar：压缩

```
tar -czvf archive.tar.gz directory/    # 创建gzip压缩包
tar -cjvf archive.tar.bz2 directory/   # 创建bzip2压缩包
```

tar：解压

```
tar -xzvf archive.tar.gz     # 解压gzip压缩包
tar -xjvf archive.tar.bz2    # 解压bzip2压缩包
```

# 五. 建议

linux命令远不止这些，但以上命令已经可以满足很多日常需求，但在项目的开发中不乏遇到需要用到较偏较杂的命令，所以linux的是一个需要长期学习，积累经验的东西，祝学习愉快。
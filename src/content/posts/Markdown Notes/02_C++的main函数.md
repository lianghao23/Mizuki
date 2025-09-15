---
title: C++的main函数
published: 2025-09-01
tags: [C++]
category: 黑马程序员
draft: false
---

# C++的main函数

# 1. main函数

① main函数是一个程序的入口

② 每个程序都必须有这么一个函数，并且有且只有一个。


```python
//下面11行代码的含义就是在屏幕中输出一个 Hello World

#include<iostream>
using namespace std;

int main() {

    cout << "hello C++" << endl;  //屏幕显示“hello C++”
    system("pause");              //按任意键继续，退出程序

    return 0;

}
```

运行结果：  
 - hello C++  
 - 请按任意键继续. . .

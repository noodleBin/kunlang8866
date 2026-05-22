# Century 系统架构代码分析
以localization模块为例，其余模块节点类似。

## 代码分析
主要文件： main.cc，century_app.cc，century_app.h。
### mian.c文件
文件中只有一行代码：CENTURY_MAIN(century::localization::Localization) ，使用宏CENTURY_MAIN，开启了Localization节点，这里localization节点开始运行，这里的的节点与ros中的node概念一致，相当于一个进程。

### CENTURY_MAIN宏解析：
 * CENTURY_MAIN宏定义位于"modules/common/century_app.h"文件。
 * 设置log和SIGINT信号处理程序，收到信号，关闭本节点。
 * 创建模块类对象，设置节点名字，调用基类（CenturyApp）的Spin()函数。

### CenturyApp类：
 * Spin()函数属于类CenturyApp public成员函数，类CenturyApp是所有模块类的基类。
 * Public成员有name()函数，用于获取模块名字。Spin()函数用于初始化、启动、当ros关闭时关闭模块节点。还有一个析构函数。
 * Protected成员都是vritual接口，子类都会重写，在Spin()函数中调用，其实现实在具体各个模块内部。Init()函数完成加载模块的配置文件，创建订阅话题。Start()函数：注册回调函数，回调函数负责节点核心任务，通常由上游话题或者timer触发。Stop()函数，结束节点，正常时不会执行到。ReportModuleStatus()返回模块状态。century_app_sigint_handler()函数，信号处理函数。
 * Spin()函数使用init()，start()和stop()函数完成模块节点的实现。此函数一般不会被重写，也就是使用CenturyApp的实现。

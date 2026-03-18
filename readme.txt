执行后完整目录结构

 ZLENS_DC/
 ├── CLAUDE.md                              # AI 自动加载的项目规则（命名规范精简版）
 ├── CMakeLists.txt                         # 顶层 CMake：双目标构建入口
 │
 ├── cmake/                                 # 🔧 构建配置
 │   ├── stm32f103rc.cmake                  #   ARM 交叉编译工具链文件（编译器路径、MCU flags）
 │   └── stm32f1xx.ld                       #   链接脚本（Flash 256KB + SRAM 48KB 内存布局）
 │
 ├── Core/                                  # 🏗️  CubeMX 生成的 HAL 初始化代码（由 STM32CubeMX工具生成，不手写）
 │   ├── Inc/                               #   头文件
 │   │   ├── main.h                         #     主函数声明、GPIO 别名定义
 │   │   ├── stm32f1xx_hal_conf.h           #     HAL 模块启用开关
 │   │   └── FreeRTOSConfig.h               #     FreeRTOS 内核配置（优先级、栈大小、钩子）
 │   ├── Src/                               #   源文件
 │   │   ├── main.cpp                       #     系统初始化入口 + FreeRTOS 任务创建
 │   │   ├── stm32f1xx_it.cpp               #     中断向量表（ISR 入口）
 │   │   └── system_stm32f1xx.c             #     系统时钟配置（CMSIS 标准）
 │   └── Startup/
 │       └── startup_stm32f103rctx.s        #   启动汇编（向量表 + 栈初始化 + Reset_Handler）
 │
 ├── Drivers/                               # 📦 第三方库（CubeMX 自动拉入，不修改）
 │   ├── STM32F1xx_HAL_Driver/              #   ST HAL 库（GPIO、TIM、SPI、UART、ADC、DAC、Flash）
 │   ├── CMSIS/                             #   ARM CMSIS 核心（Cortex-M3 寄存器定义）
 │   └── Third_Party/
 │       └── FreeRTOS/                      #   FreeRTOS 内核源码（任务、队列、信号量）
 │
 ├── App/                                   # 🎯 应用层 —— 核心业务代码（C++，我们编写的主要代码）
 │   ├── Inc/                               #   头文件（接口定义）
 │   │   ├── motor_ctrl.hpp                 #     电机控制：梯形加减速状态机、PWM 驱动
 │   │   ├── encoder.hpp                    #     编码器：TIM8 编码器模式、32位扩展计数
 │   │   ├── stall_detect.hpp               #     堵转检测：ADC 电流监测、过流保护
 │   │   ├── zoom_table.hpp                 #     倍率映射表：倍率↔编码器位置查表、Flash 存储
 │   │   ├── comm_protocol.hpp              #     通信协议：UART 帧解析、CRC16、命令处理
 │   │   ├── fram_storage.hpp               #     FRAM 存储：SPI 参数读写、双备份、紧急保存
 │   │   ├── power_monitor.hpp              #     电源监测：ADC 看门狗掉电检测
 │   │   └── system_manager.hpp             #     系统管理：状态机、自检、归零标定
 │   └── Src/                               #   源文件（实现）
 │       ├── motor_ctrl.cpp                 #     （对应同名 .hpp 的实现）
 │       ├── encoder.cpp
 │       ├── stall_detect.cpp
 │       ├── zoom_table.cpp
 │       ├── comm_protocol.cpp
 │       ├── fram_storage.cpp
 │       ├── power_monitor.cpp
 │       └── system_manager.cpp
 │
 ├── Tasks/                                 # ⚡ FreeRTOS 任务入口（调用 App 层模块）
 │   ├── Inc/
 │   │   ├── motor_task.hpp                 #     MotorTask：1ms 周期，电机控制+编码器+堵转检测
 │   │   ├── comm_task.hpp                  #     CommTask：事件触发，UART 帧接收+命令分发
 │   │   ├── storage_task.hpp               #     StorageTask：事件触发，FRAM 定期保存+恢复
 │   │   └── monitor_task.hpp               #     MonitorTask：系统状态机+自检+看门狗
 │   └── Src/
 │       ├── motor_task.cpp
 │       ├── comm_task.cpp
 │       ├── storage_task.cpp
 │       └── monitor_task.cpp
 │
 ├── Test/                                  # 🧪 单元测试（主机端 x86，Google Test）
 │   ├── CMakeLists.txt                     #   测试构建配置：链接 gtest + mock 层
 │   ├── mocks/                             #   HAL Mock 层 —— 隔离硬件依赖
 │   │   ├── stm32f1xx_hal.h                #     HAL 类型存根（GPIO_TypeDef 等最小定义）
 │   │   ├── mock_hal.hpp                   #     Mock 函数声明 + 调用日志结构
 │   │   ├── mock_hal.cpp                   #     Mock 函数实现（记录调用、返回预设值）
 │   │   ├── mock_freertos.hpp              #     FreeRTOS API mock 声明
 │   │   └── mock_freertos.cpp              #     FreeRTOS API mock 实现
 │   ├── test_encoder.cpp                   #   编码器测试：位置计算、溢出、Z 脉冲
 │   ├── test_motor_ctrl.cpp                #   电机控制测试：状态机转换、加减速、到位
 │   ├── test_stall_detect.cpp              #   堵转检测测试：阈值、屏蔽窗、过流
 │   ├── test_zoom_table.cpp                #   倍率表测试：查找、校验、角度换算
 │   ├── test_comm_protocol.cpp             #   通信协议测试：CRC16、帧解析、所有命令
 │   ├── test_fram_storage.cpp              #   FRAM 测试：序列化、CRC、双备份
 │   ├── test_power_monitor.cpp             #   电源监测测试：掉电阈值
 │   └── test_system_manager.cpp            #   系统管理测试：状态机、自检流程
 │
 └── docs/                                  # 📝 项目文档
     ├── coding-style.md                    #   完整命名规范（含示例、理由、FAQ）
     ├── ref/                               #   参考资料（芯片手册、协议文档、原理图）
     │   ├── 电动镜头通信协议v2.5.docx
     │   ├── 电动镜头通信协议v2.5_Release.pdf
     │   ├── A4950.pdf                      #     电机驱动芯片手册
     │   ├── stm32f103rc.pdf                #     MCU 数据手册
     │   ├── CMS16-2309P105PE-XX规格书.pdf  #     FRAM 规格书
     │   ├── C2842352_运算放大器_LM321A-TR_规格书_WJ19116.PDF
     │   └── ZOOM-LENS-V1_0-162.pdf         #     镜头机械图
     ├── req/
     │   └── 项目需求报告.md                #   项目需求文档
     └── superpowers/
         ├── specs/
         │   └── 2026-03-16-zlens-dc-design.md    # 系统设计文档
         └── plans/
             └── 2026-03-16-zlens-dc-implementation.md  # 详细实施计划（13 个任务）
 
 
             
 ---
 各层职责划分

 ┌──────────┬────────────────────────┬──────────┬─────────────────────────────────────────────────┐
 │    层    │          目录          │   语言   │                      职责                       │
 ├──────────┼────────────────────────┼──────────┼─────────────────────────────────────────────────┤
 │ 构建     │ cmake/, CMakeLists.txt │ CMake    │ 双目标构建：ARM 交叉编译 + x86 主机测试         │
 ├──────────┼────────────────────────┼──────────┼─────────────────────────────────────────────────┤
 │ 硬件抽象 │ Core/, Drivers/        │ C        │ CubeMX 生成，提供 HAL API，不手动修改           │
 ├──────────┼────────────────────────┼──────────┼─────────────────────────────────────────────────┤
 │ 应用逻辑 │ App/                   │ C++      │ 8 个核心模块，纯业务逻辑，通过 HAL 接口操作硬件 │
 ├──────────┼────────────────────────┼──────────┼─────────────────────────────────────────────────┤
 │ 任务调度 │ Tasks/                 │ C++      │ 4 个 FreeRTOS 任务入口，组合调用 App 层模块     │
 ├──────────┼────────────────────────┼──────────┼─────────────────────────────────────────────────┤
 │ 测试     │ Test/                  │ C++      │ 主机端 gtest 单元测试 + HAL Mock 层             │
 ├──────────┼────────────────────────┼──────────┼─────────────────────────────────────────────────┤
 │ 文档     │ docs/                  │ Markdown │ 规范、需求、设计、计划、参考资料                │
 └──────────┴────────────────────────┴──────────┴─────────────────────────────────────────────────┘

 ---
 实施顺序（6 个 Chunk，13 个 Task）

 ┌───────┬─────────────────────┬─────────────────────────────────────────────────────────────────┐
 │ Chunk │        内容         │                            产出文件                             │
 ├───────┼─────────────────────┼─────────────────────────────────────────────────────────────────┤
 │ 1     │ 构建系统 +          │ CMakeLists.txt, cmake/*, Test/CMakeLists.txt, Test/mocks/*      │
 │       │ 测试基础设施        │                                                                 │
 ├───────┼─────────────────────┼─────────────────────────────────────────────────────────────────┤
 │ 2     │ Encoder + MotorCtrl │ App/{Inc,Src}/{encoder,motor_ctrl}.*,                           │
 │       │                     │ Test/test_{encoder,motor_ctrl}.cpp                              │
 ├───────┼─────────────────────┼─────────────────────────────────────────────────────────────────┤
 │ 3     │ StallDetect +       │ App/{Inc,Src}/{stall_detect,zoom_table}.*,                      │
 │       │ ZoomTable           │ Test/test_{stall_detect,zoom_table}.cpp                         │
 ├───────┼─────────────────────┼─────────────────────────────────────────────────────────────────┤
 │ 4     │ CommProtocol +      │ App/{Inc,Src}/{comm_protocol,fram_storage}.*,                   │
 │       │ FramStorage         │ Test/test_{comm_protocol,fram_storage}.cpp                      │
 ├───────┼─────────────────────┼─────────────────────────────────────────────────────────────────┤
 │ 5     │ PowerMonitor +      │ App/{Inc,Src}/{power_monitor,system_manager}.*,                 │
 │       │ SystemManager       │ Test/test_{power_monitor,system_manager}.cpp                    │
 ├───────┼─────────────────────┼─────────────────────────────────────────────────────────────────┤
 │ 6     │ FreeRTOS Tasks +    │ Tasks/{Inc,Src}/*, Core/Src/main.cpp, Core/Inc/*                │
 │       │ main.cpp 集成       │                                                                 │
 └───────┴─────────────────────┴─────────────────────────────────────────────────────────────────┘

 每个 Task 遵循 TDD：先写测试 → 再写实现 → 测试通过 → commit。


 ---
 验证方式

 # 主机端单元测试
 mkdir build-test && cd build-test
 cmake -G Ninja -DBUILD_TESTING=ON ..
 ninja && ctest --output-on-failure

 # ARM 交叉编译（Chunk 6 完成后）
 mkdir build && cd build
 cmake -G Ninja -DCMAKE_TOOLCHAIN_FILE=../cmake/stm32f103rc.cmake ..
 ninja
 # 确认生成 .elf/.bin/.hex，无编译错误



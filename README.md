# air-pilot
A lite-version motion planning project for multicopter.

1 重构工程结构
  - 结构拆解为4块（并改了名字）
    - perception。负责grid map与ESDF等地图的构建
    - planning。分为Astar optimizer 与 manager三大块。分别对应轨迹搜索，后端优化与规划器的启动
    - controller。SO3姿态控制器
    - simulator。负责仿真飞机动力学模型与可视化。


2 修改了package.xml中许多的作者信息


3 几乎修改了所有的文件名，类名（命名风格）。大部分重要函数名字也做了修改。


4 参数从launch中全部移到yaml中

## perception
- 删除许多不必要的函数和判断
- 增加统计算力部分和一些标志位
- 添加了ESDF部分更新部分与flag
- 添加了局部地图RingBuffer更新部分与flag
## planning
- 删掉不必要的gradient decent optimizer.cpp
- 修改fsm中的初始轨迹生成，可以根据指定n个中间点生成初始轨迹，支持指定方向绕障碍
- 封装Control point类，单独弄成一个文件
- 删除了许多的optimize函数，变量等
- 封装了Cost Function类，单独弄成一个文件
- 添加一致性检验部分
## control 
- 不做修改
- Simulator部分也不修改




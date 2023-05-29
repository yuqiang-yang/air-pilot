# air-pilot
A lite-version motion planning project for multicopter.
1 重构工程结构
  - 结构拆解为4块（并改了名字）
    - perception。负责grid map与ESDF等地图的构建
    - planning。分为Astar optimizer 与 main三大块。分别对应轨迹搜索，后端优化与规划器的启动
    - controller。SO3姿态控制器
    - simulator。负责仿真飞机动力学模型与可视化。
- package.xml

## perception

## planning
删掉gradient decent optimizer.cpp
修改fsm中的初始轨迹生成
封装Control point类
删除了多余的optimize函数，变量等
修改了许多类的名字
## control 

Simulator部分不修改
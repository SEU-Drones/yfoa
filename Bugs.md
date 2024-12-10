# yfoa
避障程序。


# hybird a*搜索失败
当搜索的和障碍物最小距离过大时，可能会出现搜索失败的情况。此时，planner进入GEN_NEW_TRAJ状态，进而出现轨迹不连续的现象。

----------The 5th callReplan at time 635.380000000------------
[Replan]  max_vel: 4    max_acc: 2
[Replan]  start state:  8.55379 0.157291  4.26337       7.36075 -0.0927909    1.99306     -3.32842 -0.208904  -1.59216
[Replan]  target state:      23.5401 0.067821   3.6297
[hybird astar]: open set empty, no path!
[hybird astar]: use node num: 1
[hybird astar]: iter num: 1
----------result: 0    duration: 0.071 ms ------------
[FSM]: from REPLAN_TRAJ to GEN_NEW_TRAJ
----------The 5th callReplan at time 635.392000000------------
[Replan]  max_vel: 4    max_acc: 2
[Replan]  start state:  8.88284 0.297126  3.78684      1.72044 -0.111869  0.429073    0 0 0
[Replan]  target state:      23.8773 0.126179  3.41848
[hybird astar]: reach end
[hybird astar]: use node num: 544
[PathNlopt optimal value, f_smoothness, f_distance, f_feasibility ]: 10.9778,    0.00320193,    1.55517,    0
[PathNlopt iteration count]: 508
[Replan]  mapping duration: 2 ms
[Replan]  search duration: 0.504 ms
----------result: 1    duration: 4.711 ms ------------


# Hybird Astar

## 状态转移
控制量：加速度，持续时间
状态量：位置、速度
$$u = [-a_{max}, -0.5a_{max}, 0, 0.5a_{max}, a_{max}]$$
$$t = \tau$$

# 代价

$$ f = \int \| K\mathbf{u} \|^2 dt + \rho T$$


基于庞特利亚金最小值原理 Boundary value problem (BVP): estimateHeuristic()
\url{https://blog.csdn.net/Amos98/article/details/123261802}

https://blog.csdn.net/qq_16775293/article/details/124845417

https://blog.csdn.net/qq_16775293/article/details/124845417


[1] Mueller M W, Hehn M, D’Andrea R. A computationally efficient motion primitive for quadrocopter trajectory generation[J]. IEEE transactions on robotics, 2015, 31(6): 1294-1310.

# yfoa
避障程序。

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

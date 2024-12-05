import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np

def read_trajectory_data(file_path):
    time = []
    heading_angles = []
    positions = []
    velocities = []
    accelerations = []

    with open(file_path, 'r') as file:
        for line in file.readlines():
            data = line.strip().split(',')

            # 处理可能存在逗号代替小数点的情况
            # time_val = data[0].replace(',', '.')
            time_val = data[0]
            try:
                time.append(float(time_val))
            except ValueError as e:
                print(f"无法将 {time_val} 转换为浮点数，错误信息：{e}")
                continue

            heading_angles.append(float(data[1]))
            positions.append([float(data[2]), float(data[3]), float(data[4])])
            velocities.append([float(data[5]), float(data[6]), float(data[7])])
            accelerations.append([float(data[8]), float(data[9]), float(data[10])])

    return time, heading_angles, positions, velocities, accelerations


def plot_trajectory_3d(positions, heading_angles):
    fig_3d = go.Figure()

    positions = np.array(positions)

    # 添加散点图表示轨迹上的点
    fig_3d.add_trace(go.Scatter3d(
        x=positions[:, 0],
        y=positions[:, 1],
        z=positions[:, 2],
        mode='markers',
        marker=dict(
            size=3,
            color='blue',
            opacity=0.8
        )
    ))

    # for i in range(len(positions)):
    #     # 将航向角从度转换为弧度
    #     heading_angle_rad = np.radians(heading_angles[i])

    #     # 根据航向角计算在x和y方向上的单位向量分量
    #     dx = np.cos(heading_angles[i])
    #     dy = np.sin(heading_angle_rad)

    #     # 设置z方向的单位向量分量为0（假设在平面内移动，可根据实际情况修改）
    #     dz = 0

    #     # 添加箭头表示航向
    #     fig_3d.add_trace(go.Cone(
    #         x=[positions[i, 0]],
    #         y=[positions[i, 1]],
    #         z=[positions[i, 2]],
    #         u=[dx],
    #         v=[dy],
    #         w=[dz],
    #         sizemode="absolute",
    #         size=1,
    #         showscale=False,
    #         opacity=0.8
    #     ))
    minxy = min(min(positions[:, 0]), min(positions[:, 1]))
    maxxy = max(max(positions[:, 0]), max(positions[:, 1]))
    fig_3d.update_layout(
        scene=dict(
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z',
            
            # xaxis=dict(range=[minxy, maxxy]),
            # yaxis=dict(range=[minxy, maxxy]),
            # xaxis=dict(range=[min(positions[:, 0]), max(positions[:, 0])]),
            # yaxis=dict(range=[min(positions[:, 1]), max(positions[:, 1])]),
            # zaxis=dict(range=[min(positions[:, 2]), max(positions[:, 2])]),
            aspectratio=dict(x=1, y=1, z=1),
        ),
        title='3D Drone Trajectory'
    )

    return fig_3d


def plot_position_velocity_and_acceleration(time_val, positions, velocities, accelerations):
    # time = np.arange(len(velocities))
    time = time_val

    fig = make_subplots(rows=3, cols=1, shared_xaxes=True)
    
    # 绘制位置曲线
    fig.add_trace(go.Scatter(
        x=time,
        y=[p[0] for p in positions],
        name='X Position',
        # line=dict(color='blue')
    ), row=1, col=1)
    fig.add_trace(go.Scatter(
        x=time,
        y=[p[1] for p in positions],
        name='Y Position',
        # line=dict(color='green')
    ), row=1, col=1)
    fig.add_trace(go.Scatter(
        x=time,
        y=[p[2] for p in positions],
        name='Z Position',
        # line=dict(color='red')
    ), row=1, col=1)

    # 绘制速度曲线
    fig.add_trace(go.Scatter(
        x=time,
        y=[v[0] for v in velocities],
        name='X Velocity',
        # line=dict(color='blue')
    ), row=2, col=1)
    fig.add_trace(go.Scatter(
        x=time,
        y=[v[1] for v in velocities],
        name='Y Velocity',
        # line=dict(color='green')
    ), row=2, col=1)
    fig.add_trace(go.Scatter(
        x=time,
        y=[v[2] for v in velocities],
        name='Z Velocity',
        # line=dict(color='red')
    ), row=2, col=1)
    fig.add_trace(go.Scatter(
        x=time,
        y=[np.linalg.norm(v) for v in velocities],
        name='Resultant Velocity',
        # line=dict(color='red')
    ), row=2, col=1)

    # 绘制加速度曲线
    fig.add_trace(go.Scatter(
        x=time,
        y=[a[0] for a in accelerations],
        name='X Acceleration',
        # line=dict(color='orange')
    ), row=3, col=1)
    fig.add_trace(go.Scatter(
        x=time,
        y=[a[1] for a in accelerations],
        name='Y Acceleration',
        # line=dict(color='purple')
    ), row=3, col=1)
    fig.add_trace(go.Scatter(
        x=time,
        y=[a[2] for a in accelerations],
        name='Z Acceleration',
        # line=dict(color='brown')
    ), row=3, col=1)
    fig.add_trace(go.Scatter(
        x=time,
        y=[np.linalg.norm(a) for a in accelerations],
        name='Resultant Acceleration',
        # line=dict(color='red')
    ), row=3, col=1)
    fig.update_layout(
        title='Position, Velocity and Acceleration over Time',
        # xaxis_title='Time(s)',
        xaxis3_title='Time(s)',  # For the second x-axis (shared)
        yaxis_title='Position (Row 1)',
        yaxis2_title='Velocity (Row 2)',
        yaxis3_title='Acceleration (Row 3)',
        legend=dict(orientation="h", yanchor="bottom", y=1.0, xanchor="right", x=1.02)
    )

    return fig

if __name__ == "__main__":
    file_path = '/home/ly/ws_yfoa/searchtraj.txt'
    time, heading_angles, positions, velocities, accelerations = read_trajectory_data(file_path)
    # 创建3D轨迹图
    fig_3d = plot_trajectory_3d(positions, heading_angles)
    fig_3d.show()
    # 创建速度和加速度的2x1子图
    fig_va = plot_position_velocity_and_acceleration(time, positions, velocities, accelerations)
    fig_va.show()

    file_path = '/home/ly/ws_yfoa/optiamltraj.txt'
    time, heading_angles, positions, velocities, accelerations = read_trajectory_data(file_path)
    # 创建3D轨迹图
    fig_3d = plot_trajectory_3d(positions, heading_angles)
    fig_3d.show()
    # 创建速度和加速度的2x1子图
    fig_va = plot_position_velocity_and_acceleration(time, positions, velocities, accelerations)
    fig_va.show()

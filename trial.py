# # import numpy as np
# # import matplotlib.pyplot as plt
# # from mpl_toolkits.mplot3d import Axes3D

# # # Load ground truth data
# # # Assume ground_truth.txt is a CSV with columns: time, x, y, z
# # ground_truth_data = np.loadtxt(r'C:\Users\athar\OneDrive\Documents\Sem-1\SLAM\Project\codes\MH_01_easy\mav0\state_groundtruth_estimate0\data.csv', delimiter=',', skiprows=1)

# # # Extract x, y, z coordinates
# # x = ground_truth_data[:, 1]
# # print(x[0])
# # y = ground_truth_data[:, 2]
# # z = ground_truth_data[:, 3]

# # # Plotting
# # fig = plt.figure()
# # ax = fig.add_subplot(111, projection='3d')

# # ax.plot(x, y, z, label='Ground Truth Trajectory')
# # ax.set_xlabel('X Position')
# # ax.set_ylabel('Y Position')
# # ax.set_zlabel('Z Position')
# # ax.set_title('EuRoC MAV Ground Truth Trajectory')
# # ax.legend()

# # plt.show()

# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # Load ground truth data
# # Assume ground_truth.txt is a CSV with columns: time, x, y, z
# ground_truth_data = np.loadtxt(r'C:\Users\athar\OneDrive\Documents\Sem-1\SLAM\Project\codes\MH_01_easy\mav0\state_groundtruth_estimate0\data.csv', delimiter=',', skiprows=1)

# # Extract time and position data
# time = ground_truth_data[:, 0]  # Assuming time is in seconds
# x = ground_truth_data[:, 1]
# y = ground_truth_data[:, 2]
# z = ground_truth_data[:, 3]

# # Create subplots
# fig, axs = plt.subplots(3, 1, figsize=(10, 15), sharex=True)
# fig.suptitle('EuRoC MAV Ground Truth Trajectory')

# # Plot X position over time
# axs[0].plot(time, x, 'r-')
# axs[0].set_ylabel('X Position (m)')

# # Plot Y position over time
# axs[1].plot(time, y, 'g-')
# axs[1].set_ylabel('Y Position (m)')

# # Plot Z position over time
# axs[2].plot(time, z, 'b-')
# axs[2].set_ylabel('Z Position (m)')
# axs[2].set_xlabel('Time (s)')

# # Adjust layout and display
# plt.tight_layout()
# plt.show()

# # 3D trajectory plot
# fig = plt.figure(figsize=(10, 8))
# ax = fig.add_subplot(111, projection='3d')

# scatter = ax.scatter(x, y, z, c=time, cmap='viridis')
# ax.set_xlabel('X Position (m)')
# ax.set_ylabel('Y Position (m)')
# ax.set_zlabel('Z Position (m)')
# ax.set_title('EuRoC MAV Ground Truth 3D Trajectory')

# # Add a color bar to show time progression
# cbar = fig.colorbar(scatter, ax=ax, label='Time (s)')

# plt.show()

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

# Load ground truth data
# Assume ground_truth.csv is in the format: timestamp, p_x, p_y, p_z, q_w, q_x, q_y, q_z
data = np.loadtxt(r'C:\Users\athar\OneDrive\Documents\Sem-1\SLAM\Project\codes\MH_01_easy\mav0\state_groundtruth_estimate0\data.csv', delimiter=',', skiprows=1)

# Extract time, position, and quaternion data
time = data[:, 0]
position = data[:, 1:4]
quaternions = data[:, 4:8]

# Convert quaternions to rotation matrices
rotations = R.from_quat(quaternions)
rotation_matrices = rotations.as_matrix()

print(np.shape(rotation_matrices))

# Create subplots
fig, axs = plt.subplots(3, 2, figsize=(15, 15), sharex=True)
fig.suptitle('EuRoC MAV Ground Truth Trajectory and Rotation')

# Plot position
for i, axis in enumerate(['X', 'Y', 'Z']):
    axs[i, 0].plot(time, position[:, i])
    axs[i, 0].set_ylabel(f'{axis} Position (m)')

# Plot rotation matrix elements
for i in range(3):
    for j in range(3):
        axs[i, 1].plot(time, rotation_matrices[:, i, j], label=f'R{i+1}{j+1}')
    axs[i, 1].set_ylabel(f'Rotation Matrix Row {i+1}')
    axs[i, 1].legend()

axs[2, 0].set_xlabel('Time (s)')
axs[2, 1].set_xlabel('Time (s)')

# Adjust layout and display
plt.tight_layout()
plt.show()

# 3D trajectory plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

scatter = ax.scatter(position[:, 0], position[:, 1], position[:, 2], c=time, cmap='viridis')
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('EuRoC MAV Ground Truth 3D Trajectory')

# Add a color bar to show time progression
cbar = fig.colorbar(scatter, ax=ax, label='Time (s)')

plt.show()
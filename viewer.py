import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from multiprocessing import Queue, Process
from scipy.spatial.transform import Rotation as R

class Viewer:
    def __init__(self):
        self.image_queue = Queue()
        self.pose_queue = Queue()
        self.gt_pose_queue = Queue()
        self.gt_timestamp_queue = Queue()
        self.timestamp_queue = Queue()

        # Load ground truth data
        # Assume ground_truth.txt is a CSV with columns: time, x, y, z
        self.ground_truth_data = np.loadtxt(r'C:\Users\athar\OneDrive\Documents\Sem-1\SLAM\Project\codes\MH_01_easy\mav0\state_groundtruth_estimate0\data.csv', delimiter=',', skiprows=1)

        # Extract x, y, z coordinates
        self.x_gt = self.ground_truth_data[:, 1]
        self.y_gt = self.ground_truth_data[:, 2]
        self.z_gt = self.ground_truth_data[:, 3]
        self.time_gt = self.ground_truth_data[:, 0]/(1e9)  # Assuming time is in seconds
        self.quaternions = self.ground_truth_data[:, 4:8]

        # Convert quaternions to rotation matrices
        rotations = R.from_quat(self.quaternions)
        self.rotation_matrices = rotations.as_matrix()

        # print(self.x_gt[0])

        self.view_thread = Process(target=self.view)
        self.view_thread.start()

    def update_pose(self, pose,timestamp):
        """
        Update the pose in the queue.
        Pose should be a 4x4 transformation matrix.
        """
        if pose is None or timestamp is None:
            return
        self.pose_queue.put(pose)
        timestamp = timestamp # Already in seconds
        self.timestamp_queue.put(timestamp)

    def update_timestamp(self,timestamp):
        if timestamp is None:
            print('timestamp is none')
            return
        
        timestamp = timestamp # Already in seconds
        self.timestamp_queue.put(timestamp)
        
    def update_gt_pose(self,pose,timestamp):

        # Pose is (3,) vector
        if pose is None:
            print('gt pose is none')
            return    
        self.gt_pose_queue.put(pose)
        self.gt_timestamp_queue.put(timestamp)

    def update_image(self, image):
        """
        Update the image in the queue.
        Converts grayscale to RGB if necessary.
        """
        if image is None:
            return
        elif image.ndim == 2:  # Grayscale image
            image = np.repeat(image[..., np.newaxis], 3, axis=2)
        self.image_queue.put(image)

    def geodesic_distance(R1, R2):      # A metric to measure the angular error between 2 rotation matrices
        
        """
        Compute the geodesic distance (angular error) between two rotation matrices.

        Parameters:
            R1 (np.ndarray): First rotation matrix (3x3).
            R2 (np.ndarray): Second rotation matrix (3x3).

        Returns:
            float: Geodesic distance in radians.
        """

        # Compute the relative rotation matrix
        R_rel = np.dot(R1.T, R2)
        
        # Compute the trace of the relative rotation matrix
        trace = np.trace(R_rel)
        
        # Clamp trace to avoid numerical issues
        trace = np.clip(trace, -1.0, 3.0)
        
        # Compute the geodesic distance
        theta = np.arccos((trace - 1) / 2)
        
        return theta

    def view(self):
        """
        The visualization loop for Matplotlib and OpenCV.
        """

        print('in visualization loop')

        # Matplotlib for 3D visualization
        plt.ion()  # Interactive mode on

        # fig1 = plt.figure(1)
        # ax = fig1.add_subplot(111,projection='3d')
        # ax.set_title("3D Trajectory")
        # ax.set_xlabel("X")
        # ax.set_ylabel("Y")
        # ax.set_zlabel("Z")

        # Create an empty line object for the trajectory
        # line, = ax.plot([], [], [], label="VIO Position Estimate")
        # ax.plot(self.x_gt, self.y_gt, self.z_gt, label="Ground truth")

        gt_vals = np.column_stack((self.x_gt,self.y_gt,self.z_gt))

        # cam = ax.scatter([],[],[], c='r', label="Camera")

        fig2 = plt.figure(2)
        ax1 = fig2.add_subplot(311)
        ax1.set_title("GT vs VIO vs time")
        ax1.set_xlabel("time(s)")
        ax1.set_ylabel("x (m)")

        line1, = ax1.plot([], [], label="X- error vs time")
        
        ax2 = fig2.add_subplot(312)
        # ax2.set_title("Y error vs time")
        ax2.set_xlabel("time(s)")
        ax2.set_ylabel("y (m)")

        line2, = ax2.plot([], [], label="Y- error vs time")

        ax3 = fig2.add_subplot(313)
        # ax3.set_title("Z error vs time")
        ax3.set_xlabel("time(s)")
        ax3.set_ylabel("z (m)") 

        line3, = ax3.plot([], [], label="Z- error vs time")   

        # fig3 = plt.figure(3)
        # ax4 = fig3.add_subplot(311)
        # ax4.set_title("Rotation matrix components vs time")
        # ax4.set_xlabel("time (s)")
        # ax4.set_ylabel("Row 1 (rad)")

        # ax5 = fig3.add_subplot(312)
        # # ax5.set_title("Rotation matrix components vs time")
        # ax5.set_xlabel("time (s)")
        # ax5.set_ylabel("Row 2 (rad)")

        # ax6 = fig3.add_subplot(313)
        # # ax6.set_title("Rotation matrix components vs time")
        # ax6.set_xlabel("time (s)")
        # ax6.set_ylabel("Row 3 (rad)")

        # fig4 = plt.figure(4)
        # ax7 = fig4.add_subplot(311)
        # ax7.set_title("RMS x error vs time")
        # ax7.set_xlabel("time (s)")
        # ax7.set_ylabel("RMS x (m)")

        # ax7.set_ylim(0 , 1)  # Set y-axis limits from -0.5 to 0.5

        # line7, = ax7.plot([], [], label="RMS-x vs time")
        # ax7.plot(self.time_gt,0*self.z_gt,'--')

        # ax9 = fig4.add_subplot(313)
        # ax9.set_title("RMS z error vs time")
        # ax9.set_xlabel("time (s)")
        # ax9.set_ylabel("RMS z (m)")

        # ax9.set_ylim(0 , 1)  # Set y-axis limits from -0.5 to 0.5

        # line9, = ax9.plot([], [], label="RMS-z vs time")
        # ax9.plot(self.time_gt,0*self.z_gt,'--')

        # ax8 = fig4.add_subplot(312)
        # ax8.set_title("RMS y error vs time")
        # ax8.set_xlabel("time (s)")
        # ax8.set_ylabel("RMS y (m)")

        # ax8.set_ylim(0 , 1)  # Set y-axis limits from -0.5 to 0.5

        # line8, = ax8.plot([], [], label="RMS-y vs time")
        # ax8.plot(self.time_gt,0*self.z_gt,'--')

        # OpenCV for image display
        cv2.namedWindow("Image Viewer", cv2.WINDOW_NORMAL)

        trajectory_points = []
        reference_points = []
        timestamps = []
        timestamps_gt = []
        camera_pose = None
        rotation_matix_vals = np.empty((0, 3, 3))
        rms_x_error = []
        rms_y_error = []
        rms_z_error = []
        timestamp = 0   # Just to prevent an indexed before assignment error
    
        ax1.plot(self.time_gt,self.x_gt, label="Ground truth x vs time")  
        ax2.plot(self.time_gt,self.y_gt, label="Ground truth y vs time")
        ax3.plot(self.time_gt,self.z_gt, label="Ground truth z vs time")

        # axes = [ax4 , ax5 , ax6]

        # line_r11, = ax4.plot([], [],'r--')
        # line_r12, = ax4.plot([], [],'g--')
        # line_r13, = ax4.plot([], [],'b--')

        # line_r21, = ax5.plot([], [],'r--')
        # line_r22, = ax5.plot([], [],'g--')
        # line_r23, = ax5.plot([], [],'b--')

        # line_r31, = ax6.plot([], [],'r--')
        # line_r32, = ax6.plot([], [],'g--')
        # line_r33, = ax6.plot([], [],'b--')

        # lines_array = np.array([[line_r11 , line_r12 , line_r13] , 
        #                         [line_r21 , line_r22 , line_r23] , 
        #                         [line_r31 , line_r32 , line_r33]])

        # colors = ['r','g', 'b']

        # Plot rotation matrix elements
        # for i in range(3):
        #     for j in range(3):
        #         # print(self.rotation_matrices[:, i, j].shape)
        #         # print(self.time_gt.shape)
        #         axes[i].plot(self.time_gt, self.rotation_matrices[:, i, j],colors[j], label=f'R{i+1}{j+1}')

        # ax4.set_ylabel(f'Rotation Matrix Row {1}')
        # ax4.legend()
        # ax4.set_ylim(-1.5 , 1.5)
        # ax5.set_ylabel(f'Rotation Matrix Row {2}')
        # ax5.legend()
        # ax5.set_ylim(-1.5 , 1.5)
        # ax6.set_ylabel(f'Rotation Matrix Row {3}')
        # ax6.legend()
        # ax6.set_ylim(-1.5 , 1.5)  # Set y-axis limits from -0.5 to 0.5        

        while True:
            updated = False

            # Update pose and trajectory
            if not self.pose_queue.empty() and not self.gt_pose_queue.empty() :
                # print("updating pose visualization")
                
                # print("out of timestamp loop")

                while not self.pose_queue.empty() and not self.gt_pose_queue.empty():
                    # print('pose queue', self.pose_queue)

                    pose = self.pose_queue.get()

                    pose_array = pose.matrix()
                    position = pose_array[:3 , 3]

                    orientation = pose_array[:3,:3]

                    gt_pose = self.gt_pose_queue.get()
                    gt_timestamp = self.gt_timestamp_queue.get()

                    reference_points.append(gt_pose)
                    timestamps_gt.append(gt_timestamp)

                    # # print("gt_position" , gt_pose)
                    # gt_position = gt_pose
                    trajectory_points.append(position)  # Extract translation, pose gives transformation matrix

                    rotation_matix_vals = np.append(rotation_matix_vals, [orientation], axis=0)

                # while not self.timestamp_queue.empty():

                    timestamp = self.timestamp_queue.get()
                
                    timestamps.append(timestamp)
                
                # reference_points.append(gt_position)
                
                # print("shape of trajectory points", np.shape(np.array(trajectory_points)))

                updated = True

                index_start = np.where(self.time_gt >= timestamps[0])   # Gives first value of returned array
                index_start = index_start[0][0]

                index_end = np.where(self.time_gt <= timestamps[-1])   # Gives first value of returned array
                index_end = index_end[0][-1]

                stripped_time = self.time_gt[index_start:]
                stripped_gt_vals = gt_vals[index_start:, :]

                trajectory_array = np.array(trajectory_points)
                    # reference_array = np.array(reference_points)
                    # ax.plot(trajectory_array[:, 0], trajectory_array[:, 1], trajectory_array[:, 2], label="VIO position estimate")
                    # ax.plot(self.x_gt, self.y_gt, self.z_gt, label="Ground truth")

                time_array = np.array(timestamps)

                reference_array = np.array(reference_points)

                # print('time array', time_array)

                # print('index', index)
                # print('timestamps shape', time_array.shape)
                # print('traj array shape dim 0', trajectory_array[:, 1].shape)
                # print(gt_vals.shape)
                # print(trajectory_array.shape)
                
                # RMSE_x = np.sqrt(np.mean((trajectory_array[:,0] - stripped_gt_vals[:trajectory_array.shape[0], 0])**2))
                # RMSE_y = np.sqrt(np.mean((trajectory_array[:,1] - stripped_gt_vals[:trajectory_array.shape[0], 1])**2))
                # RMSE_z = np.sqrt(np.mean((trajectory_array[:,2] - stripped_gt_vals[:trajectory_array.shape[0], 2])**2))

                RMSE_x = np.sqrt(np.mean((trajectory_array[:,0] - reference_array[:,0])**2))
                RMSE_y = np.sqrt(np.mean((trajectory_array[:,1] - reference_array[:,1])**2))
                RMSE_z = np.sqrt(np.mean((trajectory_array[:,2] - reference_array[:,2])**2))

                # RMSE_x = np.mean(trajectory_array[:,0] - stripped_gt_vals[index_start:index_start+trajectory_array.shape[0], 0])
                # RMSE_y = np.mean(trajectory_array[:,1] - stripped_gt_vals[index_start:index_start+trajectory_array.shape[0], 1])
                # RMSE_z = np.mean(trajectory_array[:,2] - stripped_gt_vals[index_start:index_start+trajectory_array.shape[0], 2])

                print("RMSE_x:", RMSE_x)
                print("RMSE_y:", RMSE_y)
                print("RMSE_z:", RMSE_z)

                rms_x_error.append(RMSE_x)

                rms_y_error.append(RMSE_y)

                rms_z_error.append(RMSE_z)

                rms_x_error_array = np.array(rms_x_error)
                rms_y_error_array = np.array(rms_y_error)
                rms_z_error_array = np.array(rms_z_error)

                # print('gt indices length', index_end - index_start)
                print('gt array original length',gt_vals.shape)
                print('gt array stripped length', stripped_gt_vals.shape)
                print('traj indices', len(trajectory_points))
                print('timestamp length: ', len(timestamps) )
                print('length of gt queue', len(reference_points))
                print('length of gt timestamp  values', len(timestamps_gt))
                # print('check if the 2 timestamp arrays are equal: ', timestamps == timestamps_gt)
                # print('difference in array values: ', np.array(timestamps_gt) - np.array(timestamps))

                # rotation_error = self.geodesic_distance(camera_orientation,self.rotation_matrices)

            # Update image
            if not self.image_queue.empty():
                while not self.image_queue.empty():
                    img = self.image_queue.get()
                # img = cv2.resize(img, (900, 90))
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                cv2.imshow("Image Viewer", img)

            # Update Matplotlib visualization
            if updated:
                # ax.cla()
                # ax2.cla()
                # ax1.cla()
                # ax3.cla()
                # ax4.cla()

                if len(trajectory_points) > 0:

                    

                    # print('rms x shape', np.array(rms_x_error).shape)

                    # print('RMS X ERROR SHAPE',np.array(rms_x_error).shape)

                    # line.set_data(trajectory_array[:, 0], trajectory_array[:, 1])
                    # line.set_3d_properties(trajectory_array[:, 2])

                    line1.set_data(time_array, trajectory_array[:, 0])

                    line2.set_data(time_array, trajectory_array[:, 1])

                    line3.set_data(time_array, trajectory_array[:, 2])

                    # print('length of rmse array', rms_x_error_array.shape[0])
                    # print('length of time array', time_array.shape[0])
                    # line7.set_data(time_array, rms_x_error_array)
                    # line8.set_data(time_array, rms_y_error_array)
                    # line9.set_data(time_array, rms_z_error_array)

                    # for i in range(3):
                    #     for j in range(3):
                    #         # print(self.rotation_matrices[:, i, j].shape)
                    #         # print(self.time_gt.shape)
                    #         # print(lines_array.shape)
                    #         # print(lines_array[i,j])
                    #         # print(line_r11)
                    #         lines_array[i,j].set_data(time_array, rotation_matix_vals[:, i, j])

                # if camera_pose is not None:

                #     cam.set_offsets([camera_pose[0, 3], camera_pose[1, 3], camera_pose[2, 3]])

                    # ax.scatter(camera_pose[0, 3], camera_pose[1, 3], camera_pose[2, 3], c='r', label="Camera")
                    # ax.scatter(camera_gt[0], camera_gt[1], camera_gt[2], c='g', label="Camera-gt")

                    # Define unit vectors for the local coordinate axes
                    # x_axis = camera_orientation @ np.array([1, 0, 0])*0.07
                    # y_axis = camera_orientation @ np.array([0, 1, 0])*0.07
                    # z_axis = camera_orientation @ np.array([0, 0, 1])*0.07      # Scalng length

                    # # Define the start and end points of the vector
                    # start_point = camera_pose[:3, 3]
                    # end_point = camera_pose[:3, 3] + z_axis

                    # # Generate intermediate points using linear interpolation
                    # n_points = 10  # Number of points to generate between start and end
                    # intermediate_points = np.linspace(start_point, end_point, n_points)

                    # Extract x, y, z coordinates for the scatter plot
                    # x_coords = intermediate_points[:, 0]
                    # y_coords = intermediate_points[:, 1]
                    # z_coords = intermediate_points[:, 2]

                    # print('z-axis', z_axis)

                    # ax.scatter([camera_pose[0, 3], camera_pose[0, 3] + x_axis[0]], 
                    #         [camera_pose[1, 3], camera_pose[1, 3] + x_axis[1]],
                    #         [camera_pose[2, 3], camera_pose[2, 3] + x_axis[2]],
                    #         color="red", label="X-axis")
                    
                    # ax.plot([camera_pose[0, 3], camera_pose[0, 3] + y_axis[0]], 
                    #         [camera_pose[1, 3], camera_pose[1, 3] + y_axis[1]],
                    #         [camera_pose[2, 3], camera_pose[2, 3] + y_axis[2]],
                    #         color="yellow", label="Y-axis")
                    
                    # ax.scatter(x_coords, y_coords, z_coords, color="yellow", label="Z-axis")
                    
                # ax.legend()
                ax1.legend()
                ax2.legend()
                ax3.legend()
                plt.pause(0.001)  # Allow plot to refresh

            # Check for quit signal
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
                break

        # Cleanup
        # plt.close(fig1)
        plt.close(fig2)
        # plt.close(fig3)
        # plt.close(fig4)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    import time
    from scipy.spatial.transform import Rotation as R

    # Helper function to create a transformation matrix
    def create_isometry3d(rotation=None, translation=None):
        if rotation is None:
            rotation = np.eye(3)
        if translation is None:
            translation = np.zeros(3)

        transformation = np.eye(4)
        transformation[:3, :3] = rotation
        transformation[:3, 3] = translation
        return transformation

    viewer = Viewer()

    # Identity pose
    identity_pose = create_isometry3d()
    viewer.update_pose(identity_pose)

    # Example pose with rotation and translation
    rotation_matrix = R.from_euler('xyz', [45, 0, 0], degrees=True).as_matrix()  # 45° rotation about x-axis
    translation_vector = np.array([1, 2, 3])  # Translation in x, y, z
    new_pose = create_isometry3d(rotation_matrix, translation_vector)

    # Simulate pose updates
    time.sleep(1)
    viewer.update_pose(new_pose)

    # Keep the visualization running
    viewer.view_thread.join()

# # # import numpy as np
# # # import cv2
# # # from multiprocessing import Queue, Process
# # # import open3d as o3d

# # # class Viewer:
# # #     def __init__(self):
# # #         self.image_queue = Queue()
# # #         self.pose_queue = Queue()

# # #         self.view_thread = Process(target=self.view)
# # #         self.view_thread.start()

# # #     def update_pose(self, pose):
# # #         """
# # #         Update the pose in the queue.
# # #         Pose should be a 4x4 transformation matrix.
# # #         """
# # #         if pose is None:
# # #             return
# # #         self.pose_queue.put(pose)

# # #     def update_image(self, image):
# # #         """
# # #         Update the image in the queue.
# # #         Converts grayscale to RGB if necessary.
# # #         """
# # #         if image is None:
# # #             return
# # #         elif image.ndim == 2:  # Grayscale image
# # #             image = np.repeat(image[..., np.newaxis], 3, axis=2)
# # #         self.image_queue.put(image)

# # #     def view(self):
# # #         """
# # #         The visualization loop using Open3D for 3D visualization and OpenCV for image display.
# # #         """
# # #         print("In visualization loop")

# # #         # OpenCV window for image display
# # #         cv2.namedWindow("Image Viewer", cv2.WINDOW_NORMAL)

# # #         # Open3D visualization setup
# # #         vis = o3d.visualization.Visualizer()
# # #         vis.create_window(window_name="3D Viewer", width=800, height=600)

# # #         # Initialize LineSet for trajectory
# # #         trajectory_line_set = o3d.geometry.LineSet()
# # #         trajectory_points = []
# # #         trajectory_lines = []

# # #         vis.add_geometry(trajectory_line_set)

# # #         while True:
# # #             updated = False

# # #             # Update pose and trajectory
# # #             if not self.pose_queue.empty():
# # #                 while not self.pose_queue.empty():
# # #                     pose = self.pose_queue.get()
# # #                     pose_array = pose.matrix()
# # #                     position = pose_array[:3, 3]  # Extract translation
# # #                     trajectory_points.append(position)

# # #                 # Update lines and points for Open3D LineSet
# # #                 num_points = len(trajectory_points)
# # #                 if num_points > 1:
# # #                     trajectory_lines = [[i, i + 1] for i in range(num_points - 1)]
# # #                 trajectory_line_set.points = o3d.utility.Vector3dVector(trajectory_points)
# # #                 trajectory_line_set.lines = o3d.utility.Vector2iVector(trajectory_lines)
# # #                 updated = True

# # #             # Update image
# # #             if not self.image_queue.empty():
# # #                 while not self.image_queue.empty():
# # #                     img = self.image_queue.get()
# # #                 img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
# # #                 cv2.imshow("Image Viewer", img)

# # #             # Refresh Open3D visualization
# # #             if updated:
# # #                 vis.update_geometry(trajectory_line_set)
# # #                 vis.poll_events()
# # #                 vis.update_renderer()

# # #             # Check for quit signal
# # #             if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
# # #                 break

# # #         # Cleanup
# # #         vis.destroy_window()
# # #         cv2.destroyAllWindows()

# # # # Main script to simulate pose updates
# # # if __name__ == '__main__':
# # #     import time
# # #     from scipy.spatial.transform import Rotation as R

# # #     # Helper function to create a transformation matrix
# # #     def create_isometry3d(rotation=None, translation=None):
# # #         if rotation is None:
# # #             rotation = np.eye(3)
# # #         if translation is None:
# # #             translation = np.zeros(3)

# # #         transformation = np.eye(4)
# # #         transformation[:3, :3] = rotation
# # #         transformation[:3, 3] = translation
# # #         return transformation

# # #     viewer = Viewer()

# # #     # Identity pose
# # #     identity_pose = create_isometry3d()
# # #     viewer.update_pose(identity_pose)

# # #     # Example pose with rotation and translation
# # #     rotation_matrix = R.from_euler('xyz', [45, 0, 0], degrees=True).as_matrix()  # 45° rotation about x-axis
# # #     translation_vector = np.array([1, 2, 3])  # Translation in x, y, z
# # #     new_pose = create_isometry3d(rotation_matrix, translation_vector)

# # #     # Simulate pose updates
# # #     time.sleep(1)
# # #     viewer.update_pose(new_pose)

# # #     # Keep the visualization running
# # #     viewer.view_thread.join()


# # import numpy as np
# # import cv2
# # from multiprocessing import Queue, Process
# # import open3d as o3d

# # class Viewer:
# #     def __init__(self):
# #         self.image_queue = Queue()
# #         self.pose_queue = Queue()

# #         self.view_thread = Process(target=self.view)
# #         self.view_thread.start()

# #     def update_pose(self, pose):
# #         """
# #         Update the pose in the queue.
# #         Pose should be a 4x4 transformation matrix.
# #         """
# #         if pose is None:
# #             return
# #         self.pose_queue.put(pose)

# #     def update_image(self, image):
# #         """
# #         Update the image in the queue.
# #         Converts grayscale to RGB if necessary.
# #         """
# #         if image is None:
# #             return
# #         elif image.ndim == 2:  # Grayscale image
# #             image = np.repeat(image[..., np.newaxis], 3, axis=2)
# #         self.image_queue.put(image)

# #     def view(self):
# #         """
# #         The visualization loop using Open3D for 3D visualization and OpenCV for image display.
# #         """
# #         print("In visualization loop")

# #         # OpenCV window for image display
# #         cv2.namedWindow("Image Viewer", cv2.WINDOW_NORMAL)

# #         # Open3D visualization setup
# #         vis = o3d.visualization.Visualizer()
# #         vis.create_window(window_name="3D Viewer", width=800, height=600)

# #         # Initialize trajectory and coordinate axes
# #         trajectory_points = []
# #         trajectory_lines = []
# #         trajectory_line_set = o3d.geometry.LineSet()
# #         vis.add_geometry(trajectory_line_set)

# #         # # Create MaterialRecord for line thickness
# #         # material = o3d.visualization.rendering.MaterialRecord()
# #         # material.line_width = 5.0  # Adjust the thickness (default is 1.0)

# #         axes_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.005, origin=[0, 0, 0])
# #         vis.add_geometry(axes_frame)

# #         while True:
# #             updated = False

# #             # Update pose and trajectory
# #             if not self.pose_queue.empty():
# #                 while not self.pose_queue.empty():
# #                     pose = self.pose_queue.get()
# #                     pose_array = pose.matrix()
# #                     position = pose_array[:3 , 3]   # Extract translation
# #                     trajectory_points.append(position)

# #                 # Update LineSet for trajectory
# #                 num_points = len(trajectory_points)
# #                 if num_points > 1:
# #                     trajectory_lines = [[i, i + 1] for i in range(num_points - 1)]
# #                 trajectory_line_set.points = o3d.utility.Vector3dVector(trajectory_points)
# #                 trajectory_line_set.lines = o3d.utility.Vector2iVector(trajectory_lines)

# #                 # Update coordinate axes pose
# #                 pose_array = pose.matrix()
# #                 rotation_matrix = pose_array[:3, :3]
# #                 translation_vector = pose_array[:3, 3]
# #                 axes_frame.translate(translation_vector - axes_frame.get_center(), relative=False)
# #                 axes_frame.rotate(rotation_matrix)

# #                 updated = True

# #             # Update image
# #             if not self.image_queue.empty():
# #                 while not self.image_queue.empty():
# #                     img = self.image_queue.get()
# #                 img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
# #                 cv2.imshow("Image Viewer", img)

# #             # Refresh Open3D visualization
# #             if updated:
# #                 vis.update_geometry(trajectory_line_set)
# #                 vis.update_geometry(axes_frame)
# #                 vis.poll_events()
# #                 vis.update_renderer()

# #             # Check for quit signal
# #             if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
# #                 break

# #         # Cleanup
# #         vis.destroy_window()
# #         cv2.destroyAllWindows()

# # # Main script to simulate pose updates
# # if __name__ == '__main__':
# #     import time
# #     from scipy.spatial.transform import Rotation as R

# #     # Helper function to create a transformation matrix
# #     def create_isometry3d(rotation=None, translation=None):
# #         if rotation is None:
# #             rotation = np.eye(3)
# #         if translation is None:
# #             translation = np.zeros(3)

# #         transformation = np.eye(4)
# #         transformation[:3, :3] = rotation
# #         transformation[:3, 3] = translation
# #         return transformation

# #     viewer = Viewer()

# #     # Identity pose
# #     identity_pose = create_isometry3d()
# #     viewer.update_pose(identity_pose)

# #     # Example pose with rotation and translation
# #     rotation_matrix = R.from_euler('xyz', [45, 0, 0], degrees=True).as_matrix()  # 45° rotation about x-axis
# #     translation_vector = np.array([1, 2, 3])  # Translation in x, y, z
# #     new_pose = create_isometry3d(rotation_matrix, translation_vector)

# #     # Simulate pose updates
# #     time.sleep(1)
# #     viewer.update_pose(new_pose)

# #     # Keep the visualization running
# #     viewer.view_thread.join()

# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import cv2
# from multiprocessing import Queue, Process, Event
# import time

# class Viewer(object):
#     def __init__(self):
#         self.image_queue = Queue()
#         self.pose_queue = Queue()
#         self.exit_event = Event()
#         self.view_thread = Process(target=self.view)
#         self.view_thread.start()

#         # Load ground truth data
#         # Assume ground_truth.txt is a CSV with columns: time, x, y, z
#         self.ground_truth_data = np.loadtxt(r'C:\Users\athar\OneDrive\Documents\Sem-1\SLAM\Project\codes\MH_01_easy\mav0\state_groundtruth_estimate0\data.csv', delimiter=',', skiprows=1)

#         # Extract x, y, z coordinates
#         self.x_gt = self.ground_truth_data[:, 1]
#         self.y_gt = self.ground_truth_data[:, 2]
#         self.z_gt = self.ground_truth_data[:, 3]
#         self.time_gt = self.ground_truth_data[:, 0]/(1e9)  # Assuming time is in seconds

#     def update_pose(self, pose):
#         if pose is None:
#             return
#         # Assuming pose is a 4x4 transformation matrix
#         self.pose_queue.put(pose)

#     def update_timestamp(self,timestamp):
#         if timestamp is None:
#             print('timestamp is none')
#             return
        
#         timestamp = timestamp # Already in seconds
#         self.timestamp_queue.put(timestamp)
        
#     def update_gt_pose(self,pose):

#         # Pose is (3,) vector
#         if pose is None:
#             print('gt pose is none')
#             return    
#         self.gt_queue.put(pose)

#     def update_image(self, image):
#         if image is None:
#             return
#         elif image.ndim == 2:
#             image = np.repeat(image[..., np.newaxis], 3, axis=2)
#         self.image_queue.put(image)
        
#     def close(self):
#         self.exit_event.set()
#         self.view_thread.join()
#         plt.close('all')  # For matplotlib version

#     def draw_camera(self, ax, pose, scale=0.5):
#         # Camera frustum points
#         points = scale * np.array([
#             [0, 0, 0],
#             [-1, -1, 2],
#             [1, -1, 2],
#             [1, 1, 2],
#             [-1, 1, 2]
#         ])
        
#         # Transform points
#         points = (pose[:3, :3] @ points.T + pose[:3, 3:]).T
        
#         # Draw frustum lines
#         for i in range(1, 5):
#             ax.plot([points[0, 0], points[i, 0]],
#                    [points[0, 1], points[i, 1]],
#                    [points[0, 2], points[i, 2]], 'b-')
        
#         # Draw frustum base
#         ax.plot([points[1, 0], points[2, 0], points[3, 0], points[4, 0], points[1, 0]],
#                 [points[1, 1], points[2, 1], points[3, 1], points[4, 1], points[1, 1]],
#                 [points[1, 2], points[2, 2], points[3, 2], points[4, 2], points[1, 2]], 'b-')

#     def view(self):
#         plt.ion()
#         fig = plt.figure(figsize=(12, 6))
        
#         # 3D trajectory subplot
#         ax_3d = fig.add_subplot(121, projection='3d')
#         ax_3d.set_xlabel('X')
#         ax_3d.set_ylabel('Y')
#         ax_3d.set_zlabel('Z')
        
#         # Image subplot
#         ax_img = fig.add_subplot(122)
        
#         trajectory = []
#         current_image = None
        
#         while not self.exit_event.is_set():
#             try:
#                 # Update pose
#                 while not self.pose_queue.empty():
#                     pose = self.pose_queue.get()
#                     pose = pose.matrix()
#                     trajectory.append(pose[:3, 3])
                
#                 # Update image
#                 while not self.image_queue.empty():
#                     current_image = self.image_queue.get()
                
#                 # Clear and redraw
#                 ax_3d.cla()
#                 ax_3d.set_xlabel('X')
#                 ax_3d.set_ylabel('Y')
#                 ax_3d.set_zlabel('Z')
                
#                 # Draw trajectory
#                 if trajectory:
#                     trajectory_array = np.array(trajectory)
#                     ax_3d.scatter(trajectory_array[:, 0],
#                                 trajectory_array[:, 1],
#                                 trajectory_array[:, 2],
#                                 c='black', s=2)
                
#                 # Draw current camera pose
#                 if trajectory:
#                     latest_pose = np.eye(4)
#                     latest_pose[:3, 3] = trajectory[-1]
#                     self.draw_camera(ax_3d, latest_pose)
                
#                 # Draw coordinate axes
#                 ax_3d.plot([0, 1], [0, 0], [0, 0], 'r-', label='X')
#                 ax_3d.plot([0, 0], [0, 1], [0, 0], 'g-', label='Y')
#                 ax_3d.plot([0, 0], [0, 0], [0, 1], 'b-', label='Z')
                
#                 # Update image
#                 if current_image is not None:
#                     ax_img.cla()
#                     ax_img.imshow(cv2.cvtColor(current_image, cv2.COLOR_BGR2RGB))
#                     ax_img.axis('off')
                
#                 plt.draw()
#                 plt.pause(0.01)
                
#             except KeyboardInterrupt:
#                 plt.close('all')
#                 break
            
#         plt.close(fig)

# class DynamicArray(object):
#     def __init__(self, shape=3):
#         if isinstance(shape, int):
#             shape = (shape,)
#         assert isinstance(shape, tuple)
#         self.data = np.zeros((1000, *shape))
#         self.shape = shape
#         self.ind = 0

#     def clear(self):
#         self.ind = 0

#     def append(self, x):
#         self.extend([x])

#     def extend(self, xs):
#         if len(xs) == 0:
#             return
#         assert np.array(xs[0]).shape == self.shape
#         if self.ind + len(xs) >= len(self.data):
#             self.data.resize((2 * len(self.data), *self.shape), refcheck=False)
#         if isinstance(xs, np.ndarray):
#             self.data[self.ind:self.ind+len(xs)] = xs
#         else:
#             for i, x in enumerate(xs):
#                 self.data[self.ind+i] = x
#         self.ind += len(xs)

#     def array(self):
#         return self.data[:self.ind]

#     def __len__(self):
#         return self.ind

#     def __getitem__(self, i):
#         assert i < self.ind
#         return self.data[i]

#     def __iter__(self):
#         for x in self.data[:self.ind]:
#             yield x

# if __name__ == '__main__':
#     viewer = Viewer()
#     # Example usage with identity pose
#     try:
#         viewer.update_pose()
#             # Add a sleep to keep the viewer running
#         while True:
#             time.sleep(1)
#     except KeyboardInterrupt:
#         print("\nClosing viewer...")
#         viewer.close()
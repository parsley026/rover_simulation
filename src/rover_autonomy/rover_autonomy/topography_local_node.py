import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class PythonElevationMapper(Node):
    def __init__(self):
        super().__init__('python_elevation_mapper')
        
        # --- Parameters ---
        self.resolution = 0.1  # meters per cell
        self.grid_size = 10.0  # meters (10x10m local map)
        self.max_obstacle_height = 0.5 
        
        # The frame we want to flatten the data into
        self.target_frame = 'base_link' 
        
        self.grid_cells = int(self.grid_size / self.resolution)
        
        # --- TF Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- ROS Setup ---
        self.sub = self.create_subscription(
            PointCloud2, '/lidar_00/points', self.cloud_callback, 10)
        self.pub = self.create_publisher(
            OccupancyGrid, '/topography_costmap', 10)

    def cloud_callback(self, msg):
        # 1. Get the Transform Matrix from TF2
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, 
                msg.header.frame_id, 
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Waiting for TF to transform cloud: {e}')
            return

        # 2. Read points (Structured 1D Array)
        points_structured = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        if len(points_structured) == 0:
            return

        # Convert to standard 2D NumPy array (Nx3) for fast matrix math
        xyz = np.empty((len(points_structured), 3))
        xyz[:, 0] = points_structured['x']
        xyz[:, 1] = points_structured['y']
        xyz[:, 2] = points_structured['z']

        # 3. Manually apply the rotation and translation
        # Extract quaternion
        q = t.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        
        # Convert quaternion to 3x3 rotation matrix
        R = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])
        
        # Extract translation vector
        translation = np.array([t.transform.translation.x, 
                                t.transform.translation.y, 
                                t.transform.translation.z])
                                
        # Apply transform: Rotate and Translate
        points = xyz.dot(R.T) + translation

        # 4. Filter out points outside our local grid area (using standard 2D indexing again)
        half_size = self.grid_size / 2.0
        mask = (
            (points[:, 0] >= -half_size) & (points[:, 0] <= half_size) &
            (points[:, 1] >= -half_size) & (points[:, 1] <= half_size)
        )
        points = points[mask]

        if len(points) == 0:
            return

        # 5. Discretize X and Y into grid indices
        x_indices = ((points[:, 0] + half_size) / self.resolution).astype(np.int32)
        y_indices = ((points[:, 1] + half_size) / self.resolution).astype(np.int32)

        # 6. Calculate terrain cost (Roughness/Slope)
        flat_indices = y_indices * self.grid_cells + x_indices
        
        min_z = np.full(self.grid_cells * self.grid_cells, np.inf)
        max_z = np.full(self.grid_cells * self.grid_cells, -np.inf)
        
        np.minimum.at(min_z, flat_indices, points[:, 2])
        np.maximum.at(max_z, flat_indices, points[:, 2])
        
        height_diff = max_z - min_z
        valid_cells = np.isfinite(height_diff)
        
        cost_array = np.zeros_like(height_diff, dtype=np.int8)
        cost_array[valid_cells] = np.clip(
            (height_diff[valid_cells] / self.max_obstacle_height) * 254, 
            0, 254
        ).astype(np.int8)
        
        cost_array[~valid_cells] = -1

        # 7. Publish OccupancyGrid in target_frame (base_link)
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = msg.header.stamp
        grid_msg.header.frame_id = self.target_frame 
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_cells
        grid_msg.info.height = self.grid_cells
        
        grid_msg.info.origin.position.x = -half_size
        grid_msg.info.origin.position.y = -half_size
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        grid_msg.data = cost_array.tolist()
        
        self.pub.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PythonElevationMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
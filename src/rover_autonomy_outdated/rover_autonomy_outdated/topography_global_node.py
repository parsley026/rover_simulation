import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class PythonGlobalElevationMapper(Node):
    def __init__(self):
        super().__init__('python_global_elevation_mapper')
        
        # --- Parameters ---
        self.resolution = 0.1  # meters per cell
        self.max_obstacle_height = 0.5 # height diff in meters that equals lethal cost
        
        # The frame the global map is anchored to
        self.target_frame = 'map' 
        
        # --- TF Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- ROS Setup ---
        # Subscribing to RTAB-Map's global aggregated point cloud
        self.sub = self.create_subscription(
            PointCloud2, '/mapping/cloud_map', self.cloud_callback, 1)
            
        # Publishing a global costmap
        self.pub = self.create_publisher(
            OccupancyGrid, '/global_topography_costmap', 1)

    def cloud_callback(self, msg):
        # 1. Get the Transform Matrix from TF2 (Usually map -> map, which is identity, 
        # but kept here just in case RTAB-Map outputs in odom or another frame)
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, 
                msg.header.frame_id, 
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Waiting for TF: {e}')
            return

        # 2. Read points (Structured 1D Array)
        points_structured = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        if len(points_structured) == 0:
            return

        # Convert to standard 2D NumPy array (Nx3)
        xyz = np.empty((len(points_structured), 3))
        xyz[:, 0] = points_structured['x']
        xyz[:, 1] = points_structured['y']
        xyz[:, 2] = points_structured['z']

        # 3. Apply the rotation and translation to the target_frame
        q = t.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        
        R = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])
        
        translation = np.array([t.transform.translation.x, 
                                t.transform.translation.y, 
                                t.transform.translation.z])
                                
        points = xyz.dot(R.T) + translation

        # 4. Find dynamic map boundaries instead of using a fixed local window
        min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
        min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])

        # Prevent 0-width/height grids if point cloud is perfectly linear
        if min_x == max_x or min_y == max_y:
            return

        # Calculate dynamic grid dimensions
        width = int(np.ceil((max_x - min_x) / self.resolution)) + 1
        height = int(np.ceil((max_y - min_y) / self.resolution)) + 1
        total_cells = width * height

        # 5. Discretize X and Y into grid indices based on the new dynamic minimums
        x_indices = ((points[:, 0] - min_x) / self.resolution).astype(np.int32)
        y_indices = ((points[:, 1] - min_y) / self.resolution).astype(np.int32)
        
        # Clip indices to prevent out-of-bounds errors due to floating point rounding
        x_indices = np.clip(x_indices, 0, width - 1)
        y_indices = np.clip(y_indices, 0, height - 1)

        # 6. Calculate terrain cost (Roughness/Slope)
        flat_indices = y_indices * width + x_indices
        
        min_z = np.full(total_cells, np.inf)
        max_z = np.full(total_cells, -np.inf)
        
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

        # 7. Publish dynamic OccupancyGrid
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = msg.header.stamp
        grid_msg.header.frame_id = self.target_frame 
        
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = width
        grid_msg.info.height = height
        
        # Origin is now placed at the absolute minimum X and Y of the explored map
        grid_msg.info.origin.position.x = float(min_x)
        grid_msg.info.origin.position.y = float(min_y)
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        grid_msg.data = cost_array.tolist()
        
        self.pub.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PythonGlobalElevationMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np
from queue import Queue
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
import time
import cv2
from datetime import datetime


class ExplorationPlanner(Node):
    def __init__(self):
        super().__init__("exploration_planner", namespace="bot2")
        self.get_logger().info("Exploration planner node has been initialized")

        # 맵 데이터 관련 변수
        self.current_map = None
        self.map_info = {}
        self.last_callback_time = time.time()
        self.callback_interval = 5.0

        # Constants for map values
        self.WALL = 100  # Wall in occupancy grid
        self.KNOWN = 0  # Known/free space in occupancy grid
        self.UNKNOWN = -1  # Unknown space in occupancy grid
        self.CHECKED = 50  # For visualization

        # Map subscription
        self.subscription = self.create_subscription(
            OccupancyGrid, "map", self.map_callback, 10
        )

        # Navigation action client
        self.navigate_to_pose_action_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )
        self._is_navigating = False

        # 주기적 탐색 타이머
        # self.process_interval = 2.0  # 5초마다 처리
        # self.create_timer(self.process_interval, self.plan_and_navigate)

    def crop_map(self, map_data, x, y, width, height):
        # Crop a section of the map with padding
        pad_width = width // 2
        pad_height = height // 2
        map_padded = np.pad(
            map_data,
            ((pad_width, pad_width), (pad_height, pad_height)),
            mode="constant",
            constant_values=self.UNKNOWN,
        )
        return map_padded[y : y + width, x : x + height]

    def get_properties(self, map_section):
        # Get properties (wall, known, unknown) of a map section
        unique_values = np.unique(map_section)
        properties = []
        if self.WALL in unique_values:
            properties.append(self.WALL)
        if self.KNOWN in unique_values:
            properties.append(self.KNOWN)
        if self.UNKNOWN in unique_values:
            properties.append(self.UNKNOWN)
        return properties

    def is_neighbors(self, map_section, prop):
        # Check if two properties are neighbors in the map section
        # Check horizontal neighbors
        for row in map_section:
            for i in range(len(row) - 1):
                if (row[i] == prop[0] and row[i + 1] == prop[1]) or (
                    row[i] == prop[1] and row[i + 1] == prop[0]
                ):
                    return True

        # Check vertical neighbors
        for col in map_section.T:
            for i in range(len(col) - 1):
                if (col[i] == prop[0] and col[i + 1] == prop[1]) or (
                    col[i] == prop[1] and col[i + 1] == prop[0]
                ):
                    return True
        return False

    def get_wall_endpoints(self, map_data):
        # Find endpoints of walls that are adjacent to both known and unknown areas
        height, width = map_data.shape
        points = []

        for i in range(height):
            for j in range(width):
                if map_data[i, j] == self.WALL:
                    cropped = self.crop_map(map_data, j, i, 3, 3)
                    prop = self.get_properties(cropped)
                    if (
                        self.WALL in prop
                        and self.KNOWN in prop
                        and self.UNKNOWN in prop
                        and self.is_neighbors(cropped, [self.KNOWN, self.UNKNOWN])
                    ):
                        points.append((j, i))

        return points

    def get_line_coordinates(self, x1, y1, x2, y2):
        # Get coordinates of points along a line using Bresenham's algorithm
        coordinates = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        while True:
            coordinates.append((x1, y1))
            if x1 == x2 and y1 == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy

        return coordinates

    def find_center_points(self, map_data, threshold=3):
        # Find center points between wall endpoints that are suitable for exploration
        wall_endpoints = self.get_wall_endpoints(map_data)
        center_points = []

        for p1 in wall_endpoints:
            for p2 in wall_endpoints:
                if p1 == p2:
                    continue

                line_coords = self.get_line_coordinates(p1[0], p1[1], p2[0], p2[1])[
                    threshold:-threshold
                ]

                if not line_coords:
                    continue

                # Check if the line is valid (no walls, has both known and unknown areas)
                is_valid_path = True
                for p in line_coords:
                    if not (
                        0 <= p[1] < map_data.shape[0] and 0 <= p[0] < map_data.shape[1]
                    ):
                        is_valid_path = False
                        break

                    prop = self.get_properties(
                        self.crop_map(map_data, p[0], p[1], 3, 3)
                    )
                    if not (
                        self.KNOWN in prop
                        and self.UNKNOWN in prop
                        and self.WALL not in prop
                    ):
                        is_valid_path = False
                        break

                if is_valid_path:
                    # Get the middle point of the line
                    mid_point = line_coords[len(line_coords) // 2]
                    if mid_point not in center_points:
                        center_points.append(mid_point)

        return center_points

    def map_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_callback_time < self.callback_interval:
            return
        self.last_callback_time = current_time

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Convert occupancy grid to numpy array
        map_data = np.array(msg.data, dtype=np.int8).reshape(height, width)

        # Calculate robot position
        robot_x = int((-origin_x) / resolution)
        robot_y = int((-origin_y) / resolution)

        # Ensure robot position is within map bounds
        robot_x = max(0, min(robot_x, width - 1))
        robot_y = max(0, min(robot_y, height - 1))

        self.current_map = map_data
        self.map_info = {
            "resolution": resolution,
            "origin_x": origin_x,
            "origin_y": origin_y,
            "robot_position": (robot_y, robot_x),
        }

        self.plan_and_navigate()

        # self.save_map_data(map_data, msg, robot_x, robot_y)

    # [이전 코드의 나머지 메서드들은 그대로 유지]

    def save_map_data(self, map_data, msg, robot_x, robot_y):
        # Save map data to files
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_filename = f"/home/ynu/map/map_data_{timestamp}"

        image = self.convert_to_image(map_data)
        cv2.imwrite(f"{base_filename}.png", image)
        self.save_map_to_txt(map_data, f"{base_filename}_map.txt")

        with open(f"{base_filename}_info.txt", "w") as f:
            f.write(f"Map size: {msg.info.width}x{msg.info.height}\n")
            f.write(
                f"Bot position: ({msg.info.origin.position.x}, {msg.info.origin.position.y})\n"
            )
            f.write("\nMap data format:\n")
            f.write(f"{self.UNKNOWN}: Unknown area (gray)\n")
            f.write(f"{self.KNOWN}: Free space (white)\n")
            f.write(f"{self.WALL}: Wall (black)\n")
            f.write("2: Robot position (special mark)\n")

    def convert_to_image(self, map_data):
        # Convert map data to image
        image = np.zeros_like(map_data, dtype=np.uint8)
        image[map_data == self.UNKNOWN] = 128
        image[map_data == self.KNOWN] = 255
        image[map_data == self.WALL] = 0
        image[map_data == 2] = 50
        return image

    def save_map_to_txt(self, map_data, filename):
        # Save map data to text file
        with open(filename, "w") as f:
            f.write("[\n")
            for row in map_data:
                f.write("    [")
                row_str = ", ".join(f"{x:3d}" for x in row)
                f.write(f"{row_str}")
                f.write("],\n")
            f.write("]")

    def plan_and_navigate(self):
        # Plan and navigate to the next goal
        # if self._is_navigating:
        #     self.get_logger().info("Navigation in progress, skipping new goal calculation")
        #     return

        # if self.current_map is None or self.map_info is None:
        #     self.get_logger().warning("Map data or map info is unavailable")
        #     return

        # Find center points for exploration
        center_points = self.find_center_points(self.current_map)

        if center_points:
            # Find the closest center point to the robot
            robot_pos = self.map_info["robot_position"]
            closest_point = min(
                center_points,
                key=lambda p: ((p[1] - robot_pos[0]) ** 2 + (p[0] - robot_pos[1]) ** 2)
                ** 0.5,
            )
            self.navigate_to_goal(closest_point)
        else:
            # Fallback to original BFS method if no center points found
            goal = self.find_goal_bfs(self.current_map, self.map_info["robot_position"])
            if goal:
                self.navigate_to_goal(goal)
            else:
                self.get_logger().info("No valid goal found")

    def find_goal_bfs(self, map_data, robot_position):
        # Original BFS method for finding goals (used as fallback)
        height, width = map_data.shape
        visited = np.zeros_like(map_data, dtype=bool)
        queue = Queue()
        queue.put(robot_position)
        visited[robot_position] = True

        directions = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ]

        while not queue.empty():
            y, x = queue.get()
            for dy, dx in directions:
                ny, nx = y + dy, x + dx
                if 0 <= ny < height and 0 <= nx < width and not visited[ny, nx]:
                    visited[ny, nx] = True
                    if map_data[ny, nx] == self.UNKNOWN:
                        if self.is_valid_goal(map_data, ny, nx, directions):
                            return (ny, nx)
                    if map_data[ny, nx] != self.WALL:
                        queue.put((ny, nx))
        return None

    def is_valid_goal(self, map_data, y, x, directions):
        # Check if a potential goal point is valid
        for dy, dx in directions:
            ny, nx = y + dy, x + dx
            if 0 <= ny < map_data.shape[0] and 0 <= nx < map_data.shape[1]:
                if map_data[ny, nx] == self.WALL:
                    for ddy, ddx in directions:
                        check_y, check_x = ny + ddy, nx + ddx
                        if (
                            0 <= check_y < map_data.shape[0]
                            and 0 <= check_x < map_data.shape[1]
                            and map_data[check_y, check_x] == self.UNKNOWN
                        ):
                            return True
        return False

    def navigate_to_goal(self, goal):
        # Navigate to the specified goal
        while True:
            if not self.navigate_to_pose_action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn("Navigation action server not available")
                # return
            else:
                break

        self._is_navigating = True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"

        real_x = goal[0] * self.map_info["resolution"] + self.map_info["origin_x"]
        real_y = goal[1] * self.map_info["resolution"] + self.map_info["origin_y"]

        goal_msg.pose.pose.position.x = real_x
        goal_msg.pose.pose.position.y = real_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending goal: ({real_x}, {real_y})")

        self._send_goal_future = self.navigate_to_pose_action_client.send_goal_async(
            goal_msg, feedback_callback=self.navigation_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.navigation_goal_response_callback)

    def navigation_feedback_callback(self, feedback_msg):
        # Handle navigation feedback
        feedback = feedback_msg.feedback
        self.get_logger().debug(f"Navigation feedback received: {feedback}")

    def navigation_goal_response_callback(self, future):
        # Handle navigation goal response
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Navigation goal rejected")
            self._is_navigating = False
            return

        self.get_logger().info("Navigation goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        # Handle navigation result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation succeeded")
        else:
            self.get_logger().warn(f"Navigation failed with status: {status}")
        self._is_navigating = False


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ExplorationPlanner()
        try:
            while True:
                rclpy.spin_once(node)
                time.sleep(2)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Ignore if already shutdown


if __name__ == "__main__":
    main()

import rclpy
import os
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.timer import Timer
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid , Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Empty
from std_msgs.msg import Bool, String, Float32
import numpy as np
import heapq , math , random , yaml
import scipy.interpolate as si
import sys , threading , time
import math
from ament_index_python.packages import get_package_share_directory

package_path = get_package_share_directory("autonomous_exploration")
params_path = os.path.join(package_path, "config", "params.yaml")
with open(params_path, 'r') as file:
    params = yaml.load(file, Loader=yaml.FullLoader)

lookahead_distance = params["lookahead_distance"]
speed = params["speed"]
expansion_size = params["expansion_size"]
target_error = params["target_error"]
robot_r = params["robot_r"]

pathGlobal = 0

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.declare_parameter('robot_id', 'limo1')
        self.robot_id = self.get_parameter('robot_id').value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.start_topic = f'{self.robot_id}/start_mission'
        self.stop_topic = f'{self.robot_id}/stop_mission'
        self.end_topic = f'{self.robot_id}/end_mission'
        self.odom_topic = f'{self.robot_id}/odom'  
        self.scan_topic = f'{self.robot_id}/scan'
        self.cmd_vel_topic = f'{self.robot_id}/cmd_vel'
        self.distance_topic = f'{self.robot_id}/distance'
        self.navigate_topic = f'{self.robot_id}/navigate_to_pose'
        self.map_frame = f'{self.robot_id}/map' 

        self.subscription_start = self.create_subscription(Empty, self.start_topic, self.start_callback, 10)
        self.subscription_end = self.create_subscription(Empty, self.end_topic, self.end_callback, 10)
        self.subscription_map = self.create_subscription(OccupancyGrid,self.map_frame,self.map_callback,10)
        self.subscription_odom = self.create_subscription(Odometry,self.odom_topic,self.odom_callback,qos)
        self.subscription_scan = self.create_subscription(LaserScan,self.scan_topic,self.scan_callback,qos)

        self.publisher_dist = self.create_publisher(Float32, self.distance_topic, 10)
        self.publisher_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.nav_client = ActionClient(self, NavigateToPose, self.navigate_topic)
        self.nav_client.wait_for_server()
        self.current_goal_handle = None

        self.distance = 0.0
        self.mission_active = False
        self.isFirstOdom = True
        self.previus_pose = None
        self.initial_pose = None
        self.current_pos = None
        self.current_goal = None

        self.kesif = True
        self.get_logger().info("Waiting on mission to start")
        
    def exp(self):
        while True: 
            if self.mission_active == True:
                if not hasattr(self,'map_data') or not hasattr(self,'odom_data') or not hasattr(self,'scan_data'):
                    time.sleep(0.1)
                    continue
                if self.initial_pose is None:
                    self.initial_pose = (self.x, self.y)

                if self.kesif == True:
                    if isinstance(pathGlobal, int) and pathGlobal == 0:
                        column = int((self.x - self.originX)/self.resolution)
                        row = int((self.y- self.originY)/self.resolution)
                        exploration(self.data,self.width,self.height,self.resolution,column,row,self.originX,self.originY)
                        self.path = pathGlobal
                    else:
                        self.path = pathGlobal
                    if isinstance(self.path, int) and self.path == -1:
                        print("[BILGI] KESİF TAMAMLANDI")
                        sys.exit()
                    self.c = int((self.path[-1][0] - self.originX)/self.resolution) 
                    self.r = int((self.path[-1][1] - self.originY)/self.resolution) 
                    self.kesif = False
                    self.i = 0
                    print("[BILGI] YENI HEDEF BELİRLENDI")
                    t = pathLength(self.path)/speed
                    t = t - 0.2 #x = v * t formülüne göre hesaplanan sureden 0.2 saniye cikarilir. t sure sonra kesif fonksiyonu calistirilir.
                    self.t = threading.Timer(t,self.target_callback) #Hedefe az bir sure kala kesif fonksiyonunu calistirir.
                    self.t.start()
                
                #Rota Takip Blok Baslangic
                else:
                    v , w = localControl(self.scan)
                    if v == None:
                        v, w,self.i = pure_pursuit(self.x,self.y,self.yaw,self.path,self.i)
                    if(abs(self.x - self.path[-1][0]) < target_error and abs(self.y - self.path[-1][1]) < target_error):
                        v = 0.0
                        w = 0.0
                        self.kesif = True
                        print("[BILGI] HEDEFE ULASILDI")
                        self.t.join() #Thread bitene kadar bekle.
                    target_position = pathGlobal[-1]
                    self.send_navigation_goal(target_position)
                    time.sleep(0.1)
                #Rota Takip Blok Bitis

    def target_callback(self):
        exploration(self.data,self.width,self.height,self.resolution,self.c,self.r,self.originX,self.originY)
        
    def scan_callback(self,msg):
        self.scan_data = msg
        self.scan = msg.ranges

    def map_callback(self,msg):
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data

    def odom_callback(self, msg: Odometry):
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,
                                          msg.pose.pose.orientation.y,
                                          msg.pose.pose.orientation.z,
                                          msg.pose.pose.orientation.w)
        self.current_pose = msg.pose.pose.position

    def start_callback(self, msg):
        if not self.mission_active:
            self.mission_active = True
            self.exploration_thread = threading.Thread(target=self.exp)
            self.exploration_thread.start()
            self.get_logger().info("Starting mission")

    def stop_callback(self, msg):
        self.mission_active = True
        self.cancel_navigation_goal()
        self.stop_robot()
        if self.exploration_thread is not None:
            self.exploration_thread.join()
        self.get_logger().info("stopping mission")

    def end_callback(self, msg):
        self.mission_active = False
        self.get_logger().info(f"Ending mission, returning to: x={self.initial_pose.pose.position.x:.2f}, y={self.initial_pose.pose.position.y :.2f}")

        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.map_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.initial_pose.pose.position.x
        goal_pose.pose.position.y = self.initial_pose.pose.position.y
        goal_pose.pose.orientation.w = 1.0
        goal_msg.pose = goal_pose
        send_goal_future = self.nav_client.send_goal_async(goal_msg)

    def is_stock():
        print("Robot is stock")

def main(args=None):
    rclpy.init(args=args)
    navigation_control = MissionNode()
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

def euler_from_quaternion(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data = data + [start]
            data = data[::-1]
            return data
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    # If no path to goal was found, return closest path to goal
    if goal not in came_from:
        closest_node = None
        closest_dist = float('inf')
        for node in close_set:
            dist = heuristic(node, goal)
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            data = []
            while closest_node in came_from:
                data.append(closest_node)
                closest_node = came_from[closest_node]
            data = data + [start]
            data = data[::-1]
            return data
    return False

def bspline_planning(array, sn):
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        N = 2
        t = range(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)

        x_list = list(x_tup)
        xl = x.tolist()
        x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

        y_list = list(y_tup)
        yl = y.tolist()
        y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)
        path = [(rx[i],ry[i]) for i in range(len(rx))]
    except:
        path = array
    return path

def pure_pursuit(current_x, current_y, current_heading, path, index):
    global lookahead_distance
    closest_point = None
    v = speed
    for i in range(index,len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        desired_steering_angle = target_heading - current_heading
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        desired_steering_angle = target_heading - current_heading
        index = len(path)-1
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi/4
        v = 0.0
    return v,desired_steering_angle,index

def frontierB(matrix):
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] == 0.0:
                if i > 0 and matrix[i-1][j] < 0:
                    matrix[i][j] = 2
                elif i < len(matrix)-1 and matrix[i+1][j] < 0:
                    matrix[i][j] = 2
                elif j > 0 and matrix[i][j-1] < 0:
                    matrix[i][j] = 2
                elif j < len(matrix[i])-1 and matrix[i][j+1] < 0:
                    matrix[i][j] = 2
    return matrix

def assign_groups(matrix):
    group = 1
    groups = {}
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            if matrix[i][j] == 2:
                group = dfs(matrix, i, j, group, groups)
    return matrix, groups

def dfs(matrix, i, j, group, groups):
    if i < 0 or i >= len(matrix) or j < 0 or j >= len(matrix[0]):
        return group
    if matrix[i][j] != 2:
        return group
    if group in groups:
        groups[group].append((i, j))
    else:
        groups[group] = [(i, j)]
    matrix[i][j] = 0
    dfs(matrix, i + 1, j, group, groups)
    dfs(matrix, i - 1, j, group, groups)
    dfs(matrix, i, j + 1, group, groups)
    dfs(matrix, i, j - 1, group, groups)
    dfs(matrix, i + 1, j + 1, group, groups) # sağ alt çapraz
    dfs(matrix, i - 1, j - 1, group, groups) # sol üst çapraz
    dfs(matrix, i - 1, j + 1, group, groups) # sağ üst çapraz
    dfs(matrix, i + 1, j - 1, group, groups) # sol alt çapraz
    return group + 1

def fGroups(groups):
    sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
    top_five_groups = [g for g in sorted_groups[:5] if len(g[1]) > 2]    
    return top_five_groups

def calculate_centroid(x_coords, y_coords):
    n = len(x_coords)
    sum_x = sum(x_coords)
    sum_y = sum(y_coords)
    mean_x = sum_x / n
    mean_y = sum_y / n
    centroid = (int(mean_x), int(mean_y))
    return centroid

def findClosestGroup(matrix,groups, current,resolution,originX,originY):
    targetP = None
    distances = []
    paths = []
    score = []
    max_score = -1 #max score index
    for i in range(len(groups)):
        middle = calculate_centroid([p[0] for p in groups[i][1]],[p[1] for p in groups[i][1]]) 
        path = astar(matrix, current, middle)
        path = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path]
        total_distance = pathLength(path)
        distances.append(total_distance)
        paths.append(path)
    for i in range(len(distances)):
        if distances[i] == 0:
            score.append(0)
        else:
            score.append(len(groups[i][1])/distances[i])
    for i in range(len(distances)):
        if distances[i] > target_error*3:
            if max_score == -1 or score[i] > score[max_score]:
                max_score = i
    if max_score != -1:
        targetP = paths[max_score]
    else: #gruplar target_error*2 uzaklıktan daha yakınsa random bir noktayı hedef olarak seçer. Bu robotun bazı durumlardan kurtulmasını sağlar.
        index = random.randint(0,len(groups)-1)
        target = groups[index][1]
        target = target[random.randint(0,len(target)-1)]
        path = astar(matrix, current, target)
        targetP = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path]
    return targetP

def pathLength(path):
    for i in range(len(path)):
        path[i] = (path[i][0],path[i][1])
        points = np.array(path)
    differences = np.diff(points, axis=0)
    distances = np.hypot(differences[:,0], differences[:,1])
    total_distance = np.sum(distances)
    return total_distance

def costmap(data,width,height,resolution):
    data = np.array(data).reshape(height,width)
    wall = np.where(data == 100)
    for i in range(-expansion_size,expansion_size+1):
        for j in range(-expansion_size,expansion_size+1):
            if i  == 0 and j == 0:
                continue
            x = wall[0]+i
            y = wall[1]+j
            x = np.clip(x,0,height-1)
            y = np.clip(y,0,width-1)
            data[x,y] = 100
    data = data*resolution
    return data

def exploration(data,width,height,resolution,column,row,originX,originY):
        global pathGlobal #Global degisken
        data = costmap(data,width,height,resolution) #Engelleri genislet
        data[row][column] = 0 #Robot Anlık Konum
        data[data > 5] = 1 # 0 olanlar gidilebilir yer, 100 olanlar kesin engel
        data = frontierB(data) #Sınır noktaları bul
        data,groups = assign_groups(data) #Sınır noktaları gruplandır
        groups = fGroups(groups) #Grupları küçükten büyüğe sırala. En buyuk 5 grubu al
        if len(groups) == 0: #Grup yoksa kesif tamamlandı
            path = -1
        else: #Grup varsa en yakın grubu bul
            data[data < 0] = 1 #-0.05 olanlar bilinmeyen yer. Gidilemez olarak isaretle. 0 = gidilebilir, 1 = gidilemez.
            path = findClosestGroup(data,groups,(row,column),resolution,originX,originY) #En yakın grubu bul
            if path != None: #Yol varsa BSpline ile düzelt
                path = bspline_planning(path,len(path)*5)
            else:
                path = -1
        pathGlobal = path
        return

def localControl(scan):
    v = None
    w = None
    for i in range(60):
        if scan[i] < robot_r:
            v = 0.2
            w = -math.pi/4 
            break
    if v == None:
        for i in range(300,360):
            if scan[i] < robot_r:
                v = 0.2
                w = math.pi/4
                break
    return v,w
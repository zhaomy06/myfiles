import numpy as np
import heapq
from heapdict import heapdict
import reeds_shepp as rsCurve
import math


class Node:
    def __init__(self, x, y,resolution):
        self.x = x
        self.y = y
        self.x_index = int(self.x / resolution)
        self.y_index = int(self.y / resolution)
        self.h = 0
        self.g = 0
        self.parent = ""

    def __lt__(self, other):
        if self.g + self.h < other.g + other.h:
            return True
        else:
            return False

    def set_h(self, h):
        self.h = h

    def set_g(self, g):
        self.g = g

    def set_parent(self, str):
        self.parent = str

    def get_index_str(self):
        index_str = str(self.x_index) + ',' + str(self.y_index)
        return index_str


class Global_planner():

    def __init__(self, map,params):
        self.params=params
        self.map = map
        self.dir = [[-1, -1], [1, -1], [1, 1], [-1, 1], [0, 1], [0, -1], [1, 0], [-1, 0]]

    def plan(self, start_state, end_state):
        self.start_state=start_state
        self.end_state=end_state
        start_node = Node(start_state[0], start_state[1],self.params.xy_resolution)
        end_node=Node(end_state[0],end_state[1],self.params.xy_resolution)
        temp_h = abs(end_state[0] - start_state[0]) + abs(end_state[1] - start_state[1])
        start_node.set_h(temp_h)
        pq = []
        open_list = {}
        close_list = {}
        open_list[start_node.get_index_str()] = start_node
        heapq.heappush(pq,start_node)
        self.final_node=None
        while len(pq) > 0:
            temp_node = heapq.heappop(pq)
            # 这个判断是因为在更新栅格对应的node时，不知道怎么把pq里的node删了，所以都留下了，不过淘汰的node会在新node弹出后弹出，这时这个str已经进入close_list里了
            if temp_node.get_index_str() in close_list:
                continue

            open_list.pop(temp_node.get_index_str())
            close_list[temp_node.get_index_str()] = temp_node

            if temp_node.get_index_str()==end_node.get_index_str():
                self.final_node=temp_node
                break

            node_list=self.generate_node(temp_node)

            for i in range(0,len(node_list)):
                index_str=node_list[i].get_index_str()
                if index_str in close_list:
                    continue
                if index_str in open_list:
                    if open_list[index_str].g>node_list[i].g:
                        open_list[index_str]=node_list[i]
                        heapq.heappush(pq,node_list[i])
                else:
                    open_list[index_str]=node_list[i]
                    heapq.heappush(pq,node_list[i])
        #backward
        run_node=self.final_node
        x_list=[]
        y_list=[]
        while run_node.parent!='':
            x_list.append(run_node.x)
            y_list.append(run_node.y)
            run_node=close_list[run_node.parent]

        x_list=x_list[::-1]
        y_list=y_list[::-1]
        path=[x_list,y_list]
        path=np.array(path)
        path=path.T
        return path




    def generate_node(self, parent_node):
        parent_x = parent_node.x
        parent_y = parent_node.y
        node_list = []
        for i in range(0, len(self.dir)):
            temp_x = parent_x + self.dir[i][0] * self.params.xy_resolution
            temp_y = parent_y + self.dir[i][1] * self.params.xy_resolution
            new_node = Node(temp_x, temp_y,self.params.xy_resolution)
            new_node.set_parent(parent_node.get_index_str())
            if i<=3:
                new_node.set_g(parent_node.g+1.4*self.params.xy_resolution)
            else:
                new_node.set_g(parent_node.g+self.params.xy_resolution)
            new_node.set_h(abs(self.end_state[0]-temp_x)+abs(self.end_state[1]-temp_y))

            if self.map.check_obs(new_node.x,new_node.y) == 0 and self.map.is_out(new_node.x,new_node.y) == 0:
                node_list.append(new_node)
        return node_list



class Node3d:
    def __init__(self, gridIndex, traj, steeringAngle, direction, cost, parentIndex):
        self.gridIndex = gridIndex         # grid block x, y, yaw index                     表示这个点在栅格地图中的index
        self.traj = traj                   # trajectory x, y，yaw  of a simulated                表示这个点从上一个点走来的轨迹（这个不是index这种粗略的，轨迹是准数）
        self.steeringAngle = steeringAngle # steering angle throughout the trajectory       表示这个点从上一个点走来的方向盘转角
        self.direction = direction         # direction throughout the trajectory            表示这个点从上一个点走来的前后档位
        self.cost = cost                   # node cost
        self.parentIndex = parentIndex     # parent node index                              栅格地图中上一个点的index

class Cost:
    reverse = 10
    directionChange = 150
    steerAngle = 1
    steerAngleChange = 5
    hybridCost = 1.3

class Global_planner_hybrid_astar():

    def __init__(self,map,params):
        self.map=map
        self.params=params
        self.steerPresion=self.params.steerPresion
        self.astar=Global_planner(map,self.params)
        self.yaw_resolution=self.params.yaw_resolution
        self.xy_resolution=self.params.xy_resolution
        self.radius = math.tan(self.params.steer_angle_limits) / self.params.wheelbase

    def plan(self,start,goal):
        sGridIndex = [round(start[0] / self.xy_resolution), \
                      round(start[1] / self.xy_resolution), \
                      round(start[2] / self.yaw_resolution)]
        gGridIndex = [round(goal[0] / self.xy_resolution), \
                      round(goal[1] / self.xy_resolution), \
                      round(goal[2] / self.yaw_resolution)]

        # Generate all Possible motion commands to car
        motionCommand = self.motionCommands()

        # Create start and end Node
        startNode = Node3d(sGridIndex, [start], 0, 1, 0, tuple(sGridIndex))
        goalNode = Node3d(gGridIndex, [goal], 0, 1, 0, tuple(gGridIndex))

        # Find Holonomic Heuristric
        self.astar.plan([startNode.traj[-1][0], startNode.traj[-1][1]],[goalNode.traj[-1][0], goalNode.traj[-1][1]])
        holonomicHeuristics = self.astar.final_node.g
        print("起点的h是： ",holonomicHeuristics)
        # Add start node to open Set
        openSet = {self.index(startNode): startNode}
        closedSet = {}

        # Create a priority queue for acquiring nodes based on their cost's
        costQueue = heapdict()

        # Add start mode into priority queue
        costQueue[self.index(startNode)] = Cost.hybridCost * holonomicHeuristics
        counter = 0

        currentNode=None

        # Run loop while path is found or open set is empty
        while True:
            counter += 1
            # Check if openSet is empty, if empty no solution available
            # import pdb;pdb.set_trace()
            if not openSet:
                return None

            # Get first node in the priority queue
            currentNodeIndex = costQueue.popitem()[0]
            currentNode = openSet[currentNodeIndex]

            # Revove currentNode from openSet and add it to closedSet
            openSet.pop(currentNodeIndex)
            closedSet[currentNodeIndex] = currentNode

            # Get Reed-Shepp Node if available
            rSNode = self.reedsSheppNode(currentNode, goalNode)

            # Id Reeds-Shepp Path is found exit
            if rSNode:
                closedSet[self.index(rSNode)] = rSNode
                print("RS curve work")
                print("终点的档位是", rSNode.direction)
                # 必须有这句，传到trackback函数里的终点会是goalnode，而这时goalnode是空的
                currentNode = rSNode
                break

            # USED ONLY WHEN WE DONT USE REEDS-SHEPP EXPANSION OR WHEN START = GOAL
            if currentNodeIndex == self.index(goalNode):
                print("Path Found")
                print(currentNode.traj[-1])
                print("最后一个点的direction是：", currentNode.direction)
                break

            # Get all simulated Nodes from current node
            for i in range(len(motionCommand)):  # 每个motioncommand有两个值，第一个值是方向盘转角，第二个值是前后档位
                simulatedNode = self.kinematicSimulationNode(currentNode, motionCommand[i])

                # Check if path is within map bounds and is collision free
                if not simulatedNode:
                    continue

                # Draw Simulated Node
                # x, y, z = zip(*simulatedNode.traj)
                # plt.plot(x, y, linewidth=0.3, color='g')

                # Check if simulated node is already in closed set
                simulatedNodeIndex = self.index(simulatedNode)
                if simulatedNodeIndex not in closedSet:

                    self.astar.plan([simulatedNode.traj[-1][0],simulatedNode.traj[-1][1]],[goalNode.traj[-1][0],goalNode.traj[-1][1]])
                    a_star_cost=self.astar.final_node.g


                    all_rs_path=rsCurve.calc_all_paths(simulatedNode.traj[-1][0], simulatedNode.traj[-1][1], simulatedNode.traj[-1][2],
                                           goalNode.traj[-1][0], goalNode.traj[-1][1], goalNode.traj[-1][2], self.radius,
                                           self.xy_resolution / 2)

                    min_rs_cost=9999999
                    for path in all_rs_path:
                        temp_cost=self.reedsSheppCost(simulatedNode,path)
                        has_daoche=0
                        for d in path.directions:
                            if d != 1:
                                has_daoche=1
                        if has_daoche:
                            continue
                        if temp_cost<min_rs_cost:
                            min_rs_cost=temp_cost

                    # print("这次的astar损失是",a_star_cost)
                    # Check if simulated node is already in open set, if not add it open set as well as in priority queue
                    if simulatedNodeIndex not in openSet:
                        openSet[simulatedNodeIndex] = simulatedNode
                        costQueue[simulatedNodeIndex] = simulatedNode.cost+max(a_star_cost*Cost.hybridCost,min_rs_cost)
                    else:
                        if simulatedNode.cost < openSet[simulatedNodeIndex].cost:
                            # 更新节点这一步和a_star不同，a_star是更新起点到该节点的cost，即g值。
                            # 而混合a星的意思是，还是有那么多栅格，但是node和栅格不是一一对应的关系，node和栅格都是3维的，node和node之间只有x，y，yaw都一样才判断为相等，更新时是更新栅格对应的node，
                            # 这样做是有用的，因为回溯路径时虽然得到的都是离散路径的精确值，但是这还是依靠node寻找的，无论是current node还是pre node，他们对应的都是这样的三维栅格节点
                            openSet[simulatedNodeIndex] = simulatedNode
                            costQueue[simulatedNodeIndex] = simulatedNode.cost + max(a_star_cost * Cost.hybridCost,
                                                                                     min_rs_cost)

        # Backtrack
        x, y, yaw, gear = self.backtrack(startNode, currentNode, closedSet)
        return x, y, yaw, gear

    def backtrack(self,startNode, goalNode, closedSet):

        # Goal Node data
        startNodeIndex = self.index(startNode)
        currentNodeIndex = self.index(goalNode)
        currentNode = goalNode
        # 设置第一个点的dir
        x = []
        y = []
        yaw = []
        gear=[]
        change_gear_index = []

        # Iterate till we reach start node from goal node
        # 这里的bug是第一个点的direction是none
        # 其实是run函数中，如果reeds曲线直接搜索到了终点，则会创造一个没有direction的node
        # 所以我的修改是把这种点的derection设置为0作为标记
        while currentNodeIndex != startNodeIndex:
            a, b, c = zip(*currentNode.traj)
            x += a[::-1]  # [::-1]意思是倒序
            y += b[::-1]
            yaw += c[::-1]
            gear+=len(a)*[currentNode.direction]
            k = 0
            k = k + 1
            currentNodeIndex = currentNode.parentIndex
            tempNode = currentNode
            currentNode = closedSet[currentNodeIndex]
            # -1,1,0之间无论怎么变都会检测到
            if (currentNode.direction != tempNode.direction):
                change_gear_index.append(len(x) - 1 + 1)
        # 此时得到的chage_gear_index是倒数的序列，并且指向改变档位前的节点
        for i in range(0, len(change_gear_index)):
            change_gear_index[i] = len(x) - change_gear_index[i] + 1
        ##通过每段倒序拼接的x，y是整个路径的倒序，再倒序即可返回
        return x[::-1], y[::-1], yaw[::-1],gear[::-1]

    def motionCommands(self):

        # Motion commands for a Non-Holonomic Robot like a Car or Bicycle (Trajectories using Steer Angle and Direction)
        direction = 1#表示前后档位
        motionCommand = []   
        for i in np.arange(self.params.steer_angle_limits, -(self.params.steer_angle_limits + self.params.steer_angle_limits/self.steerPresion), -self.params.steer_angle_limits/self.steerPresion):
            motionCommand.append([i, direction])
            #删去了倒车选项
            motionCommand.append([i, -direction])
        return motionCommand

    def index(self,Node):
        # Index is a tuple consisting grid index, used for checking if two nodes are near/same
        return tuple([Node.gridIndex[0], Node.gridIndex[1], Node.gridIndex[2]])

    def kinematicSimulationNode(self,currentNode, motionCommand, simulationLength=0.24, step=0.12):

        # Simulate node using given current Node and Motion Commands
        traj = []
        angle = rsCurve.pi_2_pi(
            currentNode.traj[-1][2] + motionCommand[1] * step / self.params.wheelbase * math.tan(motionCommand[0]))
        traj.append([currentNode.traj[-1][0] + motionCommand[1] * step * math.cos(angle),
                     currentNode.traj[-1][1] + motionCommand[1] * step * math.sin(angle),
                     rsCurve.pi_2_pi(angle + motionCommand[1] * step / self.params.wheelbase * math.tan(motionCommand[0]))])
        for i in range(int((simulationLength / step)) - 1):
            traj.append([traj[i][0] + motionCommand[1] * step * math.cos(traj[i][2]),
                         traj[i][1] + motionCommand[1] * step * math.sin(traj[i][2]),
                         rsCurve.pi_2_pi(
                             traj[i][2] + motionCommand[1] * step / self.params.wheelbase * math.tan(motionCommand[0]))])

        # Find grid index
        gridIndex = [round(traj[-1][0] / self.xy_resolution),
                     round(traj[-1][1] / self.xy_resolution),
                     round(traj[-1][2] / self.xy_resolution)]

        # Check if node is valid
        if not self.collision_and_boundary_is_ok(traj):
            return None

        # Calculate Cost of the node
        cost = self.simulatedPathCost(currentNode, motionCommand, simulationLength)

        return Node3d(gridIndex, traj, motionCommand[0], motionCommand[1], cost, self.index(currentNode))



    def simulatedPathCost(self,currentNode, motionCommand, simulationLength):

        # Previos Node Cost
        cost = currentNode.cost

        # Distance cost
        if motionCommand[1] == 1:
            cost += simulationLength
        else:
            cost += simulationLength * Cost.reverse

        # Direction change cost
        if currentNode.direction != motionCommand[1]:
            cost += Cost.directionChange

        # Steering Angle Cost
        cost += motionCommand[0] * Cost.steerAngle

        # Steering Angle change cost
        cost += abs(motionCommand[0] - currentNode.steeringAngle) * Cost.steerAngleChange

        return cost

    def collision_and_boundary_is_ok(self,traj):

        for i in traj:
            if not (self.map.check_ego_obs(i[0],i[1],i[2])==0 and self.map.is_out(i[0],i[1])==0):
                return False

        return True

    def reedsSheppNode(self,currentNode, goalNode):

        # Get x, y, yaw of currentNode and goalNode
        startX, startY, startYaw = currentNode.traj[-1][0], currentNode.traj[-1][1], currentNode.traj[-1][2]
        goalX, goalY, goalYaw = goalNode.traj[-1][0], goalNode.traj[-1][1], goalNode.traj[-1][2]


        #  Find all possible reeds-shepp paths between current and goal node
        reedsSheppPaths = rsCurve.calc_all_paths(startX, startY, startYaw, goalX, goalY, goalYaw, self.radius, self.xy_resolution)

        # Check if reedsSheppPaths is empty
        if not reedsSheppPaths:
            return None

        # Find path with lowest cost considering non-holonomic constraints
        costQueue = heapdict()
        for path in reedsSheppPaths:
            costQueue[path] = self.reedsSheppCost(currentNode, path)

        # Find first path in priority queue that is collision free
        while len(costQueue) != 0:
            path = costQueue.popitem()[0]
            traj = []
            has_daoche = 0
            for d in path.directions:
                if d != 1:
                    has_daoche = 1
            if has_daoche:
                continue
            traj = [[path.x[k], path.y[k], path.yaw[k]] for k in range(len(path.x))]
            if self.collision_and_boundary_is_ok(traj):
                print("collision is ok")
                cost = self.reedsSheppCost(currentNode, path)
                if path.lengths[0]>0:
                    return Node3d(goalNode.gridIndex, traj, None, 2, cost, self.index(currentNode))
                if path.lengths[0]<0:
                    return Node3d(goalNode.gridIndex, traj, None, -2, cost, self.index(currentNode))

        return None

    def reedsSheppCost(self,currentNode, path):

        # Previos Node Cost
        #cost = currentNode.cost
        cost=0
        # Distance cost
        for i in path.lengths:
            if i >= 0:
                cost += 1
            else:
                cost += abs(i) * Cost.reverse

        # Direction change cost
        for i in range(len(path.lengths) - 1):
            if path.lengths[i] * path.lengths[i + 1] < 0:
                cost += Cost.directionChange

        # Steering Angle Cost
        for i in path.ctypes:
            # Check types which are not straight line
            if i != "S":
                cost += self.params.steer_angle_limits * Cost.steerAngle

        # Steering Angle change cost
        turnAngle = [0.0 for _ in range(len(path.ctypes))]
        for i in range(len(path.ctypes)):
            if path.ctypes[i] == "R":
                turnAngle[i] = - self.params.steer_angle_limits
            if path.ctypes[i] == "WB":
                turnAngle[i] = self.params.steer_angle_limits

        for i in range(len(path.lengths) - 1):
            cost += abs(turnAngle[i + 1] - turnAngle[i]) * Cost.steerAngleChange

        return cost
import tkinter as tk
from tkinter import messagebox
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def create_edges(source, goal, center, side):
    num = 4*len(center) + 2
    edges = np.arange(num)
    coordinates = []
    coordinates.append(source)
    for i in range(len(center)):
        coordinates.append((center[i][0]-0.5*side[i][0],center[i][1]+0.5*side[i][1]))
        coordinates.append((center[i][0]+0.5*side[i][0],center[i][1]+0.5*side[i][1]))
        coordinates.append((center[i][0]-0.5*side[i][0],center[i][1]-0.5*side[i][1]))
        coordinates.append((center[i][0]+0.5*side[i][0],center[i][1]-0.5*side[i][1]))
    coordinates.append(goal)

    return edges,coordinates


def point_links(point, coordinates, rectangles):
    links = []
    for i in range(len(coordinates)):
        if(i==point):
            links.append(0)
            continue
        t = 0
        dt = 0.005
        x1 = coordinates[point][0]
        y1 = coordinates[point][1]
        x2 = coordinates[i][0]
        y2 = coordinates[i][1]
        flag = 0
        while(t<=1):
            x = (1-t)*x1 + t*x2
            y = (1-t)*y1 + t*y2
            t+=dt
            t = round(t,3)
            flag = 0
            j=0
            while(j<len(rectangles)):
                if x>(rectangles[j][0]+0.01) and x<(rectangles[j+1][0]-0.01) and y>(rectangles[j][1]+0.01) and y<(rectangles[j+1][1]-0.01):
                    if i==2 and point==1:
                        print(x,y)
                    flag = 1
                    break
                j += 2

            if flag==1:
                break
        if flag==0:
            dist = np.sqrt((coordinates[point][0] - coordinates[i][0]) ** 2 + (coordinates[point][1] - coordinates[i][1]) ** 2)
            links.append(dist)
        else:
            links.append(0)
    return links

def create_graph(rectangles, start_point, goal_point, center, side):
    graph = []
    source = start_point
    goal = goal_point
    edges, coordinates = create_edges(source, goal, center, side)
    rectangles = []
    for i in range(len(center)):
        rectangles.append((center[i][0] - 0.5 * side[i][0], center[i][1] - 0.5 * side[i][1]))
        rectangles.append((center[i][0] + 0.5 * side[i][0], center[i][1] + 0.5 * side[i][1]))
    nodes_symbols = {}
    for i in edges:
        links = point_links(i, coordinates, rectangles)
        graph.append(links)
        if (i == edges[-1]):
            nodes_symbols[i] = 'Goal'
        elif (i == edges[0]):
            nodes_symbols[i] = 'Source'
        else:
            nodes_symbols[i] = chr(i + 64)
    g = Graph(len(edges), nodes_symbols)
    g.graph = graph
    g.h = coordinates

    return g , edges

def moving_average(data, window_size=5):
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

def normalize_angle(angle):
    n = angle%360
    if n>=180:
        n-=360
    return n
class Robot:
    def __init__(self,x,y,theta,v, dt, D, r):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.dt = dt
        self.D = D
        self.r = r
        self.integral = 0
        self.previous_error = 0

    def update_position(self,omega):
        self.x += self.v * np.cos(self.theta) * self.dt
        self.y += self.v * np.sin(self.theta) * self.dt
        self.theta += omega * self.dt

    def update_omega(self, error, kp, ki, kd):
        self.integral += error * self.dt
        derivative = (error - self.previous_error)/self.dt
        self.previous_error = error
        return kp*error + ki*self.integral + kd*derivative

    def get_headings(self, waypoints):
        headings = []
        for i in range(len(waypoints)-1):
            phi = np.arctan2(waypoints[i+1][1]-waypoints[i][1],waypoints[i+1][0]-waypoints[i][0])
            headings.append(phi)
        headings.append(0)
        return headings


    def get_closest_point(self, waypoints, i):
        min = float("inf")
        j = i
        while(j<len(waypoints)):
            d = np.linalg.norm([self.x-waypoints[j][0], self.y-waypoints[j][1]])
            if min>d:
                min = d
                p = j
            j+=1
        return p

    def simulate(self, kp, ki, kd, waypoints, center, side):
        headings = self.get_headings(waypoints)
        xpath = [self.x]
        ypath = [self.y]
        theta = [self.theta]
        i = self.get_closest_point(waypoints,0)
        thetaR = [0]
        thetaL = [0]
        time = [0]
        t, flag = 0, 1
        while(1):
            i = self.get_closest_point(waypoints,i)
            if i > len(waypoints)-2:
                break
            heading = headings[i]
            h_error = heading - self.theta
            error = np.linalg.norm([self.y-waypoints[i][1],self.x-waypoints[i][0]])
            yaw = np.arctan2(self.y-waypoints[i][1],self.x-waypoints[i][0])
            m = normalize_angle(heading-yaw)
            if m<0:
                error = -error
            error = max(-2, min(2, error))

            omega = self.update_omega(0.2 * error + 0.8*h_error, kp, ki, kd)
            thetaR.append((2 * self.v + omega * self.D)/(2 * self.r))
            thetaL.append((2 * self.v - omega * self.D) / (2 * self.r))
            t += self.dt
            time.append(t)
            self.update_position(omega)
            xpath.append(self.x)
            ypath.append(self.y)
            theta.append(self.theta)

        fig, ax = plt.subplots()
        ax.plot(xpath, ypath, label='Trajectory')
        for i in range(len(center)):
            rectangle = patches.Rectangle(((center[i][0] - 0.5 * side[i][0], center[i][1] - 0.5 * side[i][1])),
                                          side[i][0], side[i][1], facecolor='grey')
            ax.add_patch(rectangle)
        plt.grid()
        ax.legend()
        plt.show()

        from scipy.ndimage import gaussian_filter1d

        thetaR_smooth = gaussian_filter1d(thetaR, sigma=2)
        thetaL_smooth = gaussian_filter1d(thetaL, sigma=2)

        plt.plot(time[6:], thetaR_smooth[6:], color='red', label='thetaR')
        plt.plot(time[6:], thetaL_smooth[6:], color='blue', label='thetaL')
        plt.legend()
        plt.grid()
        plt.show()

        theta = gaussian_filter1d(theta, sigma=2)
        plt.figure(figsize=(10, 6))
        plt.plot(time, theta, label='phi')
        plt.grid()
        plt.legend()
        plt.show()

    def animate(self, kp, ki, kd, waypoints, center, side):
        headings = self.get_headings(waypoints)
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.grid()
        scat = ax.scatter([], [], color='blue')
        arrow = None
        xx = [p[0] for p in waypoints]
        yy = [p[1] for p in waypoints]
        ax.plot(xx,yy)
        for i in range(len(center)):
            rectangle = patches.Rectangle(((center[i][0] - 0.5 * side[i][0], center[i][1] - 0.5 * side[i][1])),
                                          side[i][0], side[i][1], facecolor='grey')
            ax.add_patch(rectangle)
        i = self.get_closest_point(waypoints,0)
        flag, t = 1, 0
        Vmax = self.v
        self.v = 0
        while(1):
            i = self.get_closest_point(waypoints,i)
            if i > len(waypoints)-2:
                break
            heading = headings[i]
            h_error = heading - self.theta
            error = np.linalg.norm([self.y-waypoints[i][1],self.x-waypoints[i][0]])
            yaw = np.arctan2(self.y-waypoints[i][1],self.x-waypoints[i][0])
            m = normalize_angle(heading-yaw)
            if m<0:
                error = -error
            error = max(-2, min(2, error))
            t += self.dt
            if self.v<Vmax and flag:
                self.v = Vmax*(1-np.cos(np.pi*t/2))
                if self.v >= Vmax:
                    flag = 0
            omega = self.update_omega(0.2 * error + h_error, kp, ki, kd)
            self.update_position(omega)

            scat.set_offsets([self.x, self.y])
            if arrow:
                arrow.remove()
            arrow = ax.arrow(self.x, self.y, 0.1 * np.cos(self.theta), 0.1 * np.sin(self.theta),
                             head_width=2, head_length=4, fc='red', ec='red')
            plt.pause(0.05)


class RRT_star:

    class Node:
        def __init__(self, x ,y):
            self.x = x
            self.y = y
            self.parent =None
            self.cost = 0

    class Obstacle:
        def __init__(self,center,side):
            self.xmin = center[0] - 0.5 * side[0] -1.5
            self.xmax = center[0] + 0.5 * side[0] +1.5
            self.ymin = center[1] - 0.5 * side[1] -1.5
            self.ymax = center[1] + 0.5 * side[1] +1.5


    def __init__(self, start, goal, centers, sides, map_size, step_size, num_iter):

        self.start = self.Node(start[0],start[1])
        self.goal = self.Node(goal[0], goal[1])
        self.obstacles = self.create_obstacles_list(centers,sides)
        self.map_size = map_size
        self.step_size = step_size
        self.num_iter = num_iter
        self.node_list = [self.start]

    def create_obstacles_list(self, centers, sides):
            obstacles = []
            for i in range(len(centers)):
                obj = self.Obstacle(centers[i], sides[i])
                obstacles.append(obj)
            return obstacles

    def get_random_node(self):
        if np.random.randint(0, 100) > 5:
            Nrnd = self.Node(
                np.random.randint(self.map_size[0],self.map_size[1]),
                np.random.randint(self.map_size[0],self.map_size[1]))
        else:
            Nrnd = self.Node(self.goal.x, self.goal.y)
        return Nrnd

    def find_nearest_node(self, Nrnd):
        min = 1e6
        for node in self.node_list:
            d = np.linalg.norm([node.x-Nrnd.x, node.y-Nrnd.y])
            if min>d:
                min = d
                Nnearest = node
        return Nnearest

    def create_new_node(self, Nnearest, Nrnf):
        theta = np.arctan2(Nrnf.y-Nnearest.y,Nrnf.x-Nnearest.x)
        xnew = Nnearest.x + self.step_size*np.cos(theta)
        ynew = Nnearest.y + self.step_size * np.sin(theta)
        Nnew = self.Node(xnew,ynew)
        return Nnew

    def check_collision(self, Nnew):
        for obj in self.obstacles:
            if Nnew.x > obj.xmin and Nnew.x < obj.xmax and Nnew.y > obj.ymin and Nnew.y < obj.ymax:
                return 1
        return 0

    def create_path(self):
        path = [self.node_list[-1]]
        while(1):
            path.append(path[-1].parent)
            if path[-1]==self.start:
                break
        path = list(reversed(path))

        path_points = []
        for node in path:
            path_points.append([node.x,node.y])

        return path_points

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list)
        radius = 50 * np.sqrt(np.log(nnode) / nnode)
        near_nodes = [node for node in self.node_list
                if np.linalg.norm([node.x - new_node.x, node.y - new_node.y]) < radius]

        return near_nodes

    def check_link(self, node1, node2):
        dt = 0.05
        x1 = node1.x
        y1 = node1.y
        x2 = node2.x
        y2 = node2.y
        for obj in self.obstacles:
            t = 0
            while (t <= 1):
                x = (1 - t) * x1 + t * x2
                y = (1 - t) * y1 + t * y2
                t += dt
                if x > obj.xmin and x < obj.xmax and y > obj.ymin and y < obj.ymax:
                    return 0
        return 1


    def choose_parent(self, near_nodes, nearest_node, new_node):
        min_cost = nearest_node.cost + np.linalg.norm([new_node.x - nearest_node.x, new_node.y - nearest_node.y])
        parent = nearest_node

        for neighbor in near_nodes:
            cost = neighbor.cost + np.linalg.norm([new_node.x - neighbor.x, new_node.y - neighbor.y])
            if cost < min_cost and self.check_link(new_node,neighbor):
                parent = neighbor
                min_cost = cost

        new_node.cost = min_cost
        new_node.parent = parent
        return new_node

    def rewire(self, new_node, near_nodes):
        for neighbor in near_nodes:
            cost = new_node.cost + np.linalg.norm([neighbor.x - new_node.x, neighbor.y - new_node.y])
            if cost < neighbor.cost and self.check_link(new_node, neighbor):
                neighbor.parent = new_node
                neighbor.cost = cost

    def planning(self):
        i = 0
        while(i < self.num_iter):
            print(i)
            Nrnd = self.get_random_node()
            Nnearest = self.find_nearest_node(Nrnd)
            Nnew = self.create_new_node(Nnearest, Nrnd)
            if not self.check_link(Nnew, Nnearest):
                continue
            near_nodes = self.find_near_nodes(Nnew)
            Nnew = self.choose_parent(near_nodes, Nnearest, Nnew)
            self.node_list.append(Nnew)
            self.rewire(Nnew, near_nodes)
            if np.sqrt((Nnew.x-self.goal.x)**2 + (Nnew.y-self.goal.y)**2) < 1:
                break

            i+=1
            if i==self.num_iter:
                i = 0
                self.node_list = [self.start]

        path_points = self.create_path()
        return path_points



class RRT:

    class Node:
        def __init__(self, x ,y):
            self.x = x
            self.y = y
            self.parent =None

    class Obstacle:
        def __init__(self,center,side):
            self.xmin = center[0] - 0.5 * side[0] -1.5
            self.xmax = center[0] + 0.5 * side[0] +1.5
            self.ymin = center[1] - 0.5 * side[1] -1.5
            self.ymax = center[1] + 0.5 * side[1] +1.5

    def __init__(self, start, goal, centers, sides, map_size, step_size, num_iter):

        self.start = self.Node(start[0], start[1])
        self.goal = self.Node(goal[0], goal[1])
        self.obstacles = self.create_obstacles_list(centers, sides)
        self.map_size = map_size
        self.step_size = step_size
        self.num_iter = num_iter
        self.node_list = [self.start]

    def create_obstacles_list(self, centers, sides):
            obstacles = []
            for i in range(len(centers)):
                obj = self.Obstacle(centers[i], sides[i])
                obstacles.append(obj)
            return obstacles

    def get_random_node(self):
        if np.random.randint(0, 100) > 5:
            Nrnd = self.Node(
                np.random.randint(self.map_size[0],self.map_size[1]),
                np.random.randint(self.map_size[0],self.map_size[1]))
        else:
            Nrnd = self.Node(self.goal.x, self.goal.y)
        return Nrnd

    def find_nearest_node(self, Nrnd):
        min = 1e6
        for node in self.node_list:
            d = np.linalg.norm([node.x-Nrnd.x, node.y-Nrnd.y])
            if min>d:
                min = d
                Nnearest = node
        return Nnearest

    def create_new_node(self, Nnearest, Nrnf):
        theta = np.arctan2(Nrnf.y-Nnearest.y,Nrnf.x-Nnearest.x)
        xnew = Nnearest.x + self.step_size * np.cos(theta)
        ynew = Nnearest.y + self.step_size * np.sin(theta)
        Nnew = self.Node(xnew,ynew)
        return Nnew

    def check_collision(self, Nnew):
        for obj in self.obstacles:
            if Nnew.x > obj.xmin and Nnew.x < obj.xmax and Nnew.y > obj.ymin and Nnew.y < obj.ymax:
                return 1
        return 0

    def check_link(self, node1, node2):
        dt = 0.05
        x1 = node1.x
        y1 = node1.y
        x2 = node2.x
        y2 = node2.y
        for obj in self.obstacles:
            t = 0
            while (t <= 1):
                x = (1 - t) * x1 + t * x2
                y = (1 - t) * y1 + t * y2
                t += dt
                if x > obj.xmin and x < obj.xmax and y > obj.ymin and y < obj.ymax:
                    return 0
        return 1

    def create_path(self):
        path = [self.node_list[-1]]
        while(1):
            path.append(path[-1].parent)
            if path[-1]==self.start:
                break
        path = list(reversed(path))

        path_points = []
        for node in path:
            path_points.append([node.x,node.y])

        return path_points


    def planning(self):
        i = 0
        while (i < self.num_iter):
            Nrnd = self.get_random_node()
            Nnearest = self.find_nearest_node(Nrnd)
            Nnew = self.create_new_node(Nnearest, Nrnd)
            if  not self.check_link(Nnew, Nnearest):
                continue
            self.node_list.append(Nnew)

            Nnew.parent = Nnearest
            if np.sqrt((Nnew.x-self.goal.x)**2 + (Nnew.y-self.goal.y)**2) < 1:
                break
            print(i)
            i += 1
            if i == self.num_iter-1:
                i = 0
                self.node_list = [self.start]

        path_points = self.create_path()
        return path_points


class APF:
    def __init__(self, start, goal, obstacles, side, Katt, Krep, step_size, Dgoal):
        self.start = start
        self.q_goal = np.array(goal, dtype=float)
        self.q_obj = obstacles
        self.side = side
        self.rho0 = self.create_rho()
        self.Katt = Katt
        self.Krep = Krep
        self.step_size = step_size
        self.Dgoal = Dgoal
        self.q_robot = np.array(start, dtype=float)


    def create_rho(self):
        rho = []
        for i in range(len(self.side)):
            rho.append(max(self.side[i][0], self.side[i][1]))
        rho = 1 * np.array(rho, dtype=float)
        return rho


    def dist(self, q):
        x_robot = self.q_robot[0]
        y_robot = self.q_robot[1]
        x = q[0]
        y = q[1]
        return np.linalg.norm([x_robot - x, y_robot - y])

    def Fatt(self):
        return self.Katt * (self.q_goal - self.q_robot)

    def Frep(self, q, rho):
        r = self.dist(q)
        return ((self.Krep/r**3)*(1/r - 1/rho)*(self.q_robot - q)*abs(self.q_robot-self.q_goal)**4) - (4/2)*(self.Krep*(1/r - 1/rho)**2)*((self.q_robot-self.q_goal)**3)

    def calculate_rep(self):
        sum = 0
        for i, q in enumerate(self.q_obj):
            if self.dist(q) < self.rho0[i]:
                sum += self.Frep(q, self.rho0[i])
        return sum

    def no_collision(self, point):
        for i in range(len(self.q_obj)):
            xmin = self.q_obj[i][0] - 0.5 * self.side[i][0]-self.step_size
            xmax = self.q_obj[i][0] + 0.5 * self.side[i][0]+self.step_size
            ymin = self.q_obj[i][1] - 0.5 * self.side[i][1]-self.step_size
            ymax = self.q_obj[i][1] + 0.5 * self.side[i][1]+self.step_size
            if point[0]>xmin and point[0]<xmax and point[1]>ymin and point[1]<ymax:
                return 0
        return 1

    def Jitter(self, theta_prev, delta_theta):
            x = self.q_robot[0] + 0.2 * self.step_size * np.cos(theta_prev + 0.5*delta_theta)
            y = self.q_robot[1] + 0.2 * self.step_size * np.sin(theta_prev + 0.5 * delta_theta)
            if self.no_collision([x,y]):
                self.q_robot[0] = x
                self.q_robot[1] = y
            else:
                self.q_robot[0] = self.q_robot[0] + 0.2 * self.step_size * np.cos(theta_prev + delta_theta)
                self.q_robot[1] = self.q_robot[1] + 0.2 * self.step_size * np.sin(theta_prev + delta_theta)


    def path(self):
        self.x_vals = [self.start[0]]
        self.y_vals = [self.start[1]]
        px = []
        py = []
        theta_prev = 0
        tol = self.step_size / 20
        adjacent = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
        while(1):
            if not self.no_collision(self.q_robot):
                print("collision")
                break
            if self.dist(self.q_goal) < self.Dgoal:
                print("We have reached the goal point")
                break
            Fatt = self.Fatt()
            Frep = self.calculate_rep()
            F = Frep + Fatt
            theta = np.arctan2(F[1], F[0])
            delta_theta = theta-theta_prev
            if len(self.x_vals)>1 and abs(delta_theta)>np.pi/2:
                self.Jitter(theta_prev, delta_theta)
            else:
                a = np.array([np.cos(theta), np.sin(theta)])
                self.q_robot += a * self.step_size

            self.x_vals.append(self.q_robot[0])
            self.y_vals.append(self.q_robot[1])
            theta_prev = theta


        xx = [self.x_vals[0]]
        yy = [self.y_vals[0]]
        for i in range(len(self.x_vals)):
            if abs(xx[-1]-self.x_vals[i])>4*self.step_size or abs(yy[-1]-self.y_vals[i])>4*self.step_size:
                xx.append(self.x_vals[i])
                yy.append(self.y_vals[i])

        path_points = []
        for i in range(len(xx)):
            path_points.append([xx[i], yy[i]])

        return path_points

class TwoWheeledRobot:
    def __init__(self, wheel_radius=1, robot_width=5):
        self.wheel_radius = wheel_radius
        self.robot_width = robot_width

    def update_position(self, time, epsilon, omega, c1, c2, c3, z1_0, z1_T, z2_0, z2_T, z3_0, z3_T):

        if (time <= epsilon):
            t = time
            self.x = z1_0
            self.y = z3_0
            self.theta = np.arctan(c1 * t - c1 * np.sin(omega * t) / omega + z2_0)
            self.v1 = 0
            self.v2 = c1 * (1 - np.cos(omega * t))
        elif (time <= 2 * epsilon):
            t = time - epsilon
            self.x = c2 * t - c2 * np.sin(omega * t) / omega + z1_0
            self.y = (c1 * epsilon + z2_0) * (c2 * t - c2 * np.sin(omega * t) / omega) + z3_0
            self.v1 = c2 * (1 - np.cos(omega * t))
            self.v2 = 0
        else:
            t = time - 2 * epsilon
            self.theta = np.arctan(c3 * t - c3 * np.sin(omega * t) / omega + c1 * epsilon + z2_0)
            self.v1 = 0
            self.v2 = c3 * (1 - np.cos(omega * t))

    def get_position(self):
        return self.x, self.y, self.theta

    def simulate_robot_movement(self, path_points, duration, center, side):
        dt = 0.1
        time = 0
        positions = []
        v1 = []
        v2 = []
        time_vals = []
        epsilon = duration / 3
        omega = 2 * np.pi / epsilon

        for i in range(len(path_points) - 1):
            z1_0 = path_points[i][0]
            z2_0 = np.tan(0)
            z3_0 = path_points[i][1]
            z1_T = path_points[i + 1][0]
            z2_T = np.tan(0)
            z3_T = path_points[i + 1][1]
            c2 = (z1_T - z1_0) / epsilon
            c1 = ((z3_T - z3_0) / (epsilon * c2) - z2_0) / epsilon
            c3 = (z2_T - z2_0) / epsilon - c1

            while (time <= (i + 1) * duration):
                self.update_position(time - i * duration, epsilon, omega, c1, c2, c3, z1_0, z1_T, z2_0, z2_T, z3_0,
                                     z3_T)
                time += dt
                positions.append(self.get_position())
                v1.append(self.v1)
                v2.append(self.v2)
                time_vals.append(time)

        x_vals = [pos[0] for pos in positions]
        y_vals = [pos[1] for pos in positions]
        headings = [pos[2] for pos in positions]
        u1 = np.array(v1) / np.cos(headings)
        u2 = np.array(v2) * np.cos(headings) * np.cos(headings)
        thetaRdot = u1 / self.wheel_radius + (self.robot_width / (2 * self.wheel_radius)) * u2
        thetaLdot = 2 * u1 / self.wheel_radius - thetaRdot
        xx = [p[0] for p in path_points]
        yy = [p[1] for p in path_points]

        fig, ax = plt.subplots()
        ax.plot(xx, yy, label='Trajectory')
        for i in range(len(center)):
            rectangle = patches.Rectangle(((center[i][0] - 0.5 * side[i][0], center[i][1] - 0.5 * side[i][1])),
                                          side[i][0], side[i][1],facecolor='grey')
            ax.add_patch(rectangle)

        plt.grid()
        ax.legend()
        plt.show()

        plt.figure(figsize=(10, 6))
        plt.plot(time_vals, thetaRdot, label='thetaRdot', color='red')
        plt.plot(time_vals, thetaLdot, label='thetaLdot', color='blue')
        plt.grid()
        plt.legend()
        plt.show()

        plt.figure(figsize=(10, 6))
        plt.plot(time_vals, v1, label='v1', color='red')
        plt.plot(time_vals, v2, label='v2', color='blue')
        plt.grid()
        plt.legend()
        plt.show()

        plt.figure(figsize=(10, 6))
        plt.plot(time_vals, headings, label='phi')
        plt.grid()
        plt.legend()
        plt.show()


        return thetaRdot, thetaLdot

    def animate(self, path_points, duration, center, side):
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.set_title("Robot Movement Animation with Heading")
        ax.axis('equal')
        ax.grid()
        xx = [p[0] for p in path_points]
        yy = [p[1] for p in path_points]
        ax.plot(xx, yy)
        for i in range(len(center)):
            rectangle = patches.Rectangle(((center[i][0] - 0.5 * side[i][0], center[i][1] - 0.5 * side[i][1])),
                                          side[i][0], side[i][1], facecolor='grey')
            ax.add_patch(rectangle)

        scat = ax.scatter([], [], color='blue')
        arrow = None
        r = max(path_points[-1][0], path_points[-1][1])

        dt = 0.1
        time = 0
        epsilon = duration / 3
        omega = 2 * np.pi / epsilon
        frame = 0

        for i in range(len(path_points) - 1):
            z1_0 = path_points[i][0]
            z2_0 = np.tan(0)
            z3_0 = path_points[i][1]
            z1_T = path_points[i + 1][0]
            z2_T = np.tan(0)
            z3_T = path_points[i + 1][1]
            c2 = (z1_T - z1_0) / epsilon
            c1 = ((z3_T - z3_0) / (epsilon * c2) - z2_0) / epsilon
            c3 = (z2_T - z2_0) / epsilon - c1

            while (time <= (i + 1) * duration):
                frame += 1
                self.update_position(time - i * duration, epsilon, omega, c1, c2, c3, z1_0, z1_T, z2_0, z2_T, z3_0,
                                     z3_T)
                time += dt
                x, y, theta = self.get_position()

                scat.set_offsets([x, y])

                if arrow:
                    arrow.remove()
                arrow = ax.arrow(x, y, 0.1 * np.cos(theta), 0.1 * np.sin(theta),
                                 head_width=0.1*r/6, head_length=0.175*r/6, fc='red', ec='red')

                plt.pause(0.05)

        plt.show()


class Graph():
    def __init__(self, vertices, nodes_symbols):
        self.V = vertices
        self.nodes_symbols = nodes_symbols

    def printSolution(self, dist):
        for node in range(self.V):
            print(f'{node}:   {dist[node]}')

    def printShortestPath(self, goal):
        shortest_path = [goal]
        parent = self.parents
        print(self.nodes_symbols[goal], end="<-")
        for i in range(self.V):
            if parent[goal] == self.src:
                print(self.nodes_symbols[parent[goal]])
                shortest_path.append(parent[goal])
                break
            else:
                print(self.nodes_symbols[parent[goal]], end="<-")
                shortest_path.append(parent[goal])

            goal = parent[goal]
        shortest_path = list(reversed(shortest_path))
        return shortest_path

    def minDistance(self, dist, visited):

        min = 1e6
        min_node = None

        for v in range(self.V):
            if dist[v] < min and visited[v] == False:
                min = dist[v]
                min_node = v

        return min_node

    def dijkstra(self, src):
        self.src = src
        dist = [1e6] * self.V
        dist[src] = 0
        visited = [False] * self.V
        parent = [None] * self.V

        for node in range(self.V):

            u = self.minDistance(dist, visited)
            visited[u] = True

            for v in range(self.V):
                if (self.graph[u][v] > 0 and visited[v] == False) and (dist[v] > dist[u] + self.graph[u][v]):
                    dist[v] = dist[u] + self.graph[u][v]
                    parent[v] = u

        self.parents = parent

    def heuristic(self, node, goal):
        return math.sqrt((self.h[node][0] - self.h[goal][0]) ** 2 + (self.h[node][1] - self.h[goal][1]) ** 2)

    def a_star(self, src, goal):
        self.src = src
        dist = [1e5] * self.V
        dist[src] = 0
        score = [1e5] * self.V
        score[src] = self.heuristic(src, goal)
        visited = [False] * self.V
        parent = [None] * self.V

        for node in range(self.V):

            u = self.minDistance(score, visited)
            visited[u] = True

            for v in range(self.V):
                if (self.graph[u][v] > 0 and visited[v] == False) and (
                        score[v] > dist[u] + self.graph[u][v] + self.heuristic(v, goal)):
                    dist[v] = dist[u] + self.graph[u][v]
                    score[v] = dist[u] + self.graph[u][v] + self.heuristic(v, goal)
                    parent[v] = u

        self.parents = parent


class PathPlanningGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Trajectory Planning GUI")

        # Initialize data storage
        self.rectangles = []
        self.start_point = None
        self.goal_point = None
        self.saved_data = None
        self.flag = 0

        # Create UI components
        self.create_rectangle_inputs()
        self.create_robot_inputs()
        self.create_algorithm_selection()
        self.create_buttons()

    def create_algorithm_selection(self):
        self.algorithm_frame = tk.LabelFrame(self.root, text="Path Planning Algorithm")
        self.algorithm_frame.pack(pady=10)

        self.flag = tk.IntVar()

        tk.Radiobutton(self.algorithm_frame, text="Dijkstra", variable=self.flag, value=1).pack(anchor=tk.W)
        tk.Radiobutton(self.algorithm_frame, text="A*", variable=self.flag, value=2).pack(anchor=tk.W)
        tk.Radiobutton(self.algorithm_frame, text="APF", variable=self.flag, value=3).pack(anchor=tk.W)
        tk.Radiobutton(self.algorithm_frame, text="RRT", variable=self.flag, value=4).pack(anchor=tk.W)
        tk.Radiobutton(self.algorithm_frame, text="RRT*", variable=self.flag, value=5).pack(anchor=tk.W)

    def create_rectangle_inputs(self):
        frame = tk.LabelFrame(self.root, text="Rectangle Parameters", padx=10, pady=10)
        frame.pack(padx=10, pady=5, fill="x")

        tk.Label(frame, text="Center X (cx):").grid(row=0, column=0, padx=5, pady=5)
        self.cx_entry = tk.Entry(frame)
        self.cx_entry.grid(row=0, column=1, padx=5, pady=5)

        tk.Label(frame, text="Center Y (cy):").grid(row=1, column=0, padx=5, pady=5)
        self.cy_entry = tk.Entry(frame)
        self.cy_entry.grid(row=1, column=1, padx=5, pady=5)

        tk.Label(frame, text="Size X (sx):").grid(row=2, column=0, padx=5, pady=5)
        self.sx_entry = tk.Entry(frame)
        self.sx_entry.grid(row=2, column=1, padx=5, pady=5)

        tk.Label(frame, text="Size Y (sy):").grid(row=3, column=0, padx=5, pady=5)
        self.sy_entry = tk.Entry(frame)
        self.sy_entry.grid(row=3, column=1, padx=5, pady=5)

        self.add_rectangle_button = tk.Button(frame, text="Add Rectangle", command=self.add_rectangle)
        self.add_rectangle_button.grid(row=4, columnspan=2, pady=5)

    def create_robot_inputs(self):
        frame = tk.LabelFrame(self.root, text="Robot Parameters", padx=10, pady=10)
        frame.pack(padx=10, pady=5, fill="x")

        tk.Label(frame, text="Start X (x0):").grid(row=0, column=0, padx=5, pady=5)
        self.x0_entry = tk.Entry(frame)
        self.x0_entry.grid(row=0, column=1, padx=5, pady=5)

        tk.Label(frame, text="Start Y (y0):").grid(row=1, column=0, padx=5, pady=5)
        self.y0_entry = tk.Entry(frame)
        self.y0_entry.grid(row=1, column=1, padx=5, pady=5)

        tk.Label(frame, text="Goal X (xf):").grid(row=2, column=0, padx=5, pady=5)
        self.xf_entry = tk.Entry(frame)
        self.xf_entry.grid(row=2, column=1, padx=5, pady=5)

        tk.Label(frame, text="Goal Y (yf):").grid(row=3, column=0, padx=5, pady=5)
        self.yf_entry = tk.Entry(frame)
        self.yf_entry.grid(row=3, column=1, padx=5, pady=5)

        self.set_points_button = tk.Button(frame, text="Set Points", command=self.set_points)
        self.set_points_button.grid(row=4, columnspan=2, pady=5)

    def create_buttons(self):
        frame = tk.Frame(self.root)
        frame.pack(pady=10)

        self.simulate_button = tk.Button(frame, text="Simulate", command=self.simulate)
        self.simulate_button.pack(side="left", padx=10)

        self.animate_button = tk.Button(frame, text="Animate", command=self.animate)
        self.animate_button.pack(side="left", padx=10)

        self.quit_button = tk.Button(frame, text="Quit", command=self.root.quit)
        self.quit_button.pack(side="left", padx=10)

    def add_rectangle(self):
        try:
            cx = float(self.cx_entry.get())
            cy = float(self.cy_entry.get())
            sx = float(self.sx_entry.get())
            sy = float(self.sy_entry.get())
            self.rectangles.append([[cx, cy], [sx, sy]])
            messagebox.showinfo("Success", "Rectangle added successfully!")
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numbers for the rectangle parameters.")

    def set_points(self):
        try:
            x0 = float(self.x0_entry.get())
            y0 = float(self.y0_entry.get())
            xf = float(self.xf_entry.get())
            yf = float(self.yf_entry.get())
            self.start_point = [x0, y0]
            self.goal_point = [xf, yf]
            messagebox.showinfo("Success", "Start and goal points set successfully!")
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numbers for the robot parameters.")

    def save_data(self):
        if not self.start_point or not self.goal_point:
            messagebox.showerror("Error", "Please set the start and goal points before saving.")
            return
        if self.start_point==self.goal_point:
            messagebox.showerror("Error", "Start and Goal points are the same.")
            return
        center = [element[0] for element in self.rectangles]
        side = [element[1] for element in self.rectangles]
        s = self.start_point
        g = self.goal_point
        for i in range(len(center)):
            xmin = center[i][0]-0.5*side[i][0]
            xmax = center[i][0]+0.5*side[i][0]
            ymin = center[i][1]-0.5*side[i][1]
            ymax = center[i][1]+0.5*side[i][1]
            if (s[0]>xmin and s[0]<xmax and s[1]>ymin and s[1]<ymax):
                messagebox.showerror("Error", "Start point inside a rectangle.")
                return
            if (g[0]>xmin and g[0]<xmax and g[1]>ymin and g[1]<ymax):
                messagebox.showerror("Error", "Goal point inside a rectangle.")
                return

        f = self.flag.get()
        if f!=1 and f!=2 and f!=3 and f!=4 and f!=5:
            messagebox.showerror("Error", "Please set the path planning algorithm.")
            return


        return self.rectangles, self.start_point, self.goal_point

    def simulate(self):
        flag = self.flag.get()
        data = self.save_data()
        if not data:
            return
        rectangles, start_point, goal_point = data
        center = [element[0] for element in rectangles]
        side = [element[1] for element in rectangles]
        print("Simulating")

        if flag==1:
            g, edges = create_graph(rectangles, start_point, goal_point, center, side)
            g.dijkstra(edges[0])
            print("The first rectangle has edges of names A-B-C-D and the second")
            print("one has edges of names E-F-G-H and so on....")
            shortest_path = g.printShortestPath(edges[-1])
            path_points = [g.h[i] for i in shortest_path]
        if flag==2:
            g, edges = create_graph(rectangles, start_point, goal_point, center, side)
            g.a_star(edges[0], edges[-1])
            print("The first rectangle has edges of names A-B-C-D and the second")
            print("one has edges of names E-F-G-H and so on....")
            shortest_path = g.printShortestPath(edges[-1])
            path_points = [g.h[i] for i in shortest_path]
        if flag == 3:
            abf = APF(start_point, goal_point, center, side, 0.5, 40000, 0.5, 1)
            path_points = abf.path()

        if flag == 4:
            map_size = [0, 200]
            step_size = 2
            num_iter = 5000
            r = RRT(start_point, goal_point, center, side, map_size, step_size, num_iter)
            path_points = r.planning()

        if flag == 5:
            map_size = [0, 200]
            step_size = 2
            num_iter = 5000
            r = RRT_star(start_point, goal_point, center, side, map_size, step_size, num_iter)
            path_points = r.planning()

        if flag>2:
            theta0 = np.arctan2(path_points[1][1] - path_points[0][1], path_points[1][0] - path_points[0][0])
            robot = Robot(start_point[0], start_point[1], theta0, 15, 0.01, 1, 0.3)
            robot.simulate(15, 0, 0, path_points, center, side)
        else:
            robot = TwoWheeledRobot()
            robot.simulate_robot_movement(path_points, 9, center, side)

    def animate(self):
        flag = self.flag.get()
        data = self.save_data()
        if not data:
            return
        rectangles, start_point, goal_point = data
        center = [element[0] for element in rectangles]
        side = [element[1] for element in rectangles]
        print("Animating")

        if flag == 1:
            g, edges = create_graph(rectangles, start_point, goal_point, center, side)
            g.dijkstra(edges[0])
            print("The first rectangle has edges of names A-B-C-D and the second")
            print("one has edges of names E-F-G-H and so on....")
            shortest_path = g.printShortestPath(edges[-1])
            path_points = [g.h[i] for i in shortest_path]
        if flag == 2:
            g, edges = create_graph(rectangles, start_point, goal_point, center, side)
            g.a_star(edges[0], edges[-1])
            print("The first rectangle has edges of names A-B-C-D and the second")
            print("one has edges of names E-F-G-H and so on....")
            shortest_path = g.printShortestPath(edges[-1])
            path_points = [g.h[i] for i in shortest_path]
        if flag == 3:
            abf = APF(start_point, goal_point, center, side, 0.5, 40000, 0.5, 1)
            path_points = abf.path()
        if flag == 4:
            map_size = [0, 200]
            step_size = 2
            num_iter = 5000
            r = RRT(start_point, goal_point, center, side, map_size, step_size, num_iter)
            path_points = r.planning()

        if flag == 5:
            map_size = [0, 200]
            step_size = 2
            num_iter = 5000
            r = RRT_star(start_point, goal_point, center, side, map_size, step_size, num_iter)
            path_points = r.planning()

        if flag>2:
            theta0 = np.arctan2(path_points[1][1] - path_points[0][1], path_points[1][0] - path_points[0][0])
            robot = Robot(start_point[0], start_point[1], theta0, 50, 0.01, 0.3, 0.1)
            robot.animate(15, 0, 0, path_points, center, side)
        else:
            robot = TwoWheeledRobot()
            robot.animate(path_points, 9, center, side)

if __name__ == "__main__":
    root = tk.Tk()
    app = PathPlanningGUI(root)
    root.mainloop()

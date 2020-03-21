import numpy as np
import time
import math
import matplotlib.pyplot as plt
import queue
import argparse

plt.ion()

height = 200
width = 300


def check(x, y, r, c, c1):
    # Rhombus
    if ((x * (-3 / 5) + y - 55 - r - c < 0) and (x * (3 / 5) + y - 325 - r - c < 0) and (
            x * (-3 / 5) + y - 25 + r + c > 0) and (x * (3 / 5) + y - 295 + r + c > 0)):
        return True

    # polygon - rhombus
    elif x * (7 / 5) + y - 120 > 0 and x * (-6 / 5) + y + 10 - c - r < 0 and x * (6 / 5) + y - 170 - c - r < 0 and x * (
            -7 / 5) + y + 90 + c + r > 0:
        return True

    # polygon - triangle1
    elif y - 15 + c + r > 0 and x * (7 / 5) + y - 120 < 0 and x * (-7 / 5) + y + 20 < 0:
        return True

    # polygon - triangle2
    elif y + 13 * x - 340 + c + r + c1 > 0 and x + y - 100 - r - c < 0 and x * (-7 / 5) + y + 20 > 0:
        return True

    # rectangle -angled
    elif (200 - y) - (1.73) * x + 135 + r + c > 0 and (200 - y) + (0.58) * x - 96.35 - r - c <= 0 and (200 - y) - (
            1.73) * x - 15.54 - r - c <= 0 and (200 - y) + (0.58) * x - 84.81 + r + c >= 0:
        return True

    else:
        return False


def obstaclecheck_circle(x, y, r, c):
    if (((x - 225) ** 2) + ((y - 50) ** 2) < ((25 + r + c) ** 2)):
        return True
    else:
        return False


def obstaclecheck_ellipse(x, y, r, c):
    if (((x - 150) ** 2 / (40 + c + r) ** 2) + ((y - 100) ** 2) / (20 + c + r) ** 2) <= 1:
        return True
    else:
        return False


def obstacle_check(new_i, new_j, r, c):
    c1 = 40
    if obstaclecheck_circle(new_i, new_j, r, c):
        return True
    elif obstaclecheck_ellipse(new_i, new_j, r, c):
        return True
    elif check(new_i, new_j, r, c, c1):
        return True
    elif ((new_i - r - c < 0) or (200 - new_j - r - c < 0) or (new_i + r + c > width - 1) or (
            200 - new_j + r + c > height - 1)):
        return True
    else:
        return False


def goalcheck_circle(x, y, goal_x, goal_y):
    if (((x - goal_x) ** 2) + ((y - goal_y) ** 2) < ((1.5) ** 2)):
        return True
    else:
        return False


def obstacle_map(rad, clearance):
    plot_x = []
    plot_y = []
    rigid_x = []
    rigid_y = []
    for x in range(300):
        for y in range(200):
            if obstacle_check(x, 200 - y, rad, clearance):
                rigid_x.append(x)
                rigid_y.append(y)
            #                 blank_image[x, y] = (150, 150, 150)
            if obstacle_check(x, 200 - y, 0, 0):
                plot_x.append(x)
                plot_y.append(y)
    return plot_x, plot_y, rigid_x, rigid_y


def action_model(step_size, theta, current_node):
    theta_r = math.radians(theta)
    #     theta_start_r = math.radians(theta_start)
    current_theta = math.radians(current_node[1][2])
    actions = [[step_size * math.cos(2 * theta_r + current_theta), step_size * math.sin(2 * theta_r + current_theta),
                2 * theta, step_size],
               [step_size * math.cos(theta_r + current_theta), step_size * math.sin(theta_r + current_theta), theta,
                step_size],
               [step_size * math.cos(0 + current_theta), step_size * math.sin(0 + current_theta), 0, step_size],
               [step_size * math.cos(-theta_r + current_theta), step_size * math.sin(-theta_r + current_theta), -theta,
                step_size],
               [step_size * math.cos(-2 * theta_r + current_theta), step_size * math.sin(-2 * theta_r + current_theta),
                -2 * theta, step_size]]
    return actions


def cost2go(pt1, pt2):
    dist = math.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2)
    return dist


def threshold(x, y, th, theta):
    x = (round(x * 2) / 2)
    y = (round(y * 2) / 2)
    th = (round(th / theta) * theta)

    return (x, y, th)


def a_star(start_node, goal_node, step_size, theta, rad, clearance):
    x_explored =[]
    y_explored =[]
    x_parent = []
    y_parent = []
    start_node = threshold(start_node[0], start_node[1], start_node[2], theta)
    goal_node = threshold(goal_node[0], goal_node[1], goal_node[2], theta)
    #     print(goal_node)
    visited_nodes = np.zeros((600, 400, int(360 / theta)))
    start = (0, start_node, None)  # cost, node, parent node
    goal = (0, goal_node, None)

    nodes = queue.PriorityQueue()
    path_nodes = []
    path_dict = {}
    nodes.put(start)

    while (1):
        current_node = list(nodes.get())
        current_node[0] = current_node[0] - cost2go(current_node[1], goal[1])

        if(current_node[2] != None):
            tup1 = (66,77,40)
            tup2 = (45,54,22)
            str1 = str(current_node[1][0])
            str2 = str(current_node[1][1])
            str3 = str(current_node[1][2])

            str4 = str(current_node[2][0])
            str5 = str(current_node[2][1])
            str6 = str(current_node[2][2])

            str_node = str1+','+str2+','+str3
            str_parent = str4+','+str5+','+str6
            path_dict[str_node] = str_parent

        
        else:
            str1 = str(current_node[1][0])
            str2 = str(current_node[1][1])
            str3 = str(current_node[1][2])
            str4 = str(current_node[2])

            str_node = str1+','+str2+','+str3
            str_parent = str4
            path_dict[str_node] = str_parent

        actions = action_model(step_size, theta, current_node)
        for new_pos in actions:
            angle = new_pos[2] + current_node[1][2]
            if (angle < 0):
                angle = angle + 360

            if (angle > 360):
                angle = angle - 360

            if (angle == 360):
                angle = 0

            node = (current_node[1][0] + new_pos[0],
                    current_node[1][1] + new_pos[1], angle)
            node = threshold(node[0], node[1], node[2], theta)
            node_cost = current_node[0] + new_pos[3] + cost2go(node, goal[1])
            node_parent = current_node[1]

            if obstacle_check(node[0], 200 - node[1], rad, clearance) == False:
                if (visited_nodes[int(node[0] / 0.5)][int(node[1] / 0.5)][int(node[2] / theta)] == 0):
                    visited_nodes[int(node[0] / 0.5)][int(node[1] / 0.5)][int(node[2] / theta)] = 1
                    x_explored.append(node[0])
                    y_explored.append(node[1])
                    x_parent.append(node_parent[0])
                    y_parent.append(node_parent[1])
                    new_node = (node_cost, node, node_parent)
                    nodes.put(new_node)

        if goalcheck_circle(current_node[1][0], current_node[1][1], goal_node[0], goal_node[1]):
            # print("path_nodes = ", path_nodes)
            # t = time.localtime()
            # current_time = time.strftime("%H:%M:%S", t)
            print('Goal reached')
            # print("Time taken to explore = ",current_time)
            path = []

            # plt.plot(goal_node[0], goal_node[1], color='green', marker='o', linestyle='dashed', linewidth=1, markersize=1)
            str_p1 = str(current_node[2][0])
            str_p2 = str(current_node[2][1])
            str_p3 = str(current_node[2][2])
            parent = str_p1+','+str_p2+','+str_p3
            
            while parent != "None": 
                temp = path_dict.get(parent)
#                 print(parent)
                if(parent[1]=='.' and parent[5]=='.'):
                    par_1 = float(parent[0])+float(parent[2])/10
                    par_2 = float(parent[4])+float(parent[6])/10
                if(parent[2]=='.' and parent[7]=='.'):
                    par_1 = float(parent[0]+parent[1])+float(parent[3])/10
                    par_2 = float(parent[5]+parent[6])+float(parent[8])/10
                if(parent[1]=='.' and parent[6]=='.'):
                    par_1 = float(parent[0])+float(parent[2])/10
                    par_2 = float(parent[4]+parent[5])+float(parent[7])/10
                if(parent[2]=='.' and parent[6]=='.'):
                    par_1 = float(parent[0]+parent[1])+float(parent[3])/10
                    par_2 = float(parent[5])+float(parent[7])/10
                if(parent[3]=='.' and parent[9]=='.'):
                    par_1 = float(parent[0]+parent[1]+parent[2])+float(parent[4])/10
                    par_2 = float(parent[6]+parent[7]+parent[8])+float(parent[10])/10
                if(parent[3]=='.' and parent[7]=='.'):
                    par_1 = float(parent[0]+parent[1]+parent[2])+float(parent[4])/10
                    par_2 = float(parent[6])+float(parent[8])/10
                if(parent[3]=='.' and parent[8]=='.'):
                    par_1 = float(parent[0]+parent[1]+parent[2])+float(parent[4])/10
                    par_2 = float(parent[6]+parent[7])+float(parent[9])/10
                if(parent[1]=='.' and parent[7]=='.'):
                    par_1 = float(parent[0])+float(parent[2])/10
                    par_2 = float(parent[4]+parent[5]+parent[6])+float(parent[8])/10
                if(parent[2]=='.' and parent[8]=='.'):
                    par_1 = float(parent[0]+parent[1])+float(parent[3])/10
                    par_2 = float(parent[5]+parent[6]+parent[7])+float(parent[9])/10
                path.append((par_1,par_2))
                parent = temp
                if((par_1,par_2) == (start_node[0],start_node[1])):
                    break
            # t = time.localtime()
            # current_time = time.strftime("%H:%M:%S", t)
            # print("time taken to backtrack = ",current_time)
            # plt.plot(start_node[0],start_node[1], color='green', marker='o', linestyle='dashed', linewidth=1, markersize=1)
            path.append((start_node[0], start_node[1]))
            # print("Backtracking done - shortest path found")
            return path,x_explored,y_explored,x_parent,y_parent


def main():
    Parser = argparse.ArgumentParser()
    Parser.add_argument('--user_input')

    Args = Parser.parse_args()
    user_input = int(Args.user_input)
    if user_input == 1:
        x_start = float(input('Enter the x-coordinate of start point: '))
        y_start = float(input('Enter the y-coordinate of start point: '))
        theta_start = float(input('Enter the orientation of start point: '))
        x_goal = float(input('Enter the x-coordinate of goal point: '))
        y_goal = float(input('Enter the y-coordinate of goal point: '))
        robot_radius = float(input('Enter the radius of the robot: '))
        clearance = float(input('Enter the desired clearance: '))
        theta = float(input('Enter the angle between the actions at each node: '))
        step_size = float(input('Enter the step size: '))
        theta_goal = 0.0
        

    else:
        x_start = 50.0
        y_start = 30.0
        theta_start = 60.0
        theta = 30.0

        x_goal = 150.0
        y_goal = 150.0
        theta_goal = 0.0

        robot_radius = 1.0
        clearance = 1.0
        step_size = 1.0
        
    # t = time.localtime()
    # current_time = time.strftime("%H:%M:%S", t)
    # print("start_time = ", current_time)
    start_time = time.time()
    start_node = (x_start, y_start, theta_start)
    goal_node = (x_goal, y_goal, theta_goal)
    if obstacle_check(start_node[0], 200 - start_node[1], robot_radius, clearance) == True:
        print("The start node is either in the obstacle space or out of map")
    elif obstacle_check(goal_node[0], 200 - goal_node[1], robot_radius, clearance) == True:
        print("The goal node is either in the obstacle space or out of map")
    else:

        path,x_explored,y_explored,x_parent,y_parent = a_star(start_node, goal_node, step_size, theta, robot_radius, clearance)


        if path == None:
            print("Path could not be found. Check inputs")

        else:
            end_time = time.time()
            print('Time (in seconds) taken to find the shortest path is: ',abs(end_time - start_time))


            plt.xlim(0, 300)
            plt.ylim(0, 200)
            # plt.xlim(0, 10)
            # plt.ylim(0, 10)

            plot_x, plot_y, rigid_x, rigid_y = obstacle_map(robot_radius, clearance)

            plt.plot(rigid_x, rigid_y, ".y")
            plt.plot(plot_x, plot_y, ".k")

            l = 0
            while l < len(x_explored):
                plt.plot([x_explored[l], x_parent[l]], [y_explored[l], y_parent[l]], "-c")
                l = l + 1
                plt.show()
                plt.pause(0.000000000000000000000000000000000005)


            path = path[::-1]
            x_path = [path[i][0] for i in range(len(path))]
            y_path = [path[i][1] for i in range(len(path))]
            plt.plot(x_path, y_path, "-r")

            plt.show()
            plt.pause(5)
            plt.close()
            # t = time.localtime()
            # current_time = time.strftime("%H:%M:%S", t)
            # print("Time taken to plot = ",current_time)
            


if __name__ == '__main__':
    main()

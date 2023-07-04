"""

経路をグラフに表示
3Dポテンシャルも表示
静的障害物に対して一時的なゴールを作成
"""
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import patches as pat
from matplotlib import cm

class goal():                          
    def __init__(self, locate, area):
        """   
        #Content:set up goal 
        diameter_size: size 実際の大きさ
        locate: goal position[x,y]  
        attracted_area: effective area [int a]影響を与える範囲
        """
        self.diameter_size = 0.2         
        self.locate_goal = locate
        self.attracted_area = area

class obstacles():
    def __init__(self, obstacles = None):  
        """   
        Content: set up obstacles
        obstacles: ID and position array([x1,y1][x2,y2]..)
        """
        self.locate_obstacles = obstacles
        

class vehicles():
    def __init__(self, init_locate): 
        """   
        Content: set up obstacles
        diameter_size: size 実際の大きさ
        speed: vehicles
        """
        self.diameter = 0.5
        self.speed    = 0.1
        self.locate_vehicles = init_locate
    
    def get_locate(self, locate):     #update vehicle position
        self.locate_vehicles = locate
        return self.locate_vehicles
    
class calc_APF():    
    """
    content: calculate artifitial potential field
    dist_v2goal: include distance between vehicle and goal
    attracte_k, repulse_k: include  coefficient goal and obstacle
    vehicle_speed: update distance  
    repulse_area: effective area of obstacle
    """               
    def __init__(self,vehicles_speed):
        self.dist_v2goal = None     
        self.attracte_k  = 10
        self.repulse_k   = 0.1
        self.vehicles_speed = vehicles_speed
        self.repulsed_area = 5
    
    def calc_goal_dist_theta(self, locate_goal, locate_vehicles): #calculate distance between goal and vehicle
        self.dist_v2goal = np.sqrt((locate_goal[0]-locate_vehicles[0])**2+(locate_goal[1]-locate_vehicles[1])**2)
        self.theta_goal= np.arctan2(locate_goal[1]-locate_vehicles[1], locate_goal[0]-locate_vehicles[0])
        self.goalanddistandtheta = [self.dist_v2goal, self.theta_goal]

    def calc_obs_dist_theta(self, locate_obs, locate_vehicles): #calculate distance between obstacles and vehicle
        self.dist_v2obs = []
        self.theta_obs  = []
        self.obs_distandtheta = []
        for id in range(len(locate_obs)):
            self.dist_v2obs.append(np.sqrt((locate_obs[id][0]-locate_vehicles[0])**2+(locate_obs[id][1]-locate_vehicles[1])**2))
            self.theta_obs.append(np.arctan2(locate_obs[id][1]-locate_vehicles[1], locate_obs[id][0]-locate_vehicles[0]))                  

    def calc_attractive_force(self, locate_goal, locate_vehicles): #calculate attractive_force 
        self.calc_goal_dist_theta(locate_goal, locate_vehicles)
        if self.dist_v2goal == 0:
            self.attforce = -1
        else:
            self.attforce = -1*self.attracte_k/self.dist_v2goal
    
    def calc_repulsive_force(self, locate_obs, locate_vehicles): #calculate repulse_force
        if len(locate_obs) == None:
            self.repforce = [0]
        else:
            self.repforce = [0]*len(locate_obs)
            self.calc_obs_dist_theta(locate_obs, locate_vehicles)
            for id in range(len(self.dist_v2obs)):
                if self.dist_v2obs[id] <= 0:
                    self.repforce[id] = 1
                elif self.repulsed_area >= self.dist_v2obs[id]:
                    self.repforce[id] = 1*self.repulse_k/self.dist_v2obs[id]
                else: self.repforce[id] = 0 

    def calc_sum_potentialforce(self, locate_goal, locate_vehicles, locate_obs): #calculate sum attractive_force
        self.calc_attractive_force(locate_goal, locate_vehicles)    
        self.calc_repulsive_force(locate_obs, locate_vehicles)
        self.total_pot = self.attforce
        for id in range(len(self.repforce)):
            self.total_pot = self.total_pot + self.repforce[id]
        return self.total_pot

    
    def route_creater(self, locate_goal, locate_vehicles, locate_obs): #route_creater calculates movement amount from partitial differential of potential 
        current_potential = self.calc_sum_potentialforce(locate_goal, locate_vehicles, locate_obs)
        delt_poten_x = self.calc_sum_potentialforce(locate_goal, [locate_vehicles[0]+self.vehicles_speed, locate_vehicles[1]], locate_obs)
        delt_poten_y = self.calc_sum_potentialforce(locate_goal, [locate_vehicles[0] ,locate_vehicles[1]+self.vehicles_speed], locate_obs)
        partialdiffer_x = delt_poten_x - current_potential
        partialdiffer_y = delt_poten_y - current_potential
        synthesis_v = np.sqrt(partialdiffer_x**2+partialdiffer_y**2)
        partialdiffer_x /=synthesis_v/self.vehicles_speed*-1
        partialdiffer_y /=synthesis_v/self.vehicles_speed*-1
        return partialdiffer_x, partialdiffer_y

class plot_path(): #plot vehicle trajectory
    """
    set param  ploting path
    """
    def __init__(self, start_position , goal_position):
        fig = plt.figure(figsize=(7,7))
        self.ploting_path = fig.add_subplot(111)
        self.ploting_path.set_xlabel('X-distance: m')
        self.ploting_path.set_ylabel('Y-distance: m')
        self.graph_range = [0, 60]
        plt.xlim([self.graph_range[0], self.graph_range[1]])
        plt.ylim([self.graph_range[0], self.graph_range[1]])
        self.goal_position = goal_position
        self.ploting_path.plot(start_position[0], start_position[1], "*r")
        self.ploting_path.plot(goal_position[0], goal_position[1], "*b")
        circle_goal = pat.Circle(xy=(goal_position[0], goal_position[1]), radius=2, fc ="y")
        self.ploting_path.add_patch(circle_goal)
        self.plot_obs(obs.locate_obstacles)
    
    def plot_obs(self, obstacles): #plot set obstacles area
        for i in range(len(obstacles)):
            circle_obs = pat.Circle(xy=(obstacles[i][0],obstacles[i][1]), radius=5, fc ="b", alpha = 0.3)
            self.ploting_path.add_patch(circle_obs)
            self.ploting_path.plot(obstacles[i][0],obstacles[i][1], "xk")
    
    def plot_vehicle(self, locate_vehile): #plot locate_vehicle 
        self.ploting_path.plot(locate_vehile[0], locate_vehile[1], ".r")

    def calc_all_potential(self): #calculate potential whole area
        pot = []
        for y_for_pot in range(self.graph_range[0], self.graph_range[1] + 1):
            tmp_pot = []
            for x_for_pot in range(self.graph_range[0], self.graph_range[1] + 1):
                veh1.locate_vehicles = [x_for_pot, y_for_pot]
                potential = APF.calc_sum_potentialforce(fgoal.locate_goal, veh1.locate_vehicles, obs.locate_obstacles)
                tmp_pot.append(potential)
            pot.append(tmp_pot)
        pot = np.array(pot)
        return pot
    
    def plot_3d_potential(self): #plot potential force to 3D gragh
        fig3d = plt.figure(figsize=(6,6), facecolor= "w")
        ax3d = fig3d.add_subplot(111, projection="3d")
        ax3d.tick_params(labelsize=7)    # 軸のフォントサイズ
        ax3d.set_xlabel("x", fontsize=10)
        ax3d.set_ylabel("y", fontsize=10)
        ax3d.set_zlabel("poten", fontsize=10)
        pot = self.calc_all_potential()
        x, y = np.meshgrid(np.arange(self.graph_range[0], self.graph_range[1] + 1), np.arange(self.graph_range[0], self.graph_range[1] + 1))
        surf = ax3d.plot_surface(x, y, pot, rstride=1, cstride=1, cmap= cm.coolwarm)
        plt.show()    

def main(): 
    if APF.dist_v2goal == None:
        APF.calc_goal_dist_theta(fgoal.locate_goal, veh1.locate_vehicles)
        print("if goal dist is None calc once:", APF.dist_v2goal,"\n")
    if APF.dist_v2goal != None:
        while (APF.dist_v2goal - veh1.diameter) > fgoal.diameter_size:
            partialdiffer_x, partialdiffer_y = APF.route_creater(fgoal.locate_goal, veh1.locate_vehicles, obs.locate_obstacles)
            veh1.locate_vehicles = [veh1.locate_vehicles[0]+partialdiffer_x, veh1.locate_vehicles[1]+partialdiffer_y]
            path_fig.plot_vehicle(veh1.locate_vehicles)
            plt.pause(0.0002)
        print("finish and show 3d")
        path_fig.plot_3d_potential()

if __name__ == '__main__':
    fgoal = goal([50,50],10)
    obs = obstacles([[15,20],[20,20]])
    veh1 = vehicles([0,0])
    APF = calc_APF(veh1.speed)
    path_fig = plot_path(veh1.locate_vehicles, fgoal.locate_goal)
    main()
    

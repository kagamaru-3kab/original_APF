"""
proposal dimension
potential map をグラフに表示
complete
"""
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import patches as pat
from matplotlib import cm
from math import cos, sin
import time

class goal():                          
    def __init__(self, locate, area):
        """   
        #Content:set up goal 
        diameter_size: size 実際の大きさ
        locate: goal position[x,y]  
        attracted_area: effective area [int a]影響を与える範囲
        """
        self.diameter_size = 0.5       
        self.locate_goal = locate
        self.attracted_area = area
    
    def get_locate(self, locate):
        self.locate_goal = locate

class obstacles():
    def __init__(self, obstacles = None):  
        """   
        Content: set up obstacles
        obstacles: ID and position array([x1,y1][x2,y2]..)
        """
        self.locate_obstacles = obstacles
        self.dynamic_obs_id = [0]
        self.obs_velocity_vector = [[-3, -3]]#needed proposal dimension
    
    def update_locate_obs(self):
        """   
        Content: update location dynamic obstacles ,assign dynamic obs id and displacement
        obstacles: ID and position array([x1,y1][x2,y2]..)
        self.obs_velocity_vector = [Vx, Vy]
        """
        for d in range(len(self.dynamic_obs_id)): 
           self.locate_obstacles[self.dynamic_obs_id[d]][0] += self.obs_velocity_vector[self.dynamic_obs_id[d]][0]
           self.locate_obstacles[self.dynamic_obs_id[d]][1] += self.obs_velocity_vector[self.dynamic_obs_id[d]][1]
        
class vehicles():
    def __init__(self, init_locate): 
        """   
        Content: set up obstacles
        diameter_size: size 実際の大きさ
        speed: vehicles
        """
        self.diameter = 0.5
        self.veh_maxspeed = 0.15
        self.speed    = self.veh_maxspeed
        self.locate_vehicles = init_locate
        self.beforestep_vehicle = init_locate
        self.vehicle_vector = [self.speed, self.speed]

    def find_vehicle_vector(self,locate_vehicle): #update, volocity_vector is needed for dynamic obs aboidance
        if self.beforestep_vehicle != locate_vehicle:
            vx = locate_vehicle[0]-self.beforestep_vehicle[0]
            vy = locate_vehicle[1]-self.beforestep_vehicle[1]
            self.beforestep_vehicle = locate_vehicle
            self.vehicle_vector = [vx,vy]
        else:
            self.vehicle_vector = [0,0]

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
        self.repulse_k   = 0.3
        self.vehicles_speed = vehicles_speed
        self.repulsed_area = 5
    
    def calc_goal_dist_theta(self, locate_goal, locate_vehicles): #calculate distance between goal and vehicle
        self.dist_v2goal = np.sqrt((locate_goal[0]-locate_vehicles[0])**2+(locate_goal[1]-locate_vehicles[1])**2)

    def calc_obs_dist_theta(self, locate_obs, locate_vehicles): #calculate distance between obstacles and vehicle
        self.dist_v2obs = []
        self.theta_obs  = []
        self.obs_distandtheta = []
        for id in range(len(locate_obs)):
            self.dist_v2obs.append(np.sqrt((locate_obs[id][0]-locate_vehicles[0])**2+(locate_obs[id][1]-locate_vehicles[1])**2))
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
                flag = id not in obs.dynamic_obs_id
                if flag:
                    if self.dist_v2obs[id] <= 0:
                        self.repforce[id] = 1
                    elif self.repulsed_area >= self.dist_v2obs[id]:
                        self.repforce[id] = 1*self.repulse_k/self.dist_v2obs[id]
                    else: self.repforce[id] = 0 

    def calc_sum_potentialforce(self, locate_goal, locate_vehicles, locate_obs): #calculate sum attractive_force
        self.calc_attractive_force(locate_goal, locate_vehicles)    
        self.calc_repulsive_force(locate_obs, locate_vehicles)
        self.total_pot = self.attforce
        #print("\nattractive",self.attforce)
        for id in range(len(self.repforce)):
            self.total_pot = self.total_pot + self.repforce[id]
            #print("repulsive",self.repforce)
        #print("total_poten",self.total_pot)
        return self.total_pot
   
    def route_creater(self, locate_goal, locate_vehicles, locate_obs): #route_creater calculates movement amount from partitial differential of potential 
        delt_dynamic_x = 0 #if dynamic repluse force is off, making sense
        delt_dynamic_y = 0 #if dynamic repluse force is off, making sense
        veh1.find_vehicle_vector(locate_vehicles)
        current_potential = self.calc_sum_potentialforce(locate_goal, locate_vehicles, locate_obs)
        #dynamic_repulse_force = dimention.calc_dynamic_repulsive(dimention.locate_leading_point,locate_obs)
        #current_potential += dynamic_repulse_force
        delt_poten_x = self.calc_sum_potentialforce(locate_goal, [locate_vehicles[0]+self.vehicles_speed, locate_vehicles[1]], locate_obs)
        delt_poten_y = self.calc_sum_potentialforce(locate_goal, [locate_vehicles[0] ,locate_vehicles[1]+self.vehicles_speed], locate_obs)
        #delt_dynamic_x = dimention.calc_dynamic_repulsive([dimention.locate_leading_point[0]+dimention.dx ,dimention.locate_leading_point[1]],locate_obs)
        #delt_dynamic_y = dimention.calc_dynamic_repulsive([dimention.locate_leading_point[0], dimention.locate_leading_point[1]+dimention.dy],locate_obs)
        partialdiffer_x = (delt_poten_x + delt_dynamic_x)- current_potential #partialdiffer > 0 push back, <0 go ahead in short delt <current push back
        #print("partX =delt X- currentPoten",partialdiffer_x, delt_poten_x, current_potential)
        partialdiffer_y = (delt_poten_y+ delt_dynamic_y) - current_potential
        synthesis_v = np.sqrt(partialdiffer_x**2+partialdiffer_y**2)
        partialdiffer_x /=synthesis_v/self.vehicles_speed*-1   #正規化らしい +-change 
        partialdiffer_y /=synthesis_v/self.vehicles_speed*-1
        #print("222partX delt X currentPoten",partialdiffer_x, delt_poten_x, current_potential)
        return partialdiffer_x, partialdiffer_y
    
    def route_creater_for_proposal_dynamic(self, locate_goal, locate_leading, locate_obs): #route_creater calculates movement amount from partitial differential of potential 
        delt_dynamic_x = 0 #if dynamic repluse force is off, making sense
        delt_dynamic_y = 0 #if dynamic repluse force is off, making sense
        delt_poten_x = 0
        delt_poten_y = 0
        veh1.find_vehicle_vector(veh1.locate_vehicles)
        #self.attracte_k = 1 #To respond more sensitively to dynamic obstacles
        current_potential = self.calc_sum_potentialforce(locate_goal, locate_leading, locate_obs)
        self.dynamic_repulse_force = dimention.calc_dynamic_repulsive(dimention.locate_leading_point,locate_obs)
        current_potential += self.dynamic_repulse_force
        delt_poten_x = self.calc_sum_potentialforce(locate_goal, [locate_leading[0]+self.vehicles_speed, locate_leading[1]], locate_obs) #止まってる障害物からの反力しか影響ないので，動く障害物だけのときは，attractive forceのみ返る
        delt_poten_y = self.calc_sum_potentialforce(locate_goal, [locate_leading[0] ,locate_leading[1]+self.vehicles_speed], locate_obs)
        delt_dynamic_x = dimention.calc_dynamic_repulsive([dimention.locate_leading_point[0]+self.vehicles_speed ,dimention.locate_leading_point[1]],locate_obs)
        delt_dynamic_y = dimention.calc_dynamic_repulsive([dimention.locate_leading_point[0], dimention.locate_leading_point[1]+self.vehicles_speed],locate_obs)
        if delt_dynamic_x > 0 or delt_dynamic_y > 0:
            self.next_LP_might_have_dynamicrepulse = delt_dynamic_x if delt_dynamic_x >= delt_dynamic_y else delt_dynamic_y
        else: 
            self.next_LP_might_have_dynamicrepulse = 0
        #print("\ndeltax,delty",delt_poten_x,delt_poten_y)
        #print("Dynamic deltx,delty",delt_dynamic_x,delt_dynamic_y)
        #print(" dynamic total force",self.dynamic_repulse_force)
        #print("current force", current_potential)
        partialdiffer_x = (delt_poten_x + delt_dynamic_x)- current_potential #partialdiffer > 0 push back, <0 go ahead in short delt <current push back
        partialdiffer_y = (delt_poten_y+ delt_dynamic_y) - current_potential #個々の段階ではpartialは負だと，後で-1かけて前に進む正になる
        #print("part x, y", partialdiffer_x, partialdiffer_y)
        synthesis_v = np.sqrt(partialdiffer_x**2+partialdiffer_y**2)
        partialdiffer_x /=synthesis_v/self.vehicles_speed*-1   #正規化らしい +-change 
        partialdiffer_y /=synthesis_v/self.vehicles_speed*-1
        #print("after nomalized part x, y", partialdiffer_x, partialdiffer_y,synthesis_v)
        return partialdiffer_x, partialdiffer_y

class plot_path(): #plot vehicle trajectory
    """
    set param  ploting path
    search about matplotlib with Internet
    """
    def __init__(self, start_position , goal_position):
        self.fig = plt.figure(figsize=(7,7))
        self.ploting_path = self.fig.add_subplot(111)
        self.ploting_path.set_xlabel('X-distance: m')
        self.ploting_path.set_ylabel('Y-distance: m')
        self.goal_position = goal_position
        self.start_position = start_position
        self.graph_range = [0, 150]
        plt.xlim([self.graph_range[0], self.graph_range[1]])
        plt.ylim([self.graph_range[0], self.graph_range[1]])
        self.ploting_path.plot(self.start_position[0], self.start_position[1], "*r")
        self.ploting_path.plot(self.goal_position[0], self.goal_position[1], "*b")
        circle_goal = pat.Circle(xy=(self.goal_position[0], self.goal_position[1]), radius=2, fc ="y")
        self.ploting_path.add_patch(circle_goal)

    def _initialize_fig(self):
        self.graph_range = [0, 150]
        plt.xlim([self.graph_range[0], self.graph_range[1]])
        plt.ylim([self.graph_range[0], self.graph_range[1]])
        self.ploting_path.plot(self.start_position[0], self.start_position[1], "*r")
        self.ploting_path.plot(self.goal_position[0], self.goal_position[1], "*b")
        circle_goal = pat.Circle(xy=(self.goal_position[0], self.goal_position[1]), radius=2, fc ="y")
        self.ploting_path.add_patch(circle_goal)
        self.plot_obs(obs.locate_obstacles)
    
    def plot_obs(self, obstacles): #plot set obstacles area
        r_area = APF.repulsed_area
        for i in range(len(obstacles)):
            if i not in obs.dynamic_obs_id:
                circle_obs = pat.Circle(xy=(obstacles[i][0],obstacles[i][1]), radius= r_area, fc ="b", alpha = 0.3)
                self.ploting_path.add_patch(circle_obs)
                #print("staic obs")   
                self.ploting_path.plot(obstacles[i][0],obstacles[i][1], "xk")    

            else:
                ellipse_angle = np.degrees(dimention.obs_direction)
                if dimention.denom_x_root >= dimention.denom_y_root:
                    ellipse_width = dimention.denom_x_root
                    ellipse_height = dimention.denom_y_root
                else:
                    ellipse_width = dimention.denom_y_root
                    ellipse_height = dimention.denom_x_root
                ellipse_x = obs.locate_obstacles[i][0] + obs.obs_velocity_vector[i][0]
                ellipse_y = obs.locate_obstacles[i][1] + obs.obs_velocity_vector[i][1]   
                ellipse_obs = pat.Ellipse(xy=(ellipse_x, ellipse_y), width=ellipse_width, height=ellipse_height, color="orange", alpha=0.8, angle=ellipse_angle)
                self.ploting_path.add_patch(ellipse_obs)
                #print("dynamic obs")    
                self.ploting_path.plot(obstacles[i][0],obstacles[i][1], "xk")     

    def plot_vehicle(self, locate_vehile): #plot locate_vehicle 
        anime_array_source = self.ploting_path.plot(locate_vehile[0], locate_vehile[1], ".r")
        path_fig.ploting_path.plot(dimention.locate_leading_point[0],dimention.locate_leading_point[1], ".b")
        return anime_array_source

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

class temporary_goal():
    """
    content: find the nearest obstacle on line from vehicle to goal and set temporary goal beside target obstacle
    """
    def __init__(self):
        self.tmp_goal_locate = None
        self.nearest_obs = None
        self.delta_fromobs = 1 #distance from circle

    def calc_line_from_start2goal(self, locate_goal, locate_vehicles): #calc coefficient of line which pass start and goal 
        self.a = locate_goal[1]-locate_vehicles[1]
        self.b = locate_vehicles[0]-locate_goal[0]
        self.c = locate_vehicles[1]*locate_goal[0]-locate_goal[1]*locate_vehicles[0]

    def draw_line_from_start2goal(self, locate_goal, locate_vehicles): #plot_line which pass start and goal  
        startp = int(locate_vehicles[0])
        endp = int(locate_goal[0])
        if self.b != 0 and self.a != 0 :
            self.typeline = 0
            for x in range(startp, endp):
                y = (-1*self.a*x-self.c)/self.b
                path_fig.ploting_path.plot(x, y, ".k")
        elif self.b == 0 and self.a != 0:
            self.typeline = 1
            for y in range(startp, endp):
                x = locate_vehicles[0] 
                path_fig.ploting_path.plot(x, y, ".k")

        elif self.a == 0 and self.b != 0:
            self.typeline = 2
            for x in range(startp, endp):
                y = locate_vehicles[1] 
                path_fig.ploting_path.plot(x, y, ".k")
        else:
            print("set goal and start on the same point")

    def detect_obs_ontheline(self, locate_obs): #judge which obstacles on the line
        if len(locate_obs) != 0:
            dist_from_line = [0]*len(locate_obs)
            self.register_id_obs = []
            self.Denominator = np.sqrt(self.a**2+self.b**2)
            for i in range(len(locate_obs)):
                if (fgoal.locate_goal[0]-veh1.locate_vehicles[0]) >= 0 :
                    if locate_obs[i][0] >= veh1.locate_vehicles[0] and locate_obs[i][0] <= fgoal.locate_goal[0]:
                        Numerator = abs(self.a*locate_obs[i][0]+self.b*locate_obs[i][1]+self.c)
                    else:
                        Numerator = -1
                elif (fgoal.locate_goal[0]-veh1.locate_vehicles[0]) < 0 :
                    if locate_obs[i][0] <= veh1.locate_vehicles[0] and locate_obs[i][0] >= fgoal.locate_goal[0]:
                        Numerator = abs(self.a*locate_obs[i][0]+self.b*locate_obs[i][1]+self.c)
                    else:
                        Numerator = -2
                else:
                    Numerator = -3
                if Numerator > 0:
                    dist_from_line[i] = Numerator/self.Denominator
                elif Numerator == 0:
                    dist_from_line[i] = 0
                else:
                    print("Numerator is < 0 because target_goal is out of range",Numerator)
                    dist_from_line[i] = APF.repulsed_area + 1
                #print("id + dist_from_line", i, dist_from_line[i])
                if dist_from_line[i] < APF.repulsed_area:
                    self.register_id_obs.append(i)
                    print("found obs on line")
            if len(self.register_id_obs) == 0:
                self.register_id_obs = None 
                print("obs exist but not on line")      
        elif len(locate_obs) == 0:
            self.register_id_obs = None
            print(" obstacles no exist")
        
        if self.register_id_obs == None:
            print("ゴールまでの線上のobsは None")
        else:
            print("ゴールまでの線上のobsは",len(self.register_id_obs),"個だ")
    
    def find_nearest_obs(self): #find nearest obs on the line from vehicle
        if self.register_id_obs  != None:
            for i, id_obs in enumerate(self.register_id_obs):
                if i == 0:
                    self.nearest_obs = [id_obs, APF.dist_v2obs[id_obs]]
                else:
                    if self.nearest_obs[1] > APF.dist_v2obs[id_obs]:
                        self.nearest_obs = [id_obs, APF.dist_v2obs[id_obs]]
            print("車両最近傍のobsは[id, dist_fromvehicle]", self.nearest_obs)
        else:
            self.nearest_obs = None

    def calc_tempgoal_locate(self, Dfortemp, temp_locate): #ref https://qiita.com/tydesign/items/36b42465b3f5086bd0c5
        #move  to init ->self.delta_fromobs = 1 #distance from circle
        aD = self.slope_nomal*Dfortemp  #y=a*x+c -> a*x-1*y+c
        a2b2 = self.slope_nomal**2 + 1**2
        self.rrr = APF.repulsed_area**2
        contain_root = np.sqrt((a2b2*self.rrr) - Dfortemp**2)
        x1 = (aD - contain_root)/a2b2
        x1 += temp_locate[0] -self.delta_fromobs
        x2 = (aD + contain_root)/a2b2
        x2 += temp_locate[0] +self.delta_fromobs
        y1 = self.slope_nomal*x1 +self.intercept_nomal
        y2 = self.slope_nomal*x2 +self.intercept_nomal
        p1 = [x1,y1]
        p2 = [x2,y2]
        Numerator_p1 = abs(self.a*p1[0]+self.b*p1[1]+self.c)
        Numerator_p2 = abs(self.a*p2[0]+self.b*p2[1]+self.c)
        near_goal_dist = Numerator_p1/self.Denominator
        if near_goal_dist > Numerator_p2/self.Denominator:
            return p2
        else: 
            return p1 
            
    def set_temporary_goal(self): #set temporary goal
        if self.nearest_obs !=  None:
            temporary_goal_locate = [obs.locate_obstacles[self.nearest_obs[0]][0],obs.locate_obstacles[self.nearest_obs[0]][1]]
            self.slope_nomal = self.b/self.a                                                        #法線の傾きを求めてる
            self.intercept_nomal = temporary_goal_locate[1]-(self.slope_nomal*temporary_goal_locate[0])#法線の切片を求めてる
            Dfortemp = abs(self.slope_nomal*temporary_goal_locate[0]-temporary_goal_locate[1]+self.intercept_nomal)
            near_tmpgoal = self.calc_tempgoal_locate(Dfortemp, temporary_goal_locate)
            temporary_goal_locate = near_tmpgoal          
            path_fig.ploting_path.plot(temporary_goal_locate[0], temporary_goal_locate[1], "xr")
            return temporary_goal_locate
        else:
            print("not need to change goal")
            temporary_goal_locate = None
            return temporary_goal_locate 
        
    def draw_nomalline(self): #plot_line which pass start and goal  
        if self.nearest_obs != None:
            for x in range(50):
                y = self.slope_nomal*x + self.intercept_nomal
                if y >= 0 or y <= 50:
                    path_fig.ploting_path.plot(x, y,".g")
                else:
                    break

    def integrate_temporarygoal(self,locate_goal,locate_vehicles, locate_obs): #integrate this class function
        self.calc_line_from_start2goal(locate_goal,locate_vehicles)
        self.draw_line_from_start2goal( locate_goal, locate_vehicles)
        self.detect_obs_ontheline(locate_obs)
        self.find_nearest_obs()
        self.temp_goal = self.set_temporary_goal()
        self.draw_nomalline()
        print("changed_temp_goal",self.temp_goal)

class proposal_the_other_dimension():
    """
    direction_terms: decide angle range where dynamic obstacles come from
    kx, ky :coefficient with denominator of elipse
    """
    def __init__(self):
        self.direction_terms_forward = [np.cos(np.radians(180)), np.cos(np.radians(150))]
        self.direction_terms_side = [np.cos(np.radians(120)), np.cos(np.radians(30))]
        self.direction_terms_back = [np.cos(np.radians(30)), np.cos(np.radians(0))]
        self.kx ,self.ky = 10,10 #expand cofficient ellipse
        self.minimum_repluse_area = 8
        self.dynamic_repulse_k = 0.5
        self.temp_goal_point = fgoal.locate_goal 
        self.leading_interval = 12
        self.leading_interval_max = self.leading_interval 
        self.locate_leading_point = [veh1.locate_vehicles[0]+veh1.vehicle_vector[0],veh1.locate_vehicles[1]+veh1.vehicle_vector[1]]
        #beliw init items are for the sake of processing order
        self.denom_x =0
        self.denom_x_root = 1
        self.denom_y =0
        self.denom_y_root = 1
        self.dx = veh1.speed
        self.dy= veh1.speed
        self.obs_direction = 0
        self.LP_location_1stepbefore = [0,0]
        self.LP_location_2stepbefore = [0,0]
        self.countlimit = 100
        self.locate_dynamicobs_adddelt = [[0,0]]
        self.reacting_now = 1
        self.flag_react = True 
        
    def update_leading_point(self,locate_goal):  #leading point moves by its potential
        count_calc_guidepoint = 0
        flag = True
        tempgoal4dynamic.temp_goal = None
        self.tempdynamicgoal_flag = False
        self.first_react = False
        for i in range(len(obs.dynamic_obs_id)): # use to shift tempgoal againt dynamic obs
            self.locate_dynamicobs_adddelt[i][0] = obs.locate_obstacles[obs.dynamic_obs_id[i]][0]+obs.obs_velocity_vector[i][0]
            self.locate_dynamicobs_adddelt[i][1] = obs.locate_obstacles[obs.dynamic_obs_id[i]][1]+obs.obs_velocity_vector[i][1]
        while flag:
            self.dx, self.dy = APF.route_creater_for_proposal_dynamic(locate_goal, self.locate_leading_point, obs.locate_obstacles) #get total_dynamic_repluse and return partdx,dy
            #self.locate_leading_point = [self.locate_leading_point[0]+self.dx, self.locate_leading_point[1]+self.dy ]
            self.judge_whereObscomefrom() #self.avoid_dynamicobs_flag gets the number of situation
            if self.flag_react:
                self.reactiontodynamicobs(locate_goal)   #react something by result of self.judge_whereObscomefrom() ,return new temp_goal or decelerate or no react
            elif self.flag_react == False:
                coefficient_dxdy = 3
                self.dx = coefficient_dxdy*self.dx
                self.dy = coefficient_dxdy*self.dy
            self.locate_leading_point = [self.locate_leading_point[0]+self.dx, self.locate_leading_point[1]+self.dy ]
            self.dist_leading2vehicle = np.sqrt((self.locate_leading_point[0]-veh1.locate_vehicles[0])**2+(self.locate_leading_point[1]-veh1.locate_vehicles[1])**2)
            self.dist_leading2goal = np.sqrt((locate_goal[0]-self.locate_leading_point[0])**2+(locate_goal[1]-self.locate_leading_point[1])**2)
            self.dist_v2goal = np.sqrt((locate_goal[0]-veh1.locate_vehicles[0])**2+(locate_goal[1]-veh1.locate_vehicles[1])**2)
            print("dist_LP2vehicle",self.dist_leading2vehicle)
            print("count update",count_calc_guidepoint)
            
            if self.dist_leading2vehicle >= self.leading_interval: #if LP is farther than the specified interval, the loop stops
                flag = False
                print("loop end self.dist_leading2vehicle >= self.leading_interval")
            
            if self.dist_leading2goal < fgoal.diameter_size:
                flag = False

            if count_calc_guidepoint > self.countlimit : #if the count is more than the limit, reset LP position
                print("leading point is stuck")
                self.reset_LP_location()
                count_calc_guidepoint = 0 

            if self.reacting_now == 0: #can react the same obs only once, next getting 0 dynamic repulse enables reactiontodynamicobs to react: この中はreact中の時のみ発動
                self.flag_react = False
                self.leading_interval = 6
                

            if self.first_react : #if reactiontodynamicobs react, the loop is ended
                flag = False
                print("first react")
            print("dx dy for LP, location", self.dx ,self.dy,self.locate_leading_point)
            count_calc_guidepoint += 1
            #path_fig.ploting_path.plot(dimention.locate_leading_point[0],dimention.locate_leading_point[1], ".b")
        self.countlimit = 30 #the second calc use this limit
        #print(" LP-LP1,LP",self.locate_leading_point,self.LP_location_1stepbefore,self.LP_location_2stepbefore)
        self.LP_location_2stepbefore = self.LP_location_1stepbefore
        self.LP_location_1stepbefore = self.locate_leading_point 
        
        if tempgoal4dynamic.temp_goal != None:
            return tempgoal4dynamic.temp_goal
        else:
            tempgoal4dynamic.temp_goal = None
            return tempgoal4dynamic.temp_goal 

    def judge_whereObscomefrom(self):
        print("\nloop main start")
        self.avoid_dynamicobs_flag = 0
        if APF.dynamic_repulse_force != 0 or APF.next_LP_might_have_dynamicrepulse != 0 : # if dynamic poten effect LP
            vector1 = [self.locate_leading_point[i]- self.LP_location_1stepbefore[i] for i in range(2)]
            vector2 = [self.LP_location_1stepbefore[i] - self.LP_location_2stepbefore[i] for i in range(2)]
            #print(".......vec1 LP-LP1",vector1,self.locate_leading_point,self.LP_location_1stepbefore)
            #print(".......vec2 LP1-LP2",vector2,self.LP_location_2stepbefore)

            V1 = np.array([vector1])  # ベクトルの引き算を試してみる
            V2 = np.array([vector2])
            V3 = V1 - V2 #V3は今と１つ前のベクトル変化に加わったベクトル
            #print("ベクトル3:", V3)
            magnitudeV2 = np.linalg.norm(V2)
            magnitudeV3 = np.linalg.norm(V3)
            dot_product_V2V3 = np.dot(V2, V3[0])
            if magnitudeV2 * magnitudeV3 != 0:
                cos_theta = dot_product_V2V3 / (magnitudeV2 * magnitudeV3)
                angle_radians = np.arccos(np.clip(cos_theta, -1.0, 1.0))
                #print("angle_dot",np.degrees(angle_radians))
                angle_whereobscomefrom = np.cos(angle_radians) #no need ,can be intead of above cos_theta
                #print("costheta to angle_where ha same?", cos_theta, angle_whereobscomefrom)
            else:
                # ゼロでの除算を回避する
                angle_whereobscomefrom = 2 #2 is out of range sinX   
                print("Either mag1 or mag2 vector have 0")

            if self.direction_terms_forward[0] <= angle_whereobscomefrom <= self.direction_terms_forward[1]:
                #print("Observation is in the forward direction.")
                self.avoid_dynamicobs_flag = 1

            elif self.direction_terms_side[0] <= angle_whereobscomefrom <= self.direction_terms_side[1]:
                #print("Observation is in the side direction.")
                self.avoid_dynamicobs_flag = 2

            elif self.direction_terms_back[0] <= angle_whereobscomefrom <= self.direction_terms_back[1]:
                #print("Observation is in the backward direction.")
                self.avoid_dynamicobs_flag = 3

            else:
                print("Observation is outside of the defined directions.")
                self.avoid_dynamicobs_flag = 0
        else : #LP has no total dynamic poten and LP+delt has no delt_dynamic_XY
            print(" total_repulse is 0 or None and deltdynamic_xy = 0") 
            APF.vehicles_speed =  veh1.veh_maxspeed
            self.reacting_now = 1
            self.flag_react = True 
            self.leading_interval = self.leading_interval_max
            #print("restore LP interval")


    def reactiontodynamicobs(self,locate_goal): #leading point moves by vehicle position and velocity
        if self.reacting_now == 1:# need to launch this func only once
            #print("reacting now",self.avoid_dynamicobs_flag)
            tempgoal4dynamic.temp_goal = None
            if self.avoid_dynamicobs_flag == 1:
                self.reacting_now = 0
                tempgoal4dynamic.delta_fromobs = 1.5
                tempgoal4dynamic.rrr = self.denom_x if self.denom_x >= self.denom_y else self.denom_y
                print("!!!!!obs comes from front, avoid ")
                tempgoal4dynamic.integrate_temporarygoal(fgoal.locate_goal, self.locate_leading_point,self.locate_dynamicobs_adddelt)
                if tempgoal4dynamic.temp_goal != None:
                    tempgoal4dynamic.temp_goal = [a + 2*b for a, b in zip(tempgoal4dynamic.temp_goal, obs.obs_velocity_vector[tempgoal4dynamic.nearest_obs[0]])]
                    self.dx, self.dy = APF.route_creater_for_proposal_dynamic(tempgoal4dynamic.temp_goal, self.locate_leading_point, obs.locate_obstacles)
                    #print("dx  dy",self.dx,self.dy)
                    path_fig.ploting_path.plot(tempgoal4dynamic.temp_goal[0], tempgoal4dynamic.temp_goal[1], "xy")
                    #self.tempdynamicgoal_flag = True
                    self.first_react = True
                    print("veh,  obs  tempgoal",veh1.locate_vehicles,obs.locate_obstacles,tempgoal4dynamic.temp_goal)
            elif self.avoid_dynamicobs_flag == 2:
                self.reacting_now = 0
                print("!!!!!obs comes from sideways, decelerate ")
                APF.vehicles_speed =  veh1.veh_maxspeed*(1 -APF.dynamic_repulse_force)  #deceleretion vehicle speed upon self.total_repulsive_force
                if APF.vehicles_speed < 0:
                    APF.vehicles_speed = 0
                self.dx, self.dy = APF.route_creater_for_proposal_dynamic(locate_goal, self.locate_leading_point, obs.locate_obstacles)   
                #print("dx  dy",self.dx,self.dy)
                self.first_react = True
                print("change vehicle speed because of obs coming from sideways",veh1.speed,veh1.veh_maxspeed,APF.dynamic_repulse_force)
            
            elif self.avoid_dynamicobs_flag == 3:
                print("!!!!!obs comes from itself back,but dont need to react  ")
                APF.vehicles_speed = veh1.veh_maxspeed 
                self.reacting_now = 0
            
            else:
                APF.vehicles_speed =  veh1.veh_maxspeed
                #print("donot have to avoid")
        
    def calc_dynamic_repulsive(self, locate_leading_point, locate_obstacles ): #calculate dynamic repulse_force  ellipse shape
        self.total_repulsive_force = 0
        if len(obs.dynamic_obs_id) == None:
            self.total_repulsive_force = 0
        else:
            self.dynamic_repforce= [0,0,0,0,0,0]*len(obs.dynamic_obs_id) #self.dynamic_repforce will contain [denom_x,denom_y,numer_x,numer_y,dynamic_repulse_area,dynamic_repulsive_potential]
            for id in range(len(obs.dynamic_obs_id)):
                self.denom_x = (self.kx*(obs.obs_velocity_vector[id][0])**2)
                self.denom_y = (self.ky*(obs.obs_velocity_vector[id][1])**2)
                #print("denom x, denom_y",self.denom_x,self.denom_y)
                self.denom_x +=self.minimum_repluse_area
                self.denom_y +=self.minimum_repluse_area
                self.denom_x_root = np.sqrt(self.denom_x) #used by plot ellipse obs 
                self.denom_y_root = np.sqrt(self.denom_y) #used by plot ellipse obs 
                numer_x_origin = (locate_leading_point[0]-locate_obstacles[obs.dynamic_obs_id[id]][0]-obs.obs_velocity_vector[id][0])
                numer_y_origin = (locate_leading_point[1]-locate_obstacles[obs.dynamic_obs_id[id]][1]-obs.obs_velocity_vector[id][1])
                self.obs_direction = np.arctan2(obs.obs_velocity_vector[id][1], obs.obs_velocity_vector[id][0])
                rot = np.array([[cos(self.obs_direction), -sin(self.obs_direction)], [sin(self.obs_direction), cos(self.obs_direction)]])
                v = np.array([numer_x_origin, numer_y_origin])
                rotated_xy = np.dot(rot, v)
                numer_x = (rotated_xy[0])**2
                numer_y = (rotated_xy[1])**2
                if self.denom_x * self.denom_y == 0:
                    dynamic_repulsive_area = 1 # error deal
                else:   
                        A = 0 if self.denom_x == 0 else (numer_x/self.denom_x)
                        B = 0 if self.denom_y == 0 else (numer_y/self.denom_y)
                        dynamic_repulsive_area = (A)+(B) -1 # dynamic_repulsive_area <= 0 indicates in or out ellipse 
                            
                if dynamic_repulsive_area <= 0:
                    dynamic_repulsive_poten = 1*self.dynamic_repulse_k/np.sqrt((dynamic_repulsive_area +1))
                    #print("dynamic rep area",dynamic_repulsive_area)
                    #time.sleep(0.2)  
                    #print("vehicle in dynamic obs ellipse",dynamic_repulsive_poten)
                else:
                    dynamic_repulsive_poten = 0
                self.dynamic_repforce[id] = [self.denom_x,self.denom_y,numer_x,numer_y,dynamic_repulsive_area,dynamic_repulsive_poten]
                self.total_repulsive_force += self.dynamic_repforce[id][5]
        #print("self.total_dynamic_repluse",self.total_repulsive_force)
        return self.total_repulsive_force
    
    def reset_LP_location(self):
        self.locate_leading_point = [veh1.locate_vehicles[0]+veh1.vehicle_vector[0], veh1.locate_vehicles[1]+veh1.vehicle_vector[1]]
        print("reset LP", self.locate_leading_point)
        self.countlimit = 200
        
    def reset_LP_location_for_dynamic_tmp_goal(self):
        #self.locate_leading_point = [(veh1.locate_vehicles[0]+tempgoal4dynamic.temp_goal[0])/2, (veh1.locate_vehicles[1]+tempgoal4dynamic.temp_goal[1])/2]
        #self.locate_leading_point = [(veh1.locate_vehicles[0]+tempgoal4dynamic.temp_goal[0]), (veh1.locate_vehicles[1]+tempgoal4dynamic.temp_goal[0])]
        offset = 0.2
        coordinate1 = [(veh1.locate_vehicles[0]+tempgoal4dynamic.temp_goal[0])/2, (veh1.locate_vehicles[1]+tempgoal4dynamic.temp_goal[1])/2]
        self.locate_leading_point =[(1 - offset) * coordinate1[0] + offset * tempgoal4dynamic.temp_goal[0],(1 - offset) * coordinate1[1] + offset * tempgoal4dynamic.temp_goal[1]]
        print("reset LP for dynamic", self.locate_leading_point)
        self.countlimit = 200

def main(): 
    if APF.dist_v2goal == None:   #clac distance to reach goal at first time
        APF.calc_goal_dist_theta(fgoal.locate_goal, veh1.locate_vehicles)
        APF.calc_obs_dist_theta(obs.locate_obstacles, veh1.locate_vehicles)
        #temp_goal.integrate_temporarygoal(fgoal.locate_goal, veh1.locate_vehicles, obs.locate_obstacles)
        temp_goal.temp_goal = None #itijitekini 
        if temp_goal.temp_goal != None:
            target_goal = temp_goal.temp_goal
            judgereach_finalgoal = True
        elif temp_goal.temp_goal == None:
            target_goal = fgoal.locate_goal
            judgereach_finalgoal = False
        judge_exist_dynamic_goal = False
    if APF.dist_v2goal != None:
        while (APF.dist_v2goal - veh1.diameter) > fgoal.diameter_size:  #iterate until reach goal  
            dynamic_temgoal = dimention.update_leading_point(target_goal)  
            if dynamic_temgoal != None:
                target_goal = dynamic_temgoal
                judge_exist_dynamic_goal = True
            partialdiffer_x, partialdiffer_y = APF.route_creater(target_goal, veh1.locate_vehicles, obs.locate_obstacles)
            duetoexpandmap = 10
            partialdiffer_x, partialdiffer_y = duetoexpandmap*partialdiffer_x, duetoexpandmap*partialdiffer_y #due to graph expaned, speed up
            #print("333partX delt X currentPoten",partialdiffer_x)
            veh1.locate_vehicles = [veh1.locate_vehicles[0]+partialdiffer_x, veh1.locate_vehicles[1]+partialdiffer_y]
            if dimention.first_react and dynamic_temgoal != None:
                dimention.reset_LP_location_for_dynamic_tmp_goal()
                print("reset LP position in Main loop. vehicle, LP, tempgoal",veh1.locate_vehicles,dimention.locate_leading_point, target_goal)
            obs.update_locate_obs()
            path_fig._initialize_fig()
            #print("veh, obs, veh.vector, leading point\n",veh1.locate_vehicles, obs.locate_obstacles, veh1.vehicle_vector,dimention.locate_leading_point)
            print("partiald x y" , partialdiffer_x, partialdiffer_y,APF.dist_v2goal - veh1.diameter)
            anime_array_source = path_fig.plot_vehicle(veh1.locate_vehicles)
            plt.pause(0.7)    
            plt.cla()

        if judge_exist_dynamic_goal or judgereach_finalgoal:
            APF.dist_v2goal = None
            print("reach DYNAMIC temporary goalot temp_goal but not reach final goal")
            #time.sleep(0.02)  
            return main()
        else:
            print("finish and show 3d")
            #anime_gif = ani.ArtistAnimation(path_fig.fig, animation_array, interval=100)
            #anime_gif.save("proposal_gif2.gif", writer="pillow")
            #path_fig.plot_3d_potential()
            


if __name__ == '__main__':
    fgoal = goal([120,120],10)
    obs = obstacles([[60,60]])
    veh1 = vehicles([0,0])
    dimention = proposal_the_other_dimension()
    APF = calc_APF(veh1.speed)
    path_fig = plot_path(veh1.locate_vehicles, fgoal.locate_goal)
    temp_goal = temporary_goal()
    tempgoal4dynamic = temporary_goal()
    animation_array = []
    main()
 
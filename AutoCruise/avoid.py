import math

# ==============================================================================
# -- ACS -----------------------------------------------------------------------
# ==============================================================================
class ACS(object):
    def __init__(self, world, obs, obs2):
        self.world = world
        self.obs = obs
        self.obs2 = obs2
    def tick(self):
        # 周りの車検知
        for num in range(len(self.obs.obs["other_vehicles"]["other_vehicle"])):
            dist = self.obs.obs["other_vehicles"]["transform"][num].location - self.obs.obs["hero_transform"].location
            px, py = self.rotation(math.radians(self.obs.obs["compass"]), dist.x, -dist.y)
            for i in range(40):
                
                temp_oth_loc = self.obs.obs["other_vehicles"]["transform"][num].location + self.obs.obs["other_vehicles"]["velocity"][num]/10 * i
                temp_loc = self.obs.obs["hero_transform"].location + self.obs.obs["velocity"]/10 * i    
                
                temp_dist = temp_oth_loc - temp_loc  
                if (0 < math.sqrt(temp_dist.x**2 + temp_dist.y**2 < 5) and temp_dist.z < 1.75) and (self.obs.obs["corrent_wp"].lane_id == self.obs.obs["other_vehicles"]["corrent_wp"][num].lane_id):  
                    
                    if py < 0:
                        if px >= 0:
                            self.obs.obs["acs_avoid_vehicle_danger_flag"]["right"] = True
                        elif px < 0:  
                            self.obs.obs["acs_avoid_vehicle_danger_flag"]["left"] = True
                        bounding_box = self.obs.obs["other_vehicles"]["other_vehicle"][num].bounding_box
                        bounding_box.location += self.obs.obs["other_vehicles"]["transform"][num].location
                        bounding_box.location += self.obs.obs["other_vehicles"]["velocity"][num]/10
                        self.world.debug.draw_box(bounding_box, self.obs.obs["other_vehicles"]["transform"][num].rotation, thickness=0.05,life_time=0.001)
                        self.obs.obs["acs_avoid_vehicle_danger_flag"]["back"]  = True
                        break
                if 11 > math.sqrt(dist.x**2 + dist.y**2) and i < 15:
                    if (0 < math.sqrt(temp_dist.x**2 + temp_dist.y**2 < 7) and temp_dist.z < 5):
                        if px >= 0:
                            self.obs.obs["acs_avoid_vehicle_danger_flag"]["right"] == True
                        elif px < 0:  
                            self.obs.obs["acs_avoid_vehicle_danger_flag"]["left"] == True
                        bounding_box = self.obs.obs["other_vehicles"]["other_vehicle"][num].bounding_box
                        bounding_box.location += self.obs.obs["other_vehicles"]["transform"][num].location
                        bounding_box.location += self.obs.obs["other_vehicles"]["velocity"][num]/10
                        self.world.debug.draw_box(bounding_box, self.obs.obs["other_vehicles"]["transform"][num].rotation, thickness=0.05,life_time=0.001)
                        self.obs.obs["acs_avoid_vehicle_danger_flag"]["foward"]  = True
                        break

        # 前方のwaypointに重なったら
        for num in range(len(self.obs.obs["other_vehicles"]["other_vehicle"])):
            for num2 in range(len(self.obs2.obs["foward_wps"])-3):
                dist = self.obs.obs["other_vehicles"]["transform"][num].location - self.obs2.obs["foward_wps"][num2].transform.location
                px, py = self.rotation(math.radians(self.obs.obs["compass"]), dist.x, -dist.y)

                for i in range(30):
                    if px >= 0:
                        self.obs.obs["acs_avoid_vehicle_danger_flag"]["right"] = True
                    elif px < 0:  
                        self.obs.obs["acs_avoid_vehicle_danger_flag"]["left"] = True      
                    temp_oth_loc = self.obs.obs["other_vehicles"]["transform"][num].location + self.obs.obs["other_vehicles"]["velocity"][num]/10 * i   
                    temp_dist = temp_oth_loc - self.obs2.obs["foward_wps"][num2].transform.location
                    if 0 < math.sqrt(temp_dist.x**2 + temp_dist.y**2 < 5) and temp_dist.z < 1.75 and self.obs.obs["other_vehicles"]["speed"][num] > 0.5 and\
                        self.obs.obs["other_vehicles"]["dist"][num] < self.obs.obs["other_vehicles"]["before_dist"][num] :  
                        bounding_box = self.obs.obs["other_vehicles"]["other_vehicle"][num].bounding_box
                        bounding_box.location += self.obs.obs["other_vehicles"]["transform"][num].location
                        bounding_box.location += self.obs.obs["other_vehicles"]["velocity"][num]/10
                        self.world.debug.draw_box(bounding_box, self.obs.obs["other_vehicles"]["transform"][num].rotation, thickness=0.05,life_time=0.001)
                        self.obs.obs["acs_avoid_vehicle_danger_flag"]["foward"]  = True
                        break
 
        for num in range(len(self.obs.obs["walkers"]["walker"])):
            for foward_wp in self.obs2.obs["foward_wps"]:
                dist = self.obs.obs["walkers"]["transform"][num].location - foward_wp.transform.location
                px, py = self.rotation(math.radians(self.obs.obs["compass"]), dist.x, -dist.y)
                for i in range(35):
                    if px >= 0:
                        self.obs.obs["acs_avoid_vehicle_danger_flag"]["right"] = True
                    elif px < 0:  
                        self.obs.obs["acs_avoid_vehicle_danger_flag"]["left"] = True    
                    
                    temp_oth_loc = self.obs.obs["walkers"]["transform"][num].location + self.obs.obs["walkers"]["velocity"][num]/10 * i   
                    
                    temp_dist = temp_oth_loc - foward_wp.transform.location
                    if 0 < math.sqrt(temp_dist.x**2 + temp_dist.y**2 < 2) and temp_dist.z < 1.75:  
                        bounding_box = self.obs.obs["walkers"]["walker"][num].bounding_box
                        bounding_box.location += self.obs.obs["walkers"]["transform"][num].location
                        bounding_box.location += self.obs.obs["walkers"]["velocity"][num]/10
                        self.world.debug.draw_box(bounding_box, self.obs.obs["walkers"]["transform"][num].rotation, thickness=0.05,life_time=0.001)
                        self.obs.obs["acs_avoid_vehicle_danger_flag"]["foward"]  = True
                        break
        
 
        # for num in range(len(self.obs.obs["walkers"]["walker"])):
        #     dist = self.obs.obs["walkers"]["transform"][num].location - self.obs.obs["hero_transform"].location
        #     px, py = self.kaiten(self.obs.obs["hero_transform"].rotation.yaw + 90, dist.x, dist.y)
        #     for foward_wp in self.obs2.obs["foward_wps"]:
        #         for i in range(50):
                    
        #             temp_oth_loc = self.obs.obs["walkers"]["transform"][num].location + self.obs.obs["walkers"]["velocity"][num]/10 * i
        #             temp_loc = self.obs.obs["hero_transform"].location + self.obs.obs["velocity"]/10 * i    
                    
        #             temp_dist = temp_oth_loc - temp_loc  
        #             if 0 < math.sqrt(temp_dist.x**2 + temp_dist.y**2 < 1.75) and temp_dist.z < 1.75:  
        #                 bounding_box = self.obs.obs["walkers"]["walker"][num].bounding_box
        #                 bounding_box.location += self.obs.obs["walkers"]["transform"][num].location
        #                 bounding_box.location += self.obs.obs["walkers"]["velocity"][num]/10
        #                 self.world.debug.draw_box(bounding_box, self.obs.obs["walkers"]["transform"][num].rotation, thickness=0.05,life_time=0.001)
        #                 self.obs.obs["walker_danger_flag"]  = True
        #                 break
        

    def rotation(self, r, l1, l2):
        n1 = l1 * math.cos(r) - l2 *math.sin(r)
        n2 = l1 * math.sin(r) + l2 *math.cos(r)
        return n1, n2
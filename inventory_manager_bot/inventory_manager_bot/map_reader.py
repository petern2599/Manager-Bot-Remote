import cv2
import numpy as np

class MapRender():
    """
    Node that grabs video frames from camera and publishes it to ROS.
    """
    def __init__(self,file):
        self.map_data = cv2.imread(file)
        self.resolution_factor = 0.05
        self.zero_pos_horizontal = 0
        self.zero_pos_vertical = int(self.map_data.shape[0])
        self.origin_pos_x = -2.42
        self.origin_pos_y = -0.955
        
        
        new_map = cv2.circle(self.map_data,(self.zero_pos_horizontal,self.zero_pos_vertical),3,(0,0,255),-1)

        width,height = self.get_map_dimensions()
        map_pos_x,map_pos_y=self.find_max_pos(width,height)
        print(map_pos_x,map_pos_y)

        map_cell_pos_x,map_cell_pos_y = self.interpolate_map_cell_position(map_pos_x,map_pos_y)
        print(width,height)
        print(map_cell_pos_x,map_cell_pos_y)
        
        if map_cell_pos_x > width or map_cell_pos_x < 0 or map_cell_pos_y > height or map_cell_pos_y < 0:
            print("Please enter different position, current values are out of bounds")
            cv2.imshow("Map",new_map)
            cv2.waitKey(0)
        else:
            new_map = cv2.circle(new_map,(map_cell_pos_x,map_cell_pos_y),3,(255,0,0),-1)
            #is_obstacle = self.check_for_obstacle(map_cell_pos_x+5,map_cell_pos_y+5)
            cv2.imshow("Map",new_map)
            cv2.waitKey(0)
        cv2.destroyAllWindows()
        exit(0)

    def get_map_dimensions(self):
        width = int(self.map_data.shape[1])
        height = int(self.map_data.shape[0])
        return width,height

    def interpolate_map_cell_position(self,x,y):
        map_cell_pos_x = int((x - self.origin_pos_x)/self.resolution_factor) + self.zero_pos_horizontal
        map_cell_pos_y = self.zero_pos_vertical - int((y - self.origin_pos_y)/self.resolution_factor)
        return map_cell_pos_x,map_cell_pos_y
    
    def find_max_pos(self,width,height):
        max_pos_x = round(width*self.resolution_factor + self.origin_pos_x,2)
        max_pos_y = round(height*self.resolution_factor + self.origin_pos_y,2)
        return max_pos_x,max_pos_y


    
    
    def check_for_obstacle(self,x,y):
        value = np.array(self.map_data[y][x])
        return np.array_equal(value,np.zeros(3))


def main(args=None):
    file = "/home/petern25/ros2_ws/my_map_save.pgm"
    render = MapRender(file)

if __name__ == "__main__":
    main()
        

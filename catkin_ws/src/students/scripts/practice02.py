#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2022-2
# PRACTICE 02 - MAPS 
#
# Instructions:
# Complete the code necessary to inflate the obstacles given an occupancy grid map and
# a number of cells to inflate. 
#

import rospy
import numpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from nav_msgs.srv import GetMapResponse
from nav_msgs.srv import GetMapRequest

NAME = "Gutierrez Orozpe Luis Fernando"

def get_inflated_map(static_map, inflation_cells):
    print("Inflating map by " + str(inflation_cells) + " cells")
    inflated = numpy.copy(static_map)
    [height, width] = static_map.shape
    #
    # TODO:
    # Write the code necessary to inflate the obstacles in the map a radius
    # given by 'inflation_cells' (expressed in number of cells)
    # Map is given in 'static_map' as a bidimensional numpy array.
    # Consider as occupied cells all cells with an occupation value greater than 50
    #
    for i in range(len(static_map)):
        for j in range(len(static_map[i])):
            if static_map[i,j] > 50:
                for k1 in range(-inflation_cells,inflation_cells):
                    for k2 in range(-inflation_cells,inflation_cells):
                        inflated[i+k1,j+k2] = 100
    return inflated

def callback_inflated_map(req):
    global inflated_map
    return GetMapResponse(map=inflated_map)

def main():
    global cost_map, inflated_map
    print("PRACTICE 02 - " + NAME)
    rospy.init_node("practice02")
    rospy.wait_for_service('/static_map')
    pub_map  = rospy.Publisher("/inflated_map", OccupancyGrid, queue_size=10)
    grid_map = rospy.ServiceProxy("/static_map", GetMap)().map
    map_info = grid_map.info
    width, height, res = map_info.width, map_info.height, map_info.resolution
    grid_map = numpy.reshape(numpy.asarray(grid_map.data, dtype='int'), (height, width))
    rospy.Service('/inflated_map', GetMap, callback_inflated_map)
    loop = rospy.Rate(2)
    
    inflation_radius = 0.1
    while not rospy.is_shutdown():
        if rospy.has_param("/path_planning/inflation_radius"):
            new_inflation_radius = rospy.get_param("/path_planning/inflation_radius")
        if new_inflation_radius != inflation_radius:
            inflation_radius  = new_inflation_radius
            inflated_map_data = get_inflated_map(grid_map, int(inflation_radius/res))
            inflated_map_data = numpy.ravel(numpy.reshape(inflated_map_data, (width*height, 1)))
            inflated_map      = OccupancyGrid(info=map_info, data=inflated_map_data)
        pub_map.publish(callback_inflated_map(GetMapRequest()).map)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    

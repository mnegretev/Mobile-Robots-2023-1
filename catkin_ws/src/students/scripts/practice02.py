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

NAME = "Daniel Najera Araoz"

def get_inflated_map(static_map, inflation_cells):
    print("Inflating map by " + str(inflation_cells) + " cells")
    inflated = numpy.copy(static_map)
    [height, width] = static_map.shape
     ri= inflation_cells                             
    for i in range (0,height,1):		#Se recorre el mapa original
    	for j in range (0,width,1):
    	    if inflated[i,j]>51:		#Si en el mapa original existe un obstaculo
		for k1 in range(-ri,ri,1):	#Se recorren los alrededores de la celda
		    for k2 in range(-ri,ri,1):
			inflated[i+k1,j+k2]=50	#Se calcula la inflacion
    return inflated
def get_cost_map(static_map, cost_radius):
    if cost_radius > 20:
        cost_radius = 20
    print "Calculating cost map with " +str(cost_radius) + " cells"
    cost_map = numpy.copy(static_map)           #Se hace una copia del mapa original
    [height, width] = static_map.shape
    
    rc= cost_radius
    for i in range(0,height,1):			#Se recorre el mapa original
	for j in range (0,width,1):		
	    if static_map[i,j]>50:              #Si en el mapa original existe un obstaculo
		for k3 in range(i-rc, i+rc,1):         #Se recorren los alrededores de la celda
			for k4 in range(j-rc,j+rc,1):
				c=rc-max(abs(k3-i),abs(k4-j))   #Se calcula el costo de cada celda
				cost_map[k3,k4]=max(c,cost_map[k3,k4])   #Se conserva el maximo
    return cost_map

def callback_inflated_map(req):
    global inflated_map
    return GetMapResponse(map=inflated_map)

def callback_cost_map(req):
    global cost_map
    return GetMapResponse(map=cost_map)

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
    cost_radius      = 0.1
    inflation_radius = 0.1
    while not rospy.is_shutdown():
        if rospy.has_param("/path_planning/cost_radius"):
            new_cost_radius = rospy.get_param("/path_planning/cost_radius")
        if rospy.has_param("/path_planning/inflation_radius"):
            new_inflation_radius = rospy.get_param("/path_planning/inflation_radius")
        if new_cost_radius != cost_radius:
            cost_radius   = new_cost_radius
            cost_map_data = get_cost_map(grid_map, int(cost_radius/res))
            cost_map_data = numpy.ravel(numpy.reshape(cost_map_data, (width*height, 1)))
            cost_map      = OccupancyGrid(info=map_info, data=cost_map_data)    
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
    

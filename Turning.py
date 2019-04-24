import Enum
import Drone
import Utils
import VoronoiForInitialSearch

import math

class Turning():

    def __init__(self, utils):
        self.utils = utils
    def updateTurningState(self, uavInfos):
        threshold = 10

        current_heading = uavInfos['OBJ'].getHeading()

        if abs(abs(current_heading) - abs(uavInfos['STATE_DETAIL'][Enum.TURNING]['heading'])) <= threshold:
            uavInfos['STATE'] = uavInfos['STATE_DETAIL'][Enum.TURNING]['previous_action']

            if uavInfos['STATE'] == Enum.SEARCHING:
                uavInfos['STATE_DETAIL'][Enum.SEARCHING]['current_index'] = (uavInfos['STATE_DETAIL'][Enum.SEARCHING]['current_index']+1)%(len(uavInfos['STATE_DETAIL'][Enum.SEARCHING]['total_points']))
        else :
            uavInfos['NEXT_HEADING'] = uavInfos['STATE_DETAIL'][Enum.TURNING]['heading']

    def turning(self, point, uavInfos):
        uav_lat = uavInfos['OBJ'].getLatitude()
        uav_lon = uavInfos['OBJ'].getLongitude()

        det_lat = point[0]
        det_lon = point[1]

        uavInfos['STATE_DETAIL'][Enum.TURNING]['heading'] = self.utils.getHeadingToDest(det_lat, det_lon, uav_lat, uav_lon)
        uavInfos['NEXT_HEADING'] = uavInfos['STATE_DETAIL'][Enum.TURNING]['heading'] 
        uavInfos['STATE_DETAIL'][Enum.TURNING]['previous_action'] = uavInfos['STATE']
        uavInfos['STATE'] = Enum.TURNING
        print("UAV -",uavInfos['OBJ'].getID()," is turning")
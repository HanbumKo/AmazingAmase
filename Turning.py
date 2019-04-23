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

        if abs(abs(current_heading) - abs(uavInfos['ACTION_DETAIL']['TURNING']['heading'])) <= threshold:
            uavInfos['ACTION'] = uavInfos['ACTION_DETAIL']['TURNING']['previous_action']

            if uavInfos['ACTION'] == Enum.SEARCHING:
                uavInfos['ACTION_DETAIL']['SEARCH']['current_index'] = (uavInfos['ACTION_DETAIL']['SEARCH']['current_index']+1)%(len(uavInfos['ACTION_DETAIL']['SEARCH']['total_points']))
        else :
            uavInfos['NEXT_HEADING'] = uavInfos['ACTION_DETAIL']['TURNING']['heading']

    def turning(self, point, uavInfos):
        uav_lat = uavInfos['OBJ'].getLatitude()
        uav_lon = uavInfos['OBJ'].getLongitude()

        det_lat = point[0]
        det_lon = point[1]

        uavInfos['ACTION_DETAIL']['TURNING']['heading'] = self.utils.getHeadingToDest(det_lat, det_lon, uav_lat, uav_lon)
        uavInfos['NEXT_HEADING'] = uavInfos['ACTION_DETAIL']['TURNING']['heading'] 
        uavInfos['ACTION_DETAIL']['TURNING']['previous_action'] = uavInfos['ACTION']
        uavInfos['ACTION'] = Enum.TURNING
        print("UAV -",uavInfos['OBJ'].getID()," is turning")
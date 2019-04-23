import Drone
import Utils
import VoronoiForInitialSearch

import math

class Charging():

    def __init__(self, utils):
        self.utils = utils

    def updateChargingState(self, uavInfos):
        threshold = 0.003
        fDist = self.utils.distance(uavInfos['OBJ'].getLongitude(), uavInfos['OBJ'].getLatitude(),
                                uavInfos['ACTION_DETAIL']['WELCOME']['recovery_point'][1], uavInfos['ACTION_DETAIL']['WELCOME']['recovery_point'][0])
        
        if fDist <= threshold :
            uavInfos['ACTION'] = uavInfos['ACTION_DETAIL']['CHARGING']['previous_action']
        else : 
            uavInfos['NEXT_HEADING'] = self.utils.getHeadingToDest(uavInfos['OBJ'].getLatitude(), uavInfos['OBJ'].getLongitude(),
                                uavInfos['ACTION_DETAIL']['WELCOME']['recovery_point'][0], uavInfos['ACTION_DETAIL']['WELCOME']['recovery_point'][1])



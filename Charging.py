import Drone
import Enum
import Utils
import VoronoiForInitialSearch

import math

class Charging():

    def __init__(self, utils):
        self.utils = utils

    def updateChargingState(self, uavInfos):
        threshold = 0.003
        fDist = self.utils.distance(uavInfos['OBJ'].getLongitude(), uavInfos['OBJ'].getLatitude(),
                                uavInfos['STATE_DETAIL'][Enum.INITIAL_STATE]['recovery_point'][1], uavInfos['STATE_DETAIL'][Enum.INITIAL_STATE]['recovery_point'][0])
        
        if fDist <= threshold :
            uavInfos['STATE'] = uavInfos['STATE_DETAIL'][Enum.CHARGING]['previous_action']
        else : 
            uavInfos['NEXT_HEADING'] = self.utils.getHeadingToDest(uavInfos['OBJ'].getLatitude(), uavInfos['OBJ'].getLongitude(),
                                uavInfos['STATE_DETAIL'][Enum.INITIAL_STATE]['recovery_point'][0], uavInfos['STATE_DETAIL'][Enum.INITIAL_STATE]['recovery_point'][1])



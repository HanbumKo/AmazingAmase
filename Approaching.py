import Drone
import Enum
import Utils
import VoronoiForInitialSearch

import math

class Approaching():

    def __init__(self, utils):
        self.utils = utils

    def updateApproachingState(self, uavInfos):
        # dest_point = self.detectedZones.getCenterPointInZone(uavInfos['STATE_DETAIL'][Enum.APPROACHING]['tracking_zoneID'])
        dest_point = uavInfos['STATE_DETAIL'][Enum.APPROACHING]['dest_position']
        uavInfos['NEXT_HEADING'] = self.utils.getHeadingToDest(uavInfos['OBJ'].getLatitude(), uavInfos['OBJ'].getLongitude(),
                                                                dest_point[0], dest_point[1]) + uavInfos['STATE_DETAIL'][Enum.APPROACHING]['azimuth'] 



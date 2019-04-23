import Drone
import Enum
import Utils
import VoronoiForInitialSearch

import math

class Undertaked():

    def __init__(self, utils):
        self.utils = utils

    def updateUndertakedState(self, uavInfos):
        # dest_point = self.detectedZones.getCenterPointInZone(uavInfos['STATE_DETAIL'][Enum.UNDERTAKED]['tracking_zoneID'])
        dest_point = uavInfos['STATE_DETAIL'][Enum.UNDERTAKED]['dest_position']
        uavInfos['NEXT_HEADING'] = self.utils.getHeadingToDest(uavInfos['OBJ'].getLatitude(), uavInfos['OBJ'].getLongitude(),
                                                                dest_point[0], dest_point[1])


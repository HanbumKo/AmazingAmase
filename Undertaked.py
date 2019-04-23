import Drone
import Utils
import VoronoiForInitialSearch

import math

class Undertaked():

    def __init__(self, utils):
        self.utils = utils

    def updateUndertakedState(self, uavInfos):
        # dest_point = self.detectedZones.getCenterPointInZone(uavInfos['ACTION_DETAIL']['UNDERTAKED']['tracking_zoneID'])
        dest_point = uavInfos['ACTION_DETAIL']['UNDERTAKED']['dest_position']
        uavInfos['NEXT_HEADING'] = self.utils.getHeadingToDest(uavInfos['OBJ'].getLatitude(), uavInfos['OBJ'].getLongitude(),
                                                                dest_point[0], dest_point[1])


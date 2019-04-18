from afrl.cmasi.Polygon import Polygon

import Drone
import Utils
import Enum
import math

import numpy as np
from scipy.spatial import ConvexHull

class DetectedZone():

    def __init__(self, utils):
        self.zones = {}
        self.utils = utils

    def isAlreadyDetectedZone(self, detectedPoint):
        pass
    def addNewDetectedZone(self, detectedPoint):
        zoneId = len(self.zones)
        self.zones[zoneId] = []
        self.zones[zoneId].append(detectedPoint)
        return zoneId

    def getDetectedZoneById(self, zoneId):
        if zoneId in self.zones.keys():
            return self.zones[zoneId]
        return None

    def getDetectedZones(self):
        return self.zones
    
    def sendEstimateCmd(self):
        for k, v in self.zones.items():
            all_coords = [[coord.get_Latitude(), coord.get_Longitude()] for coord in v]
            hull = ConvexHull(np.asarray(all_coords, dtype=np.float32))
            if len(all_coords) < 3:
                print("Not that much to estimate")
                return
            self.zones[k] = [v[i] for i in hull.vertices]

            # Create polygon object
            estimatedHazardZone = Polygon()

            for i in self.zones[k]:
                estimatedHazardZone.get_BoundaryPoints().append(i)

            self.utils.send_estimate_report(estimatedHazardZone, k)
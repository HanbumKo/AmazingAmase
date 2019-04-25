from afrl.cmasi.Polygon import Polygon

import Drone
import Utils
import Enum
import math
import sys

from scipy.spatial import ConvexHull
import numpy as np

class DetectedZone():

    def __init__(self, utils):
        self.zones = {}
        self.utils = utils
        self.threshold = 0.2

    def isAlreadyDetectedZone(self, detectedPoint):
        # 가장 가까운 점과의 거리가 일정거리 차이나면 새로운 존
        fShortestDist = sys.maxsize
        zoneId = 0
        for i in range(len(self.zones)):
            for point in self.zones[i]:
                fDist = self.utils.distance(point.get_Longitude(), point.get_Latitude(), detectedPoint.get_Longitude(), detectedPoint.get_Latitude()) 
                if fDist < fShortestDist:
                    zoneId = i; fShortestDist = fDist
        
        return zoneId if fShortestDist < self.threshold else -1

    def addNewDetectedZone(self, detectedPoint):
        zoneId = len(self.zones)
        self.zones[zoneId] = []
        self.zones[zoneId].append(detectedPoint)
        return zoneId, self.zones[zoneId]

    def getDetectedZoneById(self, zoneId):
        if zoneId in self.zones.keys():
            return self.zones[zoneId]
        return None

    def getDetectedZones(self):
        return self.zones
    
    def getCenterPointInZone(self, zoneId):
        lat = 0
        lon = 0

        for point in self.zones[zoneId] :
            lat += point.get_Latitude()
            lon += point.get_Longitude()
        
        return [lat/len(self.zones[zoneId]), lon/len(self.zones[zoneId])]

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
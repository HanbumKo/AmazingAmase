from afrl.cmasi.Polygon import Polygon
from afrl.cmasi.searchai.HazardZoneEstimateReport import HazardZoneEstimateReport
from afrl.cmasi.searchai.HazardZone import HazardZone
from afrl.cmasi.searchai.HazardType import HazardType



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

            hazard_estimate_report = HazardZoneEstimateReport()
            hazard_estimate_report.set_EstimatedZoneShape(estimatedHazardZone)

            hazard_estimate_report.set_UniqueTrackingID(k)
            hazard_estimate_report.set_EstimatedGrowthRate(0)
            hazard_estimate_report.set_PerceivedZoneType(HazardType.Fire)
            hazard_estimate_report.set_EstimatedZoneDirection(0)
            hazard_estimate_report.set_EstimatedZoneSpeed(0)

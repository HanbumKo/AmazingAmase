import Drone
import Utils
import Enum
import math

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
    
    
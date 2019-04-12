class RecoveryZoneInfo():
    def __init__(self):
        self.ZoneID = -1
        self.MinAltitude = -1
        self.MinAltitudeType = None
        self.MaxAltitude = -1
        self.MaxAltitudeType = None
        self.StartTime = -1
        self.EndTime = -1
        self.Padding = -1
        self.Latitude = -1
        self.Longitude = -1
        self.Altitude = -1
        self.AltitudeType = None
        self.Radius = -1

    # Getter / Setter
    def getZoneID(self):
        return self.ZoneID
    def getMinAltitude(self):
        return self.MinAltitude
    def getMinAltitudeType(self):
        return self.MinAltitudeType
    def getMaxAltitude(self):
        return self.MaxAltitude
    def getMaxAltitudeType(self):
        return self.MaxAltitudeType
    def getStartTime(self):
        return self.StartTime
    def getEndTime(self):
        return self.EndTime
    def getPadding(self):
        return self.Padding
    def getLatitude(self):
        return self.Latitude
    def getLongitude(self):
        return self.Longitude
    def getAltitude(self):
        return self.Altitude
    def getAltitudeType(self):
        return self.AltitudeType
    def getRadius(self):
        return self.Radius

    def setZoneID(self, newZoneID):
        self.ZoneID = newZoneID
    def setMinAltitude(self, newMinAltitude):
        self.MinAltitude = newMinAltitude
    def setMinAltitudeType(self, newMinAltitudeType):
        self.MinAltitudeType = newMinAltitudeType
    def setMaxAltitude(self, newMaxAltitude):
        self.MaxAltitude = newMaxAltitude
    def setMaxAltitudeType(self, newMaxAltitudeType):
        self.MaxAltitudeType = newMaxAltitudeType
    def setStartTime(self, newStartTime):
        self.StartTime = newStartTime
    def setEndTime(self, newEndTime):
        self.EndTime = newEndTime
    def setPadding(self, newPadding):
        self.Padding = newPadding
    def setLatitude(self, newLatitude):
        self.Latitude = newLatitude
    def setLongitude(self, newLongitude):
        self.Longitude = newLongitude
    def setAltitude(self, newAltitude):
        self.Altitude = newAltitude
    def setAltitudeType(self, newAltitudeType):
        self.AltitudeType = newAltitudeType
    def setRadius(self, newRadius):
        self.Radius = newRadius

    def updateRecoveryZone(self, RecoveryZone):
        self.setMinAltitudeType(RecoveryZone.get_MinAltitudeType())
        self.setMinAltitude(RecoveryZone.get_MinAltitude())
        self.setMaxAltitudeType(RecoveryZone.get_MaxAltitudeType())
        self.setMaxAltitude(RecoveryZone.get_MaxAltitude())
        self.setEndTime(RecoveryZone.get_EndTime())
        self.setPadding(RecoveryZone.get_Padding())
        self.setStartTime(RecoveryZone.get_StartTime())
        self.setZoneID(RecoveryZone.get_ZoneID())
        self.setLatitude(RecoveryZone.get_Boundary().get_CenterPoint().get_Latitude())
        self.setLongitude(RecoveryZone.get_Boundary().get_CenterPoint().get_Longitude())
        self.setAltitude(RecoveryZone.get_Boundary().get_CenterPoint().get_Altitude())
        self.setAltitudeType(RecoveryZone.get_Boundary().get_CenterPoint().get_AltitudeType())
        self.setRadius(RecoveryZone.get_Boundary().get_Radius())
        print("Recovery Zone is updated")


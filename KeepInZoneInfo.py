

class KeepInZoneInfo():
    def __init__(self):
        self.ZoneID = -1
        self.MinAltitude = -1
        self.MinAltitudeType = None
        self.MaxAltitude = -1
        self.MaxAltitudeType = None
        self.AffectedAircraft = None
        self.StartTime = -1
        self.EndTime = -1
        self.Padding = -1
        self.Label = None
        self.Width = -1
        self.Height = -1
        self.Rotation = -1
        self.CenterLatitude = -1
        self.CenterLongitude = -1
        self.CenterAltitude = -1
        self.CenterAltitudeType = -1


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
    def getAffctedAircraft(self):
        return self.AffectedAircraft
    def getStartTime(self):
        return self.StartTime
    def getEndTime(self):
        return self.EndTime
    def getPadding(self):
        return self.Padding
    def getLabel(self):
        return self.Label
    def getWidth(self):
        return self.Width
    def getHeight(self):
        return self.Height
    def getRotation(self):
        return self.Rotation
    def getCenterLatitude(self):
        return self.CenterLatitude
    def getCenterLongitude(self):
        return self.CenterLongitude
    def getCenterAltitude(self):
        return self.CenterAltitude
    def getCenterAltitudeType(self):
        return self.CenterAltitudeType
    def getPoints(self):
        points = []
        
        center_lat = self.CenterLatitude
        center_lon = self.CenterLongitude
        width = self.Width / 100000
        height = self.Height / 100000

        left = center_lon - (width / 2)
        right = center_lon + (width / 2)
        top = center_lat + (height / 2)
        bottom = center_lat - (height / 2)

        points = [[top,left], [top, right], [bottom, right], [bottom, left]]
        return points
    # ===========================================
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
    def setAffctedAircraft(self, newAffectedAircraft):
        self.AffectedAircraft = newAffectedAircraft
    def setStartTime(self, newStartTime):
        self.StartTime = newStartTime
    def setEndTime(self, newEndTime):
        self.EndTime = newEndTime
    def setPadding(self, newPadding):
        self.Padding = newPadding
    def setLabel(self, newLabel):
        self.Label = newLabel
    def setWidth(self, newWidth):
        self.Width = newWidth
    def setHeight(self, newHeight):
        self.Height = newHeight
    def setRotation(self, newRotation):
        self.Rotation = newRotation
    def setCenterLatitude(self, newCenterLatitude):
        self.CenterLatitude = newCenterLatitude
    def setCenterLongitude(self, newCenterLongitude):
        self.CenterLongitude = newCenterLongitude
    def setCenterAltitude(self, newCenterAltitude):
        self.CenterAltitude = newCenterAltitude
    def setCenterAltitudeType(self, newCenterAltitudeType):
        self.CenterAltitudeType = newCenterAltitudeType



    def updateKeepInZone(self, KeepInZone):
        self.setMinAltitudeType(KeepInZone.get_MinAltitudeType())
        self.setMinAltitude(KeepInZone.get_MinAltitude())
        self.setMaxAltitudeType(KeepInZone.get_MaxAltitudeType())
        self.setMaxAltitude(KeepInZone.get_MaxAltitude())
        self.setLabel(KeepInZone.get_Label())
        self.setAffctedAircraft(KeepInZone.get_AffectedAircraft())
        self.setEndTime(KeepInZone.get_EndTime())
        self.setPadding(KeepInZone.get_Padding())
        self.setStartTime(KeepInZone.get_StartTime())
        self.setZoneID(KeepInZone.get_ZoneID())
        self.setCenterLatitude(KeepInZone.get_Boundary().get_CenterPoint().get_Latitude())
        self.setCenterLongitude(KeepInZone.get_Boundary().get_CenterPoint().get_Longitude())
        self.setCenterAltitude(KeepInZone.get_Boundary().get_CenterPoint().get_Altitude())
        self.setCenterAltitudeType(KeepInZone.get_Boundary().get_CenterPoint().get_AltitudeType())
        self.setWidth(KeepInZone.get_Boundary().get_Width())
        self.setHeight(KeepInZone.get_Boundary().get_Height())
        self.setRotation(KeepInZone.get_Boundary().get_Rotation())
        print("Keep In Zone is updated")


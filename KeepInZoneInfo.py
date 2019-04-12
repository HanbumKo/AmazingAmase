from amase.TCPClient import AmaseTCPClient
from amase.TCPClient import IDataReceived
from afrl.cmasi.searchai.HazardZoneEstimateReport import HazardZoneEstimateReport
from afrl.cmasi.searchai.RecoveryPoint import RecoveryPoint
from afrl.cmasi.GimbalAngleAction import GimbalAngleAction
from afrl.cmasi.GimbalScanAction import GimbalScanAction
from afrl.cmasi.Rectangle import Rectangle
from afrl.cmasi.Circle import Circle
from afrl.cmasi.Polygon import Polygon
from afrl.cmasi.Waypoint import Waypoint
from afrl.cmasi.VehicleActionCommand import VehicleActionCommand
from afrl.cmasi.LoiterAction import LoiterAction
from afrl.cmasi.LoiterType import LoiterType
from afrl.cmasi.LoiterDirection import LoiterDirection
from afrl.cmasi.CommandStatusType import CommandStatusType
from afrl.cmasi.AltitudeType import AltitudeType
from afrl.cmasi.searchai.HazardZoneDetection import HazardZoneDetection
from afrl.cmasi.searchai.HazardType import HazardType
from afrl.cmasi.Location3D import Location3D
from afrl.cmasi.AirVehicleState import AirVehicleState
from afrl.cmasi.SessionStatus import SessionStatus
from afrl.cmasi.EntityState import EntityState
from afrl.cmasi.AirVehicleConfiguration import AirVehicleConfiguration
from afrl.cmasi.FlightDirectorAction import FlightDirectorAction
from afrl.cmasi.KeepInZone import KeepInZone
from afrl.cmasi.MissionCommand import MissionCommand
from afrl.cmasi.SpeedType import SpeedType
from afrl.cmasi.TurnType import TurnType
from afrl.cmasi.AbstractGeometry import AbstractGeometry
from afrl.cmasi.AbstractZone import AbstractZone
from afrl.cmasi.WeatherReport import WeatherReport
from afrl.cmasi.searchai.HazardZone import HazardZone
from afrl.cmasi.RemoveEntities import RemoveEntities
from afrl.cmasi.perceive.EntityPerception import EntityPerception
from afrl.cmasi.EntityConfiguration import EntityConfiguration


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


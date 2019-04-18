from amase.TCPClient import AmaseTCPClient
from amase.TCPClient import IDataReceived
from afrl.cmasi.searchai.HazardZoneEstimateReport import HazardZoneEstimateReport
from afrl.cmasi.searchai.RecoveryPoint import RecoveryPoint
from afrl.cmasi.GimbalAngleAction import GimbalAngleAction
from afrl.cmasi.GimbalScanAction import GimbalScanAction
from afrl.cmasi.Rectangle import Rectangle
from afrl.cmasi.Circle import Circle
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

import Drone
import Enum
import Utils
import InitialSearch
import Tracking
import DetectedZone

import sys

"""
State class manage all of drones efficiently. This must be implemented easy to see, easy to use.
The drone's ID could be different from come order. Dictionary can find drone object fast using each ID.
"""
class State():
    def __init__(self, utils):
        self.uavList = {} # Dictionary
        self.numOfDrone = 0
        self.utils = utils
        self.initialSearch = None
        self.tracking = Tracking.Tracking(utils)
        self.detectedZones = DetectedZone.DetectedZone(utils)
        
    def updateUAV(self, AirVehicleState):
        ID = AirVehicleState.get_ID()
        droneObject = {}
        if ID not in self.uavList:
            print("Error : UAV ", ID, " is not in uavList")
            exit()
        else:
            droneObject = self.uavList[ID].OBJ
        droneObject.setWindDirection(AirVehicleState.get_WindDirection())
        droneObject.setID(AirVehicleState.get_ID())
        droneObject.setWindSpeed(AirVehicleState.get_WindSpeed())
        droneObject.setAirspeed(AirVehicleState.get_Airspeed())
        droneObject.setHeading(AirVehicleState.get_Heading())
        droneObject.setLatitude(AirVehicleState.get_Location().get_Latitude())
        droneObject.setLongitude(AirVehicleState.get_Location().get_Longitude())
        droneObject.setAltitude(AirVehicleState.get_Location().get_Altitude())
        droneObject.setEnergyAvailable(AirVehicleState.get_EnergyAvailable())
        droneObject.setActualEnergyRate(AirVehicleState.get_ActualEnergyRate())
        droneObject.setCourse(AirVehicleState.get_Course())
        droneObject.setCurrentCommand(AirVehicleState.get_CurrentCommand())
        droneObject.setCurrentWaypoint(AirVehicleState.get_CurrentWaypoint())
        droneObject.setGroundspeed(AirVehicleState.get_Groundspeed())
        droneObject.setMode(AirVehicleState.get_Mode())
        droneObject.setP(AirVehicleState.get_p())
        droneObject.setPitch(AirVehicleState.get_Pitch())
        droneObject.setQ(AirVehicleState.get_q())
        droneObject.setR(AirVehicleState.get_r())
        droneObject.setRoll(AirVehicleState.get_Roll())
        droneObject.setTime(AirVehicleState.get_Time())
        droneObject.setU(AirVehicleState.get_u())
        droneObject.setUdot(AirVehicleState.get_udot())
        droneObject.setV(AirVehicleState.get_v())
        droneObject.setVdot(AirVehicleState.get_vdot())
        droneObject.setVerticalSpeed(AirVehicleState.get_VerticalSpeed())
        droneObject.setW(AirVehicleState.get_w())
        droneObject.setWdot(AirVehicleState.get_wdot())
        droneObject.setLatitude(AirVehicleState.get_Location().get_Latitude())
        droneObject.setLongitude(AirVehicleState.get_Location().get_Longitude())
        droneObject.setAltitude(AirVehicleState.get_Location().get_Altitude())
        droneObject.setAltitudeType(AirVehicleState.get_Location().get_AltitudeType())

        # Gimbal state
        droneObject.setGimbalPayloadID(AirVehicleState.get_PayloadStateList()[0].get_PayloadID())
        droneObject.setGimbalPointingMode(AirVehicleState.get_PayloadStateList()[0].get_PointingMode())
        droneObject.setGimbalAzimuth(AirVehicleState.get_PayloadStateList()[0].get_Azimuth())
        droneObject.setGimbalElevation(AirVehicleState.get_PayloadStateList()[0].get_Elevation())
        droneObject.setGimbalRotation(AirVehicleState.get_PayloadStateList()[0].get_Rotation())

        # Camera state
        droneObject.setCameraPayloadID(AirVehicleState.get_PayloadStateList()[1].get_PayloadID())
        droneObject.setCameraPointingMode(AirVehicleState.get_PayloadStateList()[1].get_PointingMode())
        droneObject.setCameraAzimuth(AirVehicleState.get_PayloadStateList()[1].get_Azimuth())
        droneObject.setCameraElevation(AirVehicleState.get_PayloadStateList()[1].get_Elevation())
        droneObject.setCameraRotation(AirVehicleState.get_PayloadStateList()[1].get_Rotation())
        droneObject.setCameraHorizontalFieldOfView(AirVehicleState.get_PayloadStateList()[1].get_HorizontalFieldOfView())
        droneObject.setCameraVerticalFieldOfView(AirVehicleState.get_PayloadStateList()[1].get_VerticalFieldOfView())
        droneObject.setCameraCenterpoint(AirVehicleState.get_PayloadStateList()[1].get_Centerpoint())

        # Hazard sensor state
        droneObject.setCameraPayloadID(AirVehicleState.get_PayloadStateList()[2].get_PayloadID())
        droneObject.setCameraPointingMode(AirVehicleState.get_PayloadStateList()[2].get_PointingMode())
        droneObject.setCameraAzimuth(AirVehicleState.get_PayloadStateList()[2].get_Azimuth())
        droneObject.setCameraElevation(AirVehicleState.get_PayloadStateList()[2].get_Elevation())
        droneObject.setCameraRotation(AirVehicleState.get_PayloadStateList()[2].get_Rotation())
        droneObject.setCameraHorizontalFieldOfView(AirVehicleState.get_PayloadStateList()[2].get_HorizontalFieldOfView())
        droneObject.setCameraVerticalFieldOfView(AirVehicleState.get_PayloadStateList()[2].get_VerticalFieldOfView())
        droneObject.setCameraCenterpoint(AirVehicleState.get_PayloadStateList()[2].get_Centerpoint())

    def addNewUAV(self, AirVehicleConfiguration):
        newDroneDict = {}
        newDroneDict['STATE'] = Enum.STATE_ALIVE
        newDroneDict['ACTION'] = Enum.ACTION_WELCOME
        newDroneDict['ACTION_DETAIL'] = {
            'WELCOME' : {
                'start_recovery_id' : 0,
                'recovery_point' : [0,0]
            },
            'SEARCH' : {
                'search_way' : 0,
                'current_index' : 0,
                'total_points' : []
            },
            'TRACKING' : {
                'msg' : 0,
                'tracking_direction' : 0,
                'tracking_zoneID' : 0,
                'last_tracking_time' : 0
            },
            'CHARGING' : {
                'recovery_point' : 0
            }
        }
        newDroneDict['NEXT_HEADING'] = 0
        newDroneDict['NEXT_ALTITUDE'] = 0
        newDroneDict['NEXT_AZIMUTH'] = 0
        newDroneDict['NEXT_ELEVATION'] = 0
        
        newDrone = Drone.Drone()
        newDrone.setID(AirVehicleConfiguration.get_ID())
        newDrone.setMaximumSpeed(AirVehicleConfiguration.get_MaximumSpeed())
        newDrone.setLabel(AirVehicleConfiguration.get_Label())
        newDrone.setMaxAltitudeType(AirVehicleConfiguration.get_MaxAltitudeType())
        newDrone.setMaximumAltitude(AirVehicleConfiguration.get_MaximumAltitude())
        newDrone.setMinAltitudeType(AirVehicleConfiguration.get_MinAltitudeType())
        newDrone.setMinimumAltitude(AirVehicleConfiguration.get_MinimumAltitude())
        newDrone.setMinimumSpeed(AirVehicleConfiguration.get_MinimumSpeed())
        newDrone.setNominalAltitude(AirVehicleConfiguration.get_NominalAltitude())
        newDrone.setNominalAltitudeType(AirVehicleConfiguration.get_NominalAltitudeType())
        newDrone.setNominalSpeed(AirVehicleConfiguration.get_NominalSpeed())

        # Gimbal Configuration
        newDrone.setGimbalPayloadID(AirVehicleConfiguration.get_PayloadConfigurationList()[0].get_PayloadID())
        newDrone.setGimbalMinAzimuth(AirVehicleConfiguration.get_PayloadConfigurationList()[0].get_MinAzimuth())
        newDrone.setGimbalMaxAzimuth(AirVehicleConfiguration.get_PayloadConfigurationList()[0].get_MaxAzimuth())
        newDrone.setGimbalIsAzimuthClamped(AirVehicleConfiguration.get_PayloadConfigurationList()[0].get_IsAzimuthClamped())
        newDrone.setGimbalMinElevation(AirVehicleConfiguration.get_PayloadConfigurationList()[0].get_MinElevation())
        newDrone.setGimbalMaxElevation(AirVehicleConfiguration.get_PayloadConfigurationList()[0].get_MaxElevation())
        newDrone.setGimbalIsElevationClamped(AirVehicleConfiguration.get_PayloadConfigurationList()[0].get_IsElevationClamped())
        newDrone.setGimbalMinRotation(AirVehicleConfiguration.get_PayloadConfigurationList()[0].get_MinRotation())
        newDrone.setGimbalMaxRotation(AirVehicleConfiguration.get_PayloadConfigurationList()[0].get_MaxRotation())
        newDrone.setGimbalIsRotationClamped(AirVehicleConfiguration.get_PayloadConfigurationList()[0].get_IsRotationClamped())
        newDrone.setGimbalMaxAzimuthSlewRate(AirVehicleConfiguration.get_PayloadConfigurationList()[0].get_MaxAzimuthSlewRate())
        newDrone.setGimbalMaxElevationSlewRate(AirVehicleConfiguration.get_PayloadConfigurationList()[0].get_MaxElevationSlewRate())
        newDrone.setGimbalMaxRotationRate(AirVehicleConfiguration.get_PayloadConfigurationList()[0].get_MaxRotationRate())

        # Camera Configuration
        newDrone.setCameraPayloadID(AirVehicleConfiguration.get_PayloadConfigurationList()[1].get_PayloadID())
        newDrone.setCameraSupportedWavelengthBand(AirVehicleConfiguration.get_PayloadConfigurationList()[1].get_SupportedWavelengthBand())
        newDrone.setCameraFieldOfViewMode(AirVehicleConfiguration.get_PayloadConfigurationList()[1].get_FieldOfViewMode())
        newDrone.setCameraMinHorizontalFieldOfView(AirVehicleConfiguration.get_PayloadConfigurationList()[1].get_MinHorizontalFieldOfView())
        newDrone.setCameraMaxHorizontalFieldOfView(AirVehicleConfiguration.get_PayloadConfigurationList()[1].get_MaxHorizontalFieldOfView())
        newDrone.setCameraVideoStreamHorizontalResolution(AirVehicleConfiguration.get_PayloadConfigurationList()[1].get_VideoStreamHorizontalResolution())
        newDrone.setCameraVideoStreamVerticalResolution(AirVehicleConfiguration.get_PayloadConfigurationList()[1].get_VideoStreamVerticalResolution())

        # Hazard sensor Configuration
        newDrone.setHazardPayloadID(AirVehicleConfiguration.get_PayloadConfigurationList()[2].get_PayloadID())
        newDrone.setHazardMaxRange(AirVehicleConfiguration.get_PayloadConfigurationList()[2].get_MaxRange())
        newDrone.setHazardHorizontalFOV(AirVehicleConfiguration.get_PayloadConfigurationList()[2].get_HorizontalFOV())
        newDrone.setHazardVerticalFOV(AirVehicleConfiguration.get_PayloadConfigurationList()[2].get_VerticalFOV())

        newDroneDict['OBJ'] = newDrone

        self.numOfDrone = self.numOfDrone + 1
        self.uavList[AirVehicleConfiguration.get_ID()] = newDroneDict
        print("UAV ", AirVehicleConfiguration.get_ID(), " is added to uavList")

    def assignInitialSearchPath(self, aKeepInZones, aRecoveryPoints, iStartWay):
        print(" - Set closestrecoverypoint to each uav")
        self.setClosestRecoveryPoint(aRecoveryPoints)
        print(" - Done")
        # cal_SearchPath
        print(" - Calc initialsearch path")
        self.initialSearch = InitialSearch.InitialSearch(self.utils, self.numOfDrone, aKeepInZones, aRecoveryPoints, iStartWay)
        print(" - Done")
        # assign each drones
        
    def updateUavAction(self, tcpClient, uavId):
        action = self.uavList[uavId].ACTION

        # state check
        if action == Enum.ACTION_WELCOME:
            # At first time, all drones will be ACTION_WELCOME, and all drones will start initial searching 
            pass
        elif action == Enum.ACTION_SEARCHING:
            # search for fire-zone and entity
            # update action_detail.
            initialSearch.updateSearchingPoint(self.uavList[uavId])
        elif action == Enum.ACTION_TRACKING:
            # tracking a fire-zone
            # still is tracking
            tracking.updateTrackingState(self.uavList[uavId])
        elif action == Enum.ACTION_CHARGING:
            # going to recovery-zone
            # still is charging
            # charging
            pass

    def getUpdateInfos(self, uavId):
        self.getElevAndAlti(uavId)
        return self.uavList[uavId].NEXT_HEADING, self.uavList[uavId].NEXT_AZIMUTH, self.uavList[uavId].NEXT_ELEVATION, self.uavList[uavId].NEXT_ALTITUDE

    def getElevAndAlti(self, uavId):
        # terrain
        uavInfos = self.uavList[uavId]
        uav_lat = uavInfos.OBJ.getLatitude()
        uav_lon = uavInfos.OBJ.getLongitude()
        originalDirection = self.tracking.getOriginalDirection(uavInfos)
        paddingAlt = uavInfos.OBJ.getHazardMaxRange()*0.5
        idealDist = uavInfos.OBJ.getHazardMaxRange()*0.7
        action = uavInfos.ACTION

        front_Alt = 0
        uavInfos.NEXT_ALTITUDE = self.utils.getElevation(uav_lat, uav_lon) + paddingAlt


        if action == Enum.ACTION_SEARCHING: #searching
            front_Alt = uavInfos.getCameraCenterpoint().get_Altitude()
        else :  # ETC.
            coord = self.utils.getLatLon(uav_lat, uav_lon, idealDist, originalDirection)
            front_Alt = self.utils.getElevation(coord[0], coord[1])

        # Q1. how does the gimbal angle adjust in front altitude higher than now.
        alt_gap = uavInfos.OBJ.getAltitude()- front_Alt

        if alt_gap <= idealDist :
            uavInfos.NEXT_ELEVATION = self.utils.getTheta(idealDist,uavInfos.OBJ.getHazardMaxRange())
            # print("optmial theta is ", self.__theta)

        elif alt_gap < uavInfos.OBJ.getHazardMaxRange() :
            uavInfos.NEXT_ELEVATION = self.utils.getTheta(alt_gap, uavInfos.OBJ.getHazardMaxRange())

    def moveFirezoneByWind(self, uavId):
        if self.uavList.keys()[-1] == uavId :
            # zone move
            wind_speed = self.uavList[uavId].getWindSpeed()
            wind_direction = self.uavList[uavId].getWindDirection()
            mDist = wind_speed*0.5

            # moving
            for zone in self.detectedZones.getDetectedZones().values():
                for i in zone:
                    coord = self.utils.getLatLon(i.get_Latitude(), i.get_Longitude(), mDist, 180 + wind_direction )
                    # [lat, lon]
                    i.set_Latitude(coord[0])
                    i.set_Longitude(coord[1])

    def hazardzoneDetected(self, hazardzoneDetected, detectedTime):
        if hazardzoneDetected.get_DetectedHazardZoneType() == 1 :
            # fire zone
            detectedPoint = hazardzoneDetected.get_DetectedLocation()
            entityId = hazardzoneDetected.get_DetectingEnitiyID()
            uavInfos = self.uavList[entityId]
            
            if detectedTime == 0:
                detectedTime = uavInfos.OBJ.getTime()
            
            if uavInfos.ACTION == Enum.ACTION_TRACKING :
                # already tracking
                zoneId = uavInfos.ACTION_DETAIL.tracking_zoneID

                if not self.putPointIntoZones(zoneId, detectedPoint):
                    return

                self.tracking.gooutFromZone(uavInfos)
                uavInfos.ACTION_DETAIL.last_tracking_time = detectedTime
            
            elif uavInfos.ACTION == Enum.ACTION_SEARCHING :
                zoneId = self.detectedZones.isAlreadyDetectedZone()
                if zoneId != -1 :
                    # already detected
                    if not self.putPointIntoZones(zoneId, detectedPoint):
                        return

                    #how ?? 

                else :
                    # new hazardzone
                    # tracking
                    uavInfos.ACTION = Enum.ACTION_TRACKING
                    uavInfos.ACTION_DETAIL.tracking_zoneID = self.detectedZones.addNewDetectedZone(detectedPoint)
                    uavInfos.ACTION_DETAIL.tracking_direction = -1 if uavInfos.OBJ.getAzimuth() > 0 else 1

                    self.tracking.gooutFromZone(uavInfos)
                    # call friends
                    
                    uavInfos.ACTION_DETAIL.last_tracking_time = detectedTime


        else :
            # smoke zone
            pass

    def putPointIntoZones(self, zoneId, detectedPoint):
        zonePoints = self.detectedZones.getDetectedZoneById(zoneId)
        if not zonePoints :
            print("The zoneId is wrong")
            return False
        zonePoints.append(detectedPoint)
        return True
    
    def estimateDetectedZone(self):
        self.detectedZones.sendEstimateCmd()

    def setClosestRecoveryPoint(self, aRecoveryPoints):
        dist = -1

        for uavInfos in self.uavList.values():
            pointId = -1

            for i in range(len(aRecoveryPoints)):
                point = aRecoveryPoints[i]
                
                new_dist = self.utils.distance(point[1], point[0], uavInfos.OBJ.getLongitude(), uavInfos.OBJ.getLatitude())

                if new_dist < dist:
                    dist = new_dist
                    pointId = i

            uavInfos.ACTION_DETAIL.start_recovery_id = pointId
            uavInfos.ACTION_DETAIL.recovery_point = aRecoveryPoints[pointId]

    def setNewDroneAction(self, uavId):
        # made right before after initial searching 
        deadList = [k for k,v in self.uavList.items() if v.STATE == Enum.STATE_DEAD ]

        if deadList.count() == 0:
            # initial searching
            self.setBestPlaceToSearch(uavId)
        else :
            # take over dead drone's action.
            # 1. find nicest actino by time to start new ACTION
            dist = float('inf')
            for i in deadList:
                # tracking or searching
                if self.getDist(uavId, i) < dist :
                    self.uavList[uavId].ACTION = self.uavList[i].ACTION
                    self.uavList[uavId].ACTION_DETAIL = self.uavList[i].ACTION_DETAIL

        self.startSearching(uavId)

    def setBestPlaceToSearch(self, uavId):
        pass

    def startSearching(self, uavId):
        pass

    def getDist(self, uavId1, uavId2):

        lat1 = self.uavList[uavId1].OBJ.get_Latitude()
        lon1 = self.uavList[uavId1].OBJ.get_Longitude()
        
        lat2 = self.uavList[uavId2].OBJ.get_Latitude()
        lon2 = self.uavList[uavId2].OBJ.get_Longitude()

        return self.utils.distance(lon1, lat1, lon2, lat2)

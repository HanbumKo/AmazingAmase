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
import Searching
import Tracking
import Undertaked
import Charging
import Turning
import DetectedZone

import sys
import math

"""
State class manage all of drones efficiently. This must be implemented easy to see, easy to use.
The drone's ID could be different from come order. Dictionary can find drone object fast using each ID.
"""
class State():
    def __init__(self, utils):
        self.aliveUavList = {} # Dictionary
        self.deadUavList = {}
        self.numOfDrone = 0
        self.utils = utils
        self.searching = None
        self.tracking = Tracking.Tracking(utils)
        self.undertaked = Undertaked.Undertaked(utils)
        self.charging = Charging.Charging(utils)
        self.turning = Turning.Turning(utils)
        self.detectedZones = DetectedZone.DetectedZone(utils)
        
        # recoverypoints
        self.aRecoveryPoints = None

    def updateUAV(self, AirVehicleState):
        ID = AirVehicleState.get_ID()
        droneObject = {}
        if ID not in self.aliveUavList:
            print("Error : UAV ", ID, " is not in aliveUavList")
            exit()
        else:
            droneObject = self.aliveUavList[ID]['OBJ']
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
        newDroneDict['ACTION'] = Enum.INITIAL_STATE
        newDroneDict['ACTION_DETAIL'] = {
            'WELCOME' : {
                'start_recovery_id' : 0,
                'recovery_point' : [0,0]
            },
            'SEARCH' : {
                'is_scanning' : False,
                'search_way' : 0,
                'current_index' : 0,
                'total_points' : []
            },
            'TRACKING' : {
                'msg' : 0,
                'tracking_direction' : 0,
                'tracking_zoneID' : -1,
                'last_tracking_time' : 0
            },
            'UNDERTAKED':{
                'tracking_zoneID' : 0,
                'dest_position' : [0,0]
            },
            'CHARGING' : {
                'previous_action' : Enum.INITIAL_STATE
            },
            'ENTITY' : {

            },
            'TURNING' : {
                'heading' : 0,
                'previous_action' : Enum.INITIAL_STATE
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
        self.aliveUavList[AirVehicleConfiguration.get_ID()] = newDroneDict

        print("UAV ", AirVehicleConfiguration.get_ID(), " is added to aliveUavList")

    def assignInitialSearchPath(self, aKeepInZones, aRecoveryPoints, iStartWay):
        self.aRecoveryPoints = aRecoveryPoints
        print(" - Set closestrecoverypoint to each uav")
        for iUavId in self.aliveUavList.keys():
            self.setClosestRecoveryPoint(iUavId) 
        print(" - Done")
        # cal_SearchPath
        print(" - Calc initialsearch path")
        self.searching = Searching.Searching(self.utils, self.numOfDrone, aKeepInZones, aRecoveryPoints, iStartWay)
        print(" - Done")
        # assign each drones
        # Get coordinate matrix to assign each drone to destination points
        print(" - Assign initialsearch path to each drone")
        waypointlists = self.searching.getWayPointLists()
        
        for i in range(len(waypointlists)):
            start_idx = 0
            for uavId,uavInfos in self.aliveUavList.items():
                if uavInfos['ACTION_DETAIL']['WELCOME']['start_recovery_id'] == i :
                    uavInfos['ACTION'] = Enum.SEARCHING
                    uavInfos['ACTION_DETAIL']['SEARCH']['current_index'] = start_idx
                    uavInfos['ACTION_DETAIL']['SEARCH']['total_points'] = waypointlists[i]
                    start_idx = (start_idx+1)%len(waypointlists[i])

                    self.utils.go_way_points(uavId,start_idx,uavInfos['ACTION_DETAIL']['SEARCH']['total_points'])
                    self.searching.updateNextHeading(uavInfos)
        print(" - Done")
    
    def checkStillInKeep(self, aKeepInZones, uavId):
        uav_lat = self.aliveUavList[uavId]['OBJ'].getLatitude()
        uav_lon = self.aliveUavList[uavId]['OBJ'].getLongitude()

        top = aKeepInZones[0][0]; bottom = aKeepInZones[2][0]
        left = aKeepInZones[0][1]; right = aKeepInZones[2][1]

        if  uav_lat >= top or uav_lat <= bottom or uav_lon >= right or uav_lon <= left :
            print("UAV -",uavId," needs to turn.")
            self.turning.turning([uav_lat,uav_lon], self.aliveUavList[uavId])
    
    def checkNeedToCharge(self, uavId):
        uavInfos = self.aliveUavList[uavId]

        if uavInfos['ACTION'] != Enum.TRACKING and uavInfos['ACTION'] != Enum.CHARGING:
            fRemainFuel = uavInfos['OBJ'].getEnergyAvailable()

            if fRemainFuel < 50 : 
                uavInfos['ACTION_DETAIL']['CHARGING']['previous_action'] = uavInfos['ACTION']
                uavInfos['ACTION_DETAIL']['SEARCH']['is_scanning'] = True
                uavInfos['ACTION'] = Enum.CHARGING
                print("UAV -", uavId," needs to charging")

    def updateUavAction(self, uavId):
        state = self.aliveUavList[uavId]['ACTION']

        # state check
        if state == Enum.INITIAL_STATE:
            # At first time, all drones will be INITIAL_STATE, and all drones will start initial searching 
            print("UAV -", uavId,"WELCOME")
            self.setNextState(uavId)
        elif state == Enum.SEARCHING:
            # search for fire-zone and entity
            # update action_detail.
            self.searching.updateSearchingState(self.aliveUavList[uavId])
        elif state == Enum.TRACKING:
            # tracking a fire-zone
            # still is tracking
            self.tracking.updateTrackingState(self.aliveUavList[uavId])
        elif state == Enum.UNDERTAKED:
            self.undertaked.updateUndertakedState(self.aliveUavList[uavId])
            # print("UAV -",uavId,"is going to zone")
        elif state == Enum.CHARGING:
            # going to recovery-zone
            # still is charging
            # charging
            self.charging.updateChargingState(self.aliveUavList[uavId])

        elif state == Enum.TURNING :
            self.turning.updateTurningState(self.aliveUavList[uavId])

    ### For initial_state
    def setNextState(self, uavInfos):
        uavId = uavInfos['OBJ'].getID()
        iDeadUavNum = len(self.deadUavList)

        if iDeadUavNum == 0 :
            # Do Searching immdiately
            self.setClosestRecoveryPoint(uavId)
            ## need to re-coding, if coordination is done.
            print(" - Assign initialsearch path to each drone")
            waypointlists = self.searching.getWayPointLists()

            for i in range(len(waypointlists)):
                start_idx = 0
                if uavInfos['ACTION_DETAIL']['WELCOME']['start_recovery_id'] == i :
                    uavInfos['ACTION'] = Enum.SEARCHING
                    uavInfos['ACTION_DETAIL']['SEARCH']['current_index'] = start_idx
                    uavInfos['ACTION_DETAIL']['SEARCH']['total_points'] = waypointlists[i]
                    start_idx = (start_idx+1)%len(waypointlists[i])

                    self.utils.go_way_points(uavId,start_idx,uavInfos['ACTION_DETAIL']['SEARCH']['total_points'])
                    self.searching.updateNextHeading(uavInfos)
            print(" - Done")
        else :
            # Be Undertaked from dead uav
            aDists = [(iUavId, self.utils.distance(uavInfos['OBJ'].getLongitude(), uavInfos['OBJ'].getLatitude(),
                                            oDeadUavInfos['OBJ'].getLongitude(), oDeadUavInfos['OBJ'].getLatitude())) 
                                            for iUavId, oDeadUavInfos in self.deadUavList.items()]

            iNearestId = min(aDists, key = lambda i : i[1])[0] 
            self.aliveUavList[uavId]['ACTION'] = self.deadUavList[iNearestId]['ACTION']
            self.aliveUavList[uavId]['ACTION_DETAIL'] = self.deadUavList[iNearestId]['ACTION_DETAIL']
            
            state = self.aliveUavList[uavId]['ACTION']
            
            if state & Enum.CHARGING :
                self.aliveUavList[uavId]['ACTION'] = self.aliveUavList[uavId]['ACTION_DETAIL']['CHARGING']['previous_action']

            if state & Enum.TRACKING :
                self.aliveUavList[uavId]['ACTION'] = Enum.UNDERTAKED
                self.aliveUavList[uavId]['ACTION_DETAIL']['UNDERTAKED']['tracking_zoneID'] = \
                    self.aliveUavList[uavId]['ACTION_DETAIL']['TRACKING']['tracking_zoneID']
            
    def getUpdateInfos(self, uavId):
        self.getElevAndAlti(uavId)
        return self.aliveUavList[uavId]['NEXT_HEADING'], self.aliveUavList[uavId]['NEXT_AZIMUTH'], self.aliveUavList[uavId]['NEXT_ELEVATION'], self.aliveUavList[uavId]['NEXT_ALTITUDE']

    def getElevAndAlti(self, uavId):
        # terrain
        uavInfos = self.aliveUavList[uavId]
        uav_lat = uavInfos['OBJ'].getLatitude()
        uav_lon = uavInfos['OBJ'].getLongitude()
        originalDirection = self.tracking.getOriginalDirection(uavInfos)
        paddingAlt = uavInfos['OBJ'].getHazardMaxRange()*0.5
        idealDist = uavInfos['OBJ'].getHazardMaxRange()*0.7
        action = uavInfos['ACTION']

        front_Alt = 0
        uavInfos['NEXT_ALTITUDE'] = self.utils.getElevation(uav_lat, uav_lon) + paddingAlt


        if action == Enum.TRACKING : #searching, charging, going, entity_searching
            coord = self.utils.getLatLon(uav_lat, uav_lon, idealDist, originalDirection)
            front_Alt = self.utils.getElevation(coord[0], coord[1])
        else :  # ETC.
            front_Alt = uavInfos['OBJ'].getCameraCenterpoint().get_Altitude()

        # Q1. how does the gimbal angle adjust in front altitude higher than now.
        alt_gap = uavInfos['OBJ'].getAltitude()- front_Alt
        
        if alt_gap <= idealDist :
            uavInfos['NEXT_ELEVATION'] = self.utils.getTheta(idealDist,uavInfos['OBJ'].getHazardMaxRange())

        elif alt_gap < uavInfos['OBJ'].getHazardMaxRange() :
            uavInfos['NEXT_ELEVATION'] = self.utils.getTheta(alt_gap, uavInfos['OBJ'].getHazardMaxRange())
        else :
            uavInfos['NEXT_ELEVATION'] = -70

    def moveFirezoneByWind(self, uavId):
        if list(self.aliveUavList.keys())[-1] == uavId :
            # zone move
            wind_speed = self.aliveUavList[uavId]['OBJ'].getWindSpeed()
            wind_direction = self.aliveUavList[uavId]['OBJ'].getWindDirection()
            mDist = wind_speed*0.5

            # moving
            for zone in self.detectedZones.getDetectedZones().values():
                for i in zone:
                    coord = self.utils.getLatLon(i.get_Latitude(), i.get_Longitude(), mDist, 180 + wind_direction )
                    # [lat, lon]
                    i.set_Latitude(coord[0])
                    i.set_Longitude(coord[1])

    def isScanning(self, uavId): return self.aliveUavList[uavId]['ACTION_DETAIL']['SEARCH']['is_scanning']
    
    def setScanning(self, uavId, state): self.aliveUavList[uavId]['ACTION_DETAIL']['SEARCH']['is_scanning'] = state

    def hazardzoneDetected(self, hazardzoneDetected, detectedTime):
        if hazardzoneDetected.get_DetectedHazardZoneType() == 1 :
            # fire zone
            detectedPoint = hazardzoneDetected.get_DetectedLocation()
            entityId = hazardzoneDetected.get_DetectingEnitiyID()
            uavInfos = self.aliveUavList[entityId]
            
            if detectedTime == 0:
                detectedTime = uavInfos['OBJ'].getTime()
            
            if uavInfos['ACTION'] == Enum.TRACKING :
                # already tracking
                zoneId = uavInfos['ACTION_DETAIL']['TRACKING']['tracking_zoneID']
                zoneIdForCheck = self.detectedZones.isAlreadyDetectedZone(detectedPoint)
                if zoneId == zoneIdForCheck:
                    if not self.putPointIntoZones(zoneId, detectedPoint):
                        return

                    self.tracking.gooutFromZone(uavInfos)
                    uavInfos['ACTION_DETAIL']['TRACKING']['last_tracking_time'] = detectedTime
                else :
                    print("Found other zone")

            elif uavInfos['ACTION'] & (Enum.SEARCHING|Enum.UNDERTAKED) :
                zoneId = self.detectedZones.isAlreadyDetectedZone(detectedPoint)
                if zoneId != -1 :
                    # already detected
                    ## need to analyse

                    if not self.putPointIntoZones(zoneId, detectedPoint):
                        return

                    
                    if self.numOfDroneInSameZone(zoneId) < 2 :
                        # partner
                        # tracking with different direction
                        uavInfos['ACTION_DETAIL']['SEARCH']['is_scanning'] = False
                        uavInfos['ACTION'] = Enum.TRACKING
                        uavInfos['ACTION_DETAIL']['TRACKING']['tracking_zoneID'] = zoneId
                        # uavInfos['ACTION_DETAIL']['TRACKING']['tracking_direction'] = -1 if uavInfos['OBJ'].getCameraAzimuth() > 0 else 1
                        self.tracking.gooutFromZone(uavInfos)
                        uavInfos['ACTION_DETAIL']['TRACKING']['last_tracking_time'] = detectedTime
                    else : 
                        # entity search
                        # 반대 방향으로 나가 버리기
                        self.turning.turning([detectedPoint.get_Latitude(), detectedPoint.get_Longitude()], uavInfos)                    
                else :
                    # new hazardzone
                    # tracking
                    print(" - Found New Fire Zone")
                    uavInfos['ACTION_DETAIL']['SEARCH']['is_scanning'] = False
                    uavInfos['ACTION'] = Enum.TRACKING
                    uavInfos['ACTION_DETAIL']['TRACKING']['tracking_zoneID'] = self.detectedZones.addNewDetectedZone(detectedPoint)
                    uavInfos['ACTION_DETAIL']['TRACKING']['tracking_direction'] = -1 if uavInfos['OBJ'].getCameraAzimuth() > 0 else 1
                    self.tracking.gooutFromZone(uavInfos)
                    uavInfos['ACTION_DETAIL']['TRACKING']['last_tracking_time'] = detectedTime
                    # call friends
                    self.callMyFriend(entityId, detectedPoint)
                    
        else :
            pass

        return hazardzoneDetected.get_DetectedHazardZoneType()
    
    def callMyFriend(self, uavId, detectedPoint):
        # fastest
        uavInfos = self.aliveUavList[uavId]
        frinedInfos = self.findNearestFriend(uavId, detectedPoint)

        frinedInfos['ACTION']= Enum.UNDERTAKED
        frinedInfos['ACTION_DETAIL']['UNDERTAKED']['tracking_zoneID'] = uavInfos['ACTION_DETAIL']['TRACKING']['tracking_zoneID']
        frinedInfos['ACTION_DETAIL']['UNDERTAKED']['dest_position'] = [detectedPoint.get_Latitude(), detectedPoint.get_Longitude()]
        frinedInfos['ACTION_DETAIL']['TRACKING']['tracking_direction'] = uavInfos['ACTION_DETAIL']['TRACKING']['tracking_direction']*(-1)
        
        print(" - Called Friend -",frinedInfos['OBJ'].getID(), "ACTION :", frinedInfos['ACTION'])

    def findNearestFriend(self, uavId, oDetectedPoint):
        # 시간으로 가장 빨리올수 있는 친구 부르고
        # 그 친구는 UNDERTAKED 상태로 바꾸고
        # tracking_zoneID를 같은 ID 로        
        aDists = [(iUavId, self.utils.distance(oDetectedPoint.get_Longitude(), oDetectedPoint.get_Latitude(),
                                            oUavInfos['OBJ'].getLongitude(), oUavInfos['OBJ'].getLatitude())/oUavInfos['OBJ'].getAirspeed()) 
                                            for iUavId, oUavInfos in self.aliveUavList.items() if iUavId != uavId and oUavInfos['STATE'] == Enum.STATE_ALIVE]

        iNearestId = min(aDists, key = lambda i : i[1])[0] 
        return self.aliveUavList[iNearestId]

    def putPointIntoZones(self, zoneId, detectedPoint):
        zonePoints = self.detectedZones.getDetectedZoneById(zoneId)
        if not zonePoints :
            print("The zoneId is wrong")
            return False
        zonePoints.append(detectedPoint)
        return True
    
    def numOfDroneInSameZone(self, zoneId):
        aUavZoneList = [ 1 for uavInfos in self.aliveUavList.values() if uavInfos['STATE'] == Enum.STATE_ALIVE and 
                        uavInfos['ACTION'] & (Enum.TRACKING) and
                        uavInfos['ACTION_DETAIL']['TRACKING']['tracking_zoneID'] == zoneId]

        return sum(aUavZoneList)

    def estimateDetectedZone(self):
        self.detectedZones.sendEstimateCmd()

    def setClosestRecoveryPoint(self, iUavId):
        uavInfos = self.aliveUavList[iUavId]
        pointId = -1
        dist = sys.maxsize

        for i in range(len(self.aRecoveryPoints)):
            point = self.aRecoveryPoints[i]
            
            new_dist = self.utils.distance(point[1], point[0], uavInfos['OBJ'].getLongitude(), uavInfos['OBJ'].getLatitude())

            if new_dist < dist:
                dist = new_dist
                pointId = i

        uavInfos['ACTION_DETAIL']['WELCOME']['start_recovery_id']= pointId
        uavInfos['ACTION_DETAIL']['WELCOME']['recovery_point'] = self.aRecoveryPoints[pointId]

    def removedUavUpdate(self, uavId):
        self.aliveUavList[uavId]['STATE'] = Enum.STATE_DEAD

    def getDist(self, uavId1, uavId2):

        lat1 = self.aliveUavList[uavId1]['OBJ'].getLatitude()
        lon1 = self.aliveUavList[uavId1]['OBJ'].getLongitude()
        
        lat2 = self.aliveUavList[uavId2]['OBJ'].getLatitude()
        lon2 = self.aliveUavList[uavId2]['OBJ'].getLongitude()

        return self.utils.distance(lon1, lat1, lon2, lat2)

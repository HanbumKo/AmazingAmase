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
from afrl.cmasi.Polygon import Polygon

import Drone
import Enum
import Utils
import Searching
import Tracking
import Undertaked
import Charging
import Turning
import DetectedZone


from scipy.spatial import ConvexHull
import sys
import math
import numpy as np
"""
State class manage all of drones efficiently. This must be implemented easy to see, easy to use.
The drone's ID could be different from come order. Dictionary can find drone object fast using each ID.
"""
class State():
    def __init__(self, utils):
        self.aliveUavList = {} # Dictionary
        self.deadUavList = {}
        self.stateForCoordination = {
            'Searching' : {},
            'Tracking': {}
        }
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

    def checkTotalState(self, uavId):
        if max(list(self.aliveUavList.keys())) == uavId :
            print(" - Check tracking state")
            self.checkTrackingState()
            print(" - Done")
            print(" - Check searching state")
            self.checkSearchingState()
            print(" - Done")
    
    def checkTrackingState(self):
        iNumOfHZ = len(self.stateForCoordination['Tracking'])

        if iNumOfHZ == 0:
            return
        
        for index in range(iNumOfHZ):
            hazardZone = self.stateForCoordination['Tracking']['HazardZone_'+str(index)]
            iNumOfDrones = len(hazardZone['tracking_drones'])
            if hazardZone['cycled'] : # 한바퀴 탐색이 끝났는 가
                for _ in range(iNumOfDrones-1):
                    uavId = hazardZone.pop()
                    self.aliveUavList[uavId]['STATE'] = Enum.INITIAL_STATE
            elif iNumOfDrones < 2 : # 최적의 개수 보다 적은 경우
                self.callFriends(hazardZone, index)

    def checkSearchingState(self):
        # 존 별로 하거나 도움이 가능한 드론 별로 하거나
        aFreeDrones = [iUavId for iUavId, oUavInfos in self.aliveUavList.items() if oUavInfos['STATE'] == Enum.INITIAL_STATE]
        threshold = 70
        # 도움을 줄 수 있는 드론 
        # 1. 자기 리커버리 영역 부터 확인
        aFreeDronesForAll = []
        # 리커버리 영역 별로 할당
        print(len(self.stateForCoordination['Searching']))
        for iRecoveryId in range(len(self.stateForCoordination['Searching'])):
            aFreeDrones = [iUavId for iUavId, oUavInfos in self.aliveUavList.items() 
                            if oUavInfos['STATE'] == Enum.INITIAL_STATE and oUavInfos['STATE_DETAIL'][Enum.INITIAL_STATE]['start_recovery_id'] == iRecoveryId ]
            print(iRecoveryId)
            if len(aFreeDrones) != 0 :
                dRecoveryArea = self.stateForCoordination['Searching']['RecoveryArea_'+str(iRecoveryId)]
                
                # 섹션 단위로 계산
                aProgs = np.array([len(dSection['searched'])/dSection['numberOfPoints']*100 for dSection in dRecoveryArea.values()])
                # 도움이 필요한 섹션
                aProgIsZero = np.where(aProgs[aProgs == 0])[0]
                aProgIsLow = np.where(aProgs[(aProgs > 0) * (aProgs < threshold)])[0]

                # 아무도 탐지하지 않은 곳 부터 처리
                print(" - Deal with Zero")
                aFreeDrones = self.dealwithZero(dRecoveryArea ,aProgIsZero, aFreeDrones)
                print(" - Done")
                # 남은 드론 수가 없다면 그냥 종료
                if len(aFreeDrones) == 0 :
                    continue

                print(" - Deal with Low")
                aFreeDrones = self.dealwithLow(dRecoveryArea, aProgIsLow, aFreeDrones)
                print(" - Done")
                # 남은 드론 수가 없다면 그냥 종료
                if len(aFreeDrones) != 0 :
                    print(" - ! Drones left")
                    print(aFreeDrones)
                    aFreeDronesForAll+=aFreeDrones

        if len(aFreeDronesForAll) != 0 :
            # 리커버리 존에 맞춰서 나누고도 남은 드론들
            # 남은 드론에 대해서 어떻게 처리하지?
            # 일단 모두 충전해서 대기
            for iUavId in aFreeDronesForAll:
                self.aliveUavList[iUavId]['STATE'] = Enum.INITIAL_STATE

    def callFriends(self, dHazardZone, index):
        ### 내가 들어온 방향과 반대로 들어올    수있는 드론, 내가 온 방향이 아닌 쪽으로 오는 드론, 인근 section에서
        aPoints = dHazardZone['area']
        
        if len(aPoints) < 3:
            return
        
        # all_coords = [[coord.get_Latitude(), coord.get_Longitude()] for coord in aPoints]
        # hull = ConvexHull(np.asarray(all_coords, dtype=np.float32))

        # dHazardZone['area'] = [aPoints[i] for i in hull.vertices]

        # aPoints = dHazardZone['area']
        # aDists = []        
        oPoint = dHazardZone['startPoint']
        # oCenterPoint = [0,0]

        # for point in aPoints:
        #     oCenterPoint[0] += point.get_Latitude()/len(aPoints)
        #     oCenterPoint[1] += point.get_Longitude()/len(aPoints)

        # # 헤딩으로 알맞은 친구부르기
        # fStartHeading = self.utils.getHeadingToDest(oPoint.get_Latitude(), oPoint.get_Longitude(), oCenterPoint[0], oCenterPoint[1])
        # # 올 수 있는 드론들
        aFreeDrones = [(iUavId, self.utils.distance(oPoint.get_Latitude(), oPoint.get_Longitude(), oUavInfos['OBJ'].getLatitude(), oUavInfos['OBJ'].getLongitude())) for iUavId, oUavInfos in self.aliveUavList.items()
                        if oUavInfos['STATE'] & (Enum.INITIAL_STATE|Enum.SEARCHING)]
        iNearestId = min(aFreeDrones, key=lambda x: x[1])[0]
        
        #
        self.returnState(iNearestId)
        self.aliveUavList[iNearestId]['STATE'] = Enum.UNDERTAKED
        self.aliveUavList[iNearestId]['STATE_DETAIL'][Enum.UNDERTAKED]['tracking_zoneID'] = index
        self.aliveUavList[iNearestId]['STATE_DETAIL'][Enum.UNDERTAKED]['dest_poisition'] = [oPoint.get_Latitude(), oPoint.get_Longitude()]
        
        dHazardZone['tracking_drones'].append(iNearestId)

    def dealwithZero(self, dRecoveryArea ,aProgIsZero, aFreeDrones):
        # 아무도 탐사하지 않은 지역에 대해서 드론 할당
        aShortestDrone = []

        for iSectionId in aProgIsZero:
            dSection = dRecoveryArea['Section_'+str(iSectionId)]
            aShortestDrone = [sys.maxsize,-1]

            for iUavId in aFreeDrones:
                aPoint = dSection['waiting'][0]

                fUav_lat = self.aliveUavList[iUavId]['OBJ'].getLatitude()
                fUav_lon = self.aliveUavList[iUavId]['OBJ'].getLongitude()
                
                fDist = self.utils.distance(aPoint[0], aPoint[1], fUav_lat, fUav_lon)
                
                if fDist < aShortestDrone[0] :
                    aShortestDrone[0] = fDist; aShortestDrone[1] = iUavId
            
            dSection['searchingUavs'].append(aShortestDrone[1])
            dSection['waiting'] = { aShortestDrone[1] : dSection['waiting']}
            aFreeDrones.remove(aShortestDrone[1])
            ## Need to change drone state!
            self.aliveUavList[aShortestDrone[1]]['STATE'] = Enum.SEARCHING
            # RecoveryId is same as self.aliveUavList - Enum.INITIAL_STATE
            self.aliveUavList[aShortestDrone[1]]['STATE_DETAIL'][Enum.SEARCHING]['sectionId'] = iSectionId

            if len(aFreeDrones) == 0 :
                break
        
        return aFreeDrones

    def dealwithLow(self, dRecoveryArea ,aProgIsLow, aFreeDrones):
        # 커버된 영역이 적은 걸 먼저 처리
        # 고로 정렬 필요
        aProgIsLow = [(iSectionId, len(dRecoveryArea['Section_'+str(iSectionId)]['searched'])/dRecoveryArea['Section_'+str(iSectionId)]['numberOfPoints']*100) for iSectionId in aProgIsLow]
        aProgIsLow.sort(key=lambda x: x[1])
        aProgIsLow = list(map(lambda x: x[0], aProgIsLow))

        aShortestDrone = []

        for iSectionId in aProgIsLow:
            dSection = dRecoveryArea['Section_'+str(iSectionId)]
            aShortestDrone = [sys.maxsize,-1]
            waitingForFreeDrone = []
            ### 기존에 있는 거중 남은 좌표의 개수가 가장 많은 거에서 나눠준다.
            if len(dSection['searchingUavs']) == 0 :
                waitingForFreeDrone = dSection['waiting']
            else :
                aRemainPoints = [(iUavId, aPoints) for iUavId, aPoints in dSection['waiting'].items()]
                tNeedToHelp = max(aRemainPoints,key=lambda x: len(x[1]))
                
                dSection['waiting'][tNeedToHelp[0]], waitingForFreeDrone \
                    = tNeedToHelp[1][:int(len(tNeedToHelp[1]/2))], tNeedToHelp[1][int(len(tNeedToHelp[1]/2)):],

            for iUavId in aFreeDrones:

                aPoint = waitingForFreeDrone[0]

                fUav_lat = self.aliveUavList[iUavId]['OBJ'].getLatitude()
                fUav_lon = self.aliveUavList[iUavId]['OBJ'].getLongitude()
                
                fDist = self.utils.distance(aPoint[0], aPoint[1], fUav_lat, fUav_lon)
                
                if fDist < aShortestDrone[0] :
                    aShortestDrone[0] = fDist; aShortestDrone[1] = iUavId
            
            dSection['searchingUavs'].append(aShortestDrone[1])
            dSection['waiting'][aShortestDrone[1]] = waitingForFreeDrone
            aFreeDrones.remove(aShortestDrone[1])
            ## Need to change drone state!
            self.aliveUavList[aShortestDrone[1]]['STATE'] = Enum.SEARCHING
            # RecoveryId is same as self.aliveUavList - Enum.INITIAL_STATE
            self.aliveUavList[aShortestDrone[1]]['STATE_DETAIL'][Enum.SEARCHING]['sectionId'] = iSectionId

            if len(aFreeDrones) == 0 :
                break
        
        return aFreeDrones

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
        newDroneDict['STATE'] = Enum.INITIAL_STATE
        newDroneDict['STATE_DETAIL'] = {
            Enum.INITIAL_STATE : {
                'start_recovery_id' : 0,
                'recovery_point' : [0,0]
            },
            Enum.SEARCHING : {
                'is_scanning' : False,
                'sectionId' : -1
            },
            Enum.TRACKING : {
                'msg' : 0,
                'tracking_direction' : 1,
                'tracking_zoneID' : -1,
                'last_tracking_time' : 0
            },
            Enum.UNDERTAKED:{
                'tracking_zoneID' : 0,
                'dest_position' : [0,0]
            },
            Enum.CHARGING : {
            },
            Enum.TURNING : {
                'heading' : 0,
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
        self.searching.setTrackingSection(self.stateForCoordination['Searching'])
        print(" - Done")
    
    def checkStillInKeep(self, aKeepInZones, uavId):
        uav_lat = self.aliveUavList[uavId]['OBJ'].getLatitude()
        uav_lon = self.aliveUavList[uavId]['OBJ'].getLongitude()

        top = aKeepInZones[0][0]; bottom = aKeepInZones[2][0]
        left = aKeepInZones[0][1]; right = aKeepInZones[2][1]

        if  uav_lat >= top or uav_lat <= bottom or uav_lon >= right or uav_lon <= left :
            print("UAV -",uavId," needs to turn.")
            self.returnState(uavId)
            self.turning.turning([uav_lat,uav_lon], self.aliveUavList[uavId])
    
    def checkNeedToCharge(self, uavId):
        uavInfos = self.aliveUavList[uavId]

        if uavInfos['STATE'] != Enum.CHARGING:
            fRemainFuel = uavInfos['OBJ'].getEnergyAvailable()

            if fRemainFuel < 50 : 
                # coord update
                self.returnState(uavId)
                
                # uavInfos['STATE_DETAIL'][Enum.CHARGING]['previous_action'] = uavInfos['STATE']
                uavInfos['STATE_DETAIL'][Enum.SEARCHING]['is_scanning'] = True
                uavInfos['STATE'] = Enum.CHARGING
                print("UAV -", uavId," needs to charging")

    def updateUavAction(self, uavId):
        state = self.aliveUavList[uavId]['STATE']

        # state check
        if state == Enum.INITIAL_STATE:
            # At first time, all drones will be INITIAL_STATE, and all drones will start initial searching
            pass
        elif state == Enum.SEARCHING:
            # search for fire-zone and entity
            # update action_detail.
            recoveryZone = self.stateForCoordination['Searching']['RecoveryArea_'+str(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.INITIAL_STATE]['start_recovery_id'])]
            dSection = recoveryZone['Section_'+str(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.SEARCHING]['sectionId'])]

            self.searching.updateSearchingState(self.aliveUavList[uavId], dSection)

        elif state == Enum.TRACKING:
            # tracking a fire-zone
            # still is tracking
            hazardZone = self.stateForCoordination['Tracking']['HazardZone_'+str(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.TRACKING]['tracking_zoneId'])]

            self.tracking.updateTrackingState(self.aliveUavList[uavId], hazardZone)
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

    ###
    def returnState(self, uavId):
        uavInfos = self.aliveUavList[uavId]

        if uavInfos['STATE'] & Enum.SEARCHING:
            recoveryZone = self.stateForCoordination['Searching']['RecoveryArea_'+str(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.INITIAL_STATE]['start_recovery_id'])]
            dSection = recoveryZone['Section_'+str(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.SEARCHING]['sectionId'])]
            
            if len(dSection['searchingUavs']) == 1:
                dSection['waiting'] = dSection['waiting'][uavId]
            else : 
                idx = dSection['searchingUavs'].index(uavId)
                waitingList = dSection['waiting'].pop(uavId)
                if idx != 0:
                    newId = dSection['seachingUavs'][idx-1]
                    dSection['waiting'][newId]+=waitingList
                else :
                    newId = dSection['seachingUavs'][idx+1]
                    dSection['waiting'][newId] = waitingList+dSection['waiting'][newId]

        elif uavInfos['STATE'] & Enum.TRACKING:
            hazardZone = self.stateForCoordination['Tracking']['HazardZone_'+str(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.TRACKING]['tracking_zoneId'])]
            hazardZone['tracking_drones'].remove(uavId)
        elif uavInfos['STATE'] & Enum.UNDERTAKED:
            hazardZone = self.stateForCoordination['Tracking']['HazardZone_'+str(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.UNDERTAKED]['tracking_zoneId'])]
            hazardZone['tracking_drones'].remove(uavId)

    ### For initial_state 
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
        action = uavInfos['STATE']

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
            # uav speed up

            #
            # moving
            for zone in self.getDetectedZones().values():
                for i in zone['area']:
                    coord = self.utils.getLatLon(i.get_Latitude(), i.get_Longitude(), mDist, 180 + wind_direction )
                    # [lat, lon]
                    i.set_Latitude(coord[0])
                    i.set_Longitude(coord[1])

    def isScanning(self, uavId): return self.aliveUavList[uavId]['STATE_DETAIL'][Enum.SEARCHING]['is_scanning']
    
    def setScanning(self, uavId, state): self.aliveUavList[uavId]['STATE_DETAIL'][Enum.SEARCHING]['is_scanning'] = state

    def hazardzoneDetected(self, hazardzoneDetected, detectedTime):
        if hazardzoneDetected.get_DetectedHazardZoneType() == 1 :
            # fire zone
            detectedPoint = hazardzoneDetected.get_DetectedLocation()
            entityId = hazardzoneDetected.get_DetectingEnitiyID()
            uavInfos = self.aliveUavList[entityId]
            
            if detectedTime == 0:
                detectedTime = uavInfos['OBJ'].getTime()
            
            if uavInfos['STATE'] == Enum.TRACKING :
                # already tracking
                zoneId = uavInfos['STATE_DETAIL'][Enum.TRACKING]['tracking_zoneID']
                zoneIdForCheck = self.isAlreadyDetectedZone(detectedPoint)
                if zoneId == zoneIdForCheck:
                    if not self.putPointIntoZones(zoneId, detectedPoint):
                        return

                    self.tracking.gooutFromZone(uavInfos)
                    uavInfos['STATE_DETAIL'][Enum.TRACKING]['last_tracking_time'] = detectedTime
                else :
                    print("Found other zone")

            elif uavInfos['STATE'] & (Enum.SEARCHING|Enum.UNDERTAKED) :
                zoneId = self.isAlreadyDetectedZone(detectedPoint)
                if zoneId != -1 :
                    # already detected
                    ## need to analyse

                    if not self.putPointIntoZones(zoneId, detectedPoint):
                        return

                    
                    if self.numOfDroneInSameZone(zoneId) < 2 :
                        # partner
                        # tracking with different direction
                        self.stateForCoordination['Tracking']['HazardZone_'+str(zoneId)]['tracking_drones'].append(entityId)
                        uavInfos['STATE_DETAIL'][Enum.SEARCHING]['is_scanning'] = False
                        uavInfos['STATE'] = Enum.TRACKING
                        uavInfos['STATE_DETAIL'][Enum.TRACKING]['tracking_zoneID'] = zoneId
                        uavInfos['STATE_DETAIL'][Enum.TRACKING]['tracking_direction'] = self.getTrackingDirection(zoneId, detectedPoint)
                        self.tracking.gooutFromZone(uavInfos)
                        uavInfos['STATE_DETAIL'][Enum.TRACKING]['last_tracking_time'] = detectedTime
                    else : 
                        # entity search
                        # 반대 방향으로 나가 버리기
                        self.returnState(entityId)
                        self.turning.turning([detectedPoint.get_Latitude(), detectedPoint.get_Longitude()], uavInfos)                    
                else :
                    # new hazardzone
                    # tracking
                    print(" - Found New Fire Zone")
                    uavInfos['STATE_DETAIL'][Enum.SEARCHING]['is_scanning'] = False
                    uavInfos['STATE'] = Enum.TRACKING
                    uavInfos['STATE_DETAIL'][Enum.TRACKING]['tracking_zoneID'] = self.addNewDetectedZone(detectedPoint)
                    uavInfos['STATE_DETAIL'][Enum.TRACKING]['tracking_direction'] = -1 if uavInfos['OBJ'].getCameraAzimuth() > 0 else 1
                    self.tracking.gooutFromZone(uavInfos)
                    uavInfos['STATE_DETAIL'][Enum.TRACKING]['last_tracking_time'] = detectedTime
                    # call friends
                    self.callMyFriend(entityId, detectedPoint)
                    
        else :
            pass

        return hazardzoneDetected.get_DetectedHazardZoneType()

    def callMyFriend(self, uavId, detectedPoint):
        # fastest
        uavInfos = self.aliveUavList[uavId]
        frinedInfos = self.findNearestFriend(uavId, detectedPoint)

        frinedInfos['STATE']= Enum.UNDERTAKED
        frinedInfos['STATE_DETAIL'][Enum.UNDERTAKED]['tracking_zoneID'] = uavInfos['STATE_DETAIL'][Enum.TRACKING]['tracking_zoneID']
        frinedInfos['STATE_DETAIL'][Enum.UNDERTAKED]['dest_position'] = [detectedPoint.get_Latitude(), detectedPoint.get_Longitude()]
        
        print(" - Called Friend -",frinedInfos['OBJ'].getID(), "ACTION :", frinedInfos['STATE'])

    def findNearestFriend(self, uavId, oDetectedPoint):
        # 시간으로 가장 빨리올수 있는 친구 부르고
        # 그 친구는 UNDERTAKED 상태로 바꾸고
        # tracking_zoneID를 같은 ID 로        
        aDists = [(iUavId, self.utils.distance(oDetectedPoint.get_Longitude(), oDetectedPoint.get_Latitude(),
                                            oUavInfos['OBJ'].getLongitude(), oUavInfos['OBJ'].getLatitude())/oUavInfos['OBJ'].getAirspeed()) 
                                            for iUavId, oUavInfos in self.aliveUavList.items() if iUavId != uavId]

        iNearestId = min(aDists, key = lambda i : i[1])[0] 
        return self.aliveUavList[iNearestId]

    def putPointIntoZones(self, zoneId, detectedPoint):
        zonePoints = self.getDetectedZoneById(zoneId)
        if not zonePoints :
            print("The zoneId is wrong")
            return False
        zonePoints.append(detectedPoint)
        return True

    ### For DetectedZone
    def getTrackingDirection(self, zoneId, detectedPoint):
        aArea = self.stateForCoordination['Tracking']['HazardZone_'+str(zoneId)]['area']
        aStartPoint = self.stateForCoordination['Tracking']['HazardZone_'+str(zoneId)]['startPoint']

        aDists = [(self.utils.distance(aStartPoint[1], aStartPoint[0], point.get_Longitude(), point.get_Latitude()),[point.get_Latitude(), point.get_Longitude()]) for point in aArea]
        aFarthestPoint = max(aDists, key=lambda x: x[0])[1]

        fDistWithSP = self.utils.distance(aStartPoint[1], aStartPoint[0], detectedPoint.get_Latitude(), detectedPoint.get_Longitude())
        fDistWithFP = self.utils.distance(aFarthestPoint[1], aFarthestPoint[0],detectedPoint.get_Latitude(), detectedPoint.get_Longitude())
        
        return 1 if fDistWithSP < fDistWithFP else -1

    def isAlreadyDetectedZone(self, detectedPoint):
        # 가장 가까운 점과의 거리가 일정거리 차이나면 새로운 존
        threshold = 0.003
        fShortestDist = sys.maxsize
        zoneId = 0
        for i in range(len(self.stateForCoordination['Tracking'])):
            for point in self.stateForCoordination['Tracking']['HazardZone_'+str(i)]['area']:
                fDist = self.utils.distance(point.get_Longitude(), point.get_Latitude(), detectedPoint.get_Longitude(), detectedPoint.get_Latitude()) 
                if fDist < fShortestDist:
                    zoneId = i; fShortestDist = fDist
        
        return zoneId if fShortestDist < threshold else -1

    def addNewDetectedZone(self, detectedPoint):
        zoneId = len(self.stateForCoordination['Tracking'])
        self.stateForCoordination['Tracking']['HazardZone_'+str(zoneId)] = {'tracking_drones':[], 'cycled' : False, 'area':[], 'startPoint': [detectedPoint.get_Latitude(), detectedPoint.get_Longitude()]}
        self.stateForCoordination['Tracking']['HazardZone_'+str(zoneId)]['area'].append(detectedPoint)
        return zoneId

    def getDetectedZoneById(self, zoneId):
        if 'HazardZone_'+str(zoneId) in self.stateForCoordination['Tracking'].keys():
            return self.stateForCoordination['Tracking']['HazardZone_'+str(zoneId)]
        return None

    def getDetectedZones(self):
        return self.stateForCoordination['Tracking']
    
    def getCenterPointInZone(self, zoneId):
        lat = 0
        lon = 0
        lenth = len( self.stateForCoordination['Tracking']['HazardZone_'+str(zoneId)]['area'])
        for point in self.stateForCoordination['Tracking']['HazardZone_'+str(zoneId)]['area'] :
            lat += point.get_Latitude()
            lon += point.get_Longitude()
        
        return [lat/lenth, lon/lenth]
    
    def numOfDroneInSameZone(self, zoneId):
        return len(self.stateForCoordination['Tracking']['HazardZone_'+str(zoneId)]['tracking_drones'])

    def estimateDetectedZone(self):
        for i in len(self.stateForCoordination['Tracking']):
            all_coords = [[coord.get_Latitude(), coord.get_Longitude()] for coord in self.stateForCoordination['HazardZone_'+str(i)]['area']]
            hull = ConvexHull(np.asarray(all_coords, dtype=np.float32))
            if len(all_coords) < 3:
                print("Not that much to estimate")
                return
            self.stateForCoordination['HazardZone_'+str(i)]['area'] = [self.stateForCoordination['HazardZone_'+str(i)]['area'][j] for j in hull.vertices]

            # Create polygon object
            estimatedHazardZone = Polygon()

            for k in self.stateForCoordination['HazardZone_'+str(i)]['area']:
                estimatedHazardZone.get_BoundaryPoints().append(k)

            self.utils.send_estimate_report(estimatedHazardZone, i)

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

        uavInfos['STATE_DETAIL'][Enum.INITIAL_STATE]['start_recovery_id']= pointId
        uavInfos['STATE_DETAIL'][Enum.INITIAL_STATE]['recovery_point'] = self.aRecoveryPoints[pointId]

    def removedUavUpdate(self, uavId):
        self.returnState(uavId)
        oDeadUavInfos = self.aliveUavList.pop(uavId)
        self.deadUavList[uavId] = oDeadUavInfos

    def getDist(self, uavId1, uavId2):

        lat1 = self.aliveUavList[uavId1]['OBJ'].getLatitude()
        lon1 = self.aliveUavList[uavId1]['OBJ'].getLongitude()
        
        lat2 = self.aliveUavList[uavId2]['OBJ'].getLatitude()
        lon2 = self.aliveUavList[uavId2]['OBJ'].getLongitude()

        return self.utils.distance(lon1, lat1, lon2, lat2)

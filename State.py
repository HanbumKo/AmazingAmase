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
import Tracing
import Patrolling
import Approaching
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
            'Tracing': {}, # Tracing - HazardZone_0
            'Patrolling' : {}
        }
        self.numOfDrone = 0
        self.utils = utils
        self.searching = None
        self.tracking = Tracing.Tracing(utils)
        self.APPROACHING = Approaching.Approaching(utils)
        self.charging = Charging.Charging(utils)
        self.turning = Turning.Turning(utils)
        self.patrolling = Patrolling.Patrolling(utils)
        self.detectedZones = DetectedZone.DetectedZone(utils)
        
        # recoverypoints
        self.aRecoveryPoints = None

    def checkTotalState(self, uavId):
        if max(list(self.aliveUavList.keys())) == uavId :
            print(" - Check tracking state")
            self.checkTrackingState()
            print(" - Done")
            # print(" - Check patrolling state")
            # self.checkPatrollingState()
            # print(" - Done")
            print(" - Check searching state")
            self.checkSearchingState()
            print(" - Done")
    
    def checkTrackingState(self):
        iNumOfHZ = len(self.stateForCoordination['Tracing'])

        if iNumOfHZ == 0:
            return
        
        for index in range(iNumOfHZ):
            hazardZone = self.stateForCoordination['Tracing']['HazardZone_'+str(index)]
            iNumOfDrones = len(hazardZone['tracking_drones'])
            if hazardZone['cycled'] : # 한바퀴 탐색이 끝났는 가
                for _ in range(iNumOfDrones-1):
                    uavId = hazardZone['tracking_drones'].pop()
                    self.aliveUavList[uavId]['STATE'] = Enum.INITIAL_STATE
            elif iNumOfDrones < 2 : # 최적의 개수 보다 적은 경우
                self.callFriendsForTracking(hazardZone, index)

    def checkPatrollingState(self):
        # it's for smokezone patrolling
        iNumOfSZ = len(self.stateForCoordination['Patrolling'])

        if iNumOfSZ == 0:
            return
        
        for index in range(iNumOfSZ):
            smokeZone = self.stateForCoordination['Patrolling']['SmokeZone_'+str(index)]
            iNumOfDrones = len(smokeZone['patrolling_drones'])

            if iNumOfDrones < 1 : # 최적의 개수 보다 적은 경우
                self.callFriendsForPatrolling(smokeZone, index)
    
    def checkSearchingState(self):
        # 존 별로 하거나 도움이 가능한 드론 별로 하거나
        aFreeDrones = [iUavId for iUavId, oUavInfos in self.aliveUavList.items() if oUavInfos['STATE'] == Enum.INITIAL_STATE]
        threshold = 55
        # 도움을 줄 수 있는 드론 
        # 1. 자기 리커버리 영역 부터 확인
        aFreeDronesForAll = []
        # 리커버리 영역 별로 할당
        for iRecoveryId in range(len(self.stateForCoordination['Searching'])):
            aFreeDrones = [iUavId for iUavId, oUavInfos in self.aliveUavList.items() 
                            if oUavInfos['STATE'] == Enum.INITIAL_STATE and oUavInfos['STATE_DETAIL'][Enum.INITIAL_STATE]['start_recovery_id'] == iRecoveryId ]
            if len(aFreeDrones) != 0 :
                dRecoveryArea = self.stateForCoordination['Searching']['RecoveryArea_'+str(iRecoveryId)]
                # 섹션 단위로 계산
                aProgs = np.array([len(dSection['searched'])/dSection['numberOfPoints']*100 for dSection in dRecoveryArea.values()])
                # drone != 0, area = 0
                # 도움이 필요한 섹션
                aProgIsZero = np.where(aProgs == 0)[0].tolist()
                # drone = 0, area = 0

                aProgIsLow = np.where((aProgs > 0) * (aProgs < threshold))[0].tolist()
                # drone !=0, area != 0
                # drone = 0, area != 0


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
                    aFreeDronesForAll+=aFreeDrones
                    #print(aFreeDrones)

        if len(aFreeDronesForAll) != 0 :
            # 리커버리 존에 맞춰서 나누고도 남은 드론들
            # 남은 드론에 대해서 어떻게 처리하지?
            # 일단 모두 충전해서 대기
            aProgs = []
            iMinIndex = 0
            for iRecoveryId in range(len(self.stateForCoordination['Searching'])):

                dRecoveryArea = self.stateForCoordination['Searching']['RecoveryArea_'+str(iRecoveryId)]
                iSearched = 0
                iAllPoints = 0
                for dSection in dRecoveryArea.values():
                    iSearched += len(dSection['searched'])
                    iAllPoints += dSection['numberOfPoints']

                # 섹션 단위로 계산
                aProgs.append(iSearched/iAllPoints*100)

            iMinIndex = aProgs.index(min(aProgs))

            for iUavId in aFreeDronesForAll:
                # 체인지 리커버리 존
                if self.aliveUavList[iUavId]['OBJ'].getEnergyAvailable() > 60 :
                    self.aliveUavList[iUavId]['STATE_DETAIL'][Enum.INITIAL_STATE]['start_recovery_id']= iMinIndex
                    self.aliveUavList[iUavId]['STATE_DETAIL'][Enum.INITIAL_STATE]['recovery_point'] = self.aRecoveryPoints[iMinIndex]
                    self.aliveUavList[iUavId]['STATE'] = Enum.INITIAL_STATE
                else :
                    self.aliveUavList[iUavId]['STATE'] = Enum.CHARGING

    def callFriendsForTracking(self, dHazardZone, index):
        ### 내가 들어온 방향과 반대로 들어올    수있는 드론, 내가 온 방향이 아닌 쪽으로 오는 드론, 인근 section에서        
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
        aFreeDrones = [(iUavId, self.utils.distance(oPoint[0], oPoint[1], oUavInfos['OBJ'].getLatitude(), oUavInfos['OBJ'].getLongitude())) for iUavId, oUavInfos in self.aliveUavList.items()
                        if oUavInfos['STATE'] & (Enum.INITIAL_STATE|Enum.SEARCHING|Enum.PATROLLING)]
        if len(aFreeDrones) != 0 :
            iNearestId = min(aFreeDrones, key=lambda x: x[1])[0]
            
            self.returnState(iNearestId)
            self.aliveUavList[iNearestId]['STATE'] = Enum.APPROACHING
            self.aliveUavList[iNearestId]['STATE_DETAIL'][Enum.APPROACHING]['tracking_zoneID'] = index
            self.aliveUavList[iNearestId]['STATE_DETAIL'][Enum.APPROACHING]['dest_position'] = [oPoint[0], oPoint[1]]
            self.aliveUavList[iNearestId]['STATE_DETAIL'][Enum.APPROACHING]['turning'] = False

            self.utils.sendWaypointsCmd(iNearestId, self.aliveUavList[iNearestId]['OBJ'].getLatitude(), self.aliveUavList[iNearestId]['OBJ'].getLongitude(),
                                    oPoint[0], oPoint[1], self.getIdealSpeed(iNearestId))
            # self.utils.go_way_points(tNeedToHelp[0],0,dSection['waiting'][tNeedToHelp[0]])
            
            dHazardZone['tracking_drones'].append(iNearestId)

    def callFriendsForPatrolling(self, dSmokeZone, iIdx):
        oPoint = dSmokeZone['point']

        aFreeDrones = [(iUavId, self.utils.distance(oPoint[0], oPoint[1], oUavInfos['OBJ'].getLatitude(), oUavInfos['OBJ'].getLongitude())) for iUavId, oUavInfos in self.aliveUavList.items()
                        if oUavInfos['STATE'] & (Enum.INITIAL_STATE|Enum.SEARCHING)]
        if len(aFreeDrones) != 0 :
            iNearestId = min(aFreeDrones, key=lambda x: x[1])[0]
            
            self.returnState(iNearestId)
            self.aliveUavList[iNearestId]['STATE'] = Enum.PATROLLING

            self.aliveUavList[iNearestId]['STATE_DETAIL'][Enum.PATROLLING]['patrolling_zoneID'] = iIdx
            self.aliveUavList[iNearestId]['STATE_DETAIL'][Enum.PATROLLING]['last_patrolling_time'] = 0
            self.aliveUavList[iNearestId]['STATE_DETAIL'][Enum.PATROLLING]['msgForCheck'] = 0
            self.aliveUavList[iNearestId]['STATE_DETAIL'][Enum.PATROLLING]['msgForCmd'] = -1

            dSmokeZone['patrolling_drones'].append(iNearestId)

    def dealwithZero(self, dRecoveryArea ,aProgIsZero, aFreeDrones):
        # 아무도 탐사하지 않은 지역에 대해서 드론 할당
        # aShortestDrone = []
        aDists = []
        #print(aProgIsZero)
        #print(aFreeDrones)
        for iSectionId in aProgIsZero:
            print("section_",iSectionId)
            dSection = dRecoveryArea['Section_'+str(iSectionId)]
            # aShortestDrone = [sys.maxsize,-1]
            waitingForFreeDrone = []
            tNeedToHelp = None

            ### 기존에 있는 거중 남은 좌표의 개수가 가장 많은 거에서 나눠준다.
            if len(dSection['searchingUavs']) == 0 :
                waitingForFreeDrone = dSection['waiting']
            else :
                aRemainPoints = [(iUavId, aPoints) for iUavId, aPoints in dSection['waiting'].items()]

                tNeedToHelp = max(aRemainPoints,key=lambda x: len(x[1]))

                waitingForFreeDrone = tNeedToHelp[1][int(len(tNeedToHelp[1])/2):]

            #print(waitingForFreeDrone)
            for iUavId in aFreeDrones:
                aPoint = waitingForFreeDrone[0]


                fUav_lat = self.aliveUavList[iUavId]['OBJ'].getLatitude()
                fUav_lon = self.aliveUavList[iUavId]['OBJ'].getLongitude()
                
                fDist = self.utils.distance(aPoint[0], aPoint[1], fUav_lat, fUav_lon)
                
                aDists.append([iSectionId, iUavId, fDist, waitingForFreeDrone])

                # if fDist < aShortestDrone[0] :
                #     aShortestDrone[0] = fDist; aShortestDrone[1] = iUavId
            # dSection['searchingUavs'].append(aShortestDrone[1])
            # dSection['waiting'] = { aShortestDrone[1] : dSection['waiting']}
            # aFreeDrones.remove(aShortestDrone[1])
            # ## Need to change drone state!
            # self.aliveUavList[aShortestDrone[1]]['STATE'] = Enum.SEARCHING
            # # RecoveryId is same as self.aliveUavList - Enum.INITIAL_STATE
            # self.aliveUavList[aShortestDrone[1]]['STATE_DETAIL'][Enum.SEARCHING]['sectionId'] = iSectionId

            # if len(aFreeDrones) == 0 :
            #     break
        while (len(aDists) != 0):
            aNearest = min(aDists, key= lambda x:x[2])
            dSection = dRecoveryArea['Section_'+str(aNearest[0])]

            ### 기존에 있는 거중 남은 좌표의 개수가 가장 많은 거에서 나눠준다.
            if len(dSection['searchingUavs']) == 0 :
                dSection['waiting'] = {}
            else :
                dSection['waiting'][tNeedToHelp[0]] = tNeedToHelp[1][:int(len(tNeedToHelp[1])/2)]

                self.utils.sendWaypointsCmd(tNeedToHelp[0], self.aliveUavList[tNeedToHelp[0]]['OBJ'].getLatitude(), self.aliveUavList[tNeedToHelp[0]]['OBJ'].getLongitude(),
                                        dSection['waiting'][tNeedToHelp[0]][0][0], dSection['waiting'][tNeedToHelp[0]][0][1], self.getIdealSpeed(tNeedToHelp[0]))
                # self.utils.go_way_points(tNeedToHelp[0],0,dSection['waiting'][tNeedToHelp[0]])

            dSection['searchingUavs'].append(aNearest[1])
            dSection['waiting'][aNearest[1]] = aNearest[3]

            aFreeDrones.remove(aNearest[1])

            ## Need to change drone state!
            self.aliveUavList[aNearest[1]]['STATE'] = Enum.SEARCHING
            # RecoveryId is same as self.aliveUavList - Enum.INITIAL_STATE
            self.aliveUavList[aNearest[1]]['STATE_DETAIL'][Enum.SEARCHING]['sectionId'] = aNearest[0]
            
            # self.utils.go_way_points(aNearest[1],0,aNearest[3])
            self.utils.sendWaypointsCmd(aNearest[1], self.aliveUavList[aNearest[1]]['OBJ'].getLatitude(), self.aliveUavList[aNearest[1]]['OBJ'].getLongitude(),
                                        aNearest[3][0][0], aNearest[3][0][1], self.getIdealSpeed(aNearest[1]))

            aTempDists = []

            for i in aDists :
                if i[0] != aNearest[0] and i[1] != aNearest[1]:
                    aTempDists.append(i)
            aDists = aTempDists
        return aFreeDrones

    def dealwithLow(self, dRecoveryArea ,aProgIsLow, aFreeDrones):
        # 커버된 영역이 적은 걸 먼저 처리
        # 고로 정렬 필요
        if len(aProgIsLow) != 0 :
            #print(aProgIsLow)
            aProgIsLow = [(iSectionId, len(dRecoveryArea['Section_'+str(iSectionId)]['searched'])/dRecoveryArea['Section_'+str(iSectionId)]['numberOfPoints']*100) for iSectionId in aProgIsLow]
            aProgIsLow.sort(key=lambda x: x[1])
            aProgIsLow = list(map(lambda x: x[0], aProgIsLow))
            #print(aProgIsLow)

        aDists = []
        tNeedToHelp = None
        for iSectionId in aProgIsLow:
            dSection = dRecoveryArea['Section_'+str(iSectionId)]
            waitingForFreeDrone = []


            ### 기존에 있는 거중 남은 좌표의 개수가 가장 많은 거에서 나눠준다.
            if len(dSection['searchingUavs']) == 0 :
                waitingForFreeDrone = dSection['waiting']
            else :
                #print("here?")
                aRemainPoints = [(iUavId, aPoints) for iUavId, aPoints in dSection['waiting'].items() ]
                #print(aRemainPoints)
                tNeedToHelp = max(aRemainPoints,key=lambda x: len(x[1]))
                #print("tNed",tNeedToHelp)
                
                waitingForFreeDrone = tNeedToHelp[1][int(len(tNeedToHelp[1])/2):]

            for iUavId in aFreeDrones:

                aPoint = waitingForFreeDrone[0]

                fUav_lat = self.aliveUavList[iUavId]['OBJ'].getLatitude()
                fUav_lon = self.aliveUavList[iUavId]['OBJ'].getLongitude()
                
                fDist = self.utils.distance(aPoint[0], aPoint[1], fUav_lat, fUav_lon)
                
                aDists.append([iSectionId, iUavId, fDist, waitingForFreeDrone])
                

        while (len(aDists) != 0):
            #print(aDists)
            aNearest = min(aDists, key= lambda x:x[2])
            dSection = dRecoveryArea['Section_'+str(aNearest[0])]

            if len(dSection['searchingUavs']) == 0:
                dSection['waiting'] = {}
            else :
                if int(len(tNeedToHelp[1])/2) == 0 :
                    dSection['waiting'].pop(tNeedToHelp[0])
                    dSection['searchingUavs'].remove(tNeedToHelp[0])
                    self.aliveUavList[tNeedToHelp[0]]['STATE'] = Enum.INITIAL_STATE
                else :
                    dSection['waiting'][tNeedToHelp[0]] = tNeedToHelp[1][:int(len(tNeedToHelp[1])/2)]
                    self.utils.sendWaypointsCmd(tNeedToHelp[0], self.aliveUavList[tNeedToHelp[0]]['OBJ'].getLatitude(), self.aliveUavList[tNeedToHelp[0]]['OBJ'].getLongitude(),
                                            dSection['waiting'][tNeedToHelp[0]][0][0], dSection['waiting'][tNeedToHelp[0]][0][1], self.getIdealSpeed(tNeedToHelp[0]))
                    # self.utils.go_way_points(tNeedToHelp[0],0,dSection['waiting'][tNeedToHelp[0]])

            dSection['searchingUavs'].append(aNearest[1])
            dSection['waiting'][aNearest[1]] = aNearest[3]

            aFreeDrones.remove(aNearest[1])

            ## Need to change drone state!
            self.aliveUavList[aNearest[1]]['STATE'] = Enum.SEARCHING
            # RecoveryId is same as self.aliveUavList - Enum.INITIAL_STATE
            self.aliveUavList[aNearest[1]]['STATE_DETAIL'][Enum.SEARCHING]['sectionId'] = aNearest[0]
            
            # self.utils.go_way_points(aNearest[1],0,aNearest[3])
            self.utils.sendWaypointsCmd(aNearest[1], self.aliveUavList[iUavId]['OBJ'].getLatitude(), self.aliveUavList[iUavId]['OBJ'].getLongitude(),
                                        aNearest[3][0][0], aNearest[3][0][1], self.getIdealSpeed(aNearest[1]))

            aTempDists = []

            for i in aDists :
                if i[0] != aNearest[0] and i[1] != aNearest[1]:
                    aTempDists.append(i)
            aDists = aTempDists
        return aFreeDrones

    ### For uav information
    # update drones information.
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
    # add new drone in list
    def addNewUAV(self, AirVehicleConfiguration):
        newDroneDict = {}
        newDroneDict['STATE'] = Enum.INITIAL_STATE
        newDroneDict['STATE_DETAIL'] = {
            Enum.INITIAL_STATE : {
                'start_recovery_id' : -1,
                'current_recovery_id' : -1,
                'recovery_point' : [0,0]
            },
            Enum.SEARCHING : {
                'is_scanning' : False,
                'sectionId' : -1
            },
            Enum.TRACING : {
                'msg' : 0,
                'tracking_direction' : 1,
                'tracking_zoneID' : -1,
                'last_tracking_time' : 0
            },
            Enum.APPROACHING:{
                'tracking_zoneID' : -1,
                'dest_position' : [0,0],
                'turning' : False,
                'azimuth' : 0
            },
            Enum.CHARGING : {
            },
            Enum.TURNING : {
                'heading' : 0,
            },
            Enum.TURNING2 : {
                'altitude' : -1,
                'prev_state' : Enum.INITIAL_STATE,
            },
            Enum.PATROLLING: {
                'msgForCheck' : 0,
                'msgForCmd' : -1,
                'patrolling_zoneID' : -1,
                'last_patrolling_time' : -1
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
    # when the scenario start first, divide whole area into section
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
    # check all drone is in keepinzone
    def checkStillInKeep(self, aKeepInZones, uavId):
        uav_lat = self.aliveUavList[uavId]['OBJ'].getLatitude()
        uav_lon = self.aliveUavList[uavId]['OBJ'].getLongitude()

        top = aKeepInZones[0][0]; bottom = aKeepInZones[2][0]
        left = aKeepInZones[0][1]; right = aKeepInZones[2][1]

        if  uav_lat >= top or uav_lat <= bottom or uav_lon >= right or uav_lon <= left :
            print("UAV -",uavId," needs to turn.")
            self.returnState(uavId)
            self.turning.turning([uav_lat,uav_lon], self.aliveUavList[uavId])
    # check which drone need to charge
    def checkNeedToCharge(self, uavId):
        uavInfos = self.aliveUavList[uavId]
        self.setClosestRecoveryPoint(uavId)

        if uavInfos['STATE'] != Enum.CHARGING:
            fRemainFuel = uavInfos['OBJ'].getEnergyAvailable()

            if fRemainFuel < 50 and uavInfos['OBJ'].getTime() < 2390000: 
                # coord update
                self.returnState(uavId)
                
                # uavInfos['STATE_DETAIL'][Enum.CHARGING]['previous_action'] = uavInfos['STATE']
                uavInfos['STATE_DETAIL'][Enum.SEARCHING]['is_scanning'] = True
                uavInfos['STATE'] = Enum.CHARGING

                self.utils.sendWaypointsCmd(uavId, uavInfos['OBJ'].getLatitude(), uavInfos['OBJ'].getLongitude(), uavInfos['STATE_DETAIL'][Enum.INITIAL_STATE]['recovery_point'][0], uavInfos['STATE_DETAIL'][Enum.INITIAL_STATE]['recovery_point'][1],self.getIdealSpeed(uavId))
                
                print("UAV -", uavId," needs to charging")
    # Do action for each state.
    def updateUavAction(self, uavId):
        print("uavId", uavId)
        state = self.aliveUavList[uavId]['STATE']

        # state check
        if state == Enum.INITIAL_STATE:
            # At first time, all drones will be INITIAL_STATE, and all drones will start initial searching
            pass
        elif state == Enum.SEARCHING:
            print("SEARCHING")
            # search for fire-zone and entity
            # update action_detail.
            #print(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.SEARCHING]['sectionId'])
            recoveryZone = self.stateForCoordination['Searching']['RecoveryArea_'+str(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.INITIAL_STATE]['start_recovery_id'])]
            dSection = recoveryZone['Section_'+str(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.SEARCHING]['sectionId'])]
            self.searching.updateSearchingState(self.aliveUavList[uavId], dSection)

        elif state == Enum.TRACING:
            # tracking a fire-zone
            # still is tracking
            print("TRACING")

            hazardZone = self.stateForCoordination['Tracing']['HazardZone_'+str(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.TRACING]['tracking_zoneID'])]

            self.tracking.updateTrackingState(self.aliveUavList[uavId], hazardZone)
        elif state == Enum.APPROACHING:
            print("APPROACHING")

            self.APPROACHING.updateApproachingState(self.aliveUavList[uavId])
            print("UAV -",uavId,"is going to zone")
        elif state == Enum.CHARGING:
            # going to recovery-zone
            # still is charging
            # charging
            print("CHARGING")

            self.charging.updateChargingState(self.aliveUavList[uavId])
        elif state == Enum.TURNING :
            print("TURNING")

            self.turning.updateTurningState(self.aliveUavList[uavId])
        elif state == Enum.TURNING2 :
            print("TURNING2")

            self.turning.updateTurning2State(self.aliveUavList[uavId])

        elif state == Enum.PATROLLING :
            # smokeZone = self.stateForCoordination['Patrolling']['SmokeZone_'+str(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.PATROLLING]['patrolling_zoneID'])]
            # self.patrolling.updatePatrollingState(self.aliveUavList[uavId], smokeZone)
            pass
    # When Change the state or be dead
    def returnState(self, uavId):
        print(" - Return state")
        uavInfos = self.aliveUavList[uavId]

        if uavInfos['STATE'] & Enum.SEARCHING:
            print(" - State SEARCHING")
            recoveryZone = self.stateForCoordination['Searching']['RecoveryArea_'+str(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.INITIAL_STATE]['start_recovery_id'])]
            dSection = recoveryZone['Section_'+str(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.SEARCHING]['sectionId'])]
            
            processingPoint = dSection['waiting'][uavId].pop(0)
            dSection['searched'].append(processingPoint)

            if len(dSection['searchingUavs']) == 1:
                dSection['waiting'] = dSection['waiting'][uavId]
                #print(dSection['waiting'])
            else : 
                idx = dSection['searchingUavs'].index(uavId)
                waitingList = dSection['waiting'].pop(uavId)
                if idx != 0:
                    newId = dSection['searchingUavs'][idx-1]
                    dSection['waiting'][newId]+=waitingList
                else :
                    newId = dSection['searchingUavs'][idx+1]
                    dSection['waiting'][newId] = waitingList+dSection['waiting'][newId]

            # if len(dSection['searched']) == dSection['numberOfPoints']:
            #     dSection['waiting'] = dSection['searched']
            #     dSection['searched'].clear()

            dSection['searchingUavs'].remove(uavId)

        elif uavInfos['STATE'] & Enum.TRACING:
            print(" - State TRACING")
            hazardZone = self.stateForCoordination['Tracing']['HazardZone_'+str(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.TRACING]['tracking_zoneID'])]
            hazardZone['tracking_drones'].remove(uavId)
            self.aliveUavList[uavId]['STATE_DETAIL'][Enum.SEARCHING]['is_scanning'] = False
        elif uavInfos['STATE'] & Enum.APPROACHING:
            print(" - State APPROACHING")
            hazardZone = self.stateForCoordination['Tracing']['HazardZone_'+str(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.APPROACHING]['tracking_zoneID'])]
            hazardZone['tracking_drones'].remove(uavId)
            self.aliveUavList[uavId]['STATE_DETAIL'][Enum.SEARCHING]['is_scanning'] = False

        elif uavInfos['STATE'] & Enum.TURNING2:
            print(" - State TURNING2")

            uavInfos['STATE'] = uavInfos['STATE_DETAIL'][Enum.TURNING2]['prev_state']
            self.returnState(uavId)

        elif uavInfos['STATE'] & Enum.PATROLLING:
            smokeZone = self.stateForCoordination['Patrolling']['SmokeZone_'+str(self.aliveUavList[uavId]['STATE_DETAIL'][Enum.PATROLLING]['patrolling_zoneID'])]
            smokeZone['patrolling_drones'].remove(uavId)
        print(" - Done")
    ######################

    ### set next heading, altitude etc 
    # main function called
    def getUpdateInfos(self, uavId):
        # self.getElevAndAlti(uavId)
        self.getElevAndAlti3(uavId)

        return self.aliveUavList[uavId], self.aliveUavList[uavId]['NEXT_HEADING'], self.aliveUavList[uavId]['NEXT_AZIMUTH'], self.aliveUavList[uavId]['NEXT_ELEVATION'], self.aliveUavList[uavId]['NEXT_ALTITUDE']
    
    def getElevAndAltiForSmokeZoneDetect(self, uavId):
        pass
    # Now using, Version3 get next altitude of drone and elevation of sensor
    def getElevAndAlti3(self, uavId):
        uavInfos = self.aliveUavList[uavId]

        iGapFromStoE = uavInfos['OBJ'].getHazardMaxRange()*0.6
        
        start_lat = uavInfos['OBJ'].getLatitude()
        start_lon = uavInfos['OBJ'].getLongitude()

        end_LatLon = self.utils.getLatLon(start_lat, start_lon, iGapFromStoE, self.utils.getOriginalDirection(uavInfos)[0])

        uav_Alt = uavInfos['OBJ'].getAltitude()
        current_Alt = self.utils.getElevation(start_lat, start_lon)
        front_Alt = self.utils.getElevation(end_LatLon[0],end_LatLon[1])

        iGapUavAndCur = uav_Alt - current_Alt # this should not be negative.
        iGapUavAndFront = uav_Alt - front_Alt # this could be negative or positive.

        fLowerThreshold = uavInfos['OBJ'].getHazardMaxRange()*0.3
        fHigherThreshold = uavInfos['OBJ'].getHazardMaxRange()*0.6
        
        uavInfos['NEXT_ALTITUDE'] = self.utils.getElevation(start_lat, start_lon) + uavInfos['OBJ'].getHazardMaxRange()*0.30

        # negative cut
        if iGapUavAndFront > 0 :
            if iGapUavAndFront < fHigherThreshold:
                uavInfos['NEXT_ELEVATION'] = self.utils.getTheta(iGapUavAndFront, uavInfos['OBJ'].getHazardMaxRange())
            else :
                uavInfos['NEXT_ELEVATION'] = -55
    # Version2 get next altitude of drone and elevation of sensor
    def getElevAndAlti2(self, uavId):
        uavInfos = self.aliveUavList[uavId]

        iGapFromStoE = 400 # uavInfos['OBJ'].getHazardMaxRange()*0.8
        
        start_lat = uavInfos['OBJ'].getLatitude()
        start_lon = uavInfos['OBJ'].getLongitude()

        end_LatLon = self.utils.getLatLon(start_lat, start_lon, iGapFromStoE, self.utils.getOriginalDirection(uavInfos))

        uav_Alt = uavInfos['OBJ'].getAltitude()
        current_Alt = self.utils.getElevation(start_lat, start_lon)
        front_Alt = self.utils.getElevation(end_LatLon[0],end_LatLon[1])

        iGapUavAndCur = uav_Alt - current_Alt # this should not be negative.
        iGapUavAndFront = uav_Alt - front_Alt # this could be negative or positive.

        fLowerThreshold = uavInfos['OBJ'].getHazardMaxRange()*0.3
        fHigherThreshold = uavInfos['OBJ'].getHazardMaxRange()*0.6

        if current_Alt < front_Alt :
            # going up
            if iGapUavAndFront < 0 :
                    self.turning.turningForChangingAlt(front_Alt + uavInfos['OBJ'].getHazardMaxRange()*0.45, uavInfos )
            else :    
                if iGapUavAndCur < fLowerThreshold : 
                    if iGapUavAndFront > fHigherThreshold : 
                        pass # it's impossible
                    else :
                        # normal or getting higher fastly.
                        # turing to go up to front_Alt + maxRange*0.45
                        self.turning.turningForChangingAlt(front_Alt + uavInfos['OBJ'].getHazardMaxRange()*0.45, uavInfos )

                elif iGapUavAndCur > fHigherThreshold :
                    if iGapUavAndFront < fLowerThreshold or iGapUavAndFront > fHigherThreshold :
                        self.turning.turningForChangingAlt(front_Alt + uavInfos['OBJ'].getHazardMaxRange()*0.45, uavInfos )
                        pass # can detect, but to go up is helpful next situation or need to go down to front_Alt + maxRange*0.45
                    else :
                        uavInfos['NEXT_ALTITUDE'] = uav_Alt
                        uavInfos['NEXT_ELEVATION'] = self.utils.getTheta(iGapUavAndFront, uavInfos['OBJ'].getHazardMaxRange())

                        pass # can detect, and no problem

                else :
                    if iGapUavAndFront < fLowerThreshold :
                        self.turning.turningForChangingAlt(front_Alt + uavInfos['OBJ'].getHazardMaxRange()*0.45, uavInfos )
                        pass # need to go up
                    elif iGapUavAndFront > fHigherThreshold :
                        pass # this doesn't make sense
                    else :
                        uavInfos['NEXT_ALTITUDE'] = uav_Alt
                        uavInfos['NEXT_ELEVATION'] = self.utils.getTheta(iGapUavAndFront, uavInfos['OBJ'].getHazardMaxRange())

                        pass # can detect, ideal state.
        else :
            # going down
            if iGapUavAndCur < fLowerThreshold : 
                if iGapUavAndFront > fHigherThreshold : 
                    aElevs, aCoords = self.utils.getLineElevs(start_lat, start_lon, end_LatLon[0], end_LatLon[1])
                    for i in aElevs :
                        if uav_Alt - i < fHigherThreshold and uav_Alt - i > fLowerThreshold :
                            uavInfos['NEXT_ALTITUDE'] = i + iGapUavAndCur
                            uavInfos['NEXT_ELEVATION'] = self.utils.getTheta(i, uavInfos['OBJ'].getHazardMaxRange())

                else :
                    # normal or getting lower slowly.
                    uavInfos['NEXT_ALTITUDE'] = uav_Alt
                    uavInfos['NEXT_ELEVATION'] = self.utils.getTheta(iGapUavAndFront, uavInfos['OBJ'].getHazardMaxRange())

            elif iGapUavAndCur > fHigherThreshold :
                if iGapUavAndFront < fLowerThreshold :
                    pass # this dosen't make sense
                elif iGapUavAndFront > fHigherThreshold :
                    self.turning.turningForChangingAlt(front_Alt + fLowerThreshold, uavInfos )
                
                else :
                    pass # this dosen't make sense

            else :
                if iGapUavAndFront < fLowerThreshold :
                    pass # this doesn't make sense
                elif iGapUavAndFront > fHigherThreshold :
                    aElevs, aCoords = self.utils.getLineElevs(start_lat, start_lon, end_LatLon[0], end_LatLon[1])
                    for i in aElevs :
                        if uav_Alt - i < fHigherThreshold and uav_Alt - i > fLowerThreshold :
                            uavInfos['NEXT_ALTITUDE'] = i + iGapUavAndCur
                            uavInfos['NEXT_ELEVATION'] = self.utils.getTheta(i, uavInfos['OBJ'].getHazardMaxRange())

                else :
                    uavInfos['NEXT_ALTITUDE'] = uav_Alt
                    uavInfos['NEXT_ELEVATION'] = self.utils.getTheta(iGapUavAndFront, uavInfos['OBJ'].getHazardMaxRange())
    # Version1 get next altitude of drone and elevation of sensor
    def getElevAndAlti(self, uavId):
        # terrain
        uavInfos = self.aliveUavList[uavId]
        uav_lat = uavInfos['OBJ'].getLatitude()
        uav_lon = uavInfos['OBJ'].getLongitude()
        originalDirection = self.utils.getOriginalDirection(uavInfos)
        paddingAlt = 0
        idealDist = 0

        paddingAlt = uavInfos['OBJ'].getHazardMaxRange()*0.5
        idealDist = uavInfos['OBJ'].getHazardMaxRange()*0.6
        alpha = 0.002
        # idealDist = uavInfos['OBJ'].getHazardMaxRange()*0.6
        action = uavInfos['STATE']

        front_Alt = 0
        aAllEle = self.utils.getElevations(uav_lat+alpha*2, uav_lon-alpha, uav_lat, uav_lon+alpha, 1 / 3600)
        fSum = 0
        iLength = 0
        fMeanAlt = 0
        iMax = 0
        iMin = 0
        for i in aAllEle:

            fSum += sum(i)
            iLength += len(i)
            if iMax < max(i):
                iMax = max(i)
            if iMin > min(i):
                iMin = min(i)

        fMeanAlt = fSum/iLength

        uavInfos['NEXT_ALTITUDE'] = self.utils.getElevation(uav_lat, uav_lon)*0.8 + paddingAlt + fMeanAlt*0.2
        gap1 = iMax - fMeanAlt
        gap2 = fMeanAlt - iMin

        if gap1 > gap2 :
            uavInfos['NEXT_ALTITUDE'] =  self.utils.getElevation(uav_lat, uav_lon) + uavInfos['OBJ'].getHazardMaxRange()*0.6
            uavInfos['NEXT_ELEVATION'] = self.utils.getTheta(idealDist,uavInfos['OBJ'].getHazardMaxRange())

        else :
            uavInfos['NEXT_ALTITUDE'] = self.utils.getElevation(uav_lat, uav_lon)+ uavInfos['OBJ'].getHazardMaxRange()*0.3
            uavInfos['NEXT_ELEVATION'] = self.utils.getTheta(idealDist,uavInfos['OBJ'].getHazardMaxRange())


            alt_gap = uavInfos['OBJ'].getAltitude() - iMin

            if alt_gap > 0 :
                if alt_gap <= idealDist :
                    uavInfos['NEXT_ELEVATION'] = self.utils.getTheta(idealDist,uavInfos['OBJ'].getHazardMaxRange())

                elif alt_gap < uavInfos['OBJ'].getHazardMaxRange() :
                    uavInfos['NEXT_ELEVATION'] = self.utils.getTheta(alt_gap, uavInfos['OBJ'].getHazardMaxRange())
                    # uavInfos['NEXT_ELEVATION'] = -70
                else :
                    uavInfos['NEXT_ELEVATION'] = -60


            # # 최고와 최저의 갭 차이에 따라서 다르게
            # if (iMax-iMin) < 350 :

            #     pass
            # else :
            #     pass
            # pass
        # if action == Enum.TRACING : #searching, charging, going, entity_searching
        #     coord = self.utils.getLatLon(uav_lat, uav_lon, idealDist, originalDirection)
        #     front_Alt = self.utils.getElevation(coord[0], coord[1])
        # else :  # ETC.
        # if uavInfos['OBJ'].getCameraCenterpoint():
        #     front_Alt = uavInfos['OBJ'].getCameraCenterpoint().get_Altitude()

        # Q1. how does the gimbal angle adjust in front altitude higher than now.
    # adjust the fire zone points according to wind
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

    ### drone is detecting a zone.
    def hazardzoneDetected(self, hazardzoneDetected, detectedTime):
        if hazardzoneDetected.get_DetectedHazardZoneType() == 1 :
            # fire zone
            detectedPoint = hazardzoneDetected.get_DetectedLocation()
            entityId = hazardzoneDetected.get_DetectingEnitiyID()
            uavInfos = self.aliveUavList[entityId]
            
            if detectedTime == 0:
                detectedTime = uavInfos['OBJ'].getTime()
            
            if uavInfos['STATE'] == Enum.TRACING :
                # already tracking
                zoneId = uavInfos['STATE_DETAIL'][Enum.TRACING]['tracking_zoneID']
                zoneIdForCheck = self.isAlreadyDetectedZone(detectedPoint)
                if zoneId == zoneIdForCheck:
                    if not self.putPointIntoZones(zoneId, detectedPoint):
                        return

                    self.tracking.gooutFromZone(uavInfos)
                    uavInfos['STATE_DETAIL'][Enum.TRACING]['last_tracking_time'] = detectedTime
                else :
                    print("Found other zone")

            elif uavInfos['STATE'] & (Enum.SEARCHING|Enum.APPROACHING) :
                zoneId = self.isAlreadyDetectedZone(detectedPoint)
                if zoneId != -1 :
                    # already detected
                    ## need to analyse

                    if not self.putPointIntoZones(zoneId, detectedPoint):
                        return

                    # if uavInfos['STATE'] & Enum.SEARCHING :
                    #     self.returnState(entityId)
                    #     self.turning.turning([detectedPoint.get_Latitude(), detectedPoint.get_Longitude()], uavInfos) 
                    # else :   
                    #     if uavInfos['STATE_DETAIL'][Enum.APPROACHING]['turning']  == False:
                    #         self.turning.turning([detectedPoint.get_Latitude(), detectedPoint.get_Longitude()], uavInfos) 
                    #         uavInfos['STATE_DETAIL'][Enum.APPROACHING]['turning'] = True
                    #         uavInfos['STATE_DETAIL'][Enum.APPROACHING]['azimuth'] = uavInfos['OBJ'].getCameraAzimuth()

                    #     else : 
                    #         uavInfos['STATE_DETAIL'][Enum.SEARCHING]['is_scanning'] = False
                    #         uavInfos['STATE'] = Enum.TRACING
                    #         uavInfos['STATE_DETAIL'][Enum.TRACING]['tracking_zoneID'] = zoneId
                    #         uavInfos['STATE_DETAIL'][Enum.TRACING]['tracking_direction'] = -1 if uavInfos['OBJ'].getCameraAzimuth() > 0 else 1
                    #         self.tracking.gooutFromZone(uavInfos)
                    #         uavInfos['STATE_DETAIL'][Enum.TRACING]['last_tracking_time'] = detectedTime
                        
                    if self.numOfDroneInSameZone(zoneId) < 2 :
                        # partner
                        # tracking with different direction
                        self.stateForCoordination['Tracing']['HazardZone_'+str(zoneId)]['tracking_drones'].append(entityId)
                        uavInfos['STATE_DETAIL'][Enum.SEARCHING]['is_scanning'] = False
                        uavInfos['STATE'] = Enum.TRACING
                        uavInfos['STATE_DETAIL'][Enum.TRACING]['tracking_zoneID'] = zoneId
                        uavInfos['STATE_DETAIL'][Enum.TRACING]['tracking_direction'] = -1 if uavInfos['OBJ'].getCameraAzimuth() > 0 else 1
                        self.tracking.gooutFromZone(uavInfos)
                        uavInfos['STATE_DETAIL'][Enum.TRACING]['last_tracking_time'] = detectedTime
                    elif entityId in self.stateForCoordination['Tracing']['HazardZone_'+str(zoneId)]['tracking_drones']:
                        uavInfos['STATE_DETAIL'][Enum.SEARCHING]['is_scanning'] = False
                        uavInfos['STATE'] = Enum.TRACING
                        uavInfos['STATE_DETAIL'][Enum.TRACING]['tracking_zoneID'] = zoneId
                        uavInfos['STATE_DETAIL'][Enum.TRACING]['tracking_direction'] = -1 if uavInfos['OBJ'].getCameraAzimuth() > 0 else 1
                        self.tracking.gooutFromZone(uavInfos)
                        uavInfos['STATE_DETAIL'][Enum.TRACING]['last_tracking_time'] = detectedTime
                    else : 
                        # entity search
                        # 반대 방향으로 나가 버리기
                        self.returnState(entityId)
                        self.turning.turning([detectedPoint.get_Latitude(), detectedPoint.get_Longitude()], uavInfos)   
                else :
                    # new hazardzone
                    # tracking
                    print(" - Found New Fire Zone")
                    # self.turning.turning([detectedPoint.get_Latitude(), detectedPoint.get_Longitude()], uavInfos)
                    # uavInfos['STATE_DETAIL'][Enum.APPROACHING]['turning'] = True
                    # uavInfos['STATE_DETAIL'][Enum.APPROACHING]['azimuth'] = uavInfos['OBJ'].getCameraAzimuth()
                    # uavInfos['STATE_DETAIL'][Enum.SEARCHING]['is_scanning'] = False
                    # uavInfos['STATE'] = Enum.APPROACHING

                    # uavInfos['STATE_DETAIL'][Enum.APPROACHING]['tracking_zoneID'] = self.addNewDetectedZoneForTracking(detectedPoint)
                    # uavInfos['STATE_DETAIL'][Enum.APPROACHING]['dest_position'] = [detectedPoint.get_Latitude(), detectedPoint.get_Longitude()]
                    uavInfos['STATE_DETAIL'][Enum.TRACING]['tracking_zoneID'] = self.addNewDetectedZoneForTracking(detectedPoint)
                    self.stateForCoordination['Tracing']['HazardZone_'+str(uavInfos['STATE_DETAIL'][Enum.TRACING]['tracking_zoneID'])]['tracking_drones'].append(entityId)
                    uavInfos['STATE_DETAIL'][Enum.TRACING]['tracking_direction'] = -1 if uavInfos['OBJ'].getCameraAzimuth() > 0 else 1
                    self.tracking.gooutFromZone(uavInfos)
                    uavInfos['STATE_DETAIL'][Enum.TRACING]['last_tracking_time'] = detectedTime
                    uavInfos['STATE_DETAIL'][Enum.SEARCHING]['is_scanning'] = False
                    uavInfos['STATE'] = Enum.TRACING
                    # # call friends
                    # self.callMyFriend(entityId, detectedPoint)
        '''            
        else :
            # Smoke zone 탐색했을 때 똑같이
            # fire zone
            detectedPoint = hazardzoneDetected.get_DetectedLocation()
            entityId = hazardzoneDetected.get_DetectingEnitiyID()
            uavInfos = self.aliveUavList[entityId]
            
            if detectedTime == 0:
                detectedTime = uavInfos['OBJ'].getTime()
            
            if uavInfos['STATE'] == Enum.PATROLLING :
                # already tracking
                self.patrolling.goStraight(uavInfos)
                self.stateForCoordination['Patrolling']['SmokeZone_'+str(uavInfos['STATE_DETAIL'][Enum.PATROLLING]['patrolling_zoneID'])]['point'] = [detectedPoint.get_Latitude(), detectedPoint.get_Longitude()]
                uavInfos['STATE_DETAIL'][Enum.PATROLLING]['last_patrolling_time'] = detectedTime

            elif uavInfos['STATE'] & Enum.SEARCHING :
                # new hazardzone
                # tracking
                print(" - Found New Smoke Zone")
                uavInfos['STATE'] = Enum.PATROLLING
                uavInfos['STATE_DETAIL'][Enum.PATROLLING]['patrolling_zoneID'] = self.addNewDetectedZoneForPatrolling(detectedPoint)
                self.stateForCoordination['Patrolling']['SmokeZone_'+str(uavInfos['STATE_DETAIL'][Enum.PATROLLING]['patrolling_zoneID'])]['patrolling_drones'].append(entityId)
                self.patrolling.goStraight(uavInfos)
                uavInfos['STATE_DETAIL'][Enum.PATROLLING]['last_patrolling_time'] = detectedTime
            elif uavInfos['STATE'] & Enum.CHARGING:
                pass
        '''
        return hazardzoneDetected.get_DetectedHazardZoneType()
    # call nearest drone
    def callMyFriend(self, uavId, detectedPoint):
        # fastest
        uavInfos = self.aliveUavList[uavId]
        friendInfos = self.findNearestFriend(uavId, detectedPoint)
        if friendInfos:
            self.returnState(friendInfos['OBJ'].getID())

            friendInfos['STATE']= Enum.APPROACHING
            friendInfos['STATE_DETAIL'][Enum.APPROACHING]['tracking_zoneID'] = uavInfos['STATE_DETAIL'][Enum.TRACING]['tracking_zoneID']
            friendInfos['STATE_DETAIL'][Enum.APPROACHING]['dest_position'] = [detectedPoint.get_Latitude(), detectedPoint.get_Longitude()]
            
            self.stateForCoordination['Tracing']['HazardZone_'+str(uavInfos['STATE_DETAIL'][Enum.TRACING]['tracking_zoneID'])]['tracking_drones'].append(friendInfos['OBJ'].getID())
            print(" - Called Friend -",friendInfos['OBJ'].getID(), "ACTION :", friendInfos['STATE'])
    # calculate which one is nearest
    def findNearestFriend(self, uavId, oDetectedPoint):
        # 시간으로 가장 빨리올수 있는 친구 부르고
        # 그 친구는 APPROACHING 상태로 바꾸고
        # tracking_zoneID를 같은 ID 로        
        aDists = [(iUavId, self.utils.distance(oDetectedPoint.get_Longitude(), oDetectedPoint.get_Latitude(),
                                            oUavInfos['OBJ'].getLongitude(), oUavInfos['OBJ'].getLatitude())/oUavInfos['OBJ'].getAirspeed()) 
                                            for iUavId, oUavInfos in self.aliveUavList.items() if iUavId != uavId and oUavInfos['STATE'] & (Enum.INITIAL_STATE|Enum.SEARCHING)]
        if len(aDists) != 0 :
            aNearest = min(aDists, key = lambda i : i[1])

            aEnergyInf = min([i for i in aDists if self.aliveUavList[i[0]]['OBJ'].getEnergyAvailable() == 100] , key = lambda i : i[1])

            return self.aliveUavList[aNearest[0]] if aNearest[1]*1.2 < aEnergyInf[1] else self.aliveUavList[aEnergyInf[0]]
        return None
    # add the detected point into hazarzone area
    def putPointIntoZones(self, zoneId, detectedPoint):
        zonePoints = self.getDetectedZoneById(zoneId)
        if not zonePoints :
            print("The zoneId is wrong")
            return False
        zonePoints['area'].append(detectedPoint)
        return True

    ### For DetectedZone
    def getTrackingDirection(self, zoneId, detectedPoint):
        aArea = self.stateForCoordination['Tracing']['HazardZone_'+str(zoneId)]['area']
        aStartPoint = self.stateForCoordination['Tracing']['HazardZone_'+str(zoneId)]['startPoint']

        aDists = [(self.utils.distance(aStartPoint[1], aStartPoint[0], point.get_Longitude(), point.get_Latitude()),[point.get_Latitude(), point.get_Longitude()]) for point in aArea]
        aFarthestPoint = max(aDists, key=lambda x: x[0])[1]

        fDistWithSP = self.utils.distance(aStartPoint[1], aStartPoint[0], detectedPoint.get_Latitude(), detectedPoint.get_Longitude())
        fDistWithFP = self.utils.distance(aFarthestPoint[1], aFarthestPoint[0],detectedPoint.get_Latitude(), detectedPoint.get_Longitude())
        
        return 1 if fDistWithSP < fDistWithFP else -1

    def isAlreadyDetectedZone(self, detectedPoint):
        # 가장 가까운 점과의 거리가 일정거리 차이나면 새로운 존
        threshold = 0.10
        fShortestDist = sys.maxsize
        zoneId = 0
        for i in range(len(self.stateForCoordination['Tracing'])):
            for point in self.stateForCoordination['Tracing']['HazardZone_'+str(i)]['area']:
                fDist = self.utils.distance(point.get_Longitude(), point.get_Latitude(), detectedPoint.get_Longitude(), detectedPoint.get_Latitude()) 
                if fDist < fShortestDist:
                    zoneId = i; fShortestDist = fDist
        
        return zoneId if fShortestDist < threshold else -1

    def addNewDetectedZoneForTracking(self, detectedPoint):
        zoneId = len(self.stateForCoordination['Tracing'])
        self.stateForCoordination['Tracing']['HazardZone_'+str(zoneId)] = {'tracking_drones':[], 'cycled' : False, 'area':[], 'startPoint': [detectedPoint.get_Latitude(), detectedPoint.get_Longitude()]}
        self.stateForCoordination['Tracing']['HazardZone_'+str(zoneId)]['area'].append(detectedPoint)
        return zoneId

    def addNewDetectedZoneForPatrolling(self, detectedPoint):
        zoneId = len(self.stateForCoordination['Patrolling'])
        self.stateForCoordination['Patrolling']['SmokeZone_'+str(zoneId)] = {'point':[detectedPoint.get_Latitude(), detectedPoint.get_Longitude()], 'patrolling_drones' : []}
        return zoneId

    def getDetectedZoneById(self, zoneId):
        if 'HazardZone_'+str(zoneId) in self.stateForCoordination['Tracing'].keys():
            return self.stateForCoordination['Tracing']['HazardZone_'+str(zoneId)]
        return None

    def getDetectedZones(self):
        return self.stateForCoordination['Tracing']
    
    def getCenterPointInZone(self, zoneId):
        lat = 0
        lon = 0
        lenth = len( self.stateForCoordination['Tracing']['HazardZone_'+str(zoneId)]['area'])
        for point in self.stateForCoordination['Tracing']['HazardZone_'+str(zoneId)]['area'] :
            lat += point.get_Latitude()
            lon += point.get_Longitude()
        
        return [lat/lenth, lon/lenth]
    
    def numOfDroneInSameZone(self, zoneId):
        return len(self.stateForCoordination['Tracing']['HazardZone_'+str(zoneId)]['tracking_drones'])

    def estimateDetectedZone(self):
        for i in range(len(self.stateForCoordination['Tracing'])):
            all_coords = [[coord.get_Latitude(), coord.get_Longitude()] for coord in self.stateForCoordination['Tracing']['HazardZone_'+str(i)]['area']]
            hull = ConvexHull(np.asarray(all_coords, dtype=np.float32))
            if len(all_coords) < 3:
                print("Not that much to estimate")
                return
            self.stateForCoordination['Tracing']['HazardZone_'+str(i)]['area'] = [self.stateForCoordination['Tracing']['HazardZone_'+str(i)]['area'][j] for j in hull.vertices]

            # Create polygon object
            estimatedHazardZone = Polygon()

            for k in self.stateForCoordination['Tracing']['HazardZone_'+str(i)]['area']:
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
        if uavInfos['STATE_DETAIL'][Enum.INITIAL_STATE]['start_recovery_id'] == -1 :
            uavInfos['STATE_DETAIL'][Enum.INITIAL_STATE]['start_recovery_id']= pointId
        uavInfos['STATE_DETAIL'][Enum.INITIAL_STATE]['current_recovery_id']= pointId
        uavInfos['STATE_DETAIL'][Enum.INITIAL_STATE]['recovery_point'] = self.aRecoveryPoints[pointId]

    def removedUavUpdate(self, uavId):
        if uavId not in self.deadUavList.keys():
            self.returnState(uavId)
            oDeadUavInfos = self.aliveUavList.pop(uavId)
            self.deadUavList[uavId] = oDeadUavInfos

    def getIdealSpeed(self, iUavId):
        return self.utils.getIdealSpeed(self.aliveUavList[iUavId])

    def getDist(self, uavId1, uavId2):

        lat1 = self.aliveUavList[uavId1]['OBJ'].getLatitude()
        lon1 = self.aliveUavList[uavId1]['OBJ'].getLongitude()
        
        lat2 = self.aliveUavList[uavId2]['OBJ'].getLatitude()
        lon2 = self.aliveUavList[uavId2]['OBJ'].getLongitude()

        return self.utils.distance(lon1, lat1, lon2, lat2)

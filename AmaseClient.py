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

import Utils
import Drone
import State
import KeepInZoneInfo
import RecoveryZoneInfo
import Enum

class PrintLMCPObject(IDataReceived):
    def dataReceived(self, lmcpObject):
        print(lmcpObject.toXMLStr(""))

class SampleHazardDetector(IDataReceived):

    def __init__(self, tcpClient):
        self.__client = tcpClient
        self.utils = Utils.Utils(tcpClient)
        self.drones = State.State(self.utils)
        self.keepInZone = KeepInZoneInfo.KeepInZoneInfo()
        self.recoveryList = []
        self.iPhase = Enum.PHASE_SETTING

        # related estimate zone
        self.isStepOne = True
        self.isStepTwo = False
        self.isStepThree = False

        # array KeepInZonePoints
        self.aKeepInZones = None
    
    def dataReceived(self, lmcpObject):
        scenarioTime = 0

        if isinstance(lmcpObject, SessionStatus):
            scenarioTime = lmcpObject.get_ScenarioTime()

        if self.isStepOne and (scenarioTime >= 1190000 and scenarioTime <= 1200000):
            self.drones.estimateDetectedZone()

            self.isStepOne = False
            self.isStepTwo = True
        elif self.isStepTwo and (scenarioTime >= 2390000 and scenarioTime <= 2400000):
            self.drones.estimateDetectedZone()

            self.isStepTwo = False
            self.isStepThree = True
        elif self.isStepThree and (scenarioTime >= 3590000 and scenarioTime <= 3600000):
            self.drones.estimateDetectedZone()

            self.isStepThree = False


        if self.iPhase == Enum.PHASE_SETTING :
            if int(scenarioTime) == 0:
                print(" - - - - - PHASE : SETTING - - - - - ")
                if isinstance(lmcpObject, AirVehicleConfiguration):
                    print(" - Add new drone")
                    self.drones.addNewUAV(lmcpObject)
                    print(" - Done")
                elif isinstance(lmcpObject, AirVehicleState):
                    print(" - Get drone info")
                    self.drones.updateUAV(lmcpObject)
                    print(" - Done")
                elif isinstance(lmcpObject, KeepInZone):
                    print(" - Get keepinzone info")
                    self.keepInZone.updateKeepInZone(lmcpObject)
                    print(" - Done")
                    print(" - Read Dted data")
                    self.aKeepInZones = self.keepInZone.getPoints()
                    #self.utils.getElevations(self.aKeepInZones[0][0], self.aKeepInZones[0][1], self.aKeepInZones[2][0], self.aKeepInZones[2][1], 1 / 3600)
                    print(" - Done")

                elif isinstance(lmcpObject, RecoveryPoint):
                    print(" - Get recoverypoint info")
                    recoveryZone = RecoveryZoneInfo.RecoveryZoneInfo()
                    recoveryZone.updateRecoveryZone(lmcpObject)
                    self.recoveryList.append(recoveryZone)
                    print(" - Done")
                print(" - - - - - - - - - - - - - - - - - - ")
            else :
                print(" - - - - - PHASE : UPDATE - - - - - ")
                # calculate for initialsearch
                self.aRecoveryPoints = [[point.getLatitude(), point.getLongitude()] for point in self.recoveryList]
                print(" - Assign initialsearch path")
                self.drones.assignInitialSearchPath(self.aKeepInZones, self.aRecoveryPoints, Enum.INIT_START_NEAREST)
                print(" - Done")
                self.iPhase = Enum.PHASE_UPDATE
                print(" - - - - - - - - - - - - - - - - - - - ")
        elif self.iPhase == Enum.PHASE_UPDATE:
            if isinstance(lmcpObject, AirVehicleState):
                self.drones.updateUAV(lmcpObject)

                # check still in keepinzone
                self.drones.checkStillInKeep(self.aKeepInZones, lmcpObject.get_ID())

                # checkNeedToCharge
                self.drones.checkNeedToCharge(lmcpObject.get_ID())

                # make a command for drones, heading, azimuth update
                self.drones.updateUavAction(lmcpObject.get_ID())

                # find nice altitude
                heading, azimuth, elevation, altitude = self.drones.getUpdateInfos(lmcpObject.get_ID())

                # wind affect
                self.drones.moveFirezoneByWind(lmcpObject.get_ID())

                # send cmd to drone
                self.utils.sendHeadingAndAltitudeCmd(lmcpObject.get_ID(), heading, altitude)


                if type(azimuth) == dict :
                    if not self.drones.isScanning(lmcpObject.get_ID()) :
                        # self.utils.sendGimbalAzimuthAndElevationScanCmd(
                        #     lmcpObject.get_ID(), azimuth['start'], azimuth['end'], azimuth['rate'],
                        #     elevation, elevation, 0, 0)
                        self.utils.sendGimbalAzimuthAndElevationScanCmd(
                            lmcpObject.get_ID(), azimuth['start'], azimuth['end'], azimuth['rate'])
                        self.drones.setScanning(lmcpObject.get_ID(), True)
                else :
                    self.utils.sendGimbalAzimuthAndElevationCmd(lmcpObject.get_ID(), azimuth, elevation)
                    
                self.drones.checkTotalState(lmcpObject.get_ID())
            elif isinstance(lmcpObject, AirVehicleConfiguration):
                print(" - Add new drone during playing")
                self.drones.addNewUAV(lmcpObject)
                print(" - Done")
            elif isinstance(lmcpObject, HazardZoneDetection):
                # detection !!! 
                if self.drones.hazardzoneDetected(lmcpObject, scenarioTime) == Enum.ZONE_FIRE:

                    # find nice altitude
                    heading, azimuth, elevation, altitude = self.drones.getUpdateInfos(lmcpObject.get_DetectingEnitiyID())

                    # send cmd to drone
                    self.utils.sendHeadingAndAltitudeCmd(lmcpObject.get_DetectingEnitiyID(), heading, altitude)

                    if type(azimuth) == dict :
                        if not self.drones.isScanning(lmcpObject.get_DetectingEnitiyID()) :
                            # self.utils.sendGimbalAzimuthAndElevationScanCmd(
                            #     lmcpObject.get_ID(), azimuth['start'], azimuth['end'], azimuth['rate'],
                            #     elevation, elevation, 0, 0)
                            self.utils.sendGimbalAzimuthAndElevationScanCmd(
                                lmcpObject.get_DetectingEnitiyID(), azimuth['start'], azimuth['end'], azimuth['rate'])
                            self.drones.setScanning(lmcpObject.get_DetectingEnitiyID(), True)
                    else :
                        self.utils.sendGimbalAzimuthAndElevationCmd(lmcpObject.get_DetectingEnitiyID(), azimuth, elevation)
                else :
                    pass
            elif isinstance(lmcpObject, RemoveEntities):
                print(" - Update removed uav state")
                self.drones.removedUavUpdate(lmcpObject.get_EntityList()[0])
                print(" - Done")

#################
## Main
#################

if __name__ == '__main__':
    myHost = 'localhost'
    myPort = 5555
    amaseClient = AmaseTCPClient(myHost, myPort)
    #amaseClient.addReceiveCallback(PrintLMCPObject())
    amaseClient.addReceiveCallback(SampleHazardDetector(amaseClient))

    try:
        # make a threaded client, listen until a keyboard interrupt (ctrl-c)
        #start client thread
        amaseClient.start()

        while True:
            #wait for keyboard interrupt
            pass
    except KeyboardInterrupt as ki:
        print("Stopping amase tcp client")
    except Exception as ex:
        print(ex)
    amaseClient.stop()

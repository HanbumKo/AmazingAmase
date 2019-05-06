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
from terrian.TerrianService import TerrianService
from terrian.DTEDTile import DTEDTile
from afrl.cmasi.AbstractGeometry import AbstractGeometry
from afrl.cmasi.AbstractZone import AbstractZone
from afrl.cmasi.WeatherReport import WeatherReport
from afrl.cmasi.searchai.HazardZone import HazardZone
from afrl.cmasi.RemoveEntities import RemoveEntities
from afrl.cmasi.perceive.EntityPerception import EntityPerception
from afrl.cmasi.EntityConfiguration import EntityConfiguration
import math

import Enum

DTEDPATH = './dted2/'

class Utils():
    def __init__(self, tcpClient):
        self.__client = tcpClient
        self.terrian_service = TerrianService()
        # Dted file read
        self.terrian_service.addDirectory(DTEDPATH)

    # Calculate distance
    def distance(self, uav1lon, uav1lat, uav2lon, uav2lat):
        return math.sqrt((uav1lon - uav2lon) ** 2 + (uav1lat - uav2lat) ** 2)

    def getHeadingToDest(self, startLat, startLon, endLat, endLon):
        dy = endLat - startLat
        dx = endLon - startLon

        heading = math.degrees(math.atan2(dx,dy))

        return heading 

    def getLatLon(self, lat, lon, mDist, azimuth):
        return self.terrian_service.getLatLon(lat,lon, mDist, azimuth)
    
    def getElevation(self, lat, lon):
        return self.terrian_service.getElevation(lat,lon)

    def getElevations(self, ullat, ullon, lrlat, lrlon, arcStep):
        return self.terrian_service.getElevations(ullat, ullon, lrlat, lrlon, arcStep)

    def getLineElevs(self, lat1, lon1, lat2, lon2):
        return self.terrian_service.getLineElevs( lat1, lon1, lat2, lon2, 2)

    def getTheta(self, altitude, maxRange):
        return -(math.degrees(math.asin(altitude/maxRange))) - 5
    
    def getOriginalDirection(self, uavInfos):
        heading = uavInfos['OBJ'].getHeading()
        course = uavInfos['OBJ'].getCourse()
        originalDirection = 0

        if heading < 0 :
            heading += 360 

        if course < 0 :
            course += 360 
        
        gap =  abs((heading-course)/2)
        
        if gap > 90 :
            gap = 180 - gap
        
        if heading > course:
            originalDirection = course + gap
        elif heading < course :
            originalDirection = course - gap
        else :
            originalDirection = course

        return originalDirection, gap

    def getIdealSpeed(self, uavInfos) :
        fAS = uavInfos['OBJ'].getAirspeed()
        fGS = uavInfos['OBJ'].getGroundspeed()
            
        
        if uavInfos['STATE'] & (Enum.TRACING|Enum.APPROACHING|Enum.TURNING2):
            # Groundspeed -> 15
            return 15 + ( fAS - fGS )
        return 25 + ( fAS - fGS )

    def turningForChangingAlt(self, altitude, uavInfos ) :

        mission_command = MissionCommand()
        mission_command.set_FirstWaypoint(1)
        mission_command.set_VehicleID(uavInfos['OBJ'].getID())
        mission_command.set_Status(CommandStatusType.Pending)
        mission_command.set_CommandID(1)

        way_point_list = mission_command.get_WaypointList()

        way_point = Waypoint()
        way_point.set_Latitude(uavInfos['OBJ'].getLatitude())
        way_point.set_Longitude(uavInfos['OBJ'].getLongitude())
        way_point.set_Speed(15)
        way_point.set_Altitude(altitude)
        way_point.set_AltitudeType(AltitudeType.MSL)
        way_point.set_Number(1)
        way_point.set_NextWaypoint(2)
        way_point.set_SpeedType(SpeedType.Airspeed)
        way_point.set_ClimbRate(0)
        way_point.set_TurnType(TurnType.TurnShort)
        way_point.set_ContingencyWaypointA(0)
        way_point.set_ContingencyWaypointB(0)

        way_point_list.append(way_point)

        self.__client.sendLMCPObject(mission_command)

        uavInfos['STATE_DETAIL'][Enum.TURNING2]['altitude'] = altitude
        uavInfos['STATE_DETAIL'][Enum.TURNING2]['prev_state'] = uavInfos['STATE']
        uavInfos['STATE'] = Enum.TURNING2

    def go_way_points(self, id, iStartIdx, aPoints, Speed=15, ClimbRate=0):
        mission_command = MissionCommand()
        mission_command.set_FirstWaypoint(1)
        mission_command.set_VehicleID(id)
        mission_command.set_Status(CommandStatusType.Pending)
        mission_command.set_CommandID(1)

        way_point_list = mission_command.get_WaypointList()

        for i in range(len(aPoints)):
            
            alti = self.terrian_service.getElevation(aPoints[i][0], aPoints[i][1])

            way_point = Waypoint()
            way_point.set_Latitude(aPoints[i][0])
            way_point.set_Longitude(aPoints[i][1])
            way_point.set_Speed(Speed)
            way_point.set_Altitude(alti)
            way_point.set_AltitudeType(AltitudeType.MSL)
            way_point.set_Number(i+1)
            way_point.set_NextWaypoint(i+2)
            way_point.set_SpeedType(SpeedType.Airspeed)
            way_point.set_ClimbRate(ClimbRate)
            way_point.set_TurnType(TurnType.TurnShort)
            way_point.set_ContingencyWaypointA(0)
            way_point.set_ContingencyWaypointB(0)
            way_point_list.append(way_point)

        self.__client.sendLMCPObject(mission_command)

    def send_loiter(self, vehicle_id, location, radius, is_clockwise, axis=0, length=0, duration=100000, Speed=15):
        vehicle_action_command = VehicleActionCommand()
        vehicle_action_command.set_VehicleID(vehicle_id)
        vehicle_action_command.set_Status(CommandStatusType.Pending)
        vehicle_action_command.set_CommandID(1)

        loiter_action = LoiterAction()
        loiter_action.set_LoiterType(LoiterType.Circular)
        loiter_action.set_Radius(radius * 100000)
        loiter_action.set_Axis(axis)
        loiter_action.set_Length(length)
        if is_clockwise:
            loiter_action.set_Direction(LoiterDirection.Clockwise)
        else:
            loiter_action.set_Direction(LoiterDirection.CounterClockwise)
        loiter_action.set_Duration(duration)
        loiter_action.set_Airspeed(Speed)
        loiter_action.set_Location(location)
        vehicle_action_command.get_VehicleActionList().append(loiter_action)

        self.__client.sendLMCPObject(vehicle_action_command)

    def send_estimate_report(self, estimated_hazardzone, estimated_hazardzone_ID, growthRate=0, zoneDirection=0, zoneSpeed=0):
        # Setting up the mission to send to the UAV
        hazard_estimate_report = HazardZoneEstimateReport()
        hazard_estimate_report.set_EstimatedZoneShape(estimated_hazardzone)

        # 기존엔 상수로 되어있었는데, 받아온 파라미터를 넣어 헤저드 존 별 차이를 둔다.
        hazard_estimate_report.set_UniqueTrackingID(estimated_hazardzone_ID)
        hazard_estimate_report.set_EstimatedGrowthRate(growthRate)
        hazard_estimate_report.set_PerceivedZoneType(HazardType.Fire)
        hazard_estimate_report.set_EstimatedZoneDirection(zoneDirection)
        hazard_estimate_report.set_EstimatedZoneSpeed(zoneSpeed)

        # Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPObject(hazard_estimate_report)

    def sendHeadingAndAltitudeCmd(self, uav_id, heading, altitude, speed):
        vehicleActionCommand = VehicleActionCommand()
        vehicleActionCommand.set_VehicleID(uav_id)
        vehicleActionCommand.set_Status(CommandStatusType.Pending)
        vehicleActionCommand.set_CommandID(Enum.CMD_DRONE_HEADING_ALT)

        flight = FlightDirectorAction()
        flight.set_Heading(heading)
        flight.set_Altitude(altitude)
        flight.set_Speed(speed)
        vehicleActionCommand.get_VehicleActionList().append(flight)

        self.__client.sendLMCPObject(vehicleActionCommand)

    def sendGimbalAzimuthAndElevationCmd(self, uav_id, Azitmuth, Elevation):
        vehicle_action_command = VehicleActionCommand()
        vehicle_action_command.set_VehicleID(uav_id)
        vehicle_action_command.set_Status(CommandStatusType.Pending)
        vehicle_action_command.set_CommandID(Enum.CMD_GIMBAL_SET)

        # gimbalangleaction
        gimbalAngle_action = GimbalAngleAction()
        gimbalAngle_action.set_PayloadID(1)
        gimbalAngle_action.set_Azimuth(Azitmuth)
        gimbalAngle_action.set_Elevation(Elevation)

        vehicle_action_command.get_VehicleActionList().append(gimbalAngle_action)

        self.__client.sendLMCPObject(vehicle_action_command)

    def sendGimbalAzimuthAndElevationScanCmd(self, uav_id, StartAzimuth=0, EndAzimuth=0, AzimuthSlewRate=0, StartElevation=-50, EndElevation=-50, ElevationSlewRate=5, cycles=0):


        vehicle_action_command = VehicleActionCommand()
        vehicle_action_command.set_VehicleID(uav_id)
        vehicle_action_command.set_Status(CommandStatusType.Pending)
        vehicle_action_command.set_CommandID(Enum.CMD_GIMBAL_SCAN)

        # start-end 까지 스캔하는 명령
        gimbalScan_action = GimbalScanAction()
        gimbalScan_action.set_PayloadID(1)  # 짐발 센서 id

        gimbalScan_action.set_StartAzimuth(StartAzimuth)
        gimbalScan_action.set_EndAzimuth(EndAzimuth)
        gimbalScan_action.set_AzimuthSlewRate(AzimuthSlewRate)

        gimbalScan_action.set_StartElevation(StartElevation)
        gimbalScan_action.set_EndElevation(EndElevation)
        gimbalScan_action.set_ElevationSlewRate(ElevationSlewRate)  # 고도 한번 스캔 시간

        gimbalScan_action.set_Cycles(cycles)

        vehicle_action_command.get_VehicleActionList().append(gimbalScan_action)

        # Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPObject(vehicle_action_command)

    def sendWaypointsCmd(self, uavId, start_lat, start_lon, end_lat, end_lon, speed) :
        iHeightForSmokeDetect = 1500
        iHeightForFireDetect = 200
        mission_command = MissionCommand()
        mission_command.set_FirstWaypoint(1)
        mission_command.set_VehicleID(uavId)
        mission_command.set_Status(CommandStatusType.Pending)
        mission_command.set_CommandID(1)

        way_point_list = mission_command.get_WaypointList()
        
        aElevs, aCoords = self.getLineElevs(start_lat, start_lon, end_lat, end_lon)

        for i in range(len(aCoords)):
            way_point = Waypoint()
            way_point.set_Latitude(aCoords[i][0])
            way_point.set_Longitude(aCoords[i][1])
            way_point.set_Speed(speed)
            way_point.set_Altitude(aElevs[i]+ iHeightForFireDetect)
            way_point.set_AltitudeType(AltitudeType.MSL)
            way_point.set_Number(i+1)
            way_point.set_NextWaypoint(i+2)
            way_point.set_SpeedType(SpeedType.Airspeed)
            way_point.set_ClimbRate(0)
            way_point.set_TurnType(TurnType.TurnShort)
            way_point.set_ContingencyWaypointA(0)
            way_point.set_ContingencyWaypointB(0)

            way_point_list.append(way_point)

        self.__client.sendLMCPObject(mission_command)

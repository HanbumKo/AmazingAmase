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

class Utils():
    def __init__(self):
        self.terrian_service = TerrianService()
        pass

    # Calculate distance
    def distance(self, uav1lon, uav1lat, uav2lon, uav2lat):
        return math.sqrt((uav1lon - uav2lon) ** 2 + (uav1lat - uav2lat) ** 2)

    def sendGimbalAngleChangeCmd(self, tcp_client, uav_id, elevation, azimuth):
        vehicle_action_command = VehicleActionCommand()
        vehicle_action_command.set_VehicleID(uav_id)
        vehicle_action_command.set_Status(CommandStatusType.Pending)
        vehicle_action_command.set_CommandID(2)

        # gimbalangleaction
        gimbalAngle_action = GimbalAngleAction()
        gimbalAngle_action.set_PayloadID(1)
        gimbalAngle_action.set_Elevation(elevation)
        gimbalAngle_action.set_Azimuth(azimuth)

        vehicle_action_command.get_VehicleActionList().append(gimbalAngle_action)

        tcp_client.sendLMCPObject(vehicle_action_command)

    def sendGimbalScanCmd(self, tcp_client, uav_id, start_angle, end_angle, rate, cycles=0, startAzimuth=0, EndAzimuth=30, AzimuthSlewRate=10):

        vehicle_action_command = VehicleActionCommand()
        vehicle_action_command.set_VehicleID(uav_id)
        vehicle_action_command.set_Status(CommandStatusType.Pending)
        vehicle_action_command.set_CommandID(2)
        # start-end 까지 스캔하는 명령
        gimbalScan_action = GimbalScanAction()
        gimbalScan_action.set_PayloadID(1)  # 짐발 센서 id
        gimbalScan_action.set_ElevationSlewRate(rate)  # 고도 한번 스캔 시간

        gimbalScan_action.set_StartElevation(start_angle)
        gimbalScan_action.set_EndElevation(end_angle)
        gimbalScan_action.set_Cycles(cycles)
        gimbalScan_action.set_StartAzimuth(startAzimuth)
        gimbalScan_action.set_EndAzimuth(EndAzimuth)
        gimbalScan_action.set_AzimuthSlewRate(AzimuthSlewRate)

        vehicle_action_command.get_VehicleActionList().append(gimbalScan_action)

        # Sending the Vehicle Action Command message to AMASE to be interpreted
        tcp_client.sendLMCPObject(vehicle_action_command)

    def go_way_point(self, tcp_client, id, Longitude, Latitude, Speed=15, ClimbRate=0):
        mission_command = MissionCommand()
        mission_command.set_FirstWaypoint(1)
        mission_command.set_VehicleID(id)
        mission_command.set_Status(CommandStatusType.Pending)
        mission_command.set_CommandID(1)

        way_point_list = mission_command.get_WaypointList()

        alti = self.terrian_service.getElevation(Latitude, Longitude)

        way_point = Waypoint()
        way_point.set_Latitude(Latitude)
        way_point.set_Longitude(Longitude)
        way_point.set_Speed(Speed)
        way_point.set_Altitude(alti)
        way_point.set_AltitudeType(AltitudeType.MSL)
        way_point.set_Number(1)
        way_point.set_NextWaypoint(2)
        way_point.set_SpeedType(SpeedType.Airspeed)
        way_point.set_ClimbRate(ClimbRate)
        way_point.set_TurnType(TurnType.TurnShort)
        way_point.set_ContingencyWaypointA(0)
        way_point.set_ContingencyWaypointB(0)
        way_point_list.append(way_point)

        tcp_client.sendLMCPObject(mission_command)

    def send_loiter(self, tcp_client, vehicle_id, location, radius, is_clockwise, axis=0, length=0, duration=100000, Speed=15):
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
        tcp_client.sendLMCPObject(vehicle_action_command)

    def send_estimate_report(self, tcp_client, estimated_hazardzone, estimated_hazardzone_ID, growthRate=0, zoneDirection=0, zoneSpeed=0):
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
        tcp_client.sendLMCPObject(hazard_estimate_report)


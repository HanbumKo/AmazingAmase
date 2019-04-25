import itertools

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
from terrian.TerrianService import TerrianService
from terrian.DTEDTile import DTEDTile
from afrl.cmasi.RemoveEntities import RemoveEntities
from afrl.cmasi.perceive.EntityPerception import EntityPerception
from afrl.cmasi.EntityConfiguration import EntityConfiguration
from scipy.spatial import ConvexHull

import random as rd
import math
import numpy as np
import sys

# Check gimbal sensor's state.
HIGH = 1
MID = 2
LOW = 3
DTEDPATH = './dted2/'


class Competition(IDataReceived):
    def __init__(self, tcp_client):
        self.__waypointAlt1 = 2300
        self.__safeAlt = 2300
        self.__initial_speed = 15

        # Parameters
        self.__num_of_row = 3
        self.__initial_strategy = 0  # 1 -> loiter, 0 -> rec, 2 -> Voronoi
        self.__loiter_clockwise = 1  # 1 -> Clockwise, 0 -> Counter clockwise
        self.__maxDist = 500  # distance from uav point to front point of the going way, meter
        self.__maxGap = 600  # Maximum gap between altitude of uav and land
        self.__minGap = 500  # Minimum gap between altitude of uav and land
        self.__maxAlti_firzone = 1200  # the altitude of fire zone
        self.__gimbal_searchMode = -90
        self.__gimbal_trackingMode = -90
        self.__default_speed = 15
        self.__radius = 0.0005
        self.__hazardzone_threshold = 0.2
        self.__maxPoints = 40
        self.__minSpeed = 15

        # Basic Variable
        self.__client = tcp_client
        self.__estimatedHazardZone = Polygon()
        self.__terrian_service = TerrianService()
        self.__scenarioTime = 0
        self.__groupflag1 = True
        self.__groupflag2 = True

        # Phase Flag
        self.__phase0 = True
        self.__phase1 = False
        self.__phase2 = False
        self.__phase3 = False

        # Flag Variable
        self.__step_one = True
        self.__step_two = False
        self.__step_three = False
        self.__first_detection = False
        self.__distance_caculated = False
        self.__who_closest = False
        self.__keep_info = False
        self.__first_loiter = False
        self.__do_not_enter = False
        self.__uav_array_clear = False

        # Entity Information
        self.__entity_id = []
        self.__entity_state = []

        # UAV Information
        self.__uav_object = []
        self.__uav_location = []
        self.__uav_current_status = []
        self.__uav_polygon = []
        self.__uav_all = []
        self.__num_of_uav = 0
        self.__is_tracking = []
        self.__uav_max_speed = []
        self.__uav_id = []
        self.__uav_idx = []
        self.__uav_in_the_zone = []
        self.__uav_zigzag_heading = []
        self.__uav_gap_check_time = []
        self.__uav_first_gap_check = []
        self.__uav_ideal_altitude = []
        self.__uav_gimbal_state = []  # 1 - STARE, keep one elevation, # 2 - SCAN, look over changing elevation.
        self.__uav_total_state = []
        self.__added_uav = []
        self.__removed_uav = []

        # UAV distance to each vertex
        self.__distance = []
        self.__optimal_location = []
        self.__time = []

        # Detected UAV location
        self.__detected_uav_location = []

        # KeepInZone Information
        self.__center_lat = 0
        self.__center_lon = 0
        self.__width = 0
        self.__height = 0
        self.__top = 0
        self.__bottom = 0
        self.__left = 0
        self.__right = 0
        self.__keep_in_zone = []
        self.__num_of_uav_per_row = 0
        self.__num_of_full_uav_row = 0
        self.__remainder = 0

        # left top, right top, left bottom, right bottom
        self.__closest_vertex = []

        # Polygon Information
        self.__polygon_center_lon = 0.0
        self.__polygon_center_lat = 0.0
        self.__min_size = 0.0

        self.__for_keep_going = []
        self.__keep_going_flag = []
        self.__keep_going_stop = False

        # For tracking
        self.__uav_detection_count = []
        self.__uav_detection_time = []
        self.__uav_detection = []
        self.__uav_heading = []
        self.__uav_circle_way = []
        self.__goto_center = []
        self.__uav_change_heading = []
        self.__uav_hazardzone = []
        self.__hazardzones = []
        self.__hazardzone_detect_count = []
        self.__hazardzone_center_point = []
        self.__hazardzone_firstDetect_point = []

        # For recovery point
        self.__recovery_points = []
        self.__closest_recovery_point = []
        self.__climb_start_time = 0

        # Dted file read
        self.__terrian_service.addDirectory(DTEDPATH)

        # For group
        self.__grouped_uav_idx = []  # Will be [[0, 1, 4, ...], [2, 3, 5, ...], [6, 8, 10, ...], ...]
        self.__grouped_uav_id = []  # Will be [[1, 2, 5, ...], [3, 7, 11, ...], [4, 22, 24, ...], ...]

        # For recovery
        self.__is_recharging = []
        self.__saved_location = []

        self.__previousPolygon = []

        self.__msgState = 0

    # Function to make group
    def grouping(self):
        for recovery in self.__recovery_points:
            one_group_idx = []
            one_group_id = []
            for uav_idx in range(self.__num_of_uav):
                if self.distance(self.__uav_total_state[uav_idx][4][0], self.__uav_total_state[uav_idx][4][1],
                                 recovery[0],
                                 recovery[1]) < recovery[2] / 100000:
                    one_group_idx.append(uav_idx)
                    one_group_id.append(self.__uav_id[uav_idx])
            self.__grouped_uav_idx.append(one_group_idx)
            self.__grouped_uav_id.append(one_group_id)

        print("Grouped UAV idx = ", self.__grouped_uav_idx)
        print("Grouped UAV id = ", self.__grouped_uav_id)
        print("End of grouping")

    def uav_update(self, uav_object):
        uav_id = uav_object.get_ID()
        uav_idx = self.__uav_id.index(uav_id)

        # Location
        uav_lon = uav_object.get_Location().get_Longitude()
        uav_lat = uav_object.get_Location().get_Latitude()
        uav_alt = uav_object.get_Location().get_Altitude()
        self.__uav_total_state[uav_idx][4] = [uav_lon, uav_lat, uav_alt]

        # State
        # self.__uav_total_state[uav_idx][5]

        # Info of state
        # self.__uav_total_state[uav_idx][6]

        # Heading
        self.__uav_total_state[uav_idx][7] = uav_object.get_Heading()

        # Gimbal elevation
        self.__uav_total_state[uav_idx][8] = uav_object.get_PayloadStateList()[0].get_Elevation()

        # Gimbal azimuth
        self.__uav_total_state[uav_idx][9] = uav_object.get_PayloadStateList()[0].get_Azimuth()

        # Air speed
        self.__uav_total_state[uav_idx][10] = uav_object.get_Airspeed()

        # Wind speed
        self.__uav_total_state[uav_idx][11] = uav_object.get_WindSpeed()

        # Wind direction
        self.__uav_total_state[uav_idx][12] = uav_object.get_WindDirection()

        # Polygon list
        # self.__uav_total_state[uav_idx][13] = []

        # Which recovery group
        # self.__uav_total_state[uav_idx][14] = []

        # what time detect a hazard zone
        # self.__uav_total_state[uav_idx][15] = []

    # Function to know the number of UAVs
    def uav_init(self, uav_object):
        list_to_be_attended = []
        list_to_be_attended.append(uav_object.get_ID())
        list_to_be_attended.append(uav_object.get_PayloadConfigurationList()[2].MaxRange)
        list_to_be_attended.append(uav_object.get_MaximumSpeed())
        list_to_be_attended.append(uav_object.get_NominalFlightProfile().get_EnergyRate())
        for i in range(11):
            list_to_be_attended.append(0)
        self.__uav_id.append(uav_object.get_ID())
        self.__uav_idx.append(self.__num_of_uav)
        list_to_be_attended[13] = []
        list_to_be_attended.append([])  # recovery ID
        list_to_be_attended.append(0)  # detection time

        self.__uav_total_state.append(list_to_be_attended)
        self.__num_of_uav = self.__num_of_uav + 1

        print("End of UAV configuration")

    # Function for getting information about KeepInZone
    def keep_in_zone_info(self, lmcpObject):
        # Variables
        boundary = lmcpObject.get_Boundary()

        center_point = boundary.get_CenterPoint()
        self.__center_lat = center_point.get_Latitude()
        self.__center_lon = center_point.get_Longitude()
        self.__width = boundary.get_Width() / 100000 + 0.1
        self.__height = boundary.get_Height() / 100000 - 0.1

        self.__left = self.__center_lon - (self.__width / 2)
        self.__right = self.__center_lon + (self.__width / 2)
        self.__top = self.__center_lat + (self.__height / 2)
        self.__bottom = self.__center_lat - (self.__height / 2)

        print("geting dted data...")
        self.__terrian_service.getElevations(self.__top, self.__left, self.__bottom, self.__right, 1 / 3600)
        print("done!")

    # Calculate distance
    def distance(self, uav1lon, uav1lat, uav2lon, uav2lat):
        return math.sqrt((uav1lon - uav2lon) ** 2 + (uav1lat - uav2lat) ** 2)

    """
    # Function for calculating UAVs distance to KeepInZone
    def calculate_from_uav(self):
        self.__distance = [[0 for x in range(self.__num_of_uav)] for y in range(self.__num_of_uav)]
        for i in range(self.__num_of_uav):
            for j in range(self.__num_of_uav):
                self.__distance[i][j] = math.sqrt((self.__keep_in_zone[i][0] - self.__uav_location[j][0]) ** 2 + (
                        self.__keep_in_zone[i][1] - self.__uav_location[j][1]) ** 2)
        print("End of calculate_from_uav")
    """

    # Decide optimal drone location using row parameter
    def decide_location(self, num_of_row):
        # 거리를 저장해놓는 배열 구한다.
        distances_sum = [0 for i in range(self.__num_of_uav)]
        for i in range(len(distances_sum)):
            for recovery in self.__recovery_points:
                distances_sum[i] = distances_sum[i] + self.distance(self.__keep_in_zone[i][0],
                                                                    self.__keep_in_zone[i][1],
                                                                    recovery[0], recovery[1])

        # 거리의 합이 가장 큰 구역의 인덱스부터 저장하는 배열을 구한다.
        first_to_last = [0 for i in range(self.__num_of_uav)]
        for i in range(len(first_to_last)):
            max_idx = distances_sum.index(max(distances_sum))
            first_to_last[i] = max_idx
            distances_sum[max_idx] = -1

        print("max distance to min distance")
        print(first_to_last)

        self.__optimal_location = [0 for i in range(self.__num_of_uav)]

        # 거리의 합이 가장 큰 존부터 자신에게 가장 가까운 uav를 선택한다.
        for zone_idx in first_to_last:
            min_idx = self.__distance[zone_idx].index(min(self.__distance[zone_idx]))
            self.__optimal_location[zone_idx] = min_idx
            for i in range(len(self.__distance)):
                self.__distance[i][min_idx] = sys.maxsize

        """
        for i in range(self.__num_of_uav):
            idx = self.__time[i].index(min(self.__time[i]))
            self.__optimal_location.append(idx)
            for j in range(self.__num_of_uav):
                self.__time[j][idx] = sys.maxsize
        """
        print("optimal", self.__optimal_location)
        print("End of decide_location_with_row")

    def sendGimbalAngleChangeCmd(self, uav_id, elevation):
        uav_id_index = self.__uav_id.index(uav_id)

        vehicle_action_command = VehicleActionCommand()
        vehicle_action_command.set_VehicleID(uav_id)
        vehicle_action_command.set_Status(CommandStatusType.Pending)
        vehicle_action_command.set_CommandID(2)

        # gimbalangleaction
        gimbalAngle_action = GimbalAngleAction()
        gimbalAngle_action.set_PayloadID(1)
        gimbalAngle_action.set_Elevation(elevation)
        gimbalAngle_action.set_Azimuth(self.__uav_total_state[uav_id_index][9])

        vehicle_action_command.get_VehicleActionList().append(gimbalAngle_action)

        self.__client.sendLMCPObject(vehicle_action_command)

    def sendGimbalScanCmd(self, uav_id, start_angle, end_engle, rate):

        vehicle_action_command = VehicleActionCommand()
        vehicle_action_command.set_VehicleID(uav_id)
        vehicle_action_command.set_Status(CommandStatusType.Pending)
        vehicle_action_command.set_CommandID(2)
        # start-end 까지 스캔하는 명령
        gimbalScan_action = GimbalScanAction()
        gimbalScan_action.set_PayloadID(1)  # 짐발 센서 id
        gimbalScan_action.set_ElevationSlewRate(rate)  # 고도 한번 스캔 시간

        gimbalScan_action.set_StartElevation(start_angle)
        gimbalScan_action.set_EndElevation(end_engle)
        gimbalScan_action.set_Cycles(0)
        gimbalScan_action.set_StartAzimuth(0)
        gimbalScan_action.set_EndAzimuth(30)
        gimbalScan_action.set_AzimuthSlewRate(10)

        vehicle_action_command.get_VehicleActionList().append(gimbalScan_action)

        # Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPObject(vehicle_action_command)

    # Function for setting first way point
    def first_way_point(self):
        # Is loiter?
        if self.__initial_strategy == 1:
            self.decide_location(self.__num_of_row)
            self.initial_loiter(self.__num_of_row)
        elif self.__initial_strategy == 2:
            print("Voronoi")
            # self.Voronoi()
        elif self.__initial_strategy == 0:
            self.decide_location(self.__num_of_row)
            self.initial_lec_upgrade(self.__num_of_row)

        print("End of first_way_point")

    # Initial strategy using time and row parameter
    def initial_lec(self, num_of_row):
        # From first row to (Last-1) row
        keep_num = 0
        for row in range(self.__num_of_full_uav_row):
            for i in range(self.__num_of_uav_per_row):
                width_term = self.__width / self.__num_of_uav_per_row
                height_term = self.__height / self.__num_of_row

                mission_command = MissionCommand()
                mission_command.set_FirstWaypoint(1)
                mission_command.set_VehicleID(self.__uav_id[self.__optimal_location[keep_num]])
                mission_command.set_Status(CommandStatusType.Pending)
                mission_command.set_CommandID(1)

                way_point_list = mission_command.get_WaypointList()

                keep_lon = self.__keep_in_zone[keep_num][0]
                keep_lat = self.__keep_in_zone[keep_num][1]

                uav_id_index = self.__optimal_location[keep_num]
                line1_altis, coords = self.__terrian_service.getLineElevs(self.__uav_total_state[uav_id_index][4][1],
                                                                          self.__uav_total_state[uav_id_index][4][0],
                                                                          keep_lat, keep_lon, 2)
                line1_alti = self.__safeAlt
                # Position 1
                way_point = Waypoint()
                way_point.set_Latitude(keep_lat)
                way_point.set_Longitude(keep_lon)
                way_point.set_Altitude(line1_alti)
                way_point.set_Number(1)
                way_point.set_NextWaypoint(2)
                way_point.set_Speed(self.__default_speed)
                way_point.set_SpeedType(SpeedType.Airspeed)
                way_point.set_ClimbRate(0)
                way_point.set_TurnType(TurnType.TurnShort)
                way_point.set_ContingencyWaypointA(0)
                way_point.set_ContingencyWaypointB(0)
                way_point_list.append(way_point)

                line2_altis, coords = self.__terrian_service.getLineElevs(keep_lat, keep_lon,
                                                                          keep_lat + height_term / 4,
                                                                          keep_lon + width_term / 4, 2)
                line2_alti = self.__safeAlt
                # Position 2
                way_point = Waypoint()
                way_point.set_Latitude(keep_lat + height_term / 8 * 3)
                way_point.set_Longitude(keep_lon + width_term / 8 * 3)
                way_point.set_Altitude(line2_alti)
                way_point.set_Number(2)
                way_point.set_NextWaypoint(3)
                way_point.set_Speed(self.__default_speed)
                way_point.set_SpeedType(SpeedType.Airspeed)
                way_point.set_ClimbRate(0)
                way_point.set_TurnType(TurnType.TurnShort)
                way_point.set_ContingencyWaypointA(0)
                way_point.set_ContingencyWaypointB(0)
                way_point_list.append(way_point)

                line3_altis, coords = self.__terrian_service.getLineElevs(keep_lat + height_term / 4,
                                                                          keep_lon + width_term / 4,
                                                                          keep_lat + height_term / 4,
                                                                          keep_lon - width_term / 4, 2)
                line3_alti = self.__safeAlt
                # Position 3
                way_point = Waypoint()
                way_point.set_Latitude(keep_lat + height_term / 8 * 3)
                way_point.set_Longitude(keep_lon - width_term / 8 * 3)
                way_point.set_Altitude(line3_alti)
                way_point.set_Number(3)
                way_point.set_NextWaypoint(4)
                way_point.set_Speed(self.__default_speed)
                way_point.set_SpeedType(SpeedType.Airspeed)
                way_point.set_ClimbRate(0)
                way_point.set_TurnType(TurnType.TurnShort)
                way_point.set_ContingencyWaypointA(0)
                way_point.set_ContingencyWaypointB(0)
                way_point_list.append(way_point)

                line4_altis, coords = self.__terrian_service.getLineElevs(keep_lat + height_term / 4,
                                                                          keep_lon - width_term / 4,
                                                                          keep_lat - height_term / 4,
                                                                          keep_lon - width_term / 4, 2)
                line4_alti = self.__safeAlt
                # Position 4
                way_point = Waypoint()
                way_point.set_Latitude(keep_lat - height_term / 8 * 3)
                way_point.set_Longitude(keep_lon - width_term / 8 * 3)
                way_point.set_Altitude(line4_alti)
                way_point.set_Number(4)
                way_point.set_NextWaypoint(5)
                way_point.set_Speed(self.__default_speed)
                way_point.set_SpeedType(SpeedType.Airspeed)
                way_point.set_ClimbRate(0)
                way_point.set_TurnType(TurnType.TurnShort)
                way_point.set_ContingencyWaypointA(0)
                way_point.set_ContingencyWaypointB(0)
                way_point_list.append(way_point)

                line5_altis, coords = self.__terrian_service.getLineElevs(keep_lat - height_term / 4,
                                                                          keep_lon - width_term / 4,
                                                                          keep_lat - height_term / 4,
                                                                          keep_lon + width_term / 4, 2)
                line5_alti = self.__safeAlt
                # Position 5
                way_point = Waypoint()
                way_point.set_Latitude(keep_lat - height_term / 8 * 3)
                way_point.set_Longitude(keep_lon + width_term / 8 * 3)
                way_point.set_Altitude(line5_alti)
                way_point.set_Number(5)
                way_point.set_NextWaypoint(2)
                way_point.set_Speed(self.__default_speed)
                way_point.set_SpeedType(SpeedType.Airspeed)
                way_point.set_ClimbRate(0)
                way_point.set_TurnType(TurnType.TurnShort)
                way_point.set_ContingencyWaypointA(0)
                way_point.set_ContingencyWaypointB(0)
                way_point_list.append(way_point)

                loiter_action = LoiterAction()
                loiter_action.set_LoiterType(LoiterType.Circular)
                loiter_action.set_Radius(width_term / 5 * 100000)
                loiter_action.set_Axis(0)
                loiter_action.set_Length(0)
                loiter_action.set_Direction(LoiterDirection.Clockwise)
                loiter_action.set_Duration(100000)
                loiter_action.set_Airspeed(30)

                location = Location3D()
                location.set_AltitudeType(AltitudeType.MSL)
                location.set_Altitude(line1_alti)
                location.set_Longitude(keep_lon)
                location.set_Latitude(keep_lat)
                loiter_action.set_Location(location)

                way_point.VehicleActionList.append(loiter_action)

                self.__client.sendLMCPObject(mission_command)

                keep_num = keep_num + 1

        """
        for i in range(self.__num_of_uav):
            self.go_way_point(self.__optimal_location[i]+1, self.__keep_in_zone[i][0], self.__keep_in_zone[i][1], 0)

        print("To be implemented...(initialstrategy_with_row"))
        """

        print("End of initialstrategy_with_row")

    def initial_lec2(self, num_of_row):
        # From first row to (Last-1) row
        keep_num = 0
        for row in range(self.__num_of_full_uav_row):
            for i in range(self.__num_of_uav_per_row):
                width_term = self.__width / self.__num_of_uav_per_row
                height_term = self.__height / self.__num_of_row

                mission_command = MissionCommand()
                mission_command.set_FirstWaypoint(1)
                mission_command.set_VehicleID(self.__uav_id[self.__optimal_location[keep_num]])
                mission_command.set_Status(CommandStatusType.Pending)
                mission_command.set_CommandID(1)

                way_point_list = mission_command.get_WaypointList()

                keep_lon = self.__keep_in_zone[keep_num][0]
                keep_lat = self.__keep_in_zone[keep_num][1]

                # Position 1
                way_point = Waypoint()
                way_point.set_Latitude(keep_lat)
                way_point.set_Longitude(keep_lon)
                way_point.set_Altitude(self.__safeAlt)
                way_point.set_Number(1)
                way_point.set_NextWaypoint(2)
                way_point.set_Speed(self.__default_speed)
                way_point.set_SpeedType(SpeedType.Airspeed)
                way_point.set_ClimbRate(0)
                way_point.set_TurnType(TurnType.TurnShort)
                way_point.set_ContingencyWaypointA(0)
                way_point.set_ContingencyWaypointB(0)
                way_point_list.append(way_point)

                # Position 2
                way_point = Waypoint()
                way_point.set_Latitude(keep_lat + height_term / 4)
                way_point.set_Longitude(keep_lon + width_term / 4)
                way_point.set_Altitude(self.__safeAlt)
                way_point.set_Number(2)
                way_point.set_NextWaypoint(3)
                way_point.set_Speed(self.__default_speed)
                way_point.set_SpeedType(SpeedType.Airspeed)
                way_point.set_ClimbRate(0)
                way_point.set_TurnType(TurnType.TurnShort)
                way_point.set_ContingencyWaypointA(0)
                way_point.set_ContingencyWaypointB(0)
                way_point_list.append(way_point)

                # Position 3
                way_point = Waypoint()
                way_point.set_Latitude(keep_lat + height_term / 4)
                way_point.set_Longitude(keep_lon - width_term / 4)
                way_point.set_Altitude(self.__safeAlt)
                way_point.set_Number(3)
                way_point.set_NextWaypoint(4)
                way_point.set_Speed(self.__default_speed)
                way_point.set_SpeedType(SpeedType.Airspeed)
                way_point.set_ClimbRate(0)
                way_point.set_TurnType(TurnType.TurnShort)
                way_point.set_ContingencyWaypointA(0)
                way_point.set_ContingencyWaypointB(0)
                way_point_list.append(way_point)

                # Position 4
                way_point = Waypoint()
                way_point.set_Latitude(keep_lat - height_term / 4)
                way_point.set_Longitude(keep_lon - width_term / 4)
                way_point.set_Altitude(self.__safeAlt)
                way_point.set_Number(4)
                way_point.set_NextWaypoint(5)
                way_point.set_Speed(self.__default_speed)
                way_point.set_SpeedType(SpeedType.Airspeed)
                way_point.set_ClimbRate(0)
                way_point.set_TurnType(TurnType.TurnShort)
                way_point.set_ContingencyWaypointA(0)
                way_point.set_ContingencyWaypointB(0)
                way_point_list.append(way_point)

                # Position 5
                way_point = Waypoint()
                way_point.set_Latitude(keep_lat - height_term / 4)
                way_point.set_Longitude(keep_lon + width_term / 4)
                way_point.set_Altitude(self.__safeAlt)
                way_point.set_Number(5)
                way_point.set_NextWaypoint(2)
                way_point.set_Speed(self.__default_speed)
                way_point.set_SpeedType(SpeedType.Airspeed)
                way_point.set_ClimbRate(0)
                way_point.set_TurnType(TurnType.TurnShort)
                way_point.set_ContingencyWaypointA(0)
                way_point.set_ContingencyWaypointB(0)
                way_point_list.append(way_point)

                self.__client.sendLMCPObject(mission_command)

                keep_num = keep_num + 1

        # Last row
        for i in range(self.__remainder):
            width_term = self.__width / self.__remainder
            height_term = self.__height / self.__num_of_row

            mission_command = MissionCommand()
            mission_command.set_FirstWaypoint(1)
            mission_command.set_VehicleID(self.__uav_id[self.__optimal_location[keep_num]])
            mission_command.set_Status(CommandStatusType.Pending)
            mission_command.set_CommandID(1)

            way_point_list = mission_command.get_WaypointList()

            keep_lon = self.__keep_in_zone[keep_num][0]
            keep_lat = self.__keep_in_zone[keep_num][1]

            # Position 1
            way_point = Waypoint()
            way_point.set_Latitude(keep_lat)
            way_point.set_Longitude(keep_lon)
            way_point.set_Altitude(self.__safeAlt)
            way_point.set_Number(1)
            way_point.set_NextWaypoint(2)
            way_point.set_Speed(self.__default_speed)
            way_point.set_SpeedType(SpeedType.Airspeed)
            way_point.set_ClimbRate(0)
            way_point.set_TurnType(TurnType.TurnShort)
            way_point.set_ContingencyWaypointA(0)
            way_point.set_ContingencyWaypointB(0)
            way_point_list.append(way_point)

            # Position 2
            way_point = Waypoint()
            way_point.set_Latitude(keep_lat + height_term / 4)
            way_point.set_Longitude(keep_lon + width_term / 4)
            way_point.set_Altitude(self.__safeAlt)
            way_point.set_Number(2)
            way_point.set_NextWaypoint(3)
            way_point.set_Speed(self.__default_speed)
            way_point.set_SpeedType(SpeedType.Airspeed)
            way_point.set_ClimbRate(0)
            way_point.set_TurnType(TurnType.TurnShort)
            way_point.set_ContingencyWaypointA(0)
            way_point.set_ContingencyWaypointB(0)
            way_point_list.append(way_point)

            # Position 3
            way_point = Waypoint()
            way_point.set_Latitude(keep_lat + height_term / 4)
            way_point.set_Longitude(keep_lon - width_term / 4)
            way_point.set_Altitude(self.__safeAlt)
            way_point.set_Number(3)
            way_point.set_NextWaypoint(4)
            way_point.set_Speed(self.__default_speed)
            way_point.set_SpeedType(SpeedType.Airspeed)
            way_point.set_ClimbRate(0)
            way_point.set_TurnType(TurnType.TurnShort)
            way_point.set_ContingencyWaypointA(0)
            way_point.set_ContingencyWaypointB(0)
            way_point_list.append(way_point)

            # Position 4
            way_point = Waypoint()
            way_point.set_Latitude(keep_lat - height_term / 4)
            way_point.set_Longitude(keep_lon - width_term / 4)
            way_point.set_Altitude(self.__safeAlt)
            way_point.set_Number(4)
            way_point.set_NextWaypoint(5)
            way_point.set_Speed(self.__default_speed)
            way_point.set_SpeedType(SpeedType.Airspeed)
            way_point.set_ClimbRate(0)
            way_point.set_TurnType(TurnType.TurnShort)
            way_point.set_ContingencyWaypointA(0)
            way_point.set_ContingencyWaypointB(0)
            way_point_list.append(way_point)

            # Position 5
            way_point = Waypoint()
            way_point.set_Latitude(keep_lat - height_term / 4)
            way_point.set_Longitude(keep_lon + width_term / 4)
            way_point.set_Altitude(self.__safeAlt)
            way_point.set_Number(5)
            way_point.set_NextWaypoint(2)
            way_point.set_Speed(self.__default_speed)
            way_point.set_SpeedType(SpeedType.Airspeed)
            way_point.set_ClimbRate(0)
            way_point.set_TurnType(TurnType.TurnShort)
            way_point.set_ContingencyWaypointA(0)
            way_point.set_ContingencyWaypointB(0)
            way_point_list.append(way_point)

            self.__client.sendLMCPObject(mission_command)

            keep_num = keep_num + 1

        """
        for i in range(self.__num_of_uav):
            self.go_way_point(self.__optimal_location[i]+1, self.__keep_in_zone[i][0], self.__keep_in_zone[i][1], 0)

        print("To be implemented...(initialstrategy_with_row"))
        """

        print("End of initialstrategy_with_row")

    # Initial strategy with loiter
    def initial_loiter(self, num_of_row):
        # From first row to (Last-1) row
        keep_num = 0
        for row in range(self.__num_of_full_uav_row):
            for i in range(self.__num_of_uav_per_row):
                width_term = self.__width / self.__num_of_uav_per_row
                height_term = self.__height / self.__num_of_row
                location = Location3D()
                location.set_Latitude(self.__keep_in_zone[keep_num][1])
                location.set_Longitude(self.__keep_in_zone[keep_num][0])
                location.set_Altitude(self.__safeAlt)
                self.send_loiter(self.__uav_id[self.__optimal_location[keep_num]], location, height_term / 8 * 3,
                                 self.__loiter_clockwise)
                keep_num = keep_num + 1

        # Last row
        for i in range(self.__remainder):
            width_term = self.__width / self.__remainder
            height_term = self.__height / self.__num_of_row
            location = Location3D()
            location.set_Latitude(self.__keep_in_zone[keep_num][1])
            location.set_Longitude(self.__keep_in_zone[keep_num][0])
            location.set_Altitude(self.__safeAlt)

            self.send_loiter(self.__uav_id[self.__optimal_location[keep_num]], location, height_term / 8 * 3,
                             self.__loiter_clockwise)
            keep_num = keep_num + 1

        print("End of initial_loiter")

    # Function to make it go
    def go_way_point(self, id, Longitude, Latitude, area):
        mission_command = MissionCommand()
        mission_command.set_FirstWaypoint(1)
        mission_command.set_VehicleID(id)
        mission_command.set_Status(CommandStatusType.Pending)
        mission_command.set_CommandID(1)

        way_point_list = mission_command.get_WaypointList()

        uav_id_index = self.__uav_id.index(id)
        alti = self.__terrian_service.getElevation(Latitude, Longitude)

        way_point = Waypoint()
        way_point.set_Latitude(Latitude)
        way_point.set_Longitude(Longitude)
        way_point.set_Speed(self.__default_speed)
        way_point.set_Altitude(alti)
        self.sendGimbalAngleChangeCmd(id, self.__gimbal_searchMode)

        way_point.set_AltitudeType(AltitudeType.MSL)
        way_point.set_Number(1)
        way_point.set_NextWaypoint(2)
        way_point.set_SpeedType(SpeedType.Airspeed)
        way_point.set_ClimbRate(0)
        way_point.set_TurnType(TurnType.TurnShort)
        way_point.set_ContingencyWaypointA(0)
        way_point.set_ContingencyWaypointB(0)
        way_point_list.append(way_point)

        way_point = Waypoint()
        way_point.set_Latitude(Latitude)
        way_point.set_Longitude(Longitude)
        way_point.set_Speed(self.__default_speed)
        way_point.set_Altitude(alti)
        self.sendGimbalAngleChangeCmd(id, self.__gimbal_searchMode)

        way_point.set_AltitudeType(AltitudeType.MSL)
        way_point.set_Number(2)
        way_point.set_NextWaypoint(3)
        way_point.set_SpeedType(SpeedType.Airspeed)
        way_point.set_ClimbRate(0)
        way_point.set_TurnType(TurnType.TurnShort)
        way_point.set_ContingencyWaypointA(0)
        way_point.set_ContingencyWaypointB(0)
        way_point_list.append(way_point)

        loiter_action = LoiterAction()
        loiter_action.set_LoiterType(LoiterType.Circular)
        loiter_action.set_Radius(3000)
        loiter_action.set_Axis(0)
        loiter_action.set_Length(0)
        loiter_action.set_Direction(LoiterDirection.Clockwise)
        loiter_action.set_Duration(100000)
        loiter_action.set_Airspeed(30)

        location = Location3D()
        location.set_AltitudeType(AltitudeType.MSL)
        location.set_Altitude(alti)
        location.set_Longitude(Longitude)
        location.set_Latitude(Latitude)
        loiter_action.set_Location(location)

        way_point.VehicleActionList.append(loiter_action)

        self.__client.sendLMCPObject(mission_command)

    def go_two_way_point(self, id, lon1, lat1, lon2, lat2):
        mission_command = MissionCommand()
        mission_command.set_FirstWaypoint(1)
        mission_command.set_VehicleID(id)
        mission_command.set_Status(CommandStatusType.Pending)
        mission_command.set_CommandID(1)

        way_point_list = mission_command.get_WaypointList()
        uav_id_index = self.__uav_id.index(id)

        way_point = Waypoint()
        way_point.set_Latitude(lat1)
        way_point.set_Longitude(lon1)
        way_point.set_Speed(self.__default_speed)
        way_point.set_Altitude(self.__safeAlt)
        self.sendGimbalAngleChangeCmd(id, self.__gimbal_searchMode)
        way_point.set_AltitudeType(AltitudeType.MSL)
        way_point.set_Number(1)
        way_point.set_NextWaypoint(2)
        way_point.set_SpeedType(SpeedType.Airspeed)
        way_point.set_ClimbRate(0)
        way_point.set_TurnType(TurnType.TurnShort)
        way_point.set_ContingencyWaypointA(0)
        way_point.set_ContingencyWaypointB(0)
        way_point_list.append(way_point)

        way_point = Waypoint()
        way_point.set_Latitude(lat2)
        way_point.set_Longitude(lon2)
        way_point.set_Speed(self.__default_speed)
        way_point.set_Altitude(self.__safeAlt)
        self.sendGimbalAngleChangeCmd(id, self.__gimbal_searchMode)
        way_point.set_AltitudeType(AltitudeType.MSL)
        way_point.set_Number(2)
        way_point.set_NextWaypoint(3)
        way_point.set_SpeedType(SpeedType.Airspeed)
        way_point.set_ClimbRate(0)
        way_point.set_TurnType(TurnType.TurnShort)
        way_point.set_ContingencyWaypointA(0)
        way_point.set_ContingencyWaypointB(0)
        way_point_list.append(way_point)

        loiter_action = LoiterAction()
        loiter_action.set_LoiterType(LoiterType.Circular)
        loiter_action.set_Radius(5000)
        loiter_action.set_Axis(0)
        loiter_action.set_Length(0)
        loiter_action.set_Direction(LoiterDirection.Clockwise)
        loiter_action.set_Duration(100000)
        loiter_action.set_Airspeed(30)

        location = Location3D()
        location.set_AltitudeType(AltitudeType.MSL)
        location.set_Altitude(self.__safeAlt)
        location.set_Longitude(lon2)
        location.set_Latitude(lat2)
        loiter_action.set_Location(location)

        way_point.VehicleActionList.append(loiter_action)

        self.__client.sendLMCPObject(mission_command)

    def go_two_way_point2(self, id, lon1, lat1, lon2, lat2):
        mission_command = MissionCommand()
        mission_command.set_FirstWaypoint(1)
        mission_command.set_VehicleID(id)
        mission_command.set_Status(CommandStatusType.Pending)
        mission_command.set_CommandID(1)

        way_point_list = mission_command.get_WaypointList()
        uav_id_index = self.__uav_id.index(id)

        line1_altis, coords = self.__terrian_service.getLineElevs(lat1, lon1, lat2, lon2, 2)
        line1_alti = np.max(line1_altis) + self.__maxGap

        way_point = Waypoint()
        way_point.set_Latitude(lat1)
        way_point.set_Longitude(lon1)
        way_point.set_Speed(self.__default_speed)
        way_point.set_Altitude(line1_alti)
        self.sendGimbalAngleChangeCmd(id, self.__gimbal_searchMode)
        way_point.set_AltitudeType(AltitudeType.MSL)
        way_point.set_Number(1)
        way_point.set_NextWaypoint(2)
        way_point.set_SpeedType(SpeedType.Airspeed)
        way_point.set_ClimbRate(0)
        way_point.set_TurnType(TurnType.TurnShort)
        way_point.set_ContingencyWaypointA(0)
        way_point.set_ContingencyWaypointB(0)
        way_point_list.append(way_point)

        way_point = Waypoint()
        way_point.set_Latitude(lat2)
        way_point.set_Longitude(lon2)
        way_point.set_Speed(self.__default_speed)
        way_point.set_Altitude(line1_alti)
        self.sendGimbalAngleChangeCmd(id, self.__gimbal_searchMode)
        way_point.set_AltitudeType(AltitudeType.MSL)
        way_point.set_Number(2)
        way_point.set_NextWaypoint(3)
        way_point.set_SpeedType(SpeedType.Airspeed)
        way_point.set_ClimbRate(0)
        way_point.set_TurnType(TurnType.TurnShort)
        way_point.set_ContingencyWaypointA(0)
        way_point.set_ContingencyWaypointB(0)
        way_point_list.append(way_point)

        loiter_action = LoiterAction()
        loiter_action.set_LoiterType(LoiterType.Circular)
        loiter_action.set_Radius(5000)
        loiter_action.set_Axis(0)
        loiter_action.set_Length(0)
        loiter_action.set_Direction(LoiterDirection.Clockwise)
        loiter_action.set_Duration(100000)
        loiter_action.set_Airspeed(30)

        location = Location3D()
        location.set_AltitudeType(AltitudeType.MSL)
        location.set_Altitude(line1_alti)
        location.set_Longitude(lon2)
        location.set_Latitude(lat2)
        loiter_action.set_Location(location)

        way_point.VehicleActionList.append(loiter_action)

        self.__client.sendLMCPObject(mission_command)

    # Function to calculate time to go each first keep in zone with row parameter
    def calculate_each_time(self, num_of_row):
        self.__remainder = -1
        # When remainder is 0
        if self.__num_of_uav % num_of_row is 0:
            self.__num_of_uav_per_row = self.__num_of_uav // num_of_row
            self.__num_of_full_uav_row = num_of_row
            self.__remainder = 0
        # When remainder is not 0
        else:
            self.__num_of_uav_per_row = -1
            self.__num_of_full_uav_row = num_of_row - 1
            self.__remainder = self.__num_of_uav % num_of_row
            for uavnum in range(self.__num_of_uav, 0, -1):
                if uavnum * (num_of_row - 1) < self.__num_of_uav:
                    self.__num_of_uav_per_row = uavnum
                    self.__remainder = self.__num_of_uav - (self.__num_of_full_uav_row * self.__num_of_uav_per_row)
                    break
            if self.__num_of_uav_per_row is 0:
                print("Error in calculating __num_of_uav_per_row")

        # When remainder is 0
        if self.__remainder is 0:
            # Calculate location of keep in zone at each row
            for row in range(num_of_row):
                for i in range(self.__num_of_uav_per_row):
                    width_term = self.__width / self.__num_of_uav_per_row
                    height_term = self.__height / num_of_row
                    self.__keep_in_zone.append([self.__left + (i * width_term + (i + 1) * width_term) / 2,
                                                self.__top - height_term / 2 - row * height_term])

        elif self.__remainder > 0:
            # Not last row
            for row in range(self.__num_of_full_uav_row):
                for i in range(self.__num_of_uav_per_row):
                    width_term = self.__width / self.__num_of_uav_per_row
                    height_term = self.__height / num_of_row
                    self.__keep_in_zone.append([self.__left + (i * width_term + (i + 1) * width_term) / 2,
                                                self.__top - (row * height_term + (row + 1) * height_term) / 2])

            # Last row
            for i in range(self.__remainder):
                width_term = self.__width / self.__remainder
                height_term = self.__height / num_of_row
                self.__keep_in_zone.append(
                    [self.__left + (i * width_term + (i + 1) * width_term) / 2, self.__bottom + height_term / 2])
        else:
            print("Error in calculate remainder")

        # --- Finish calculating keep in zone ---

        # Calculate distance
        for i in range(self.__num_of_uav):
            # distance_tmp = [0 for i in range(self.__num_of_uav)]
            distance_tmp = []
            for j in range(self.__num_of_uav):
                distance_tmp.append(
                    self.distance(self.__uav_total_state[j][4][0], self.__uav_total_state[j][4][1],
                                  self.__keep_in_zone[i][0],
                                  self.__keep_in_zone[i][1]))
            self.__distance.append(distance_tmp)

        # time = distance / velocity
        # Calculate time
        for i in range(self.__num_of_uav):
            time_tmp = []
            for j in range(self.__num_of_uav):
                time_tmp.append(self.__distance[i][j] / self.__initial_speed)
            self.__time.append(time_tmp)
        print("End of calculate_each_time")

    # Send loiter
    def send_loiter(self, vehicle_id, location, radius, is_clockwise):
        vehicle_action_command = VehicleActionCommand()
        vehicle_action_command.set_VehicleID(vehicle_id)
        vehicle_action_command.set_Status(CommandStatusType.Pending)
        vehicle_action_command.set_CommandID(1)

        loiter_action = LoiterAction()
        loiter_action.set_LoiterType(LoiterType.Circular)
        loiter_action.set_Radius(radius * 100000)
        loiter_action.set_Axis(0)
        loiter_action.set_Length(0)
        if is_clockwise:
            loiter_action.set_Direction(LoiterDirection.Clockwise)
        else:
            loiter_action.set_Direction(LoiterDirection.CounterClockwise)
        loiter_action.set_Duration(100000)
        loiter_action.set_Airspeed(self.__default_speed)
        loiter_action.set_Location(location)
        vehicle_action_command.get_VehicleActionList().append(loiter_action)
        self.__client.sendLMCPObject(vehicle_action_command)

    def send_estimate_report(self, estimated_hazardzone, estimated_hazardzone_ID):
        # Setting up the mission to send to the UAV
        hazard_estimate_report = HazardZoneEstimateReport()
        hazard_estimate_report.set_EstimatedZoneShape(estimated_hazardzone)

        # 기존엔 상수로 되어있었는데, 받아온 파라미터를 넣어 헤저드 존 별 차이를 둔다.
        hazard_estimate_report.set_UniqueTrackingID(estimated_hazardzone_ID)
        hazard_estimate_report.set_EstimatedGrowthRate(0)
        hazard_estimate_report.set_PerceivedZoneType(HazardType.Fire)
        hazard_estimate_report.set_EstimatedZoneDirection(0)
        hazard_estimate_report.set_EstimatedZoneSpeed(0)

        # Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPObject(hazard_estimate_report)

    # 헤저드 존 별로 그룹화가 이뤄져야한다.
    def polygon(self, hz_index):
        # Initialize array
        # 헤저드 존 별로 각각 폴리곤을 형성해야하므로 현재 uav가 탐색한 hz 번호를 구한다.
        self.__uav_all.clear()

        # Variables
        self.__polygon_center_lat = 0
        self.__polygon_center_lon = 0

        # - CM
        # Insert all detected location into uav_all
        # hazard_zone - 두개 이상이므로 이에 맞게 처리필요
        # uav_all - zone에 맞게
        # 현재 uav 와 같은 존을 탐색하는 uav들이 찍은 모든 점을 temp_polygon에 담아놓은후 self.__uav_all에 추가한다.
        temp_polygon = [self.__uav_total_state[i][13] for i in range(self.__num_of_uav) if
                        self.__uav_total_state[i][5] == 2 and self.__uav_total_state[i][6] == hz_index]
        self.__uav_all = list(itertools.chain(*temp_polygon))
        # -----------------
        for location in self.__previousPolygon[hz_index]:
            self.__uav_all.append(location)

        self.__previousPolygon[hz_index].clear()

        print("point num : ", len(self.__uav_all))

        all_coords = [[coord.get_Longitude(), coord.get_Latitude()] for coord in self.__uav_all]
        if len(all_coords) < 3:
            return
        hull = ConvexHull(np.asarray(all_coords, dtype=np.float32))

        self.__uav_all = [self.__uav_all[i] for i in hull.vertices]

        # Create polygon object
        self.__estimatedHazardZone = Polygon()

        for i in self.__uav_all:
            self.__estimatedHazardZone.get_BoundaryPoints().append(i)
            self.__previousPolygon[hz_index].append(i)
        self.send_estimate_report(self.__estimatedHazardZone, hz_index)
        # -------------

    def recovery_point_info(self, recovery_point):
        # [[lon1, lat1, rad1], [lon2, lat2, rad2], ...]
        self.__recovery_points.append(
            [recovery_point.Boundary.CenterPoint.Longitude, recovery_point.Boundary.CenterPoint.Latitude,
             recovery_point.get_Boundary().Radius])

    def calculate_closest_recovery_point(self):
        for uav_idx in range(self.__num_of_uav):
            current_uav_lon = self.__uav_total_state[uav_idx][4][0]
            current_uav_lat = self.__uav_total_state[uav_idx][4][1]
            dist = sys.maxsize
            save_recovery = []
            for recovery_list in self.__recovery_points:
                recovery_lon = recovery_list[0]
                recovery_lat = recovery_list[1]
                new_dist = self.distance(current_uav_lon, current_uav_lat, recovery_lon, recovery_lat)
                if new_dist < dist:
                    dist = new_dist
                    save_recovery = [current_uav_lon, current_uav_lat]
            self.__uav_total_state[uav_idx][14] = save_recovery

    def climb_altitude(self):
        for i in range(self.__num_of_uav):
            mission_command = MissionCommand()
            mission_command.set_FirstWaypoint(1)
            mission_command.set_VehicleID(self.__uav_id[i])
            mission_command.set_Status(CommandStatusType.Pending)
            mission_command.set_CommandID(1)

            way_point_list = mission_command.get_WaypointList()

            rp_lat = self.__closest_recovery_point[i][1]
            rp_lon = self.__closest_recovery_point[i][0]
            print("id:", self.__uav_id[i], "point:", rp_lat, ",", rp_lon)
            # Position 1
            way_point = Waypoint()
            way_point.set_Latitude(rp_lat)
            way_point.set_Longitude(rp_lon)
            way_point.set_Altitude(self.__safeAlt)
            way_point.set_Number(1)
            way_point.set_NextWaypoint(2)
            way_point.set_Speed(self.__default_speed)
            way_point.set_SpeedType(SpeedType.Airspeed)
            way_point.set_ClimbRate(0)
            way_point.set_TurnType(TurnType.TurnShort)
            way_point.set_ContingencyWaypointA(0)
            way_point.set_ContingencyWaypointB(0)
            way_point_list.append(way_point)

            self.__client.sendLMCPObject(mission_command)

    # Function to get maximum altitude
    def get_maxalti(self, uav_point, front_point):
        elevs, coords = self.__terrian_service.getLineElevs(uav_point.get_Latitude(), uav_point.get_Longitude(),
                                                            math.degrees(front_point[0]), math.degrees(front_point[1]),
                                                            2)
        return np.max(elevs)

    # Function to move to ideal altitude

    def check_alti(self, uav_object):
        # 드론 현 위치의 고도와 디텍 센서되는 지형의 고도 차를 225 정도, 1200보다 크게 맞추기 위한 함수
        uav_id = uav_object.get_ID()
        uav_id_index = self.__uav_id.index(uav_id)

        uav_point = uav_object.get_Location()
        # 700m 앞의 좌표 구하기
        front_point = self.__terrian_service.getLatLon(math.radians(uav_point.get_Latitude()),
                                                       math.radians(uav_point.get_Longitude()), self.__maxDist,
                                                       uav_object.get_Heading());

        max_alti = self.get_maxalti(uav_point, front_point)
        gap = abs(uav_point.get_Altitude() - max_alti)
        # print("uav_id:", uav_id, "max_alti:",max_alti)

        # if max_alti + self.__minGap < self.__maxAlti_firzone:
        #     self.__uav_gap_check_time[uav_id_index] = uav_object.get_Time()
        #     if self.__uav_gimbal_state[uav_id_index] == LOW or self.__uav_gimbal_state[uav_id_index] == MID:
        #         self.sendGimbalAngleChangeCmd(uav_id, self.__gimbal_trackingMode - 20)
        #         self.__uav_gimbal_state[uav_id_index] = HIGH
        #     return

        if gap < self.__minGap:
            self.moveto_ideal_alti(uav_id, uav_object.get_Heading(), max_alti)
            self.__uav_gap_check_time[uav_id_index] = uav_object.get_Time()
            if self.__uav_gimbal_state[uav_id_index] == HIGH:
                self.sendGimbalAngleChangeCmd(uav_id, self.__gimbal_trackingMode)
                self.__uav_gimbal_state[uav_id_index] = LOW
        elif gap > self.__maxGap:
            self.moveto_ideal_alti(uav_id, uav_object.get_Heading(), max_alti)
            self.__uav_gap_check_time[uav_id_index] = uav_object.get_Time()
            if self.__uav_gimbal_state[uav_id_index] == LOW or self.__uav_gimbal_state[uav_id_index] == MID:
                self.sendGimbalAngleChangeCmd(uav_id, self.__gimbal_trackingMode - 20)
                self.__uav_gimbal_state[uav_id_index] = HIGH
        else:
            self.__uav_gap_check_time[uav_id_index] = uav_object.get_Time()
            if self.__uav_gimbal_state[uav_id_index] == HIGH or self.__uav_gimbal_state[uav_id_index] == LOW:
                self.sendGimbalAngleChangeCmd(uav_id, self.__gimbal_trackingMode)
                self.__uav_gimbal_state[uav_id_index] = MID

    """
    def all_arr_setting(self):
        # init array for tracking
        self.__uav_detection_count = [0 for i in range(self.__num_of_uav)]
        self.__uav_detection_time = [0 for i in range(self.__num_of_uav)]
        self.__uav_detection = [False for i in range(self.__num_of_uav)]
        self.__uav_heading = [0 for i in range(self.__num_of_uav)]
        self.__uav_circle_way = [1 for i in range(self.__num_of_uav)]
        self.__goto_center = [False for i in range(self.__num_of_uav)]
        self.__uav_change_heading = [0 for i in range(self.__num_of_uav)]
        self.__uav_current_status = [0 for i in range(self.__num_of_uav)]
        self.__uav_polygon = [[] for i in range(self.__num_of_uav)]
        self.__uav_gap_check_time = [0 for i in range(self.__num_of_uav)]
        self.__uav_first_gap_check = [False for i in range(self.__num_of_uav)]
        self.__uav_gimbal_state = [1 for i in range(
            self.__num_of_uav)]  # 1 - STARE, keep one elevation, # 2 - SCAN, look over changing elevation.
        self.__closest_recovery_point = [None for i in range(self.__num_of_uav)]
        self.__uav_hazardzone = [-1 for i in range(self.__num_of_uav)]
        self.__uav_in_the_zone = [0 for i in range(self.__num_of_uav)]
        self.__detected_uav_location = [None for i in range(self.__num_of_uav)]
        self.__is_recharging = [False for i in range(self.__num_of_uav)]
    """

    def addNewUav(self, lmcpObject):
        if lmcpObject.get_ID() not in self.__uav_id:
            self.uav_init(lmcpObject)
            self.__uav_detection_count.append(0)
            self.__uav_detection_time.append(0)
            self.__uav_detection.append(False)
            self.__uav_heading.append(0)
            self.__uav_circle_way.append(1)
            self.__goto_center.append(False)
            self.__uav_change_heading.append(0)
            self.__uav_current_status.append(0)
            self.__uav_polygon.append([])
            self.__uav_gap_check_time.append(0)
            self.__uav_first_gap_check.append(False)
            self.__uav_gimbal_state.append(1)
            self.__closest_recovery_point.append(None)
            self.__uav_hazardzone.append(-1)
            self.__uav_in_the_zone.append(0)
            self.__detected_uav_location.append(None)
            self.__is_tracking.append(False)
            self.__is_recharging.append(False)

            # recovery point.
            dist = sys.maxsize
            save_recovery = []
            for recovery_list in self.__recovery_points:
                recovery_lon = recovery_list[0]
                recovery_lat = recovery_list[1]
                new_dist = self.distance(self.__uav_location[self.__uav_id.index(lmcpObject.get_ID())][0],
                                         self.__uav_location[self.__uav_id.index(lmcpObject.get_ID())][1], recovery_lon,
                                         recovery_lat)
                if new_dist < dist:
                    dist = new_dist
                    save_recovery = [self.__uav_location[self.__uav_id.index(lmcpObject.get_ID())][0],
                                     self.__uav_location[self.__uav_id.index(lmcpObject.get_ID())][1]]
            self.__closest_recovery_point[self.__uav_id.index(lmcpObject.get_ID())] = save_recovery

    def check_hazardzone(self, hazard_lon, hazard_lat, threshold):
        for i in range(len(self.__hazardzone_firstDetect_point)):
            dist = math.sqrt((self.__hazardzone_firstDetect_point[i][0] - hazard_lon) ** 2 + (
                    self.__hazardzone_firstDetect_point[i][1] - hazard_lat) ** 2)

            if dist < threshold:
                return i

        return -1

    def callOtherDrone(self, lmcpObject):
        # new harzard zone
        hazard_lon = lmcpObject.get_DetectedLocation().get_Longitude()
        hazard_lat = lmcpObject.get_DetectedLocation().get_Latitude()

        if self.check_hazardzone(hazard_lon, hazard_lat, self.__hazardzone_threshold) == -1:
            hzNum = len(self.__hazardzones)
            self.__hazardzones.append(hzNum)
            self.__for_keep_going.append([])
            self.__hazardzone_firstDetect_point.append([hazard_lon, hazard_lat])
            self.__hazardzone_center_point.append(None)
            self.__uav_zigzag_heading.append(0)
            self.__hazardzone_detect_count.append(0)
            self.__previousPolygon.append([])
            uav_id = lmcpObject.get_DetectingEnitiyID()
            uav_id_index = self.__uav_id.index(uav_id)

            self.__uav_total_state[uav_id_index][6] = hzNum

            # Call near(time) drones that is not called by other drone
            time = []
            for i in range(self.__num_of_uav):
                # 자기자신은 선택 안하도록 최댓값
                if i == uav_id_index:
                    time.append(sys.maxsize)
                elif self.__uav_id[i] in self.__removed_uav:
                    time.append(sys.maxsize)
                else:
                    time.append(
                        self.distance(hazard_lon, hazard_lat, self.__uav_total_state[i][4][0],
                                      self.__uav_total_state[i][4][1]))
            print("UAV", uav_id, " detect hazard zone", hzNum)

            print("time : ", time)

            num_of_call = 2
            for i in range(num_of_call):
                min_index = time.index(min(time))
                if self.__uav_total_state[uav_id_index][5] not in (2, 3):
                    self.go_way_point(self.__uav_id[min_index], hazard_lon, hazard_lat, 0)
                    self.__uav_total_state[min_index][5] = 3  # 3 : 불러서 가는 중
                    time[min_index] = sys.maxsize
                    self.__uav_total_state[min_index][6] = hzNum  # 가는 hz_id
                    print("Call uav", self.__uav_id[min_index], "to detect hazard zone ", hzNum)
                else:
                    time[min_index] = sys.maxsize
                    i = i - 1
                print("time : ", time)
            else:
                pass
            print("added Hz")

    def doLineTracer(self, lmcpObject):
        # 1 . 발견하면 멈추고
        # print("lineTracer")
        uav_id = lmcpObject.get_DetectingEnitiyID()
        uav_id_index = self.__uav_id.index(uav_id)
        state = self.__uav_total_state[uav_id_index][5]
        if lmcpObject.get_DetectedHazardZoneType() == 1:

            # print("uav_id:",uav_id)

            if state not in (2, 3):
                # new harzard zone
                self.callOtherDrone(lmcpObject)
                state = 2

            # Already tracking
            if state == 2 or state == 3:
                # 디텍ing
                # 점을 저장
                # 90도 나감
                # 35 속도
                self.stopAndMakeLoiter(lmcpObject)
                state = 2

        self.__uav_total_state[uav_id_index][5] = state

    # Flight 명령 주기
    def sendFlightCmd(self, uav_id, alti, heading, speed):
        uav_id_index = self.__uav_id.index(uav_id)

        vehicle_action_command = VehicleActionCommand()
        vehicle_action_command.set_VehicleID(uav_id)
        vehicle_action_command.set_Status(CommandStatusType.Pending)
        vehicle_action_command.set_CommandID(1)

        flight_action = FlightDirectorAction()
        flight_action.set_Speed(speed)

        flight_action.set_Altitude(alti)
        flight_action.set_Heading(heading)
        # Adding the loiter action to the vehicle action list
        vehicle_action_command.get_VehicleActionList().append(flight_action)

        # Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPObject(vehicle_action_command)

    # 일정 각도 만큼 꺾기, state = 1 -> 밖에서 안으로, 0 -> 안에서 밖으로
    def changeUavHeading(self, uav_id, added_angle, direction):

        uav_id_index = self.__uav_id.index(uav_id)
        # print(self.__for_keep_going)
        loiter_direction = (self.__for_keep_going[self.__uav_total_state[uav_id_index][6]].index(
            uav_id) + direction) % 2

        heading = self.__uav_total_state[uav_id_index][7]
        if loiter_direction == 1:  # 시계방향
            heading = heading - added_angle
            if heading <= -180:
                heading += 360
        else:
            heading = heading + added_angle
            if heading >= 180:
                heading -= 360
        return heading

    def check_energy(self, lmcpObject):
        uav_id = lmcpObject.get_ID()
        # print("Airvehicle:", uav_id)
        uav_idx = self.__uav_id.index(uav_id)

        recovery_lon = self.__closest_recovery_point[uav_idx][0]
        recovery_lat = self.__closest_recovery_point[uav_idx][1]

        uav_lat = lmcpObject.get_Location().get_Latitude()
        uav_lon = lmcpObject.get_Location().get_Longitude()

        battery_remaining = lmcpObject.get_EnergyAvailable()
        if battery_remaining <= 50:
            if not self.__is_recharging[uav_idx]:
                self.__is_recharging[uav_idx] = True
                self.go_two_way_point2(lmcpObject.get_ID(), recovery_lon, recovery_lat, uav_lon, uav_lat)
        ## 에너지가 98보다 큰 즉 100 인체로 유지되는 친구는 무시
        if battery_remaining >= 98 and battery_remaining < 100:
            self.__is_recharging[uav_idx] = False
            self.__uav_detection_time[uav_idx] = 0
            self.__uav_change_heading[uav_idx] = 0

    def stopAndMakeLoiter(self, hazardZoneDetection):
        # 위치 저장
        uav_id = hazardZoneDetection.get_DetectingEnitiyID()
        uav_id_index = self.__uav_id.index(uav_id)

        self.__uav_total_state[uav_id_index][15] = self.__scenarioTime

        if self.__uav_detection_count[uav_id_index] % 10 == 0 and self.__uav_detection_count[uav_id_index] < 20:
            if not self.__uav_detection[uav_id_index]:

                # rotation way
                if uav_id not in self.__for_keep_going[self.__uav_total_state[uav_id_index][6]]:
                    self.__for_keep_going[self.__uav_total_state[uav_id_index][6]].append(uav_id)

                self.__uav_detection[uav_id_index] = True
                self.__uav_detection_count[uav_id_index] = 0

            center_location = hazardZoneDetection.get_DetectedLocation()

            front_point = self.__terrian_service.getLatLon(math.radians(center_location.get_Latitude()),
                                                           math.radians(center_location.get_Longitude()),
                                                           self.__maxDist,
                                                           self.__uav_total_state[uav_id_index][7])

            self.__uav_total_state[uav_id_index][13].append(center_location)
            front = Location3D()
            front.set_Longitude(front_point[1])
            front.set_Latitude(front_point[0])

            #self.__uav_total_state[uav_id_index][13].append(front)

            alti = self.__uav_total_state[uav_id_index][4][2]
            speed = self.__uav_total_state[uav_id_index][2]
            heading = self.changeUavHeading(uav_id, 120, 1)

            self.sendFlightCmd(uav_id, alti, heading, speed)

        # print("stop-done")
        self.__uav_detection_count[uav_id_index] += 1

    def alti_angle_update(self, uav_object):
        # 고도와 각도, 현재 상황에 맞게 변화
        # 고도는 300m gap 유지
        alpha = 10
        uav_id = uav_object.get_ID()
        uav_id_index = self.__uav_id.index(uav_id)

        current_uav_alti = self.__uav_total_state[uav_id_index][4][2]
        current_angle = self.__uav_total_state[uav_id_index][8]
        current_pos_alti = self.__terrian_service.getElevation(self.__uav_total_state[uav_id_index][4][1],
                                                               self.__uav_total_state[uav_id_index][4][0])

        current_alt_gap = current_uav_alti - current_pos_alti
        if current_alt_gap < 0:
            print("gap-error")

        maxRange = self.__uav_total_state[uav_id_index][1]

        if current_alt_gap < maxRange:
            radian = math.acos(current_alt_gap / maxRange)

            angle = 90 - math.degrees(radian)

            self.sendGimbalAngleChangeCmd(uav_id, (-1) * angle - 10)

        if self.__uav_total_state[uav_id_index][5] == 2:

            self.sendGimbalAngleChangeCmd(uav_id, -60)
        #     print("h")

        if self.__uav_total_state[uav_id_index][5] == 0 or self.__uav_total_state[uav_id_index][5]==1:

            self.sendGimbalAngleChangeCmd(uav_id, -70)

        # front_point = self.__terrian_service.getLatLon(math.radians(self.__uav_total_state[uav_id_index][4][1]),
        #                                                     math.radians(self.__uav_total_state[uav_id_index][4][0]), self.__maxDist,
        #                                                     self.__uav_total_state[uav_id_index][7]);

        # elevs, coords = self.__terrian_service.getLineElevs(self.__uav_total_state[uav_id_index][4][1], self.__uav_total_state[uav_id_index][4][0],
        #                                                     math.degrees(front_point[0]), math.degrees(front_point[1]),
        #                                                     2)

        # max_alti = np.max(elevs)
        if self.__uav_total_state[uav_id_index][5] == 2:
            # if self.__scenarioTime - self.__uav_total_state[uav_id_index][15] >10000:
            self.__uav_total_state[uav_id_index][4][2] = current_pos_alti + self.__maxGap
            # self.sendFlightCmd(uav_id, current_pos_alti + 100, self.__uav_total_state[uav_id_index][7], 15)
            # print("j")

        elif self.__uav_total_state[uav_id_index][5] == 3:
            self.__uav_total_state[uav_id_index][4][2] = current_pos_alti + self.__maxGap
            #self.sendFlightCmd(uav_id, current_pos_alti + 100, self.__uav_total_state[uav_id_index][7], 15)

    def initial_lec_upgrade(self, num_of_row):
        # From first row to (Last-1) row
        keep_num = 0

        width_term = self.__width / self.__num_of_uav_per_row
        height_term = self.__height / self.__num_of_row

        mission_command = MissionCommand()
        mission_command.set_FirstWaypoint(1)
        mission_command.set_VehicleID(self.__uav_id[self.__optimal_location[keep_num]])
        mission_command.set_Status(CommandStatusType.Pending)
        mission_command.set_CommandID(1)

        way_point_list = mission_command.get_WaypointList()

        keep_lon = self.__keep_in_zone[keep_num][0]
        keep_lat = self.__keep_in_zone[keep_num][1]

        uav_id_index = self.__optimal_location[keep_num]
        line1_altis, coords = self.__terrian_service.getLineElevs(self.__uav_total_state[uav_id_index][4][1],
                                                                  self.__uav_total_state[uav_id_index][4][0],
                                                                  keep_lat, keep_lon, 2)

        line1_alti = max(line1_altis) + self.__maxGap
        # Position 1
        way_point = Waypoint()
        way_point.set_Latitude(keep_lat - height_term / 8 * 3)
        way_point.set_Longitude(keep_lon + width_term / 8 * 3)
        way_point.set_Altitude(line1_alti)
        way_point.set_Number(1)
        way_point.set_NextWaypoint(2)
        way_point.set_Speed(self.__default_speed)
        way_point.set_SpeedType(SpeedType.Airspeed)
        way_point.set_ClimbRate(0)
        way_point.set_TurnType(TurnType.TurnShort)
        way_point.set_ContingencyWaypointA(0)
        way_point.set_ContingencyWaypointB(0)
        way_point_list.append(way_point)

        line2_altis, coords = self.__terrian_service.getLineElevs(keep_lat, keep_lon,
                                                                  keep_lat - height_term / 8 * 3,
                                                                  keep_lon + width_term / 8 * 3, 2)
        line2_alti = max(line2_altis) + self.__maxGap
        # Position 2
        way_point = Waypoint()
        way_point.set_Latitude(keep_lat - height_term / 8 * 3)
        way_point.set_Longitude(keep_lon + width_term / 8 * 3)
        way_point.set_Altitude(line2_alti)
        way_point.set_Number(2)
        way_point.set_NextWaypoint(3)
        way_point.set_Speed(self.__default_speed)
        way_point.set_SpeedType(SpeedType.Airspeed)
        way_point.set_ClimbRate(0)
        way_point.set_TurnType(TurnType.TurnShort)
        way_point.set_ContingencyWaypointA(0)
        way_point.set_ContingencyWaypointB(0)
        way_point_list.append(way_point)

        line3_altis, coords = self.__terrian_service.getLineElevs(keep_lat - height_term / 8 * 3,
                                                                  keep_lon + width_term / 8 * 3,
                                                                  keep_lat + height_term / 8 * 3,
                                                                  keep_lon + width_term / 8 * 3, 2)
        line3_alti = max(line3_altis) + self.__maxGap
        # Position 3
        way_point = Waypoint()
        way_point.set_Latitude(keep_lat + height_term / 8 * 3)
        way_point.set_Longitude(keep_lon + width_term / 8 * 3)
        way_point.set_Altitude(line3_alti)
        way_point.set_Number(3)
        way_point.set_NextWaypoint(5)
        way_point.set_Speed(self.__default_speed)
        way_point.set_SpeedType(SpeedType.Airspeed)
        way_point.set_ClimbRate(0)
        way_point.set_TurnType(TurnType.TurnShort)
        way_point.set_ContingencyWaypointA(0)
        way_point.set_ContingencyWaypointB(0)
        way_point_list.append(way_point)

        line4_altis, coords = self.__terrian_service.getLineElevs(keep_lat + height_term / 8 * 3,
                                                                  keep_lon + width_term / 8 * 3,
                                                                  keep_lat + height_term / 8 * 3,
                                                                  keep_lon - width_term / 8 * 3, 2)
        line4_alti = max(line4_altis) + self.__maxGap
        # Position 4
        way_point = Waypoint()
        way_point.set_Latitude(keep_lat + height_term / 8 * 3)
        way_point.set_Longitude(keep_lon - width_term / 8 * 3)
        way_point.set_Altitude(line4_alti)
        way_point.set_Number(4)
        way_point.set_NextWaypoint(5)
        way_point.set_Speed(self.__default_speed)
        way_point.set_SpeedType(SpeedType.Airspeed)
        way_point.set_ClimbRate(0)
        way_point.set_TurnType(TurnType.TurnShort)
        way_point.set_ContingencyWaypointA(0)
        way_point.set_ContingencyWaypointB(0)
        way_point_list.append(way_point)

        line5_altis, coords = self.__terrian_service.getLineElevs(keep_lat + height_term / 8 * 3,
                                                                  keep_lon - width_term / 8 * 3,
                                                                  keep_lat - height_term / 8 * 3,
                                                                  keep_lon - width_term / 8 * 3, 2)
        line5_alti = max(line5_altis) + self.__maxGap
        # Position 5
        way_point = Waypoint()
        way_point.set_Latitude(keep_lat - height_term / 8 * 3)
        way_point.set_Longitude(keep_lon - width_term / 8 * 3)
        way_point.set_Altitude(line5_alti)
        way_point.set_Number(5)
        way_point.set_NextWaypoint(2)
        way_point.set_Speed(self.__default_speed)
        way_point.set_SpeedType(SpeedType.Airspeed)
        way_point.set_ClimbRate(0)
        way_point.set_TurnType(TurnType.TurnShort)
        way_point.set_ContingencyWaypointA(0)
        way_point.set_ContingencyWaypointB(0)
        way_point_list.append(way_point)

        loiter_action = LoiterAction()
        loiter_action.set_LoiterType(LoiterType.Circular)
        loiter_action.set_Radius(width_term / 5 * 100000)
        loiter_action.set_Axis(0)
        loiter_action.set_Length(0)
        loiter_action.set_Direction(LoiterDirection.Clockwise)
        loiter_action.set_Duration(100000)
        loiter_action.set_Airspeed(30)

        location = Location3D()
        location.set_AltitudeType(AltitudeType.MSL)
        location.set_Altitude(line1_alti)
        location.set_Longitude(keep_lon)
        location.set_Latitude(keep_lat)
        loiter_action.set_Location(location)

        way_point.VehicleActionList.append(loiter_action)

        self.__client.sendLMCPObject(mission_command)

        keep_num = keep_num + 1

        # ====================================================================================================
        # 2

        width_term = self.__width / self.__num_of_uav_per_row
        height_term = self.__height / self.__num_of_row

        mission_command = MissionCommand()
        mission_command.set_FirstWaypoint(1)
        mission_command.set_VehicleID(self.__uav_id[self.__optimal_location[keep_num]])
        mission_command.set_Status(CommandStatusType.Pending)
        mission_command.set_CommandID(1)

        way_point_list = mission_command.get_WaypointList()

        keep_lon = self.__keep_in_zone[keep_num][0]
        keep_lat = self.__keep_in_zone[keep_num][1]

        uav_id_index = self.__optimal_location[keep_num]
        line1_altis, coords = self.__terrian_service.getLineElevs(self.__uav_total_state[uav_id_index][4][1],
                                                                  self.__uav_total_state[uav_id_index][4][0],
                                                                  keep_lat, keep_lon, 2)
        line1_alti = max(line1_altis) + self.__maxGap
        # Position 1
        way_point = Waypoint()
        way_point.set_Latitude(keep_lat - height_term / 8 * 3)
        way_point.set_Longitude(keep_lon)
        way_point.set_Altitude(line1_alti)
        way_point.set_Number(1)
        way_point.set_NextWaypoint(2)
        way_point.set_Speed(self.__default_speed)
        way_point.set_SpeedType(SpeedType.Airspeed)
        way_point.set_ClimbRate(0)
        way_point.set_TurnType(TurnType.TurnShort)
        way_point.set_ContingencyWaypointA(0)
        way_point.set_ContingencyWaypointB(0)
        way_point_list.append(way_point)

        line2_altis, coords = self.__terrian_service.getLineElevs(keep_lat,
                                                                  keep_lon,
                                                                  keep_lat - height_term / 8 * 3,
                                                                  keep_lon, 2)
        line2_alti = max(line2_altis) + self.__maxGap
        # Position
        way_point = Waypoint()
        way_point.set_Latitude(keep_lat - height_term / 8 * 3)
        way_point.set_Longitude(keep_lon)
        way_point.set_Altitude(line2_alti)
        way_point.set_Number(2)
        way_point.set_NextWaypoint(3)
        way_point.set_Speed(self.__default_speed)
        way_point.set_SpeedType(SpeedType.Airspeed)
        way_point.set_ClimbRate(0)
        way_point.set_TurnType(TurnType.TurnShort)
        way_point.set_ContingencyWaypointA(0)
        way_point.set_ContingencyWaypointB(0)
        way_point_list.append(way_point)

        line3_altis, coords = self.__terrian_service.getLineElevs(keep_lat - height_term / 8 * 3, keep_lon,
                                                                  keep_lat - height_term / 8 * 3,
                                                                  keep_lon + width_term / 8 * 3, 2)
        line3_alti = max(line3_altis) + self.__maxGap
        # Position 2
        way_point = Waypoint()
        way_point.set_Latitude(keep_lat - height_term / 8 * 3)
        way_point.set_Longitude(keep_lon + width_term / 8 * 3)
        way_point.set_Altitude(line3_alti)
        way_point.set_Number(3)
        way_point.set_NextWaypoint(4)
        way_point.set_Speed(self.__default_speed)
        way_point.set_SpeedType(SpeedType.Airspeed)
        way_point.set_ClimbRate(0)
        way_point.set_TurnType(TurnType.TurnShort)
        way_point.set_Contin

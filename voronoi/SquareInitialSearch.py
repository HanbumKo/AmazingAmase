import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from scipy.spatial import Voronoi, voronoi_plot_2d
import math
import sys

class SquareSearch():

    def __init__(self, pointlist, number_keepinzone, number_recoveryzone, numberofdroneeachrecoveryzone):
        self.points = pointlist
        self.number_keepinzone = number_keepinzone
        self.number_recoveryzone = number_recoveryzone
        self.number_drone_each_recoveryzone = numberofdroneeachrecoveryzone
        self.searchcoord = []
        self.searchroute = []
        self.__recovery_points = pointlist[4:]
        self.__num_of_uav = numberofdroneeachrecoveryzone * number_recoveryzone
        self.__keep_in_zone = pointlist[:3]



    def distance(self, uav1lon, uav1lat, uav2lon, uav2lat):
        return math.sqrt((uav1lon - uav2lon) ** 2 + (uav1lat - uav2lat) ** 2)

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

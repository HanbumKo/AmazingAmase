import Drone
import Utils
import VoronoiForInitialSearch

import math
class Searching():

    def __init__(self, utils, NumberofDrones, keepinzone, recoveryzone, startway):
        self.utils = utils
        self.threshold = 0.003

        # nkeepinzone = 4
        # nrecoveryzone = 3
        # numberofdroneeachrecoveryzone = 3
        # pointlist = []

        nkeepinzone = len(keepinzone)
        nrecoveryzone = len(recoveryzone)
        numberofdroneeachrecoveryzone = NumberofDrones / nrecoveryzone
        pointlist = [[] for _ in range(nkeepinzone + nrecoveryzone)]

        for i in range(nkeepinzone):
            pointlist[i] = keepinzone[i]
        for i in range(nrecoveryzone):
            pointlist[i+nkeepinzone] = recoveryzone[i]
        # print("DeBug...")
        '''
        startway
        0 = nearest
        1 = farthest
        2 = smallest
        3 = largest
        '''
        try:
            self.initialsearchpoints = VoronoiForInitialSearch.VoronoiSearch(pointlist, nkeepinzone, nrecoveryzone, numberofdroneeachrecoveryzone, startway)
            # print("DeBug...")
            self.initialsearchpoints.voronoialgo()
            print("SEARCHCOORD\n", self.initialsearchpoints.searchcoord)
            print("SEARCHROUTE\n", self.initialsearchpoints.searchroute)
            self.waypointlists = self.returnwaypointlists()
        except:
            ### TODO : Implement the way to set waypoints without voronoi
            print("CAN'T VORONOI!!")
    def returnwaypointlists(self):
        waypointlist = []
        for i in range(len(self.initialsearchpoints.searchcoord)):
            temp = []
            for j in range(len(self.initialsearchpoints.searchcoord[i])):
                t = self.initialsearchpoints.searchroute[i][j]
                temp.append(self.initialsearchpoints.searchcoord[i][t])
            waypointlist.append(temp)

        return waypointlist

    def getWayPointLists(self): return self.waypointlists


    def updateSearchingState(self, uavInfos):
        # if current drone finished searching current_point => this condition depends on the searching way.

        # uavInfos = { 
        #   OBJ : <uav_object>
        #   STATE : <uav_state>
        #   ACTION : <uav_action>
        #   ACTION_DETAIL : ...
        # }
        # ACTION_DETAIL = {
        #   search_way : 0, 1, 2
        #   current_index : 0
        #   next_heading : 0 ~ 360
        #   total points : loc1-loc2-loc3- ... 
        # } 

        if self.checkWhetherSearched(uavInfos):
            self.movetoNextPoint(uavInfos)
        else : 
            self.updateNextHeading(uavInfos)

    def checkWhetherSearched(self, uavInfos):
        current_seacrh_point_idx = uavInfos['STATE_DETAIL'][Enum.SEARCHING]['current_index']
        current_seacrh_point_loc = uavInfos['STATE_DETAIL'][Enum.SEARCHING]['total_points'][current_seacrh_point_idx]

        uav_lat = uavInfos['OBJ'].getLatitude()
        uav_lon = uavInfos['OBJ'].getLongitude()

        return self.utils.distance(uav_lon, uav_lat, current_seacrh_point_loc[1], current_seacrh_point_loc[0]) <= self.threshold
        
    def movetoNextPoint(self, uavInfos):
        next_seacrh_point_idx = (uavInfos['STATE_DETAIL'][Enum.SEARCHING]['current_index'] + 1)%len(uavInfos['STATE_DETAIL'][Enum.SEARCHING]['total_points'])
        next_seacrh_point_loc = uavInfos['STATE_DETAIL'][Enum.SEARCHING]['total_points'][next_seacrh_point_idx]

        uav_lat = uavInfos['OBJ'].getLatitude()
        uav_lon = uavInfos['OBJ'].getLongitude()

        heading = self.utils.getHeadingToDest(uav_lat, uav_lon, next_seacrh_point_loc[0], next_seacrh_point_loc[1])

        print("next heading will be ", heading)

        # need to fix the direction
        uavInfos['NEXT_HEADING'] = heading
        uavInfos['NEXT_AZIMUTH'] = {'start': -45, 'end': 45, 'rate': 45}

        uavInfos['STATE_DETAIL'][Enum.SEARCHING]['current_index'] = next_seacrh_point_idx 

    def updateNextHeading(self, uavInfos):
        current_seacrh_point_idx = uavInfos['STATE_DETAIL'][Enum.SEARCHING]['current_index']
        current_seacrh_point_loc = uavInfos['STATE_DETAIL'][Enum.SEARCHING]['total_points'][current_seacrh_point_idx]

        uav_lat = uavInfos['OBJ'].getLatitude()
        uav_lon = uavInfos['OBJ'].getLongitude()

        heading = self.utils.getHeadingToDest(uav_lat, uav_lon, current_seacrh_point_loc[0], current_seacrh_point_loc[1])

        # print("next heading will be ", heading)

        # need to fix the direction
        uavInfos['NEXT_HEADING'] = heading
        uavInfos['NEXT_AZIMUTH'] = {'start': -45, 'end': 45, 'rate': 45}



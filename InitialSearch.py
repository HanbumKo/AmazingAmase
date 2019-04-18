import Drone
import Utils
import VoronoiForInitialSearch

import math
class InitialSearch():

    def __init__(self, utils, NumberofDrones, keepinzone, recoveryzone, startway):
        self.utils = utils
        self.threshold = 0.001

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
            pointlist[i+nkeepinzone+1] = recoveryzone[i]

        '''
        startway
        0 = nearest
        1 = farthest
        2 = smallest
        3 = largest
        '''
        self.initialsearchpoints = VoronoiForInitialSearch.VoronoiSearch(pointlist, nkeepinzone, nrecoveryzone, numberofdroneeachrecoveryzone, startway)
        self.initialsearchpoints.voronoialgo()
        print("SEARCHCOORD\n", self.initialsearchpoints.searchcoord)
        print("SEARCHROUTE\n", self.initialsearchpoints.searchroute)
        self.waypointlists = self.returnwaypointlists()
        pass

    def returnwaypointlists(self):
        waypointlist = []
        for i in range(len(self.initialsearchpoints.searchcoord)):
            temp = []
            for j in range(len(self.initialsearchpoints.searchcoord[i])):
                t = self.initialsearchpoints.searchroute[i][j]
                temp.append(self.initialsearchpoints.searchcoord[i][t])
            waypointlist.append(temp)

        return waypointlist



    def updateSearchingPoint(self, uavInfos):
        # if current drone finished searching current_point => this condition depends on the searching way.

        # uavInfos = { 
        #   OBJ : <uav_object>
        #   STATE : <uav_state>
        #   ACTION : <uav_action>
        #   ACTION_DETIAL : ...
        # }
        # ACTION_DETIAL = {
        #   search_way : 0, 1, 2
        #   current_index : 0
        #   next_heading : 0 ~ 360
        #   total points : loc1-loc2-loc3- ... 
        # } 

        if self.checkWhetherSearched(uavInfos):
            self.movetoNextPoint(uavInfos)

    def checkWhetherSearched(self, uavInfos):
        current_seacrh_point_idx = uavInfos.ACTION_DETIAL.current_index
        current_seacrh_point_loc = uavInfos.ACTION_DETIAL.total_points[current_seacrh_point_idx]

        uav_lat = uavInfos.OBJ.get_latitude()
        uav_lon = uavInfos.OBJ.get_longitude()

        return self.utils.distance(uav_lon, uav_lat, current_seacrh_point_loc[1], current_seacrh_point_loc[0]) <= self.threshold
        
    def movetoNextPoint(self, uavInfos):
        current_seacrh_point_idx = (uavInfos.ACTION_DETIAL.current_index + 1)%len(uavInfos.ACTION_DETIAL.total_points)
        current_seacrh_point_loc = uavInfos.ACTION_DETIAL.total_points[current_seacrh_point_idx]

        uav_lat = uavInfos.OBJ.get_latitude()
        uav_lon = uavInfos.OBJ.get_longitude()

        dy = uav_lat - current_seacrh_point_loc[1]
        dx = uav_lon - current_seacrh_point_loc[0]

        heading = math.atan2(dy,dx)*180/math.pi

        print("next heading will be ", heading)

        # need to fix the direction
        uavInfos.NEXT_HEADING = heading
        uavInfos.NEXT_AZIMUTH = {'start': -45, 'end': 45, 'rate': 45}

        uavInfos.ACTION_DETIAL.current_index = current_seacrh_point_idx 



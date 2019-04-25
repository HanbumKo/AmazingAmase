import Drone
import Utils
import VoronoiForInitialSearch
import Enum
import StraightForInitialSearch

import numpy as np
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
            self.initialsearchpoints = VoronoiForInitialSearch.VoronoiSearch(np.array(pointlist), nkeepinzone, nrecoveryzone, numberofdroneeachrecoveryzone, startway)
            print("DeBug...")
            self.waypointlists = self.initialsearchpoints.voronoialgo()
        except:
            ### TODO : Implement the way to set waypoints without voronoi
            self.initialsearchpoints = StraightForInitialSearch.StraightSearch(np.array(pointlist), nkeepinzone, nrecoveryzone, numberofdroneeachrecoveryzone)
            self.waypointlists = self.initialsearchpoints.straightalgo()
            print("CAN'T VORONOI!!")
    
    def setTrackingSection(self, searchMap):
        print(self.waypointlists)
        for i in range(len(self.waypointlists)):
            print(" - RecoveryArea_"+str(i)+"setting")
            searchMap['RecoveryArea_'+str(i)] = {}
            
            for j in range(len(self.waypointlists[i])):
                searchMap['RecoveryArea_'+str(i)]['Section_'+str(j)] = {
                    'waiting' : self.waypointlists[i][j],
                    'searched' : [],
                    'numberOfPoints' : len(self.waypointlists[i][j]),
                    'searchingUavs' : []
                }
            print(" - Done")
        print(searchMap)
    
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


    def updateSearchingState(self, uavInfos, dSection):
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

        if self.checkWhetherSearched(uavInfos, dSection):
            self.movetoNextPoint(uavInfos, dSection)
        else : 
            self.updateNextHeading(uavInfos, dSection)

    def checkWhetherSearched(self, uavInfos, dSection):
        current_seacrh_point_loc = dSection['waiting'][uavInfos.getID()][0]

        uav_lat = uavInfos['OBJ'].getLatitude()
        uav_lon = uavInfos['OBJ'].getLongitude()

        return self.utils.distance(uav_lon, uav_lat, current_seacrh_point_loc[1], current_seacrh_point_loc[0]) <= self.threshold
        
    def movetoNextPoint(self, uavInfos, dSection):
        uavId = uavInfos['OBJ'].getID()
        current_seacrh_point_loc = dSection['waiting'][uavInfos.getID()].pop(0)
        dSection['searched'].append(current_seacrh_point_loc)

        if len(dSection['waiting'][uavInfos.getID()]) != 0:
                
            next_seacrh_point_loc = dSection['waiting'][uavInfos.getID()][0]

            uav_lat = uavInfos['OBJ'].getLatitude()
            uav_lon = uavInfos['OBJ'].getLongitude()

            heading = self.utils.getHeadingToDest(uav_lat, uav_lon, next_seacrh_point_loc[0], next_seacrh_point_loc[1])

            print("next heading will be ", heading)

            # need to fix the direction
            uavInfos['NEXT_HEADING'] = heading
            uavInfos['NEXT_AZIMUTH'] = {'start': -45, 'end': 45, 'rate': 45}
        else :            
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
            uavInfos['STATE'] = Enum.INITIAL_STATE

    def updateNextHeading(self, uavInfos, dSection):        
        current_seacrh_point_loc = dSection['waiting'][uavInfos.getID()][0]

        uav_lat = uavInfos['OBJ'].getLatitude()
        uav_lon = uavInfos['OBJ'].getLongitude()

        heading = self.utils.getHeadingToDest(uav_lat, uav_lon, current_seacrh_point_loc[0], current_seacrh_point_loc[1])

        # print("next heading will be ", heading)

        # need to fix the direction
        uavInfos['NEXT_HEADING'] = heading
        uavInfos['NEXT_AZIMUTH'] = {'start': -45, 'end': 45, 'rate': 45}



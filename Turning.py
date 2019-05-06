import Enum
import Drone
import Utils
import VoronoiForInitialSearch

import math

class Turning():

    def __init__(self, utils):
        self.utils = utils
    def updateTurningState(self, uavInfos):
        threshold = 10

        current_heading = uavInfos['OBJ'].getHeading()

        if abs(abs(current_heading) - abs(uavInfos['STATE_DETAIL'][Enum.TURNING]['heading'])) <= threshold:
            # if uavInfos['STATE_DETAIL'][Enum.APPROACHING]['tracking_zoneID'] != -1:
            #     uavInfos['STATE'] = Enum.APPROACHING

            #     uav_lat = uavInfos['OBJ'].getLatitude()
            #     uav_lon = uavInfos['OBJ'].getLongitude()

            #     self.utils.sendWaypointsCmd(uavInfos['OBJ'].getID(), uav_lat, uav_lon, uavInfos['STATE_DETAIL'][Enum.APPROACHING]['dest_position'][0], uavInfos['STATE_DETAIL'][Enum.APPROACHING]['dest_position'][1], 15)

            uavInfos['STATE'] = Enum.INITIAL_STATE
        else :
            uavInfos['NEXT_HEADING'] = uavInfos['STATE_DETAIL'][Enum.TURNING]['heading']

    def updateTurning2State(self, uavInfos):
        threshold = 10

        current_heading = uavInfos['OBJ'].getHeading()
        current_altitude = uavInfos['OBJ'].getAltitude()

        if abs(abs(current_altitude) - abs(uavInfos['STATE_DETAIL'][Enum.TURNING2]['altitude'])) <= threshold:
            uavInfos['STATE'] = uavInfos['STATE_DETAIL'][Enum.TURNING2]['prev_state']
        else :
            uavInfos['NEXT_ALTITUDE'] = uavInfos['STATE_DETAIL'][Enum.TURNING2]['altitude']
            uavInfos['NEXT_HEADING'] = current_heading - 90
            pass

    def turning(self, point, uavInfos):
        uav_lat = uavInfos['OBJ'].getLatitude()
        uav_lon = uavInfos['OBJ'].getLongitude()

        det_lat = point[0]
        det_lon = point[1]

        uavInfos['STATE_DETAIL'][Enum.TURNING]['heading'] = self.utils.getHeadingToDest(det_lat, det_lon, uav_lat, uav_lon)
        uavInfos['NEXT_HEADING'] = uavInfos['STATE_DETAIL'][Enum.TURNING]['heading'] 
        uavInfos['STATE'] = Enum.TURNING
        print("UAV -",uavInfos['OBJ'].getID()," is turning")

    def turningForChangingAlt(self, altitude, uavInfos):
        uavInfos['STATE_DETAIL'][Enum.TURNING2]['altitude'] = altitude
        uavInfos['STATE_DETAIL'][Enum.TURNING2]['prev_state'] = uavInfos['STATE']
        uavInfos['STATE'] = Enum.TURNING2
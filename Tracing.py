import Drone
import Utils
import Enum
import math
import sys

class Tracing():

    def __init__(self, utils):
        self.utils = utils
        self.change = 5

        pass
    
    def updateTrackingState(self, uavInfos, hazardZone):
        # uavInfos = { 
        #   OBJ : <uav_object>
        #   STATE : <uav_state>
        #   ACTION : <uav_action>
        #   ACTION_DETIAL : ...
        # }
        # ACTION_DETIAL = {
        #   tracking_direction : clockwise, unclockwise
        #   tracking_zoneID : 0
        #   last_tracking_time : 0,
        #   next_azimuth : 0,
        #   next_heading : 0,
        #   msg : 0
        # }
        if self.isStillTracking(uavInfos):
            #Still tracking
            self.gointoZone(uavInfos)
        else : 
            # got lost
            print(" - Re APPROACHING")
            uavInfos['STATE'] = Enum.APPROACHING
            fShortestDist = sys.maxsize
            for point in hazardZone['area']:
                fDist = self.utils.distance(point.get_Longitude(), point.get_Latitude(), uavInfos['OBJ'].getLongitude(), uavInfos['OBJ'].getLatitude()) 
                if fDist < fShortestDist:
                    fShortestDist = fDist
                    uavInfos['STATE_DETAIL'][Enum.APPROACHING]['dest_position'] = [point.get_Latitude(), point.get_Longitude()]
            print(" - Got nice point")
            uav_lat = uavInfos['OBJ'].getLatitude()
            uav_lon = uavInfos['OBJ'].getLongitude()

            self.utils.sendWaypointsCmd(uavInfos['OBJ'].getID(), uav_lat, uav_lon, uavInfos['STATE_DETAIL'][Enum.APPROACHING]['dest_position'][0], uavInfos['STATE_DETAIL'][Enum.APPROACHING]['dest_position'][1], 
                                        self.utils.getIdealSpeed(uavInfos))
            print(" - Send Waypoints list")
            uavInfos['STATE_DETAIL'][Enum.SEARCHING]['is_scanning'] = False
            # uavInfos['STATE_DETAIL'][Enum.APPROACHING]['turning'] = False
            uavInfos['NEXT_AZIMUTH'] = {'start': -45, 'end': 45, 'rate': 45}       
            print(" - Done")

    def isStillTracking(self, uavInfos):
        # tracking condition : interval between current time and last_tracking_time
        last_tracking_time = uavInfos['STATE_DETAIL'][Enum.TRACING]['last_tracking_time']
        current_time = uavInfos['OBJ'].getTime()

        return (current_time - last_tracking_time)//1000 < 35
    
    def gointoZone(self, uavInfos):
        msg = uavInfos['STATE_DETAIL'][Enum.TRACING]['msg']

        if msg == 1:
            uavInfos['STATE_DETAIL'][Enum.TRACING]['msg'] = -1
        elif msg == -1 :
            originalDirection, gap = self.utils.getOriginalDirection(uavInfos)
            azimuth = uavInfos['OBJ'].getCameraAzimuth()
            direction = uavInfos['STATE_DETAIL'][Enum.TRACING]['tracking_direction']

            # course 쪽이냐, heading 쪽이냐
            if self.adaptWindState(originalDirection+direction*(self.change)*(-1), gap, uavInfos) :
                uavInfos['NEXT_HEADING'] =  originalDirection+direction*(self.change+gap)*(-1)
            else :
                uavInfos['NEXT_HEADING'] =  originalDirection+direction*(self.change+gap)*(-1)

            if azimuth > 60 or azimuth < -60 :
                uavInfos['NEXT_AZIMUTH'] = azimuth + direction*(self.change)

    def gooutFromZone(self, uavInfos):
        direction = uavInfos['STATE_DETAIL'][Enum.TRACING]['tracking_direction']
        uavInfos['STATE_DETAIL'][Enum.TRACING]['msg'] = 1

        originalDirection, gap = self.utils.getOriginalDirection(uavInfos)
        azimuth = uavInfos['OBJ'].getCameraAzimuth()

        # course 쪽이냐, heading 쪽이냐
        if self.adaptWindState(originalDirection+direction*(self.change), gap, uavInfos) :
            uavInfos['NEXT_HEADING'] =  originalDirection+direction*(self.change+gap)
        else :
            uavInfos['NEXT_HEADING'] =  originalDirection+direction*(self.change+gap)


        if azimuth < 60 and azimuth > -60 :
            uavInfos['NEXT_AZIMUTH'] = azimuth + direction*(self.change)*(-1)
    
    def adaptWindState(self, nextHeading, gap, uavInfos):

        heading = uavInfos['OBJ'].getHeading()
        course = uavInfos['OBJ'].getCourse()

        if heading < 0 :
            heading += 360 

        if course < 0 :
            course += 360 
        
        iGapNextHeadingAndCurHeading = abs(nextHeading - heading)
        iGapNextHeadingAndCourse = abs(nextHeading - course)

        if iGapNextHeadingAndCourse < iGapNextHeadingAndCurHeading : 
            return True
        else : return False

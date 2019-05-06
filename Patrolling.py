import Drone
import Utils
import Enum
import math

class Patrolling():

    def __init__(self, utils):
        self.utils = utils

        pass
    
    def updatePatrollingState(self, uavInfos, smokeZone):
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
        if self.isStillPatrolling(uavInfos):
            #Still tracking
            self.turnRightAngle(uavInfos)
        else : 
            # got lost
            uav_lat = uavInfos['OBJ'].getLatitude()
            uav_lon = uavInfos['OBJ'].getLongitude()

            uavInfos['NEXT_HEADING'] =  self.utils.getHeadingToDest(uav_lat, uav_lon, smokeZone['point'][0], smokeZone['point'][1])
        
    def isStillPatrolling(self, uavInfos):
        # tracking condition : interval between current time and last_tracking_time
        last_tracking_time = uavInfos['STATE_DETAIL'][Enum.PATROLLING]['last_patrolling_time']
        current_time = uavInfos['OBJ'].getTime()

        return (current_time - last_tracking_time)//1000 < 15

    
    def turnRightAngle(self, uavInfos):
        msgForCheck = uavInfos['STATE_DETAIL'][Enum.PATROLLING]['msgForCheck']
        msgForCmd = uavInfos['STATE_DETAIL'][Enum.PATROLLING]['msgForCmd']

        if msgForCheck == 1 :
            uavInfos['STATE_DETAIL'][Enum.PATROLLING]['msgForCheck'] = -1
        elif msgForCheck == -1 and msgForCmd != 1:
            originalDirection = self.getOriginalDirection(uavInfos)
            uavInfos['NEXT_HEADING'] = originalDirection+90
            uavInfos['STATE_DETAIL'][Enum.PATROLLING]['msgForCmd'] = 1

    def goStraight(self, uavInfos):
        uavInfos['STATE_DETAIL'][Enum.PATROLLING]['msgForCheck'] = 1
        uavInfos['STATE_DETAIL'][Enum.PATROLLING]['msgForCmd'] = 0

        originalDirection = self.getOriginalDirection(uavInfos)
        uavInfos['NEXT_HEADING'] =  originalDirection

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

        return originalDirection
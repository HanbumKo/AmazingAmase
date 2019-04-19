import Drone
import Utils
import Enum
import math

class Tracking():

    def __init__(self, utils):
        self.utils = utils
        self.change = 5

        pass
    
    def updateTrackingState(self, uavInfos):
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
            uavInfos['ACTION'] = Enum.ACTION_INITIAL_SEARCHING
        
    def isStillTracking(self, uavInfos):
        # tracking condition : interval between current time and last_tracking_time
        last_tracking_time = uavInfos['ACTION_DETAIL']['TRACKING']['last_tracking_time']
        current_time = uavInfos['OBJ'].getTime()

        return (current_time - last_tracking_time)//1000 < 45
    
    def gointoZone(self, uavInfos):
        msg = uavInfos['ACTION_DETAIL']['TRACKING']['msg']

        if msg == 1:
            uavInfos['ACTION_DETAIL']['TRACKING']['msg'] = -1
        elif msg == -1 :
            originalDirection = self.getOriginalDirection(uavInfos)
            azimuth = uavInfos['OBJ'].getCameraAzimuth()
            direction = uavInfos['ACTION_DETAIL']['TRACKING']['tracking_direction']

            uavInfos['NEXT_HEADING'] = originalDirection+direction*(self.change)*(-1)
            if azimuth > 45 or azimuth < -45 :
                uavInfos['NEXT_AZIMUTH'] = azimuth + direction*(self.change)

    def gooutFromZone(self, uavInfos):
        direction = uavInfos['ACTION_DETAIL']['TRACKING']['tracking_direction']
        uavInfos['ACTION_DETAIL']['TRACKING']['msg'] = 1

        originalDirection = self.getOriginalDirection(uavInfos)
        azimuth = uavInfos['OBJ'].getCameraAzimuth()


        uavInfos['NEXT_HEADING'] =  originalDirection+direction*(self.change)

        if azimuth < 90 and azimuth > -90:
            uavInfos['NEXT_AZIMUTH'] = azimuth + direction*(self.change)*(-1)
            

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
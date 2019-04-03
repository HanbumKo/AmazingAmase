from lmcp.LMCPObject import *

## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

from afrl.cmasi.searchai import HazardZone
from afrl.cmasi.searchai import HazardZoneDetection
from afrl.cmasi.searchai import HazardZoneEstimateReport
from afrl.cmasi.searchai import RecoveryPoint
from afrl.cmasi.searchai import HazardZoneChangeCommand
from afrl.cmasi.searchai import HazardSensorConfiguration
from afrl.cmasi.searchai import HazardSensorState


SERIES_NAME = "SEARCHAI"
#Series Name turned into a long for quick comparisons.
SERIES_NAME_ID = 6000273900112986441
SERIES_VERSION = 5


class SeriesEnum:

    def getName(self, type_):
        if(type_ ==  1): return "HazardZone"
        if(type_ ==  2): return "HazardZoneDetection"
        if(type_ ==  3): return "HazardZoneEstimateReport"
        if(type_ ==  4): return "RecoveryPoint"
        if(type_ ==  5): return "HazardZoneChangeCommand"
        if(type_ ==  6): return "HazardSensorConfiguration"
        if(type_ ==  7): return "HazardSensorState"


    def getType(self, name):
        if ( name == "HazardZone"): return 1
        if ( name == "HazardZoneDetection"): return 2
        if ( name == "HazardZoneEstimateReport"): return 3
        if ( name == "RecoveryPoint"): return 4
        if ( name == "HazardZoneChangeCommand"): return 5
        if ( name == "HazardSensorConfiguration"): return 6
        if ( name == "HazardSensorState"): return 7

        return -1

    def getInstance(self, type_):
        if(type_ ==  1): return HazardZone.HazardZone()
        if(type_ ==  2): return HazardZoneDetection.HazardZoneDetection()
        if(type_ ==  3): return HazardZoneEstimateReport.HazardZoneEstimateReport()
        if(type_ ==  4): return RecoveryPoint.RecoveryPoint()
        if(type_ ==  5): return HazardZoneChangeCommand.HazardZoneChangeCommand()
        if(type_ ==  6): return HazardSensorConfiguration.HazardSensorConfiguration()
        if(type_ ==  7): return HazardSensorState.HazardSensorState()

        return None

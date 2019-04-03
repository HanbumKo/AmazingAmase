#! /usr/bin/python

import sys, struct
import xml.dom.minidom
from lmcp import LMCPObject

## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

from afrl.cmasi import CameraState


class HazardSensorState(CameraState.CameraState):

    def __init__(self):
        CameraState.CameraState.__init__(self)
        self.LMCP_TYPE = 7
        self.SERIES_NAME = "SEARCHAI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.searchai.HazardSensorState"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6000273900112986441
        self.SERIES_VERSION = 5

        #Define message fields


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(CameraState.CameraState.pack(self))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = CameraState.CameraState.unpack(self, buffer, _pos)
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        CameraState.CameraState.unpackFromXMLNode(self, el, seriesFactory)

        return

    def unpackFromDict(self, d, seriesFactory):
        CameraState.CameraState.unpackFromDict(self, d, seriesFactory)

        return



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = CameraState.CameraState.toString(self)
        buf += "From HazardSensorState:\n"

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if ("SEARCHAI" is None) or ("SEARCHAI" is ""): # this should never happen
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/HazardSensorState")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("SEARCHAI" + "/HazardSensorState")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        CameraState.CameraState.toDictMembers(self, d)

        return

    def getLMCPType(self):
        return self.LMCP_TYPE

    def getSeriesName(self):
        return self.SERIES_NAME

    def getSeriesNameID(self):
        return self.SERIES_NAME_ID

    def getSeriesVersion(self):
        return self.SERIES_VERSION

    def toXMLStr(self, ws):
        str = ws + '<HazardSensorState Series="SEARCHAI" >\n';
        #str +=CameraState.CameraState.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</HazardSensorState>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += CameraState.CameraState.toXMLMembersStr(self, ws)

        return buf
        

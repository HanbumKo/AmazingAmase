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

from afrl.cmasi import AbstractZone
from afrl.cmasi.searchai import HazardType


class HazardZone(AbstractZone.AbstractZone):

    def __init__(self):
        AbstractZone.AbstractZone.__init__(self)
        self.LMCP_TYPE = 1
        self.SERIES_NAME = "SEARCHAI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.searchai.HazardZone"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6000273900112986441
        self.SERIES_VERSION = 5

        #Define message fields
        self.ZoneType = HazardType.HazardType.Undefined   #HazardType


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(AbstractZone.AbstractZone.pack(self))
        buffer.extend(struct.pack(">i", self.ZoneType))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = AbstractZone.AbstractZone.unpack(self, buffer, _pos)
        self.ZoneType = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        AbstractZone.AbstractZone.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "ZoneType" and len(e.childNodes) > 0 :
                    self.ZoneType = HazardType.get_HazardType_str(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        AbstractZone.AbstractZone.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "ZoneType":
                self.ZoneType = d[key]

        return

    def get_ZoneType(self):
        return self.ZoneType

    def set_ZoneType(self, value):
        self.ZoneType = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = AbstractZone.AbstractZone.toString(self)
        buf += "From HazardZone:\n"
        buf +=    "ZoneType = " + str( self.ZoneType ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if ("SEARCHAI" is None) or ("SEARCHAI" is ""): # this should never happen
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/HazardZone")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("SEARCHAI" + "/HazardZone")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        AbstractZone.AbstractZone.toDictMembers(self, d)
        d['ZoneType'] = self.ZoneType

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
        str = ws + '<HazardZone Series="SEARCHAI" >\n';
        #str +=AbstractZone.AbstractZone.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</HazardZone>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += AbstractZone.AbstractZone.toXMLMembersStr(self, ws)
        buf += ws + "<ZoneType>" + HazardType.get_HazardType_int(self.ZoneType) + "</ZoneType>\n"

        return buf
        

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


class RecoveryPoint(AbstractZone.AbstractZone):

    def __init__(self):
        AbstractZone.AbstractZone.__init__(self)
        self.LMCP_TYPE = 4
        self.SERIES_NAME = "SEARCHAI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.searchai.RecoveryPoint"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6000273900112986441
        self.SERIES_VERSION = 5

        #Define message fields
        self.LocationName = ""   #string


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(AbstractZone.AbstractZone.pack(self))
        buffer.extend(struct.pack(">H", len(self.LocationName) ))
        if len(self.LocationName) > 0:
            if (sys.version_info > (3, 0)):
                buffer.extend(struct.pack( repr(len(self.LocationName)) + "s", bytearray(self.LocationName,'ascii')))
            else:
                buffer.extend(struct.pack( repr(len(self.LocationName)) + "s", self.LocationName))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = AbstractZone.AbstractZone.unpack(self, buffer, _pos)
        _strlen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        if _strlen > 0:
            self.LocationName = struct.unpack_from( repr(_strlen) + "s", buffer, _pos )[0]
            _pos += _strlen
        else:
             self.LocationName = ""
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        AbstractZone.AbstractZone.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "LocationName" and len(e.childNodes) > 0 :
                    self.LocationName = str(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        AbstractZone.AbstractZone.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "LocationName":
                self.LocationName = d[key]

        return

    def get_LocationName(self):
        return self.LocationName

    def set_LocationName(self, value):
        self.LocationName = str( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = AbstractZone.AbstractZone.toString(self)
        buf += "From RecoveryPoint:\n"
        buf +=    "LocationName = " + str( self.LocationName ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if ("SEARCHAI" is None) or ("SEARCHAI" is ""): # this should never happen
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/RecoveryPoint")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("SEARCHAI" + "/RecoveryPoint")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        AbstractZone.AbstractZone.toDictMembers(self, d)
        d['LocationName'] = self.LocationName

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
        str = ws + '<RecoveryPoint Series="SEARCHAI" >\n';
        #str +=AbstractZone.AbstractZone.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</RecoveryPoint>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += AbstractZone.AbstractZone.toXMLMembersStr(self, ws)
        buf += ws + "<LocationName>" + str(self.LocationName) + "</LocationName>\n"

        return buf
        

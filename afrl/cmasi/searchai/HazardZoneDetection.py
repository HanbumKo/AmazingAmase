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

from afrl.cmasi import Location3D
from afrl.cmasi.searchai import HazardType


class HazardZoneDetection(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 2
        self.SERIES_NAME = "SEARCHAI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.searchai.HazardZoneDetection"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6000273900112986441
        self.SERIES_VERSION = 5

        #Define message fields
        self.DetectedLocation = Location3D.Location3D()   #Location3D
        self.SensorPayloadID = 0   #uint32
        self.DetectingEnitiyID = 0   #uint32
        self.DetectedHazardZoneType = HazardType.HazardType.Undefined   #HazardType


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack("B", self.DetectedLocation != None ))
        if self.DetectedLocation != None:
            buffer.extend(struct.pack(">q", self.DetectedLocation.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.DetectedLocation.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.DetectedLocation.SERIES_VERSION))
            buffer.extend(self.DetectedLocation.pack())
        buffer.extend(struct.pack(">I", self.SensorPayloadID))
        buffer.extend(struct.pack(">I", self.DetectingEnitiyID))
        buffer.extend(struct.pack(">i", self.DetectedHazardZoneType))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        _valid = struct.unpack_from("B", buffer, _pos )[0]
        _pos += 1
        if _valid:
            _series = struct.unpack_from(">q", buffer, _pos)[0]
            _pos += 8
            _type = struct.unpack_from(">I", buffer, _pos)[0]
            _pos += 4
            _version = struct.unpack_from(">H", buffer, _pos)[0]
            _pos += 2
            from lmcp import LMCPFactory
            self.DetectedLocation = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.DetectedLocation.unpack(buffer, _pos)
        else:
            self.DetectedLocation = None
        self.SensorPayloadID = struct.unpack_from(">I", buffer, _pos)[0]
        _pos += 4
        self.DetectingEnitiyID = struct.unpack_from(">I", buffer, _pos)[0]
        _pos += 4
        self.DetectedHazardZoneType = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "DetectedLocation" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.DetectedLocation = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.DetectedLocation != None:
                                self.DetectedLocation.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "SensorPayloadID" and len(e.childNodes) > 0 :
                    self.SensorPayloadID = int(e.childNodes[0].nodeValue)
                elif e.localName == "DetectingEnitiyID" and len(e.childNodes) > 0 :
                    self.DetectingEnitiyID = int(e.childNodes[0].nodeValue)
                elif e.localName == "DetectedHazardZoneType" and len(e.childNodes) > 0 :
                    self.DetectedHazardZoneType = HazardType.get_HazardType_str(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "DetectedLocation":
                self.DetectedLocation = seriesFactory.unpackFromDict(d[key])
            elif key == "SensorPayloadID":
                self.SensorPayloadID = d[key]
            elif key == "DetectingEnitiyID":
                self.DetectingEnitiyID = d[key]
            elif key == "DetectedHazardZoneType":
                self.DetectedHazardZoneType = d[key]

        return

    def get_DetectedLocation(self):
        return self.DetectedLocation

    def set_DetectedLocation(self, value):
        self.DetectedLocation = value 

    def get_SensorPayloadID(self):
        return self.SensorPayloadID

    def set_SensorPayloadID(self, value):
        self.SensorPayloadID = int( value )

    def get_DetectingEnitiyID(self):
        return self.DetectingEnitiyID

    def set_DetectingEnitiyID(self, value):
        self.DetectingEnitiyID = int( value )

    def get_DetectedHazardZoneType(self):
        return self.DetectedHazardZoneType

    def set_DetectedHazardZoneType(self, value):
        self.DetectedHazardZoneType = value 



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From HazardZoneDetection:\n"
        buf +=    "DetectedLocation = " + str( self.DetectedLocation ) + "\n" 
        buf +=    "SensorPayloadID = " + str( self.SensorPayloadID ) + "\n" 
        buf +=    "DetectingEnitiyID = " + str( self.DetectingEnitiyID ) + "\n" 
        buf +=    "DetectedHazardZoneType = " + str( self.DetectedHazardZoneType ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if ("SEARCHAI" is None) or ("SEARCHAI" is ""): # this should never happen
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/HazardZoneDetection")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("SEARCHAI" + "/HazardZoneDetection")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        if self.DetectedLocation == None:
            d['DetectedLocation'] = None
        else:
            d['DetectedLocation'] = self.DetectedLocation.toDict()
        d['SensorPayloadID'] = self.SensorPayloadID
        d['DetectingEnitiyID'] = self.DetectingEnitiyID
        d['DetectedHazardZoneType'] = self.DetectedHazardZoneType

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
        str = ws + '<HazardZoneDetection Series="SEARCHAI" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</HazardZoneDetection>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<DetectedLocation>\n"
        if self.DetectedLocation == None:
            buf += ws + "    <null/>\n"
        else:
            buf += ws + self.DetectedLocation.toXMLStr(ws + "    ") 
        buf += ws + "</DetectedLocation>\n"
        buf += ws + "<SensorPayloadID>" + str(self.SensorPayloadID) + "</SensorPayloadID>\n"
        buf += ws + "<DetectingEnitiyID>" + str(self.DetectingEnitiyID) + "</DetectingEnitiyID>\n"
        buf += ws + "<DetectedHazardZoneType>" + HazardType.get_HazardType_int(self.DetectedHazardZoneType) + "</DetectedHazardZoneType>\n"

        return buf
        

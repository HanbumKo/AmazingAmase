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

from afrl.cmasi import AbstractGeometry
from afrl.cmasi.searchai import HazardType


class HazardZoneEstimateReport(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 3
        self.SERIES_NAME = "SEARCHAI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.searchai.HazardZoneEstimateReport"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6000273900112986441
        self.SERIES_VERSION = 5

        #Define message fields
        self.UniqueTrackingID = 0   #uint32
        self.EstimatedZoneShape = AbstractGeometry.AbstractGeometry()   #AbstractGeometry
        self.EstimatedGrowthRate = 0   #real32
        self.PerceivedZoneType = HazardType.HazardType.Undefined   #HazardType
        self.EstimatedZoneDirection = 0   #real32
        self.EstimatedZoneSpeed = 0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">I", self.UniqueTrackingID))
        buffer.extend(struct.pack("B", self.EstimatedZoneShape != None ))
        if self.EstimatedZoneShape != None:
            buffer.extend(struct.pack(">q", self.EstimatedZoneShape.SERIES_NAME_ID))
            buffer.extend(struct.pack(">I", self.EstimatedZoneShape.LMCP_TYPE))
            buffer.extend(struct.pack(">H", self.EstimatedZoneShape.SERIES_VERSION))
            buffer.extend(self.EstimatedZoneShape.pack())
        buffer.extend(struct.pack(">f", self.EstimatedGrowthRate))
        buffer.extend(struct.pack(">i", self.PerceivedZoneType))
        buffer.extend(struct.pack(">f", self.EstimatedZoneDirection))
        buffer.extend(struct.pack(">f", self.EstimatedZoneSpeed))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.UniqueTrackingID = struct.unpack_from(">I", buffer, _pos)[0]
        _pos += 4
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
            self.EstimatedZoneShape = LMCPFactory.LMCPFactory().createObject(_series, _version, _type )
            _pos = self.EstimatedZoneShape.unpack(buffer, _pos)
        else:
            self.EstimatedZoneShape = None
        self.EstimatedGrowthRate = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.PerceivedZoneType = struct.unpack_from(">i", buffer, _pos)[0]
        _pos += 4
        self.EstimatedZoneDirection = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.EstimatedZoneSpeed = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "UniqueTrackingID" and len(e.childNodes) > 0 :
                    self.UniqueTrackingID = int(e.childNodes[0].nodeValue)
                elif e.localName == "EstimatedZoneShape" and len(e.childNodes) > 0 :
                    for n in e.childNodes:
                        if n.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.EstimatedZoneShape = seriesFactory.createObjectByName(n.getAttribute('Series'), n.localName)
                            if self.EstimatedZoneShape != None:
                                self.EstimatedZoneShape.unpackFromXMLNode(n, seriesFactory)
                elif e.localName == "EstimatedGrowthRate" and len(e.childNodes) > 0 :
                    self.EstimatedGrowthRate = float(e.childNodes[0].nodeValue)
                elif e.localName == "PerceivedZoneType" and len(e.childNodes) > 0 :
                    self.PerceivedZoneType = HazardType.get_HazardType_str(e.childNodes[0].nodeValue)
                elif e.localName == "EstimatedZoneDirection" and len(e.childNodes) > 0 :
                    self.EstimatedZoneDirection = float(e.childNodes[0].nodeValue)
                elif e.localName == "EstimatedZoneSpeed" and len(e.childNodes) > 0 :
                    self.EstimatedZoneSpeed = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "UniqueTrackingID":
                self.UniqueTrackingID = d[key]
            elif key == "EstimatedZoneShape":
                self.EstimatedZoneShape = seriesFactory.unpackFromDict(d[key])
            elif key == "EstimatedGrowthRate":
                self.EstimatedGrowthRate = d[key]
            elif key == "PerceivedZoneType":
                self.PerceivedZoneType = d[key]
            elif key == "EstimatedZoneDirection":
                self.EstimatedZoneDirection = d[key]
            elif key == "EstimatedZoneSpeed":
                self.EstimatedZoneSpeed = d[key]

        return

    def get_UniqueTrackingID(self):
        return self.UniqueTrackingID

    def set_UniqueTrackingID(self, value):
        self.UniqueTrackingID = int( value )

    def get_EstimatedZoneShape(self):
        return self.EstimatedZoneShape

    def set_EstimatedZoneShape(self, value):
        self.EstimatedZoneShape = value 

    def get_EstimatedGrowthRate(self):
        return self.EstimatedGrowthRate

    def set_EstimatedGrowthRate(self, value):
        self.EstimatedGrowthRate = float( value )

    def get_PerceivedZoneType(self):
        return self.PerceivedZoneType

    def set_PerceivedZoneType(self, value):
        self.PerceivedZoneType = value 

    def get_EstimatedZoneDirection(self):
        return self.EstimatedZoneDirection

    def set_EstimatedZoneDirection(self, value):
        self.EstimatedZoneDirection = float( value )

    def get_EstimatedZoneSpeed(self):
        return self.EstimatedZoneSpeed

    def set_EstimatedZoneSpeed(self, value):
        self.EstimatedZoneSpeed = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From HazardZoneEstimateReport:\n"
        buf +=    "UniqueTrackingID = " + str( self.UniqueTrackingID ) + "\n" 
        buf +=    "EstimatedZoneShape = " + str( self.EstimatedZoneShape ) + "\n" 
        buf +=    "EstimatedGrowthRate = " + str( self.EstimatedGrowthRate ) + "\n" 
        buf +=    "PerceivedZoneType = " + str( self.PerceivedZoneType ) + "\n" 
        buf +=    "EstimatedZoneDirection = " + str( self.EstimatedZoneDirection ) + "\n" 
        buf +=    "EstimatedZoneSpeed = " + str( self.EstimatedZoneSpeed ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if ("SEARCHAI" is None) or ("SEARCHAI" is ""): # this should never happen
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/HazardZoneEstimateReport")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("SEARCHAI" + "/HazardZoneEstimateReport")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['UniqueTrackingID'] = self.UniqueTrackingID
        if self.EstimatedZoneShape == None:
            d['EstimatedZoneShape'] = None
        else:
            d['EstimatedZoneShape'] = self.EstimatedZoneShape.toDict()
        d['EstimatedGrowthRate'] = self.EstimatedGrowthRate
        d['PerceivedZoneType'] = self.PerceivedZoneType
        d['EstimatedZoneDirection'] = self.EstimatedZoneDirection
        d['EstimatedZoneSpeed'] = self.EstimatedZoneSpeed

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
        str = ws + '<HazardZoneEstimateReport Series="SEARCHAI" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</HazardZoneEstimateReport>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<UniqueTrackingID>" + str(self.UniqueTrackingID) + "</UniqueTrackingID>\n"
        buf += ws + "<EstimatedZoneShape>\n"
        if self.EstimatedZoneShape == None:
            buf += ws + "    <null/>\n"
        else:
            buf += ws + self.EstimatedZoneShape.toXMLStr(ws + "    ") 
        buf += ws + "</EstimatedZoneShape>\n"
        buf += ws + "<EstimatedGrowthRate>" + str(self.EstimatedGrowthRate) + "</EstimatedGrowthRate>\n"
        buf += ws + "<PerceivedZoneType>" + HazardType.get_HazardType_int(self.PerceivedZoneType) + "</PerceivedZoneType>\n"
        buf += ws + "<EstimatedZoneDirection>" + str(self.EstimatedZoneDirection) + "</EstimatedZoneDirection>\n"
        buf += ws + "<EstimatedZoneSpeed>" + str(self.EstimatedZoneSpeed) + "</EstimatedZoneSpeed>\n"

        return buf
        

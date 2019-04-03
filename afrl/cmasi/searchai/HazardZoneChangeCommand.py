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



class HazardZoneChangeCommand(LMCPObject.LMCPObject):

    def __init__(self):

        self.LMCP_TYPE = 5
        self.SERIES_NAME = "SEARCHAI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.searchai.HazardZoneChangeCommand"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6000273900112986441
        self.SERIES_VERSION = 5

        #Define message fields
        self.ZoneID = 0   #uint32
        self.GrowthRate = 0   #real32
        self.TranslationRate = 0   #real32
        self.TranslationDirection = 0   #real32


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(LMCPObject.LMCPObject.pack(self))
        buffer.extend(struct.pack(">I", self.ZoneID))
        buffer.extend(struct.pack(">f", self.GrowthRate))
        buffer.extend(struct.pack(">f", self.TranslationRate))
        buffer.extend(struct.pack(">f", self.TranslationDirection))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = LMCPObject.LMCPObject.unpack(self, buffer, _pos)
        self.ZoneID = struct.unpack_from(">I", buffer, _pos)[0]
        _pos += 4
        self.GrowthRate = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.TranslationRate = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.TranslationDirection = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        LMCPObject.LMCPObject.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "ZoneID" and len(e.childNodes) > 0 :
                    self.ZoneID = int(e.childNodes[0].nodeValue)
                elif e.localName == "GrowthRate" and len(e.childNodes) > 0 :
                    self.GrowthRate = float(e.childNodes[0].nodeValue)
                elif e.localName == "TranslationRate" and len(e.childNodes) > 0 :
                    self.TranslationRate = float(e.childNodes[0].nodeValue)
                elif e.localName == "TranslationDirection" and len(e.childNodes) > 0 :
                    self.TranslationDirection = float(e.childNodes[0].nodeValue)

        return

    def unpackFromDict(self, d, seriesFactory):
        LMCPObject.LMCPObject.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "ZoneID":
                self.ZoneID = d[key]
            elif key == "GrowthRate":
                self.GrowthRate = d[key]
            elif key == "TranslationRate":
                self.TranslationRate = d[key]
            elif key == "TranslationDirection":
                self.TranslationDirection = d[key]

        return

    def get_ZoneID(self):
        return self.ZoneID

    def set_ZoneID(self, value):
        self.ZoneID = int( value )

    def get_GrowthRate(self):
        return self.GrowthRate

    def set_GrowthRate(self, value):
        self.GrowthRate = float( value )

    def get_TranslationRate(self):
        return self.TranslationRate

    def set_TranslationRate(self, value):
        self.TranslationRate = float( value )

    def get_TranslationDirection(self):
        return self.TranslationDirection

    def set_TranslationDirection(self, value):
        self.TranslationDirection = float( value )



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = LMCPObject.LMCPObject.toString(self)
        buf += "From HazardZoneChangeCommand:\n"
        buf +=    "ZoneID = " + str( self.ZoneID ) + "\n" 
        buf +=    "GrowthRate = " + str( self.GrowthRate ) + "\n" 
        buf +=    "TranslationRate = " + str( self.TranslationRate ) + "\n" 
        buf +=    "TranslationDirection = " + str( self.TranslationDirection ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if ("SEARCHAI" is None) or ("SEARCHAI" is ""): # this should never happen
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/HazardZoneChangeCommand")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("SEARCHAI" + "/HazardZoneChangeCommand")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        LMCPObject.LMCPObject.toDictMembers(self, d)
        d['ZoneID'] = self.ZoneID
        d['GrowthRate'] = self.GrowthRate
        d['TranslationRate'] = self.TranslationRate
        d['TranslationDirection'] = self.TranslationDirection

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
        str = ws + '<HazardZoneChangeCommand Series="SEARCHAI" >\n';
        #str +=LMCPObject.LMCPObject.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</HazardZoneChangeCommand>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += LMCPObject.LMCPObject.toXMLMembersStr(self, ws)
        buf += ws + "<ZoneID>" + str(self.ZoneID) + "</ZoneID>\n"
        buf += ws + "<GrowthRate>" + str(self.GrowthRate) + "</GrowthRate>\n"
        buf += ws + "<TranslationRate>" + str(self.TranslationRate) + "</TranslationRate>\n"
        buf += ws + "<TranslationDirection>" + str(self.TranslationDirection) + "</TranslationDirection>\n"

        return buf
        

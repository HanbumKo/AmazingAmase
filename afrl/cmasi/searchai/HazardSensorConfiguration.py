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

from afrl.cmasi import PayloadConfiguration
from afrl.cmasi.searchai import HazardType


class HazardSensorConfiguration(PayloadConfiguration.PayloadConfiguration):

    def __init__(self):
        PayloadConfiguration.PayloadConfiguration.__init__(self)
        self.LMCP_TYPE = 6
        self.SERIES_NAME = "SEARCHAI"
        self.FULL_LMCP_TYPE_NAME = "afrl.cmasi.searchai.HazardSensorConfiguration"
        #Series Name turned into a long for quick comparisons.
        self.SERIES_NAME_ID = 6000273900112986441
        self.SERIES_VERSION = 5

        #Define message fields
        self.MaxRange = 0   #real32
        self.HorizontalFOV = 0   #real32
        self.VerticalFOV = 0   #real32
        self.DetectableHazardTypes = []   #HazardType


    def pack(self):
        """
        Packs the object data and returns a string that contains all of the serialized
        members.
        """
        buffer = bytearray()
        buffer.extend(PayloadConfiguration.PayloadConfiguration.pack(self))
        buffer.extend(struct.pack(">f", self.MaxRange))
        buffer.extend(struct.pack(">f", self.HorizontalFOV))
        buffer.extend(struct.pack(">f", self.VerticalFOV))
        buffer.extend(struct.pack(">H", len(self.DetectableHazardTypes) ))
        for x in self.DetectableHazardTypes:
            buffer.extend(struct.pack(">i", x ))

        return buffer

    def unpack(self, buffer, _pos):
        """
        Unpacks data from a bytearray and sets class members
        """
        _pos = PayloadConfiguration.PayloadConfiguration.unpack(self, buffer, _pos)
        self.MaxRange = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.HorizontalFOV = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        self.VerticalFOV = struct.unpack_from(">f", buffer, _pos)[0]
        _pos += 4
        _arraylen = struct.unpack_from(">H", buffer, _pos )[0]
        _pos += 2
        self.DetectableHazardTypes = [None] * _arraylen
        if _arraylen > 0:
            self.DetectableHazardTypes = struct.unpack_from(">" + repr(_arraylen) + "i", buffer, _pos )
            _pos += 4 * _arraylen
        return _pos


    def unpackFromXMLNode(self, el, seriesFactory):
        PayloadConfiguration.PayloadConfiguration.unpackFromXMLNode(self, el, seriesFactory)
        for e in el.childNodes:
            if e.nodeType == xml.dom.Node.ELEMENT_NODE:
                if e.localName == "MaxRange" and len(e.childNodes) > 0 :
                    self.MaxRange = float(e.childNodes[0].nodeValue)
                elif e.localName == "HorizontalFOV" and len(e.childNodes) > 0 :
                    self.HorizontalFOV = float(e.childNodes[0].nodeValue)
                elif e.localName == "VerticalFOV" and len(e.childNodes) > 0 :
                    self.VerticalFOV = float(e.childNodes[0].nodeValue)
                elif e.localName == "DetectableHazardTypes" and len(e.childNodes) > 0 :
                    self.DetectableHazardTypes = []
                    for c in e.childNodes:
                        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                            self.DetectableHazardTypes.append( HazardType.get_HazardType_str(c.childNodes[0].nodeValue) )

        return

    def unpackFromDict(self, d, seriesFactory):
        PayloadConfiguration.PayloadConfiguration.unpackFromDict(self, d, seriesFactory)
        for key in d:
            if key == "MaxRange":
                self.MaxRange = d[key]
            elif key == "HorizontalFOV":
                self.HorizontalFOV = d[key]
            elif key == "VerticalFOV":
                self.VerticalFOV = d[key]
            elif key == "DetectableHazardTypes":
                self.DetectableHazardTypes = []
                for c in d[key]:
                    self.DetectableHazardTypes.append( c )

        return

    def get_MaxRange(self):
        return self.MaxRange

    def set_MaxRange(self, value):
        self.MaxRange = float( value )

    def get_HorizontalFOV(self):
        return self.HorizontalFOV

    def set_HorizontalFOV(self, value):
        self.HorizontalFOV = float( value )

    def get_VerticalFOV(self):
        return self.VerticalFOV

    def set_VerticalFOV(self, value):
        self.VerticalFOV = float( value )

    def get_DetectableHazardTypes(self):
        return self.DetectableHazardTypes



    def toString(self):
        """
        Returns a string representation of all variables
        """
        buf = PayloadConfiguration.PayloadConfiguration.toString(self)
        buf += "From HazardSensorConfiguration:\n"
        buf +=    "MaxRange = " + str( self.MaxRange ) + "\n" 
        buf +=    "HorizontalFOV = " + str( self.HorizontalFOV ) + "\n" 
        buf +=    "VerticalFOV = " + str( self.VerticalFOV ) + "\n" 
        buf +=    "DetectableHazardTypes = " + str( self.DetectableHazardTypes ) + "\n" 

        return buf;

    def toDict(self):
        m = {}
        self.toDictMembers(m)
        d = {}
        if ("SEARCHAI" is None) or ("SEARCHAI" is ""): # this should never happen
            # need to fill this with error message
            d["datatype"] = str("DEBUG_PROBLEM_HERE" + "/HazardSensorConfiguration")
            d["datastring"] = str(m)
        else:
            d['datatype'] = str("SEARCHAI" + "/HazardSensorConfiguration")
            d['datastring'] = str(m)
        return d

    def toDictMembers(self, d):
        PayloadConfiguration.PayloadConfiguration.toDictMembers(self, d)
        d['MaxRange'] = self.MaxRange
        d['HorizontalFOV'] = self.HorizontalFOV
        d['VerticalFOV'] = self.VerticalFOV
        d['DetectableHazardTypes'] = []
        for x in self.DetectableHazardTypes:
            d['DetectableHazardTypes'].append(x)

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
        str = ws + '<HazardSensorConfiguration Series="SEARCHAI" >\n';
        #str +=PayloadConfiguration.PayloadConfiguration.toXMLMembersStr(self, ws + "  ")
        str += self.toXMLMembersStr(ws + "  ")
        str += ws + "</HazardSensorConfiguration>\n";
        return str

    def toXMLMembersStr(self, ws):
        buf = ""
        buf += PayloadConfiguration.PayloadConfiguration.toXMLMembersStr(self, ws)
        buf += ws + "<MaxRange>" + str(self.MaxRange) + "</MaxRange>\n"
        buf += ws + "<HorizontalFOV>" + str(self.HorizontalFOV) + "</HorizontalFOV>\n"
        buf += ws + "<VerticalFOV>" + str(self.VerticalFOV) + "</VerticalFOV>\n"
        buf += ws + "<DetectableHazardTypes>\n"
        for x in self.DetectableHazardTypes:
            buf += ws + "<HazardType>" + HazardType.get_HazardType_int(x) + "</HazardType>\n"
        buf += ws + "</DetectableHazardTypes>\n"

        return buf
        

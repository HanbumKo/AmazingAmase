## ===============================================================================
## Authors: AFRL/RQQA
## Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
## 
## Copyright (c) 2017 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

## This file was auto-created by LmcpGen. Modifications will be overwritten.

class HazardType:

    Undefined = 0
    Fire = 1
    Smoke = 2



def get_HazardType_str(str):
    """
    Returns a numerical value from a string
    """
    if str == "Undefined": return HazardType.Undefined
    if str == "Fire": return HazardType.Fire
    if str == "Smoke": return HazardType.Smoke


def get_HazardType_int(val):
    """
    Returns a string representation from an int
    """
    if val == HazardType.Undefined: return "Undefined"
    if val == HazardType.Fire: return "Fire"
    if val == HazardType.Smoke: return "Smoke"
    return HazardType.Undefined



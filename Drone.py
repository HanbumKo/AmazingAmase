

class Drone():
    def __init__(self):
        self.ID = -1
        self.u = -1
        self.v = -1
        self.w = -1
        self.udot = -1
        self.vdot = -1
        self.wdot = -1
        self.Heading = -1
        self.Pitch = -1
        self.Roll = -1
        self.p = -1
        self.q = -1
        self.r = -1
        self.Course = -1
        self.Groundspeed = -1
        self.Latitude = -1
        self.Longitude = -1
        self.Altitude = -1
        self.AltitudeType = None
        self.EnergyAvailable = -1
        self.ActualEnergyRate = -1
        self.CurrentWaypoint = -1
        self.CurrentCommand = -1
        self.Mode = None
        self.Time = -1
        self.Airspeed = -1
        self.VerticalSpeed = -1
        self.WindSpeed = -1
        self.WindDirection = -1
        self.Label = None
        self.NominalSpeed = -1
        self.NominalAltitude = -1
        self.NominalAltitudeType = None
        self.MinimumSpeed = -1
        self.MaximumSpeed = -1
        self.MinimumAltitude = -1
        self.MinAltitudeType = None
        self.MaximumAltitude = -1
        self.MaxAltitudeType = None

        # Gimbal
        self.GimbalPayloadID = -1
        self.GimbalPointingMode = None
        self.GimbalAzimuth = -1
        self.GimbalElevation = -1
        self.GimbalRotation = -1
        self.GimbalMinAzimuth = -1
        self.GimbalMaxAzimuth = -1
        self.GimbalIsAzimuthClamped = None
        self.GimbalMinElevation = -1
        self.GimbalMaxElevation = -1
        self.GimbalIsElevationClamped = None
        self.GimbalMinRotation = -1
        self.GimbalMaxRotation = -1
        self.GimbalIsRotationClamped = None
        self.GimbalMaxAzimuthSlewRate = -1
        self.GimbalMaxElevationSlewRate = -1
        self.GimbalMaxRotationRate = -1

        # Camera
        self.CameraPayloadID = -1
        self.CameraPointingMode = None
        self.CameraAzimuth = -1
        self.CameraElevation = -1
        self.CameraRotation = -1
        self.CameraHorizontalFieldOfView  = -1
        self.CameraVerticalFieldOfView = -1
        self.CameraCenterpoint = None
        self.CameraSupportedWavelengthBand = None
        self.CameraFieldOfViewMode = None
        self.CameraMinHorizontalFieldOfView = -1
        self.CameraMaxHorizontalFieldOfView = -1
        self.CameraVideoStreamHorizontalResolution = -1
        self.CameraVideoStreamVerticalResolution = -1

        # Hazard sensor
        self.HazardPayloadID = -1
        self.HazardAzimuth = -1
        self.HazardElevation = -1
        self.HazardRotation = -1
        self.HazardHorizontalFieldOfView = -1
        self.HazardVerticalFieldOfView = -1
        self.HazardCenterpoint = None
        self.HazardMaxRange = -1
        self.HazardHorizontalFOV = -1
        self.HazardVerticalFOV = -1


    # Getter / Setter
    def getID(self):
        return self.ID

    def getU(self):
        return self.u

    def getV(self):
        return self.v

    def getUdot(self):
        return self.Udot

    def getVdot(self):
        return self.Vdot

    def getWdot(self):
        return self.Wdot

    def getHeading(self):
        return self.Heading

    def getPitch(self):
        return self.Pitch

    def getRoll(self):
        return self.Roll

    def getP(self):
        return self.p

    def getQ(self):
        return self.q

    def getR(self):
        return self.r

    def getCourse(self):
        return self.Course

    def getGroundspeed(self):
        return self.Groundspeed

    def getLatitude(self):
        return self.Latitude

    def getLongitude(self):
        return self.Longitude

    def getAltitude(self):
        return self.Altitude

    def getAltitudeType(self):
        return self.AltitudeType

    def getEnergyAvailable(self):
        return self.EnergyAvailable

    def getActualEnergyRate(self):
        return self.ActualEnergyRate

    def getCurrentWaypoint(self):
        return self.CurrentWaypoint

    def getCurrentCommand(self):
        return self.CurrentCommand

    def getMode(self):
        return self.Mode

    def getTime(self):
        return self.Time

    def getAirspeed(self):
        return self.Airspeed

    def getVerticalSpeed(self):
        return self.VerticalSpeed

    def getWindSpeed(self):
        return self.WindSpeed

    def getWindDirection(self):
        return self.WindDirection

    def getLabel(self):
        return self.Label

    def getNominalSpeed(self):
        return self.NominalSpeed

    def getNominalAltitude(self):
        return self.NominalAltitude

    def getNominalAltitudeType(self):
        return self.NominalAltitudeType

    def getMinimumSpeed(self):
        return self.MinimumSpeed

    def getMaximumSpeed(self):
        return self.MaximumSpeed

    def getMinimumAltitude(self):
        return self.MinimumAltitude

    def getMinAltitudeType(self):
        return self.MinAltitudeType

    def getMaximumAltitude(self):
        return self.MaximumAltitude

    def getMaxAltitudeType(self):
        return self.MaxAltitudeType

    def getGimbalPayloadID(self):
        return self.GimbalPayloadID

    def getGimbalPointingMode(self):
        return self.GimbalPointingMode

    def getGimbalAzimuth(self):
        return self.GimbalAzimuth

    def getGimbalElevation(self):
        return self.GimbalElevation

    def getGimbalRotation(self):
        return self.GimbalRotation

    def getGimbalMinAzimuth(self):
        return self.GimbalMinAzimuth

    def getGimbalMaxAzimuth(self):
        return self.GimbalMaxAzimuth

    def getGimbalIsAzimuthClamped(self):
        return self.GimbalIsAzimuthClamped

    def getGimbalMinElevation(self):
        return self.GimbalMinElevation

    def getGimbalMaxElevation(self):
        return self.GimbalMaxElevation

    def getGimbalIsElevationClamped(self):
        return self.GimbalIsElevationClamped

    def getGimbalMinRotation(self):
        return self.GimbalMinRotation

    def getGimbalMaxRotation(self):
        return self.GimbalMaxRotation

    def getGimbalIsRotationClamped(self):
        return self.GimbalIsRotationClamped

    def getGimbalMaxAzimuthSlewRate(self):
        return self.GimbalMaxAzimuthSlewRate

    def getGimbalMaxElevationSlewRate(self):
        return self.GimbalMaxElevationSlewRate

    def getGimbalMaxRotationRate(self):
        return self.GimbalMaxRotationRate

    def getCameraPayloadID(self):
        return self.CameraPayloadID

    def getCameraPointingMode(self):
        return self.CameraPointingMode

    def getCameraAzimuth(self):
        return self.CameraAzimuth

    def getCameraElevation(self):
        return self.CameraElevation

    def getCameraRotation(self):
        return self.CameraRotation

    def getCameraHorizontalFieldOfView(self):
        return self.CameraHorizontalFieldOfView

    def getCameraVerticalFieldOfView(self):
        return self.CameraVerticalFieldOfView

    def getCameraCenterpoint(self):
        return self.CameraCenterpoint

    def getCameraSupportedWavelengthBand(self):
        return self.CameraSupportedWavelengthBand

    def getCameraFieldOfViewMode(self):
        return self.CameraFieldOfViewMode

    def getCameraMinHorizontalFieldOfView(self):
        return self.CameraMinHorizontalFieldOfView

    def getCameraMaxHorizontalFieldOfView(self):
        return self.CameraMaxHorizontalFieldOfView

    def getCameraVideoStreamHorizontalResolution(self):
        return self.CameraVideoStreamHorizontalResolution

    def getCameraVideoStreamVerticalResolution(self):
        return self.CameraVideoStreamVerticalResolution

    def getHazardPayloadID(self):
        return self.HazardPayloadID

    def getHazardAzimuth(self):
        return self.HazardAzimuth

    def getHazardElevation(self):
        return self.HazardElevation

    def getHazardRotation(self):
        return self.HazardRotation

    def getHazardHorizontalFieldOfView(self):
        return self.HazardHorizontalFieldOfView

    def getHazardVerticalFieldOfView(self):
        return self.HazardVerticalFieldOfView

    def getHazardCenterpoint(self):
        return self.HazardCenterpoint

    def getHazardMaxRange(self):
        return self.HazardMaxRange

    def getHazardHorizontalFOV(self):
        return self.HazardHorizontalFOV

    def getHazardVerticalFOV(self):
        return self.HazardVerticalFOV

    # ===============================

    def setID(self, newID):
        self.ID = newID

    def setU(self, newU):
        self.u = newU

    def setV(self, newV):
        self.v = newV

    def setW(self, newW):
        self.v = newW

    def setUdot(self, newUdot):
        self.Udot = newUdot

    def setVdot(self, newVdot):
        self.Vdot = newVdot

    def setWdot(self, newWdot):
        self.Wdot = newWdot

    def setHeading(self, newHeading):
        self.Heading = newHeading

    def setPitch(self, newPitch):
        self.Pitch = newPitch

    def setRoll(self, newRoll):
        self.Roll = newRoll

    def setP(self, newP):
        self.p = newP

    def setQ(self, newQ):
        self.q = newQ

    def setR(self, newR):
        self.r = newR

    def setCourse(self, newCourse):
        self.Course = newCourse

    def setGroundspeed(self, newGroundspeed):
        self.Groundspeed = newGroundspeed

    def setLatitude(self, newLatitude):
        self.Latitude = newLatitude

    def setLongitude(self, newLongitude):
        self.Longitude = newLongitude

    def setAltitude(self, newAltitude):
        self.Altitude = newAltitude

    def setAltitudeType(self, newAltitudeType):
        self.AltitudeType = newAltitudeType

    def setEnergyAvailable(self, newEnergyAvailable):
        self.EnergyAvailable = newEnergyAvailable

    def setActualEnergyRate(self, newActualEnergyRate):
        self.ActualEnergyRate = newActualEnergyRate

    def setCurrentWaypoint(self, newCurrentWaypoint):
        self.CurrentWaypoint = newCurrentWaypoint

    def setCurrentCommand(self, newCurrentCommand):
        self.CurrentCommand = newCurrentCommand

    def setMode(self, newMod):
        self.Mode = newMod

    def setTime(self, newTime):
        self.Time = newTime

    def setAirspeed(self, newAirspeed):
        self.Airspeed = newAirspeed

    def setVerticalSpeed(self, newVerticalSpeed):
        self.VerticalSpeed = newVerticalSpeed

    def setWindSpeed(self, newWindSpeed):
        self.WindSpeed = newWindSpeed

    def setWindDirection(self, newWindDirection):
        self.WindDirection = newWindDirection

    def setLabel(self, newLabel):
        self.Label = newLabel

    def setNominalSpeed(self, newNominalSpeed):
        self.NominalSpeed = newNominalSpeed

    def setNominalAltitude(self, newNominalAltitude):
        self.NominalAltitude = newNominalAltitude

    def setNominalAltitudeType(self, newNominalAltitudeType):
        self.NominalAltitudeType = newNominalAltitudeType

    def setMinimumSpeed(self, newMinimumSpeed):
        self.MinimumSpeed = newMinimumSpeed

    def setMaximumSpeed(self, newMaximumSpeed):
        self.MaximumSpeed = newMaximumSpeed

    def setMinimumAltitude(self, newMinimumAltitude):
        self.MinimumAltitude = newMinimumAltitude

    def setMinAltitudeType(self, newMinAltitudeType):
        self.MinAltitudeType = newMinAltitudeType

    def setMaximumAltitude(self, newMaximumAltitude):
        self.MaximumAltitude = newMaximumAltitude

    def setMaxAltitudeType(self, newMaxAltitudeType):
        self.MaxAltitudeType = newMaxAltitudeType

    def setGimbalPayloadID(self, newGimbalPayloadID):
        self.GimbalPayloadID = newGimbalPayloadID

    def setGimbalPointingMode(self, newGimbalPointingMode):
        self.GimbalPointingMode = newGimbalPointingMode

    def setGimbalAzimuth(self, newGimbalAzimuth):
        self.GimbalAzimuth = newGimbalAzimuth

    def setGimbalElevation(self, newGimbalElevation):
        self.GimbalElevation = newGimbalElevation

    def setGimbalRotation(self, newGimbalRotation):
        self.GimbalRotation = newGimbalRotation

    def setGimbalMinAzimuth(self, newGImbalMinAzimuth):
        self.GimbalMinAzimuth = newGImbalMinAzimuth

    def setGimbalMaxAzimuth(self, newGimbalMaxAzimuth):
        self.GimbalMaxAzimuth = newGimbalMaxAzimuth

    def setGimbalIsAzimuthClamped(self, newGimbalIsAzimuthClamped):
        self.GimbalIsAzimuthClamped = newGimbalIsAzimuthClamped

    def setGimbalMinElevation(self, newGimbalMinElevation):
        self.GimbalMinElevation = newGimbalMinElevation

    def setGimbalMaxElevation(self, newGimbalMaxElevation):
        self.GimbalMaxElevation = newGimbalMaxElevation

    def setGimbalIsElevationClamped(self, newGimbalIsElevationClamped):
        self.GimbalIsElevationClamped = newGimbalIsElevationClamped

    def setGimbalMinRotation(self, newGimbalMinRotation):
        self.GimbalMinRotation = newGimbalMinRotation

    def setGimbalMaxRotation(self, newGimbalMaxRotation):
        self.GimbalMaxRotation = newGimbalMaxRotation

    def setGimbalIsRotationClamped(self, newGimbalIsRotationClamped):
        self.GimbalIsRotationClamped = newGimbalIsRotationClamped

    def setGimbalMaxAzimuthSlewRate(self, newGimbalMaxAzimuthSlewRate):
        self.GimbalMaxAzimuthSlewRate = newGimbalMaxAzimuthSlewRate

    def setGimbalMaxElevationSlewRate(self, newGimbalMaxElevationSlewRate):
        self.GimbalMaxElevationSlewRate = newGimbalMaxElevationSlewRate

    def setGimbalMaxRotationRate(self, newGimbalMaxRotationRate):
        self.GimbalMaxRotationRate = newGimbalMaxRotationRate

    def setCameraPayloadID(self, newCameraPayloadID):
        self.CameraPayloadID = newCameraPayloadID

    def setCameraPointingMode(self, newCameraPointingMode):
        self.CameraPointingMode = newCameraPointingMode

    def setCameraAzimuth(self, newCameraAzimuth):
        self.CameraAzimuth = newCameraAzimuth

    def setCameraElevation(self, newCameraElevation):
        self.CameraElevation = newCameraElevation

    def setCameraRotation(self, newCameraRotation):
        self.CameraRotation = newCameraRotation

    def setCameraHorizontalFieldOfView(self, newCameraHorizontalFieldOfView):
        self.CameraHorizontalFieldOfView = newCameraHorizontalFieldOfView

    def setCameraVerticalFieldOfView(self, newCameraVerticalFieldOfView):
        self.CameraVerticalFieldOfView = newCameraVerticalFieldOfView

    def setCameraCenterpoint(self, newCameraCenterpoint):
        self.CameraCenterpoint = newCameraCenterpoint

    def setCameraSupportedWavelengthBand(self, newCameraSupportedWavelengthBand):
        self.CameraSupportedWavelengthBand = newCameraSupportedWavelengthBand

    def setCameraFieldOfViewMode(self, newCameraFieldOfViewMode):
        self.CameraFieldOfViewMode = newCameraFieldOfViewMode

    def setCameraMinHorizontalFieldOfView(self, newCameraMinHorizontalFieldOfView):
        self.CameraMinHorizontalFieldOfView = newCameraMinHorizontalFieldOfView

    def setCameraMaxHorizontalFieldOfView(self, newCameraMaxHorizontalFieldOfView):
        self.CameraMaxHorizontalFieldOfView = newCameraMaxHorizontalFieldOfView

    def setCameraVideoStreamHorizontalResolution(self, newCameraVideoStreamHorizontalResolution):
        self.CameraVideoStreamHorizontalResolution = newCameraVideoStreamHorizontalResolution

    def setCameraVideoStreamVerticalResolution(self, newCameraVideoStreamVerticalResolution):
        self.CameraVideoStreamVerticalResolution = newCameraVideoStreamVerticalResolution

    def setHazardPayloadID(self, newHazardPayloadID):
        self.HazardPayloadID = newHazardPayloadID

    def setHazardAzimuth(self, newHazardAzimuth):
        self.HazardAzimuth = newHazardAzimuth

    def setHazardElevation(self, newHazardElevation):
        self.HazardElevation = newHazardElevation

    def setHazardRotation(self, newHazardRotation):
        self.HazardRotation = newHazardRotation

    def setHazardHorizontalFieldOfView(self, newHazardHorizontalFieldOfView):
        self.HazardHorizontalFieldOfView = newHazardHorizontalFieldOfView

    def setHazardVerticalFieldOfView(self, newHazardVerticalFieldOfView):
        self.HazardVerticalFieldOfView = newHazardVerticalFieldOfView

    def setHazardCenterpoint(self, newHazardCenterpoint):
        self.HazardCenterpoint = newHazardCenterpoint

    def setHazardMaxRange(self, newHazardMaxRange):
        self.HazardMaxRange = newHazardMaxRange

    def setHazardHorizontalFOV(self, newHazardHorizontalFOV):
        self.HazardHorizontalFOV = newHazardHorizontalFOV

    def setHazardVerticalFOV(self, newHazardVerticalFOV):
        self.HazardVerticalFOV = newHazardVerticalFOV





class drone():
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

    def setID(self):
        return self.ID

    def setU(self):
        return self.u

    def setV(self):
        return self.v

    def setUdot(self):
        return self.Udot

    def setVdot(self):
        return self.Vdot

    def setWdot(self):
        return self.Wdot

    def setHeading(self):
        return self.Heading

    def setPitch(self):
        return self.Pitch

    def setRoll(self):
        return self.Roll

    def setP(self):
        return self.p

    def setQ(self):
        return self.q

    def setR(self):
        return self.r

    def setCourse(self):
        return self.Course

    def setGroundspeed(self):
        return self.Groundspeed

    def setLatitude(self):
        return self.Latitude

    def setLongitude(self):
        return self.Longitude

    def setAltitude(self):
        return self.Altitude

    def setAltitudeType(self):
        return self.AltitudeType

    def setEnergyAvailable(self):
        return self.EnergyAvailable

    def setActualEnergyRate(self):
        return self.ActualEnergyRate

    def setCurrentWaypoint(self):
        return self.CurrentWaypoint

    def setCurrentCommand(self):
        return self.CurrentCommand

    def setMode(self):
        return self.Mode

    def setTime(self):
        return self.Time

    def setAirspeed(self):
        return self.Airspeed

    def setVerticalSpeed(self):
        return self.VerticalSpeed

    def setWindSpeed(self):
        return self.WindSpeed

    def setWindDirection(self):
        return self.WindDirection

    def setLabel(self):
        return self.Label

    def setNominalSpeed(self):
        return self.NominalSpeed

    def setNominalAltitude(self):
        return self.NominalAltitude

    def setNominalAltitudeType(self):
        return self.NominalAltitudeType

    def setMinimumSpeed(self):
        return self.MinimumSpeed

    def setMaximumSpeed(self):
        return self.MaximumSpeed

    def setMinimumAltitude(self):
        return self.MinimumAltitude

    def setMinAltitudeType(self):
        return self.MinAltitudeType

    def setMaximumAltitude(self):
        return self.MaximumAltitude

    def setMaxAltitudeType(self):
        return self.MaxAltitudeType

    def setGimbalPointingMode(self):
        return self.GimbalPointingMode

    def setGimbalAzimuth(self):
        return self.GimbalAzimuth

    def setGimbalElevation(self):
        return self.GimbalElevation

    def setGimbalRotation(self):
        return self.GimbalRotation

    def setGimbalMinAzimuth(self):
        return self.GimbalMinAzimuth

    def setGimbalMaxAzimuth(self):
        return self.GimbalMaxAzimuth

    def setGimbalIsAzimuthClamped(self):
        return self.GimbalIsAzimuthClamped

    def setGimbalMinElevation(self):
        return self.GimbalMinElevation

    def setGimbalMaxElevation(self):
        return self.GimbalMaxElevation

    def setGimbalIsElevationClamped(self):
        return self.GimbalIsElevationClamped

    def setGimbalMinRotation(self):
        return self.GimbalMinRotation

    def setGimbalMaxRotation(self):
        return self.GimbalMaxRotation

    def setGimbalIsRotationClamped(self):
        return self.GimbalIsRotationClamped

    def setGimbalMaxAzimuthSlewRate(self):
        return self.GimbalMaxAzimuthSlewRate

    def setGimbalMaxElevationSlewRate(self):
        return self.GimbalMaxElevationSlewRate

    def setGimbalMaxRotationRate(self):
        return self.GimbalMaxRotationRate

    def setCameraPointingMode(self):
        return self.CameraPointingMode

    def setCameraAzimuth(self):
        return self.CameraAzimuth

    def setCameraElevation(self):
        return self.CameraElevation

    def setCameraRotation(self):
        return self.CameraRotation

    def setCameraHorizontalFieldOfView(self):
        return self.CameraHorizontalFieldOfView

    def setCameraVerticalFieldOfView(self):
        return self.CameraVerticalFieldOfView

    def setCameraCenterpoint(self):
        return self.CameraCenterpoint

    def setCameraSupportedWavelengthBand(self):
        return self.CameraSupportedWavelengthBand

    def setCameraFieldOfViewMode(self):
        return self.CameraFieldOfViewMode

    def setCameraMinHorizontalFieldOfView(self):
        return self.CameraMinHorizontalFieldOfView

    def setCameraMaxHorizontalFieldOfView(self):
        return self.CameraMaxHorizontalFieldOfView

    def setCameraVideoStreamHorizontalResolution(self):
        return self.CameraVideoStreamHorizontalResolution

    def setCameraVideoStreamVerticalResolution(self):
        return self.CameraVideoStreamVerticalResolution

    def setHazardAzimuth(self):
        return self.HazardAzimuth

    def setHazardElevation(self):
        return self.HazardElevation

    def setHazardRotation(self):
        return self.HazardRotation

    def setHazardHorizontalFieldOfView(self):
        return self.HazardHorizontalFieldOfView

    def setHazardVerticalFieldOfView(self):
        return self.HazardVerticalFieldOfView

    def setHazardCenterpoint(self):
        return self.HazardCenterpoint

    def setHazardMaxRange(self):
        return self.HazardMaxRange

    def setHazardHorizontalFOV(self):
        return self.HazardHorizontalFOV

    def setHazardVerticalFOV(self):
        return self.HazardVerticalFOV



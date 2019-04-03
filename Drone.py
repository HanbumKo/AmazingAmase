

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
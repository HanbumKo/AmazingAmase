from os import listdir
from os.path import isdir, isfile, join
import re
from .DTEDTile import DTEDTile
import math
import numpy as np

class TerrianService(object):
    __LON_PATTERN = "[eEwW]\\d+"
    __LAT_PATTERN = "[nNsS]\\d+[.][dD][tT]\\d"
    
    __currentTile = None
    __tileMap = {}

    __cachedTiles = []
    __MAX_CACHE_SIZE = 8   
    
    def distanceToHorizon(self, meterAlt):
        return math.sqrt(1.2756274E7 * meterAlt + meterAlt * meterAlt)

    def getLatLon(self, lat, lon, mDist, azimuth):
        radLat = math.radians(lat)
        radLon = math.radians(lon)
        radAzimuth = math.radians(azimuth)

        radDist = mDist / 6378137.0
        coslat1 = math.cos(radLat)
        sinlat1 = math.sin(radLat)
        cosAz = math.cos(radAzimuth)
        sinAz = math.sin(radAzimuth)
        sinc = math.sin(radDist)
        cosc = math.cos(radDist)
        radLat2 = math.asin(sinlat1 * cosc + coslat1 * sinc * cosAz)
        sinlat2 = math.sin(radLat2)
        radLon2 = radLon + math.atan2(sinAz * sinc * coslat1, cosc - sinlat1 * sinlat2 )

        return [math.degrees(radLat2), math.degrees(radLon2)]

    def addDirectory(self, dir):
        # 경로 파일 읽고, 널이 아니면
        # input dir = dted2가 있는 곳, ex =data/dted/
        # dted2 > lonDir(wWeE\\d) > latFile(sSnN\\d.dt(level))
        if isdir(dir):
            checkLonDir = re.compile(self.__LON_PATTERN)
            checkLatFile = re.compile(self.__LAT_PATTERN)

            # lonDirs = file name list of lon
            lonDirs = [join(dir, f) for f in listdir(dir) if isdir(join(dir, f)) and checkLonDir.match(f)]
            for lonDir in lonDirs:
                latFiles = [join(lonDir, f) for f in listdir(lonDir) if isfile(join(lonDir, f)) and checkLatFile.match(f)]
                for latFile in latFiles:
                    tile = DTEDTile(latFile)
                    
                    levelMap = {}
                    if tile.getLevel() not in self.__tileMap:
                        self.__tileMap[tile.getLevel()] = levelMap
                    else :
                        levelMap = self.__tileMap[tile.getLevel()]
                    
                    lonMap = {}

                    if tile.getLon() not in levelMap:
                        levelMap[tile.getLon()] = lonMap
                    else :
                        lonMap = levelMap[tile.getLon()]

                    lonMap[tile.getLat()] = tile

            self.__currentTile = None

    def getTile(self, lat, lon):
        intLat = int(math.floor(lat))
        intLon = int(math.floor(lon))

        if self.__currentTile != None and self.__currentTile.getLat() == intLat and self.__currentTile.getLon() == intLon :
            return self.__currentTile
        else :
            levelMap = None
            lonMap = None

            for i in range(2,-1, -1):
                if i in self.__tileMap:
                    levelMap = self.__tileMap[i]
                    if intLon in levelMap:
                        lonMap = levelMap[intLon]
                        if intLat in lonMap:
                            t = lonMap[intLat]
                            self.addToCache(t)
                            self.__currentTile = t
                            return t
            return None

    def getElevation(self, lat, lon):
        tile = self.getTile(lat, lon)
        return tile.getElevation(lat, lon) if tile is not None else 0

    def getElevations(self, ullat, ullon, lrlat, lrlon, arcStep):
        tmp = 0
        
        if ullat < lrlat:
            ullat, lrlat = lrlat, ullat
        
        if lrlon < ullon:
            lrlon, ullon = ullon, lrlon
        
        numlats = int((ullat - lrlat) / arcStep)
        numlats = 1 if numlats == 0 else numlats
        numlons = int((lrlon - ullon) / arcStep)
        numlons = 1 if numlons == 0 else numlons
        ptsPerDegree = int(1.0/arcStep)
        posts = [ [ 0 for i in range(numlats)] for j in range(numlons)]
        xs = 0
        xe = 0
        ys = 0
        while ( xs < numlons) :
            lon = ullon + xs*arcStep
            xe = xs + int((math.ceil(lon) - lon) * ptsPerDegree)

            ye = 0

            xe = numlons if xe > numlons else xe
            while( ys < numlats ):
                lat = lrlat + ys*arcStep
                ye = ys + int((math.ceil(lat) - lat) * ptsPerDegree )
                ye = numlats if ye > numlats else ye
                
                tile = self.getTile(lat, lon)

                for i in range(xs,xe) :
                    if not tile:
                        posts[i][ys:ye] = [ 0 for i in range(ys,ye)]
                    else:
                        lon = ullon + i*arcStep

                        for j in range(ys, ye) :
                            lat = lrlat + j*arcStep
                            posts[i][j] = tile.getElevation(lat, lon)

                ys = ye + 1

            ys = 0

            xs = xe + 1
        # 90도 반시계 회전필요
        posts = np.rot90(posts)
        return posts
    
    def getElevationInterp(self, lat, lon):
        tile = self.getTile(lat, lon)
        return tile.getElevationInterp(lat, lon) if tile is not None else 0.0
    
    def getLineElevs(self, lat1, lon1, lat2, lon2, level):
        arcStep = DTEDTile.getPostSpacing(level)
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        totalDist = math.hypot(dlon, dlat)
        numSteps = int(totalDist / arcStep)
        numSteps = 1 if numSteps == 0 else numSteps
        latStep = dlat / numSteps
        lonStep = dlon / numSteps
        elevs = [ 0 for i in range(numSteps+1)]
        coords = [ 0 for i in range(numSteps+1)]
        lat = 0.0
        lon = 0.0

        for i in range(numSteps+1) :
            lat = lat1 + i * latStep
            lon = lon1 + i * lonStep
            coords[i] = (lat,lon)
            elevs[i] = self.getElevation(lat, lon)

        return elevs, coords


    def getInterceptPoint(self, startLat, startLon, startAlt, heading, slopeRad, maxDist, level):
        tanSlope = math.tan(slopeRad)
        if (slopeRad >= 0.0 or maxDist <= 0.0) :
            maxDist = self.distanceToHorizon(startAlt)

        maxDist = math.min(maxDist, math.abs(startAlt / tanSlope))
        ll = self.getLatLon(startLat, startLon, maxDist, heading);
        lat2 = math.degrees(ll[0])
        lon2 = math.degrees(ll[1])
        numsteps = int(math.hypot(lat2 - startLat, lon2 - startLon) / DTEDTile.getPostSpacing(level));
        if (numsteps == 0) :
            return [startLat, startLon, self.getElevation(startLat, startLon)]
        else :
            latstep = (lat2 - startLat) / numsteps;
            lonstep = (lon2 - startLon) / numsteps;
            hstep = -startAlt / numsteps;
            h2 = 0.0
            h1 = startAlt
            hterr1 = 0.0
            hterr2 = 0.0
            lat = 0.0
            lon = 0.0

            for i in range(numsteps) :
                lat = startLat + latstep * i
                lon = startLon + lonstep * i
                hterr2 = self.getElevation(lat, lon)
                h2 = startAlt + hstep * i
                if (h2 <= hterr2) :
                    stepRatio = (hterr1 - h1) / (h2 - h1 - hterr2 + hterr1)
                    lat = startLat + latstep * ((i - 1) + stepRatio)
                    lon = startLon + lonstep * ((i - 1) + stepRatio)
                    hterr = hterr1 + (hterr2 - hterr1) * stepRatio
                    return [lat, lon, hterr]

                h1 = h2
                hterr1 = hterr2

            return [lat2, lon2, self.getElevation(lat2, lon2)]
    
    def isLineOfSight(self, lat1, lon1, h1, lat2, lon2, h2, level):
        numsteps = int(math.hypot(lat2 - lat1, lon2 - lon1) / DTEDTile.getPostSpacing(level))
        latstep = (lat2 - lat1) / numsteps
        lonstep = (lon2 - lon1) / numsteps
        hstart = self.getElevation(lat1, lon1) + h1
        hend = self.getElevation(lat2, lon2) + h2
        hstep = (hend - hstart) / numsteps

        for i in range(numsteps):
            lat = lat1 + latstep * i
            lon = lon1 + lonstep * i
            hterr = self.getElevation(lat, lon)
            hray = hstart + hstep * i
            if (hray < hterr) :
                return False

        return True

    def addToCache(self, t):
        if t in self.__cachedTiles:
            self.__cachedTiles.remove(t)
        self.__cachedTiles.insert(0, t)
        if len(self.__cachedTiles) > self.__MAX_CACHE_SIZE and not self.__cachedTiles :
            t = self.__cachedTiles.pop()
            if t:
                t.dumpData()
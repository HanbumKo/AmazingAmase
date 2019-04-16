import math
import copy

class ParseError(Exception): pass

class DTEDTile(object):

    __UHL = 80
    __DSI = 648
    __ACC = 2700
    __lat = 0
    __lon = 0
    __numlats = 0
    __numlons = 0
    __file = None
    __level = 0
    __data = None
    __rf = None

    def __init__(self, filepath):
        self.__file = open(filepath, 'rb')
        self.__lat = self.readLat()
        self.__lon = self.readLon()
        self.__level = self.getLevel(filepath)
        self.__numlats = self.readNumLats()
        self.__numlons = self.readNumLons()
        self.__data = [None for i in range(self.__numlons)]

        print(self.__lat, self.__lon, self.__level, self.__numlats, self.__numlons)
    def getLevel(self, *filepath):
        if not filepath:
            return self.__level
        
        return int(filepath[0][-1])

    def getLon(self):
        return self.__lon

    def getLat(self):
        return self.__lat
    
    @staticmethod
    def getPostSpacing(level):
        if level == 1:
            return 0.008333333333333333
        elif level == 2:
            return 8.333333333333334E-4
        elif level == 3:
            return 2.777777777777778E-4
        else : 
            return 0.008333333333333333

    def getElevation(self, lat, lon):
        
        if (math.floor(lat) == self.__lat and math.floor(lon) == self.__lon):
            rlat = lat - self.__lat
            rlon = lon - self.__lon
            latIndex = int(round(rlat * (self.__numlats - 1)))
            lonIndex = int(round(rlon * (self.__numlons - 1)))
            self.readData(lonIndex)
            return self.__data[lonIndex][latIndex]
        else :
            return 0

    def getElevationInterp(self, lat, lon):
        if (math.floor(lat) == self.__lat and math.floor(lon) == self.__lon) :
            rlat = (lat - self.__lat) * (self.__numlats - 1)
            rlon = (lon - self.__lon) * (self.__numlons - 1)
            floorLatIndex = int(math.floor(rlat))
            floorLonIndex = int(math.floor(rlon))
            ceilLatIndex = int(math.ceil(rlat))
            ceilLonIndex = int(math.ceil(rlon))
            self.readData(floorLonIndex)
            self.readData(ceilLonIndex)
            latDiff = rlat - floorLatIndex
            lonDiff = rlon - floorLonIndex
            h1 = self.__data[floorLonIndex][floorLatIndex]
            h2 = self.__data[floorLonIndex][ceilLatIndex]
            h3 = self.__data[ceilLonIndex][ceilLatIndex]
            h4 = self.__data[ceilLonIndex][floorLatIndex]
            h = (h2 * latDiff + h1 * (1.0 - latDiff)) * (1.0 - lonDiff) + (h3 * latDiff + h4 * (1.0 - latDiff)) * lonDiff
            return h
        else :
            return 0.0
    
    def getAllElevations(self) :
        for i in range(self.__numlons) :
            if self.__data[i] == None:
                self.readData(i)

        return copy.deepcopy(self.__data)

    def dumpData(self):
        self.__data = [None for i in range(self.__numlons)]

    def readData(self, column):
        if (self.__data[column] is None):
            self.__file.seek(self.__UHL + self.__DSI + self.__ACC + column * (12 + 2*self.__numlats) + 8)
            col = []
            self.__data[column] = col
            
            rowdata = self.__file.read(2 * self.__numlats)
            for j in range(self.__numlats):
                h = rowdata[j*2] << 8 | rowdata[j*2+1] & 255
                col.append( 0 if h == -32768 else h)

    def readLat(self):
        lat = self.readInt(12, 3)
        self.__file.seek(19)
        h = self.__file.read(1).decode()
        
        lat = -lat if h != 'N' and h != 'n' else lat
        return lat

    def readLon(self):
        lon = self.readInt(4, 3)
        self.__file.seek(11)
        h = self.__file.read(1).decode()
        lon = -lon if h != 'E' and h != 'e' else lon
        return lon

    def readLonSpacing(self):
        return self.readInt(20, 4) / 3600

    def readLatSpacing(self):
        return self.readInt(24, 4) / 3600

    def readNumLats(self):
        return self.readInt(51, 4)

    def readNumLons(self):
        return self.readInt(47, 4)

    def readInt(self, position, length):
        try:
            self.__file.seek(position)
            return int(self.__file.read(length).decode('utf-8'))
        except:
            print("exception")
            return 0

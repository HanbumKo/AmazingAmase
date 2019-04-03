import socket
from lmcp import LMCPFactory

## ===============================================================================
## Authors: Abe Stoker
## University of Dayton Research Institute Applied Sensing Division
##
## Copyright (c) 2018 Government of the United State of America, as represented by
## the Secretary of the Air Force.  No copyright is claimed in the United States under
## Title 17, U.S. Code.  All Other Rights Reserved.
## ===============================================================================

from lmcp import LMCPFactory
from lmcp import LMCPObject
import abc
import threading

class IDataReceived(abc.ABC):
    @abc.abstractmethod
    def dataReceived(self, lmcpObject):
        pass

class AmaseTCPClient(threading.Thread):
    def __init__(self, host, port):
        super().__init__()
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__stop_reading = False
        self.__factory = LMCPFactory.LMCPFactory()
        self.__recvCallbacks = []
        self.__host = host
        self.__port = port

    def connect(self):
        while True and not self.__stop_reading:
            try:
                self.__socket.settimeout(5)
                self.__socket.connect((self.__host, self.__port))
                return True
            except Exception as ex:
                print("Timed out waiting for server connection, trying again")

    def sendLMCPObject(self, lmcpObj):
        if isinstance(lmcpObj, LMCPObject.LMCPObject):
            buf = bytearray()
            buf.extend(LMCPFactory.packMessage(lmcpObj, True))
            self.__socket.send(buf)
        else:
            raise ValueError("Not an LMCP Object.  Non-LMCP message to AMASE not supported")

    def addReceiveCallback(self, iDataRcv):
        if(isinstance(iDataRcv, IDataReceived)):
            self.__recvCallbacks.append(iDataRcv)
        else:
            raise TypeError("Receive callback is not an instance of IDataReceived")

    def stop(self):
        self.__stop_reading = True

    def run(self):
        if(self.connect()):
            while (not self.__stop_reading):
                try:
                    lmcpObj = self.__readLMCPDataFromSocket()
                    for idataRcv in self.__recvCallbacks:
                        idataRcv.dataReceived(lmcpObj)
                except socket.timeout:
                    continue
                except ValueError as ve:
                    print(ve)
                    continue
                except Exception as ex:
                    print("Unknown Error reading AMASE data")
                    print(ex)
                    return False
        return True

    def __readLMCPDataFromSocket(self):
        data = bytearray(self.__socket.recv(LMCPFactory.HEADER_SIZE))
        if(len(data) >= LMCPFactory.HEADER_SIZE):
            size = LMCPFactory.getSize(data)
            data.extend(bytearray(self.__socket.recv(size+4))) # compensate for checksum
            recv_obj = self.__factory.getObject(data)
            if recv_obj != None:
                return recv_obj
            else:
                raise ValueError("Invalid object received.")
        if(len(data) == 0):
            return
        raise ValueError("Data read not enough for an LMCP header")


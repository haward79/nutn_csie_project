
from time import sleep
import socket as s
from threading import Thread
from typing import final
from textLog import logText


class Communication:

    def __init__(self, globalData):

        self.globalData = globalData

        self.com = s.socket(s.AF_INET, s.SOCK_STREAM)
        self.com.settimeout(0.5)

        isBound = False
        while not isBound:
            try:
                self.com.bind(('127.0.0.1', 8100))
                isBound = True
            except OSError:
                logText('Another program or communicator is running on binding port ...... (waiting)')

            sleep(0.5)

        self.com.listen()

        self.con = None
        self.listen()

        Thread(target=self.checkClose).start()


    def listen(self) -> None:

        con = None
        logText('Wait for communicator ......')

        while not self.globalData.isTerminated and self.con == None:
            try:
                (con, clientAddress) = self.com.accept()
            except s.timeout:
                pass
            
            if con != None:
                self.con = con
                logText('Communicator connected.')


    def sendData(self, msg: str) -> bool:

        if self.con != None:
            self.con.send(str.encode(msg))
            return True
        else:
            return False


    def receiveData(self, msg: str) -> str:

        rtn = ''

        if self.con != None:
            self.con.send(str.encode(msg))
            data = self.con.recv(1024)

            rtn = data.decode('utf-8')

            if rtn.endswith('\n'):
                rtn = rtn[:-1]

        return rtn


    def checkClose(self) -> None:

        while not self.globalData.isTerminated:
            sleep(0.5)

        if self.con != None:
            self.con.close()
            self.con = None


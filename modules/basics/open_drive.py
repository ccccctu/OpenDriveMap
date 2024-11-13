# open_drive.py

class OpenDrive(object):

    def __init__(self):
        self._header = None
        self._roads = []
        self._controllers = []
        self._junctions = []
        self._junctionGroups = []
        self._stations = []
        self._nodes = []

    @property
    def header(self):
        return self._header

    @property
    def roads(self):
        return self._roads

    def getRoad(self, id):
        for road in self._roads:
            if road.id == id:
                return road

        return None
    
    def addNode(self, node):
        self._nodes.append(node)

    def getNode(self, unique_id):
        if 0 <= unique_id < len(self._nodes):
            return self._nodes[unique_id]
        return None 

    @property
    def controllers(self):
        return self._controllers

    @property
    def junctions(self):
        return self._junctions

    @property
    def junctionGroups(self):
        return self._junctionGroups

    @property
    def stations(self):
        return self._stations
    
    @property
    def nodes(self):
        return self._nodes

class Header(object):

    def __init__(self):
        self._revMajor = None
        self._revMinor = None
        self._name = None
        self._version = None
        self._date = None
        self._north = None
        self._south = None
        self._east = None
        self._west = None
        self._vendor = None
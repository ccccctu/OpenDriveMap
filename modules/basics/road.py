# road.py

import numpy as np
from eulerspiral import eulerspiral
from modules.basics.lane import Lanes
from modules.basics.road_plan_view import PlanView
import copy

class Road(object):

    def __init__(self):
        self._id = None
        self._name = None
        self._junction = None
        self._length = None
        self._header = None # TODO
        self._link = Link()
        self._types = []
        self._planView = PlanView()
        self._lanes = Lanes()
        self._ref_points = None
        self._center_line = None

    @property
    def id(self):
        return self._id

    @id.setter
    def id(self, value):
        self._id = int(value)

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self._name = str(value)

    @property
    def junction(self):
        return self._junction

    @junction.setter
    def junction(self, value):
        if not isinstance(value, int) and value is not None:
            raise TypeError("Property must be a int or NoneType")

        if value == -1:
            value = None

        self._junction = value

    @property
    def length(self):
        return self._length
    
    @length.setter
    def length(self, value):
        self._length = float(value)

    @property
    def link(self):
        return self._link

    @property
    def types(self):
        return self._types

    @property
    def planView(self):
        return self._planView

    @property
    def lanes(self):
        return self._lanes
    
    @property
    def ref_points(self):
        return self._ref_points
    
    def setRefPoints(self, ref_points):
        self._ref_points = copy.deepcopy(ref_points)

    @property
    def center_line(self):
        return self._center_line
    
    def setCenterLine(self, center_line):
        self._center_line = copy.deepcopy(center_line)


class Type(object):

    allowedTypes = ["unknown", "rural", "motorway", "town", "lowSpeed", "pedestrian", "bicycle"]

    def __init__(self):
        self._sPos = None
        self._type = None
        self._speed = None

    @property
    def sPos(self):
        return self._sPos

    @sPos.setter
    def sPos(self, value):
        self._sPos = float(value)

    @property
    def type(self):
        return self._type

    @type.setter
    def type(self, value):
        if value not in self.allowedTypes:
            raise AttributeError("Type not allowed.")

        self._type = value

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, value):
        if not isinstance(value, Speed):
            raise TypeError("Value must be instance of Speed.")

        self._speed = value


class Speed(object):

    def __init__(self):
        self._max = None
        self._unit = None

    @property
    def max(self):
        return self._max

    @max.setter
    def max(self, value):
        self._max = str(value)

    @property
    def unit(self):
        return self._unit

    @unit.setter
    def unit(self, value):
        # TODO validate input
        self._unit = str(value)

class Link(object):

    def __init__(self):
        self._id = None
        self._predecessor = None
        self._successor = None
        self._neighbors = []

    def __str__(self):
        return " > link id " + str(self._id) + " | successor: " + str(self._successor)

    @property
    def id(self):
        return self._id

    @id.setter
    def id(self, value):
        self._id = int(value)

    @property
    def predecessor(self):
        return self._predecessor

    @predecessor.setter
    def predecessor(self, value):
        if not isinstance(value, Predecessor):
            raise TypeError("Value must be Predecessor")

        self._predecessor = value

    @property
    def successor(self):
        return self._successor

    @successor.setter
    def successor(self, value):
        if not isinstance(value, Successor):
            raise TypeError("Value must be Successor")

        self._successor = value

    @property
    def neighbors(self):
        return self._neighbors

    @neighbors.setter
    def neighbors(self, value):
        if not isinstance(value, list) or not all(isinstance(x, Neighbor) for x in value):
            raise TypeError("Value must be list of instances of Neighbor.")

        self._neighbors = value

    def addNeighbor(self, value):
        if not isinstance(value, Neighbor):
            raise TypeError("Value must be Neighbor")

        self._neighbors.append(value)


class Predecessor(object):

    def __init__(self):
        self._elementType = None
        self._elementId = None
        self._contactPoint = None

    def __str__(self):
        return str(self._elementType) + " with id " + str(self._elementId) + " contact at " + str(self._contactPoint)

    @property
    def elementType(self):
        return self._elementType

    @elementType.setter
    def elementType(self, value):
        if value not in ["road", "junction"]:
            raise AttributeError("Value must be road or junction")

        self._elementType = value

    @property
    def elementId(self):
        return self._elementId

    @elementId.setter
    def elementId(self, value):
        self._elementId = int(value)

    @property
    def contactPoint(self):
        return self._contactPoint

    @contactPoint.setter
    def contactPoint(self, value):
        if value not in ["start", "end"] and value is not None:
            raise AttributeError("Value must be start or end")

        self._contactPoint = value

class Successor(Predecessor):
    pass

    # def __init__(self):
    #     self._elementType = None
    #     self._elementId = None
    #     self._contactPoint = None

    # @property
    # def elementType(self):
    #     return self._elementType

    # @elementType.setter
    # def elementType(self, value):
    #     if value not in ["road", "junction"]:
    #         raise AttributeError("Value must be road or junction")

    #     self._elementType = value

    # @property
    # def elementId(self):
    #     return self._elementId

    # @elementId.setter
    # def elementId(self, value):
    #     self._elementId = int(value)

    # @property
    # def contactPoint(self):
    #     return self._contactPoint

    # @contactPoint.setter
    # def contactPoint(self, value):
    #     if value not in ["start", "end"] and value is not None:
    #         raise AttributeError("Value must be start or end")

    #     self._contactPoint = value


class Neighbor(object):

    def __init__(self):
        self._side = None
        self._elementId = None
        self._direction = None

    @property
    def side(self):
        return self._side

    @side.setter
    def side(self, value):
        if value not in ["left", "right"]:
            raise AttributeError("Value must be left or right")

        self._side = value

    @property
    def elementId(self):
        return self._elementId

    @elementId.setter
    def elementId(self, value):
        self._elementId = int(value)

    @property
    def direction(self):
        return self._direction

    @direction.setter
    def direction(self, value):
        if value not in ["same", "opposite"]:
            raise AttributeError("Value must be same or opposite")

        self._direction = value



# road_plan_view.py

import abc
import numpy as np
from eulerspiral import eulerspiral

class PlanView(object):

    def __init__(self):
        self._geometries = []

    @property
    def geometries(self):
        return self._geometries

    def addLine(self, start_s, startPosition, heading, length):
        self._geometries.append(Line(start_s, startPosition, heading, length))

    def addSpiral(self, start_s, startPosition, heading, length, curvStart, curvEnd):
        self._geometries.append(Spiral(start_s, startPosition, heading, length, curvStart, curvEnd))

    def addArc(self, start_s, startPosition, heading, length, curvature):
        self._geometries.append(Arc(start_s, startPosition, heading, length, curvature))

    def addParamPoly3(self, start_s, startPosition, heading, length, aU, bU, cU, dU, aV, bV, cV, dV, pRange):
        self._geometries.append(ParamPoly3(start_s, startPosition, heading, length, aU, bU, cU, dU, aV, bV, cV, dV, pRange))

    def getLength(self):
        """ Get length of whole plan view """

        length = 0

        for geometry in self._geometries:
            length += geometry.getLength()

        return length

    def calc(self, sPos):
        """ Calculate position and tangent at sPos """

        for geometry in self._geometries:
            if geometry.getLength() < sPos and not np.isclose(geometry.getLength(), sPos):
                sPos -= geometry.getLength()
                continue
            return geometry.calcPosition(sPos)

        raise Exception("Tried to calculate a position outside of the borders of the trajectory by s=" + str(sPos))


class Geometry(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def getStartPosition(self):
        """ Returns the overall geometry length """
        return

    @abc.abstractmethod
    def getLength(self):
        """ Returns the overall geometry length """
        return
    
    @abc.abstractmethod
    def getRefPoints(self):
        return

    @abc.abstractmethod
    def calcPosition(self, s):
        """ Calculates the position of the geometry as if the starting point is (0/0) """
        return
    
    @abc.abstractmethod
    def calcRefPoints(self, step=0.01):
        return

class Line(Geometry):

    def __init__(self, start_s, startPosition, heading, length):
        self.start_s = start_s
        self.startPosition = np.array(startPosition)
        self.heading = heading
        self.length = length
        self.refPoints = []

    def getStartPosition(self):
        return self.startPosition

    def getLength(self):
        return self.length
    
    def getRefPoints(self):
        return self.refPoints

    def calcPosition(self, s):
        pos = self.startPosition + np.array([s * np.cos(self.heading), s * np.sin(self.heading)])
        tangent = self.heading

        return (pos, tangent)
    
    def calcRefPoints(self, step=0.1):
        pos, tangent = self.calcPosition(0)
        x, y = pos
        one_point = {
            "position": (x, y),
            "tangent": tangent,
            "s_geometry": 0.0,
            "s_road": self.start_s
        }
        self.refPoints.append(one_point)

        point_nums = int(self.length / step)
        for i in range(1, point_nums + 1):
            s_geometry = i * step
            s_road = s_geometry + self.start_s
            pos, tangent = self.calcPosition(s_geometry)
            x, y = pos
            one_point = {
                "position": (x, y),
                "tangent": tangent,
                "s_geometry": s_geometry,
                "s_road": s_road
            }
            self.refPoints.append(one_point)
        
        if (self.length % step) != 0:
            s_geometry = self.length
            s_road = s_geometry + self.start_s
            pos, tangent = self.calcPosition(s_geometry)
            x, y = pos
            one_point = {
                "position": (x, y),
                "tangent": tangent,
                "s_geometry": s_geometry,
                "s_road": s_road
            }
            self.refPoints.append(one_point)

class Arc(Geometry):

    def __init__(self, start_s, startPosition, heading, length, curvature):
        self.start_s = start_s
        self.startPosition = np.array(startPosition)
        self.heading = heading
        self.length = length
        self.curvature = curvature
        self.refPoints = []

    def getStartPosition(self):
        return self.startPosition

    def getLength(self):
        return self.length
    
    def getRefPoints(self):
        return self.refPoints

    def calcPosition(self, s):
        c = self.curvature
        hdg = self.heading - np.pi / 2

        a = 2 / c * np.sin(s * c / 2)
        alpha = (np.pi - s * c) / 2 - hdg

        dx = -1 * a * np.cos(alpha)
        dy = a * np.sin(alpha)

        pos = self.startPosition + np.array([dx, dy])
        tangent = self.heading + s * self.curvature

        return (pos, tangent)
    
    def calcRefPoints(self, step=0.01):
        pos, tangent = self.calcPosition(0)
        x, y = pos
        one_point = {
            "position": (x, y),
            "tangent": tangent,
            "s_geometry": 0.0,
            "s_road": self.start_s
        }
        self.refPoints.append(one_point)

        point_nums = int(self.length / step)
        for i in range(1, point_nums + 1):
            s_geometry = i * step
            s_road = s_geometry + self.start_s
            pos, tangent = self.calcPosition(s_geometry)
            x, y = pos
            one_point = {
                "position": (x, y),
                "tangent": tangent,
                "s_geometry": s_geometry,
                "s_road": s_road
            }
            self.refPoints.append(one_point)

        if (self.length % step) != 0:
            s_geometry = self.length
            s_road = s_geometry + self.start_s
            pos, tangent = self.calcPosition(s_geometry)
            x, y = pos
            one_point = {
                "position": (x, y),
                "tangent": tangent,
                "s_geometry": s_geometry,
                "s_road": s_road
            }
            self.refPoints.append(one_point)

class Spiral(Geometry):

    def __init__(self, start_s, startPosition, heading, length, curvStart, curvEnd):
        self._start_s = start_s
        self._startPosition = np.array(startPosition)
        self._heading = heading
        self._length = length
        self._curvStart = curvStart
        self._curvEnd = curvEnd
        self._refPoints = []

        self._spiral = eulerspiral.EulerSpiral.createFromLengthAndCurvature(self._length, self._curvStart, self._curvEnd)

    def getStartPosition(self):
        return self._startPosition

    def getLength(self):
        return self._length
    
    def getRefPoints(self):
        return self._refPoints

    def calcPosition(self, s):
        (x, y, t) = self._spiral.calc(s, self._startPosition[0], self._startPosition[1], self._curvStart, self._heading)

        return (np.array([x, y]), t)
    
    def calcRefPoints(self, step=0.01):
        pos, tangent = self.calcPosition(0)
        x, y = pos
        one_point = {
            "position": (x, y),
            "tangent": tangent,
            "s_geometry": 0.0,
            "s_road": self._start_s
        }
        self._refPoints.append(one_point)

        point_nums = int(self._length / step)
        for i in range(1, point_nums + 1):
            s_geometry = i * step
            s_road = s_geometry + self._start_s
            pos, tangent = self.calcPosition(s_geometry)
            x, y = pos
            one_point = {
                "position": (x, y),
                "tangent": tangent,
                "s_geometry": s_geometry,
                "s_road": s_road
            }
            self._refPoints.append(one_point)

        if (self._length % step) != 0:
            s_geometry = self._length
            s_road = s_geometry + self._start_s
            pos, tangent = self.calcPosition(s_geometry)
            x, y = pos
            one_point = {
                "position": (x, y),
                "tangent": tangent,
                "s_geometry": s_geometry,
                "s_road": s_road
            }
            self._refPoints.append(one_point)

class Poly3(Geometry):

    def __init__(self, start_s, startPosition, heading, length, a, b, c, d):
        self._start_s = start_s
        self._startPosition = np.array(startPosition)
        self._heading = heading
        self._length = length
        self._a = a
        self._b = b
        self._c = c
        self._d = d
        self._refPoints = []

        raise NotImplementedError()

    def getStartPosition(self):
        return self._startPosition

    def getLength(self):
        return self._length
    
    def getRefPoints(self):
        return self._refPoints

    def calcPosition(self, s):
        # TODO untested

        # Calculate new point in s/t coordinate system
        coeffs = [self._a, self._b, self._c, self._d]

        t = np.polynomial.polynomial.polyval(s, coeffs)

        # Rotate and translate
        srot = s * np.cos(self._heading) - t * np.sin(self._heading)
        trot = s * np.sin(self._heading) + t * np.cos(self._heading)

        # Derivate to get heading change
        dCoeffs = coeffs[1:] * np.array(np.arange(1, len(coeffs)))
        tangent = np.polynomial.polynomial.polyval(s, dCoeffs)

        return (self._startPosition + np.array([srot, trot]), self._heading + tangent)
    
    def calcRefPoints(self, step=0.01):
        pos, tangent = self.calcPosition(0)
        x, y = pos
        one_point = {
            "position": (x, y),
            "tangent": tangent,
            "s_geometry": 0.0,
            "s_road": self._start_s
        }
        self.refPoints.append(one_point)

        point_nums = int(self._length / step)
        for i in range(1, point_nums + 1):
            s_geometry = i * step
            s_road = s_geometry + self._start_s
            pos, tangent = self.calcPosition(s_geometry)
            x, y = pos
            one_point = {
                "position": (x, y),
                "tangent": tangent,
                "s_geometry": s_geometry,
                "s_road": s_road
            }
            self._refPoints.append(one_point)

        if (self._length % step) != 0:
            s_geometry = self._length
            s_road = s_geometry + self._start_s
            pos, tangent = self.calcPosition(s_geometry)
            x, y = pos
            one_point = {
                "position": (x, y),
                "tangent": tangent,
                "s_geometry": s_geometry,
                "s_road": s_road
            }
            self._refPoints.append(one_point)

class ParamPoly3(Geometry):

    def __init__(self, start_s, startPosition, heading, length, aU, bU, cU, dU, aV, bV, cV, dV, pRange):
        self._start_s = start_s
        self._startPosition = np.array(startPosition)
        self._heading = heading
        self._length = length
        self._refPoints = []

        self._aU = aU
        self._bU = bU
        self._cU = cU
        self._dU = dU
        self._aV = aV
        self._bV = bV
        self._cV = cV
        self._dV = dV

        if pRange is None:
            self._pRange = 1.0
        else:
            self._pRange = pRange

    def getStartPosition(self):
        return self._startPosition

    def getLength(self):
        return self._length
    
    def getRefPoints(self):
        return self._refPoints

    def calcPosition(self, s):

        # Position
        pos = (s / self._length) * self._pRange

        coeffsU = [self._aU, self._bU, self._cU, self._dU]
        coeffsV = [self._aV, self._bV, self._cV, self._dV]

        x = np.polynomial.polynomial.polyval(pos, coeffsU)
        y = np.polynomial.polynomial.polyval(pos, coeffsV)

        xrot = x * np.cos(self._heading) - y * np.sin(self._heading)
        yrot = x * np.sin(self._heading) + y * np.cos(self._heading)

        # Tangent is defined by derivation
        dCoeffsU = coeffsU[1:] * np.array(np.arange(1, len(coeffsU)))
        dCoeffsV = coeffsV[1:] * np.array(np.arange(1, len(coeffsV)))

        dx = np.polynomial.polynomial.polyval(pos, dCoeffsU)
        dy = np.polynomial.polynomial.polyval(pos, dCoeffsV)

        tangent = np.arctan2(dy, dx)


        return (self._startPosition + np.array([xrot, yrot]), self._heading + tangent)
    
    def calcRefPoints(self, step=0.01):
        pos, tangent = self.calcPosition(0)
        x, y = pos
        one_point = {
            "position": (x, y),
            "tangent": tangent,
            "s_geometry": 0.0,
            "s_road": self._start_s
        }
        self.refPoints.append(one_point)

        point_nums = int(self._length / step)
        for i in range(1, point_nums + 1):
            s_geometry = i * step
            s_road = s_geometry + self._start_s
            pos, tangent = self.calcPosition(s_geometry)
            x, y = pos
            one_point = {
                "position": (x, y),
                "tangent": tangent,
                "s_geometry": s_geometry,
                "s_road": s_road
            }
            self._refPoints.append(one_point)

        if (self._length % step) != 0:
            s_geometry = self._length
            s_road = s_geometry + self._start_s
            pos, tangent = self.calcPosition(s_geometry)
            x, y = pos
            one_point = {
                "position": (x, y),
                "tangent": tangent,
                "s_geometry": s_geometry,
                "s_road": s_road
            }
            self._refPoints.append(one_point)
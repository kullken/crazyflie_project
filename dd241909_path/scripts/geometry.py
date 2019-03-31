#!/usr/bin/env python

from __future__ import division

import math
from collections import namedtuple



# Since math.isclose() does not exist in python 2.7
# Inspired by example provided by PEP author
def isclose(a, b, rel_tol=1e-9, abs_tol=0.0):
    if a == b:
        return True
    diff = abs(b - a)
    return (((diff <= abs(rel_tol * b)) or
             (diff <= abs(rel_tol * a))) or
             (diff <= abs_tol))

Waypoint = namedtuple('Waypoint', ('x', 'y', 'z', 'vx', 'vy', 'vz', 'yaw'))

class Line(object):

    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

    def __abs__(self):
        return abs(self.p2 - self.p1)


class Bezier(object):

    def __init__(self, points, duration):
        self.points = points
        self.T = duration
        self._derivative = None
        self._coeffs = None

    def __repr__(self):
        return 'Bezier({}, {})'.format(self.points, self.T)

    @property
    def degree(self):
        return len(self.points) - 1

    @property
    def derivative(self):
        if not self._derivative:
            newpoints = []
            for i in range(len(self.points)-1):
                newpoint = self.degree/self.T * (self.points[i+1] - self.points[i])
                newpoints.append(newpoint)
            if not newpoints:
                print(self)
            self._derivative = Bezier(newpoints, self.T)
        return self._derivative

    @property
    def coeffs(self):
        if not self._coeffs:
            self._coeffs = self.calc_coeffs(self.points, self.T, self.degree)
        return self._coeffs
    
    def reversed(self):
        return Bezier([reversed(self.points)], self.T)

    def pos(self, t):
        if t == 0:
            return self.points[0]
        elif t == self.T or t == -1:
            return self.points[-1]
        else:
            return self._value(t)

    def vel(self, t):
        return self.derivative.pos(t)

    def acc(self, t):
        return self.derivative.vel(t)

    def split(self, t):

        if t == 0:
            return Bezier([self.points[0]], 0), Bezier(self.points, self.T)
        elif t == self.T:
            return Bezier(self.points, self.T), Bezier([self.points[-1]], 0)

        # Scale t to interval 0-1
        t = t / self.T

        # Taken from:
        # https://pomax.github.io/bezierinfo/#splitting
        lower = []
        upper = []
        def split(points, t):
            if len(points) == 1:
                lower.append(points[0])
                upper.append(points[0])
            else:
                newpoints = []
                for i in range(len(points)-1):
                    if i == 0:
                        lower.append(points[i])
                    if i == len(points)-2:
                        upper.append(points[i+1])
                    newpoints.append((1-t) * points[i] + t * points[i+1])
                split(newpoints, t)

        split(self.points, t)

        return Bezier(lower, t*self.T), Bezier(upper, (1-t)*self.T)

    def _value(self, t):
        res = self.coeffs[-1]
        for c in reversed(self.coeffs[:-1]):
            res = t*res + c
        return res

    @staticmethod
    def de_casteljau(points, T, t):
        if T != 1:
            t = t / T
        
        if len(points) == 1:
            return points[0]
        else:
            newpoints = []
            for i in range(len(points)-1):
                newpoint = (1-t)*points[i] + t*points[i+1]
                newpoints.append(newpoint)
            return Bezier.value(newpoints, 1, t)

    @staticmethod
    def calc_coeffs(points, T, degree):
        if degree == 4:
            p0, p1, p2, p3, p4 = points
            c0 = p0
            c1 = 4/T * (p1 - p0)
            c2 = 6/T**2 * (p2 - 2*p1 + p0)
            c3 = 4/T**3 * (p3 - 3*p2 + 3*p1 - p0)
            c4 = 1/T**4 * (p4 - 4*p3 + 6*p2 - 4*p1 + p0)
            return [c0, c1, c2, c3, c4]
        if degree == 3:
            p0, p1, p2, p3 = points
            c0 = p0
            c1 = 3/T * (p1 - p0)
            c2 = 3/T**2 * (p2 - 2*p1 + p0)
            c3 = 1/T**3 * (p3 - 3*p2 + 3*p1 - p0)
            return [c0, c1, c2, c3]
        else:
            # https://en.wikipedia.org/wiki/B%C3%A9zier_curve#Polynomial_form
            coeffs = []
            n = degree
            for j in range(n + 1):
                prod = 1 / T**j
                for m in range(j):
                    prod *= n - m
                sum = Vec3(0, 0, 0)
                for i in range(j + 1):
                    sum += (-1)**(i+j) / (math.factorial(i)*math.factorial(j-i)) * points[i]
                coeffs.append(prod * sum)
            return coeffs


class Vec3(object):
    __slots__ = '_x', '_y', '_z'

    # To hopefully override at least some numpy operations
    __array_priority__ = 100

    def __init__(self, x, y, z):
        self._x, self._y, self._z = x, y, z

    @property
    def x(self):
        return self._x
    @property
    def y(self):
        return self._y
    @property
    def z(self):
        return self._z

    def __getitem__(self, key):
        if key == 0:
            return self.x
        elif key == 1:
            return self.y
        elif key == 2:
            return self.z
        else:
            raise IndexError()

    def __repr__(self):
        return 'Vec3({}, {}, {})'.format(round(self.x, 4), round(self.y, 4), round(self.z, 4))
    def __str__(self):
        return 'Vec3({}, {}, {})'.format(round(self.x, 4), round(self.y, 4), round(self.z, 4))

    def __hash__(self):
        return hash((self._x, self._y, self._z))

    def __len__(self):
        return 3

    def __getitem__(self, key):
        if key == 0:
            return self._x
        elif key == 1:
            return self._y
        elif key == 2:
            return self._z
        else:
            raise IndexError()

    def __eq__(self, other):
        #return self._x == other._x and self._y == other._y and self._z == other._z
        return isclose(self._x, other._x) and isclose(self._y, other._y) and isclose(self._z, other._z)
    def __ne__(self, other):
        #return self._x != other._x or self._y != other._y or self._z != other._z
        return not isclose(self._x, other._x) or not isclose(self._y, other._y) or not isclose(self._z, other._z)

    def __lt__(self, other):
        return self._x < other._x and self._y < other._y and self._z < other._z
    def __le__(self, other):
        return self._x <= other._x and self._y <= other._y and self._z <= other._z
    def __gt__(self, other):
        return self._x > other._x and self._y > other._y and self._z > other._z
    def __ge__(self, other):
        return self._x >= other._x and self._y >= other._y and self._z >= other._z

    def __abs__(self):
        return (self._x**2 + self._y**2 + self._z**2)**0.5

    def __neg__(self):
        return Vec3(-self._x, -self._y, -self._z)

    def __add__(self, other):
        return Vec3(self._x+other._x, self._y+other._y, self._z+other._z)
    def __radd__(self, other):
        return Vec3(other._x+self._x, other._y+self._y, other._z+self._z)

    def __sub__(self, other):
        return Vec3(self._x-other._x, self._y-other._y, self._z-other._z)
    def __rsub__(self, other):
        return Vec3(other._x-self._x, other._y-self._y, other._z-self._z)

    def __mul__(self, other):
        return Vec3(self._x*other, self._y*other, self._z*other)
    def __rmul__(self, other):
        return Vec3(other*self._x, other*self._y, other*self._z)

    def __div__(self, other):
        return Vec3(self._x/other, self._y/other, self._z/other)
    def __truediv__(self, other):
        return Vec3(self._x/other, self._y/other, self._z/other)

    def unit(self):
        return self/abs(self)

    def dot(self, other):
        return self._x*other._x + self._y*other._y + self._z*other._z

    def cross(self, other):
        x = self._y*other._z - self._z*other._y
        y = self._z*other._x - self._x*other._z
        z = self._x*other._y - self._y*other._x
        return Vec3(x, y, z)

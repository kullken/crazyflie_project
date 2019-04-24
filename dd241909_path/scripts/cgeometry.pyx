# cython: profile=True

from __future__ import division

import math
from collections import namedtuple

#from cpython.object cimport Py_EQ, Py_NE, Py_LT, Py_LE, Py_GT, Py_GE
cimport cpython.object

from myprofiling import profile

print('Hello from inside cython geometry module!')


# Since math.isclose() does not exist in python 2.7
# Inspired by example provided by PEP author
cpdef bint isclose(a, b, rel_tol=1e-9, abs_tol=1e-9):
    cdef double diff
    if a == b:
        return True
    diff = abs(b - a)
    return (((diff <= abs(rel_tol * b)) or
             (diff <= abs(rel_tol * a))) or
             (diff <= abs_tol))

Waypoint = namedtuple('Waypoint', ('id', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'yaw'))

cdef class Line:
    cdef Vec3 p1, p2

    def __init__(self, Vec3 p1, Vec3 p2):
        self.p1 = p1
        self.p2 = p2

    def __abs__(self):
        return abs(self.p2 - self.p1)


cdef class Bezier:
    cdef readonly list points
    cdef readonly double T
    cdef readonly int degree
    cdef readonly list coeffs
    cdef Bezier _derivative

    def __init__(self, list points, double duration):
        self.points = points
        self.T = duration
        self.degree = len(points) - 1
        self.coeffs = Bezier._calc_coeffs(self.points, self.T, self.degree)
        self._derivative = None

    def __repr__(self):
        return 'Bezier({}, {})'.format(self.points, self.T)

    @property
    def derivative(self):
        if not self._derivative:
            self._derivative = Bezier._calc_derivative(self.points, self.T, self.degree)
        return self._derivative

    cdef Bezier get_derivative(self):
        """Fast cython only derivative getter."""
        if not self._derivative:
            self._derivative = Bezier._calc_derivative(self.points, self.T, self.degree)
        return self._derivative
    
    def reversed(self):
        return Bezier(self.points[::-1], self.T)

    cpdef Vec3 pos(self, double t):
        if t == 0.0:
            return self.points[0]
        elif t == self.T or t == -1.0:
            return self.points[-1]
        else:
            return self._value(t)

    cpdef Vec3 vel(self, double t):
        return self.get_derivative().pos(t)

    cpdef Vec3 acc(self, double t):
        return self.get_derivative().vel(t)

    def split(self, double t):

        if t == 0.0:
            return Bezier([self.points[0]], 0.0), Bezier(self.points, self.T)
        elif t == self.T or t == -1.0:
            return Bezier(self.points, self.T), Bezier([self.points[-1]], 0.0)

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
        upper.reverse()

        return Bezier(lower, t*self.T), Bezier(upper, (1-t)*self.T)
    
    cdef Vec3 _value(self, double t):
        cdef Vec3 pos, c
        cdef double x = 0.0
        cdef double y = 0.0
        cdef double z = 0.0
        for c in reversed(self.coeffs):
            x = x*t + c.x
            y = y*t + c.y
            z = z*t + c.z
        pos = Vec3(x, y, z)

        return pos

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
    cdef Bezier _calc_derivative(list points, double T, int degree):
        cdef list newpoints = []
        cdef Vec3 newpoint
        cdef int i

        for i in range(degree):
            newpoint = degree/T * (points[i+1] - points[i])
            newpoints.append(newpoint)
        return Bezier(newpoints, T)

    @staticmethod
    cdef list _calc_coeffs(list points, double T, int degree):
        if degree == 5:
            p0, p1, p2, p3, p4, p5 = points
            c0 = p0
            c1 = 5/T * (p1 - p0)
            c2 = 10/T**2 * (p2 - 2*p1 + p0)
            c3 = 10/T**3 * (p3 - 3*p2 + 3*p1 - p0)
            c4 = 5/T**4 * (p4 - 4*p3 + 6*p2 - 4*p1 + p0)
            c5 = 1/T**5 * (p5 - 5*p4 + 10*p3 - 10*p2 + 5*p1 - p0)
            return [c0, c1, c2, c3, c4, c5]
        elif degree == 4:
            p0, p1, p2, p3, p4 = points
            c0 = p0
            c1 = 4/T * (p1 - p0)
            c2 = 6/T**2 * (p2 - 2*p1 + p0)
            c3 = 4/T**3 * (p3 - 3*p2 + 3*p1 - p0)
            c4 = 1/T**4 * (p4 - 4*p3 + 6*p2 - 4*p1 + p0)
            return [c0, c1, c2, c3, c4]
        elif degree == 3:
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

    @staticmethod
    def newPenticBezier(Vec3 pos1, Vec3 vel1, Vec3 acc1, Vec3 pos2, Vec3 vel2, Vec3 acc2, double T):
        cdef Vec3 p0, p1, p2, p3, p4, p5
        # Initial conditions
        p0 = pos1
        p1 = T/5 * vel1 + p0
        p2 = T**2/20 * acc1 + 2*p1 - p0

        p5 = pos2
        p4 = -T/5 * vel2 + p5
        p3 = T**2/20 * acc2 + 2*p4 - p5

        return Bezier([p0, p1, p2, p3, p4, p5], T)


cdef class Vec3:
    cdef readonly double x, y, z

    # To hopefully override at least some numpy operations
    __array_priority__ = 100
    #cdef readonly int __array_priority__

    def __init__(self, double x, double y, double z):
        self.x = x
        self.y = y
        self.z = z
        #self.__array_priority__ = 100

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
        return hash((self.x, self.y, self.z))

    def __len__(self):
        return 3

    def __richcmp__(Vec3 self, Vec3 other, int op):
        if op == cpython.object.Py_EQ:
            return isclose(self.x, other.x) and isclose(self.y, other.y) and isclose(self.z, other.z)
        elif op == cpython.object.Py_NE:
            return not isclose(self.x, other.x) or not isclose(self.y, other.y) or not isclose(self.z, other.z)
        elif op == cpython.object.Py_LT:
            return self.x < other.x and self.y < other.y and self.z < other.z
        elif op == cpython.object.Py_LE:
            return self.x <= other.x and self.y <= other.y and self.z <= other.z
        elif op == cpython.object.Py_GT:
            return self.x > other.x and self.y > other.y and self.z > other.z
        elif op == cpython.object.Py_GE:
            return self.x >= other.x and self.y >= other.y and self.z >= other.z
        else:
            raise ValueError('Error: Unknown operator: {}'.format(op))

    def __abs__(self):
        return (self.x**2 + self.y**2 + self.z**2)**0.5

    def __neg__(self):
        return Vec3(-self.x, -self.y, -self.z)

    def __add__(Vec3 left, Vec3 right):
        return Vec3(left.x+right.x, left.y+right.y, left.z+right.z)
    def __iadd__(self, Vec3 other):
        self.x += other.x
        self.y += other.y
        self.z += other.z
        return self

    def __sub__(Vec3 left, Vec3 right):
        return Vec3(left.x-right.x, left.y-right.y, left.z-right.z)
    def __isub__(self, Vec3 other):
        self.x -= other.x
        self.y -= other.y
        self.z -= other.z
        return self

    def __mul__(left, right):
        cdef Vec3 vec
        cdef double scalar
        if isinstance(left, Vec3):
            vec = <Vec3>left
            scalar = <double>right
        else:
            vec = <Vec3>right
            scalar = <double>left
        return Vec3(vec.x*scalar, vec.y*scalar, vec.z*scalar)
    def __imul__(self, double other):
        self.x *= other
        self.y *= other
        self.z *= other
        return self

    def __div__(Vec3 self, double scalar):
        return Vec3(self.x/scalar, self.y/scalar, self.z/scalar)
    def __idiv__(self, double other):
        self.x /= other
        self.y /= other
        self.z /= other
        return self
        
    def __truediv__(Vec3 self, double scalar):
        return Vec3(self.x/scalar, self.y/scalar, self.z/scalar)
    def __itruediv__(self, double other):
        self.x /= other
        self.y /= other
        self.z /= other
        return self

    cdef Vec3 add(self, Vec3 other):
        """Fast cython only addition operator."""
        return Vec3(self.x+other.x, self.y+other.y, self.z+other.z)
    cdef void iadd(self, Vec3 other):
        """Fast cython only in-place addition operator."""
        self.x += other.x
        self.y += other.y
        self.z += other.z

    cdef Vec3 sub(self, Vec3 other):
        """Fast cython only subtraction operator."""
        return Vec3(self.x-other.x, self.y-other.y, self.z-other.z)
    cdef void isub(self, Vec3 other):
        """Fast cython only in-place subtraction operator."""
        self.x -= other.x
        self.y -= other.y
        self.z -= other.z

    cdef Vec3 mul(self, double scalar):
        """Fast cython only multiplication operator."""
        return Vec3(self.x*scalar, self.y*scalar, self.z*scalar)
    cdef void imul(self, double scalar):
        """Fast cython only in-place multiplication operator."""
        self.x *= scalar
        self.y *= scalar
        self.z *= scalar

    cdef Vec3 div(self, double scalar):
        """Fast cython only division operator."""
        return Vec3(self.x/scalar, self.y/scalar, self.z/scalar)
    cdef void idiv(self, double scalar):
        """Fast cython only in-place division operator."""
        self.x /= scalar
        self.y /= scalar
        self.z /= scalar

    cpdef Vec3 copy(self):
        return Vec3(self.x, self.y, self.z)

    cpdef Vec3 unit(self):
        return self.div(abs(self))

    cpdef double dot(self, Vec3 other):
        return self.x*other.x + self.y*other.y + self.z*other.z

    cpdef Vec3 cross(self, Vec3 other):
        cdef double x = self.y*other.z - self.z*other.y
        cdef double y = self.z*other.x - self.x*other.z
        cdef double z = self.x*other.y - self.y*other.x
        return Vec3(x, y, z)

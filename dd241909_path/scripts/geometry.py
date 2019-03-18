#!/usr/bin/env python

from __future__ import division


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
        return self._x == other._x and self._y == other._y and self._z == other._z
    def __ne__(self, other):
        return self._x != other._x or self._y != other._y or self._z != other._z

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

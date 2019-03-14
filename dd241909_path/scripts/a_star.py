#!/usr/bin/env python

import math
import heapq
from collections import namedtuple

import rospy

from geometry import Vec3


class AStar(object):

    def __init__(self, map):
        self.map = map

        # Parameters
        self.grid_size = 0.1

    def plan_path(self, start_wp, waypoints):
        rospy.loginfo('A*: Starting...')

        # Dummy return variable
        path = []

        # Create start node
        xi, yi = self.exact_to_index(start_wp.x, start_wp.y)
        startnode = Node(xi, yi, wpi=0)
        startnode.parent = None
        startnode.cost = 0
        startnode.heuristic = self.heuristic(startnode, waypoints)

        # A* loop setup
        openset = HeapPriorityQueue()
        openset.push(startnode)
        closedset = set()
        iter = 0

        # A* main loop
        while not openset.is_empty():
            currentnode = openset.pop()
            closedset.add(currentnode.key)

            iter += 1
            rospy.loginfo_throttle(1, 'A*: Iteration {}'.format(iter))

            if self.reached_goal(currentnode, waypoints):
                path = self.retrace_path(currentnode)
                break
            else:
                self.update_neighbours(currentnode, waypoints, openset, closedset)

        if path:
            rospy.loginfo('A*: Goal reached, success!')
            rospy.loginfo('A*: {} total iterations'.format(iter))
        else:
            rospy.logwarn('A*: Failed!')
            rospy.loginfo('A*: {} total iterations'.format(iter))
        return path

    def update_neighbours(self, currentnode, waypoints, openset, closedset):

        neighbours = {(dx,dy) for dx in (-1,0,1) for dy in (-1,0,1) if (dx or dy)}

        for dx, dy in neighbours:
            newnode = self.get_newnode(currentnode, dx, dy, waypoints)

            x, y = self.index_to_exact(newnode.xi, newnode.yi)
            position = Vec3(x, y, 0.4)

            if newnode.key in closedset:
                continue
            elif self.map.query([position]):
                continue
            elif newnode.key in openset:
                old_prio = openset.get_item(newnode.key).prio
                if newnode.prio <= old_prio:
                    openset.update_queue(newnode)
            else:
                openset.push(newnode)

    def get_newnode(self, parent, dx, dy, waypoints):
        newnode = Node(parent.xi+dx, parent.yi+dy, parent.wpi)
        if self.reached_wp(newnode, waypoints[parent.wpi]):
            newnode = Node(parent.xi+dx, parent.yi+dy, parent.wpi+1)

        newnode.parent = parent
        newnode.cost = parent.cost + (dx**2 + dy**2)**0.5 * self.grid_size
        newnode.heuristic = self.heuristic(newnode, waypoints)
        return newnode

    def retrace_path(self, node):
        Waypoint = namedtuple('Waypoint', ('x', 'y', 'z', 'yaw'))
        path = []
        while not node is None:
            x, y = self.index_to_exact(node.xi, node.yi)
            path.append(Waypoint(x, y, 0.4, 0))
            node = node.parent
        path.reverse()
        return path

    def reached_goal(self, node, waypoints):
        return node.wpi == len(waypoints)

    def reached_wp(self, node, waypoint):
        nx, ny = self.index_to_exact(node.xi, node.yi)
        wpx, wpy = waypoint.x, waypoint.y
        return ((nx-wpx)**2 + (ny-wpy)**2)**0.5 <= 0.1

    def heuristic(self, node, waypoints):
        heuristic = 0
        x1, y1 = self.index_to_exact(node.xi, node.yi)
        for wp in waypoints[node.wpi:]:
            x2, y2 = wp.x, wp.y
            heuristic += ((x1-x2)**2 + (y1-y2)**2)**0.5
            x1, y1 = x2, y2
        return heuristic

    def exact_to_index(self, x, y):
        return int(math.floor(x / self.grid_size)), int(math.floor(y / self.grid_size))

    def index_to_exact(self, xi, yi):
        return xi * self.grid_size, yi * self.grid_size




class Node(object):

    def __init__(self, xi, yi, wpi):
        self._xi = xi
        self._yi = yi
        self._wpi = wpi

    def __repr__(self):
        return 'Node({}, {}, {})'.format(self.xi, self.yi, self.wpi)

    def __hash__(self):
        return hash(self.key)
    def __eq__(self, other):
        return self.key == other.key
    def __ne__(self, other):
        return self.key != other.key

    @property
    def key(self):
        return (self.xi, self.yi, self.wpi)
    @property
    def xi(self):
        return self._xi
    @property
    def yi(self):
        return self._yi
    @property
    def wpi(self):
        return self._wpi
    
    @property
    def prio(self):
        return self.cost + self.heuristic


class HeapPriorityQueue(object):
    """Priority queue based on heapq module."""

    def __init__(self):
        self._heap = []
        self._dict = {}

    def push(self, item):
        """Push item into queue."""
        heapq.heappush(self._heap, (item.prio, item.key))
        self._dict[item.key] = item

    def pop(self):
        """Pop smallest item from queue."""
        key = heapq.heappop(self._heap)[1]
        item = self._dict.pop(key)
        return item

    def __contains__(self, key):
        return key in self._dict

    def get_item(self, key):
        """Returns item saved with key."""
        return self._dict[key]

    def is_empty(self):
        """Return True if queue is empty, False otherwise."""
        return len(self._heap) == 0

    def size(self):
        """Returns number of nodes in queue."""
        return len(self._heap)

    def lowest_prio(self):
        """Get priority of first node in queue."""
        # Queue might be empty.
        if self.is_empty():
            return -1
        else:
            return self._heap[0][0]

    def update_queue(self, newnode):
        """Updates node and resort queue."""
        # Remove old node with same key
        oldnode = self._dict[newnode.key]
        self._heap.remove((oldnode.prio, oldnode.key))
        heapq.heapify(self._heap)
        # Then push new node as usual
        self.push(newnode)

    def remove_everything_before_wp_index(self, input_wpi):
        """Remove all entries with wpi < input_wpi."""
        new_heap = []
        for entry in self._heap:
            key = entry[1]
            if key[-1] >= input_wpi:
                new_heap.append(entry)
            else:
                self._dict.pop(key)

        self._heap = new_heap
        heapq.heapify(self._heap)

#!/usr/bin/env python

import heapq


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

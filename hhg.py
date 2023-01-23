from collections import defaultdict
import math
import numpy as np
import pybloomfilter

def floor_log2(n):
    lg = 0
    while n > 1:
        lg += 1
        n //= 2
    return lg


class HHG:
    
    @staticmethod
    def calculate_box_position(low, high):
    
        span = max(high - low)
        level = floor_log2(int(math.floor(1/span)))
        space = 1 / (2**level)
        lo = np.floor(low / space).astype(int)
        hi = np.floor(high / space).astype(int)
    
        delta = sum(hi - lo)
    
        while delta != 0:
            level -= 1
            lo //= 2
            hi //= 2
        
            delta = sum(hi - lo)
    
        return level, lo[0], lo[1] 
    
    def __init__(self, box = None, reverse_index_scale = None):
        self.grids = defaultdict(lambda: defaultdict(set))
        self.boxes = dict()
        self.filter = pybloomfilter.BloomFilter(2**reverse_index_scale, 0.01) if reverse_index_scale else None
        # And store the optional scaling box if one other than [0,1]^2 is provided
        self.low = box[0] if box else None
        self.span = box[1] - box[0] if box else None
             
    def insert_bbox(self,low,high,i):
        # If we're working in a transformed coordinate space, transform the bounding box
        if self.low is not None:
            low = (low - self.low) / self.span
            high = (high - self.low) / self.span
        
        level,ix,iy = HHG.calculate_box_position(low,high)
        self.grids[level][(ix,iy)].add(i)
        self.boxes[i] = (level,ix,iy)
        
        while level > -1:
            if self.filter:
                self.filter.update([(level,ix,iy)])
            level -= 1
            ix //= 2
            iy //= 2
                
            if level in self.grids and (ix,iy) in self.grids[level]:
                break          
    
    def intersection_sets(self, dead):
        for level in sorted(self.grids.keys(), key = lambda x: 0 -x):
            grid = self.grids[level]
            for idx, items in grid.items():
                if len(items) != 1:
                    yield items, items
                else:
                    m, = items
                    if m in dead:
                        continue
                    
                ix,iy = idx
                for j in range(level-1,-1,-1):
                    ix //= 2
                    iy //= 2
                    
                    if j not in self.grids:
                        continue
                    grid = self.grids[j]

                    if (ix,iy) not in grid:
                        continue
                    other = grid[(ix,iy)]
                    
                    if len(other) == 1:
                        m, = other
                        if m in dead:
                            continue
                    yield items, other
                    
    def intersections(self, dead):
        for x,y in self.intersection_sets(dead):
            for a in x:
                for b in y:
                    if (x is y and b <= a) or (a in dead) or (b in dead):
                        continue
                    yield a,b


    def check_upwards(self, other, strict = False, flip = False):
        """ Check the elements of this HHG against the equal-to-larger elements
        of another HHG. If strict is true, only check strictly larger ones.

        Yields a list of possible intersection candidates, with the first element being
        from this grid, and the second from the other grid. If flip is true,
        reverse this order. """
        for i,g in self.grids.items():
            for (idx,idy),v in g.items():
                level = i
                if strict:
                    idx //= 2
                    idy //= 2
                    level -= 1
                for _ in range(i+1):
                    if level in other.grids and (idx,idy) in other.grids[level]:
                        for a in other.grids[level][(idx,idy)]:
                            for b in v:
                                yield (a,b) if flip else (b,a) 
                    idx //= 2
                    idy //= 2
                    level -= 1

    def merge(self, other, filter = None):
        """ Takes another HHG (must have an identical bounding box - not checked!), and adds all of its elements
        to this one. If a filter function is provided, it first checks them with it and only adds passing elements. """

        for level, grid in other.grids.items():
            for idx, elements in grid.items():
                for v in elements:
                    if filter is not None and not filter(v):
                        continue
                    
                    self.grids[level][idx].add(v)

    def elements(self):
        for grid in self.grids.values():
            for bucket in grid.values():
                for v in bucket:
                    yield v

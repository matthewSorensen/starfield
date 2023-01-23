#!/usr/bin/env python
# coding: utf-8

# In[22]:


import numpy as np
get_ipython().run_line_magic('matplotlib', 'inline')
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.tri as tri
from numpy.random import rand
import math
from collections import defaultdict
matplotlib.rcParams['figure.figsize'] = [14.0,14.0]
import inspect
import ezdxf
from dataclasses import dataclass
import time
from hhg import HHG
from itertools import chain


# In[23]:


def annulus_sampler(r,k):
    u,v = rand(k), rand(k)
    u *= 2 * math.pi
    rtrans = r * np.sqrt(1 + v)
    return np.vstack((rtrans * np.cos(u), rtrans * np.sin(u))).T

def pdsample(width, height, radius, points, criteria = None, k = 10):
    
    cell_size = radius / math.sqrt(2)
    grid = np.zeros((2 + math.ceil(width / cell_size),2 + math.ceil(height / cell_size)), dtype = int) - 1
    n,m = grid.shape
    n -= 2
    m -= 2
    
    def idx(pt):
        x,y = pt
        return math.floor(x / cell_size), math.floor(y / cell_size)
    
    parents, queue = [],[]
    
    for i,p in enumerate(points):
        ix,iy = idx(p)
        if grid[ix,iy] != -1:
            print("Collision in initial points!")
            return
        grid[ix,iy] = i
        queue.append(i)
    
    def is_viable(pt):
        x,y = pt
        if x < 0 or x > width or y < 0 or y > width:
            return False
        
        ix,iy = idx(pt)
        if ix < 0 or iy < 0 or n <= ix or m <= iy:
            return False
        if grid[ix,iy] != -1:
            return False
        
        for dx in range(max(0, ix - 2),min(n, ix + 3)):
            for dy in range(max(0, iy - 2),min(m, iy + 3)):
                g = grid[dx,dy]
                if g != -1:
                    delta = pt - points[g]
                    if delta.dot(delta) < radius**2:
                        return False
        if criteria:
            return criteria(pt)
        
        return True
    
    while len(queue):
        i = math.floor(len(queue) * rand(1))

        success = False
        
        for candidate in annulus_sampler(radius, k):
        
            candidate += points[queue[i]]
            
            if is_viable(candidate):
                new_index = len(points)
                
                points.append(candidate)
                #parents.append(queue[i])
                
                grid[idx(candidate)] = new_index
                queue.append(new_index)
                success = True
                
        if not success:
            queue[i] = queue[-1]
            queue.pop()
        
    
    return points


# In[24]:


def inside_box(dimension, radius):
    def criteria(pt):
        x,y = pt
        
        if x < radius or y < radius:
            return False
        
        if x > (dimension - radius) or y > (dimension - radius):
            return False
        
        return True
    return criteria


# In[35]:


dimension = 2 * 25 # Overall size that we should fill with the pattern
trace = 2 * 0.003 * 25.4 # Minimum feature size we can fabricate
space = 2 * 0.003 * 25.4 # Minimum distance between features we can fabricate
fraction = 0.25      # Scale between poisson disc spacing and circle radius - must be less than 0.25
keepout = 1.1
starting_feature = 1 # The radius of the largest disc in the pattern
exponent = 0.75 # Each time we iterate, how much do we shrink everything?

points = [np.array((0.5 * dimension,0.5 * dimension))]
scale, generation, generations =  starting_feature, 0, {0: {0}}
radii = [scale]

while True:
    # Calculate the new feature size, check if it's too small to make
    scale *= exponent
    if 2 * scale < trace:
        break
    radii.append(scale)
    # Compute the new points
    old_count = len(points)
    pdsample(dimension, dimension, scale / fraction, points, criteria = inside_box(dimension, scale))
    # Record them in generation log
    generation += 1
    generations[generation] =set(range(old_count, len(points)))

points = np.array(points)
radii = np.array(radii)


# In[53]:


def multiscale_poisson(dimension, starting_radius, ending_radius, fraction = 0.25, iterations = 10):
    points = [np.array((0.5 * dimension,0.5 * dimension))]
    radii = np.exp(np.linspace(math.log(starting_radius), math.log(ending_radius), iterations))
    generations = {0: {0}}
    for generation,r in enumerate(radii):
        old_count = len(points)
        pdsample(dimension, dimension, r / fraction, points, criteria = inside_box(dimension, r))
        generations[generation] = set(range(old_count, len(points)))

    
    return radii, np.array(points), generations

radii, points, generations = multiscale_poisson(dimension, starting_feature, 0.5 * space, iterations = 10)


# In[55]:


def prune_circles(centers, radii, keepout, clearance):
    effective_radii = np.maximum(keepout * scales, space + scales)
    # Build an empty hierarchical hash grid that we will merge each successive generation of
    # circles into. Each circle is stored as a pair of the index of its center and its generation.
    margin = np.max(effective_radii)
    bbox = np.min(centers, axis = 0) - margin, np.max(centers, axis = 0) + margin
    acc = HHG(bbox)
    
    for i in range(len(generations)):
        # Build an HHG of this generation, using the same bbox as the accumulator HHG
        # so indicies are directly comparable.
        new = HHG(bbox)
        r = effective_radii[i]
        for j in generations[i]:
            new.insert_bbox(centers[j] - r, centers[j] + r, (j,i))

        # Then filter this generation against the previous ones and merge the elements
        # that don't have any collisions
        dead = set()
        collisions = chain(acc.check_upwards(new),new.check_upwards(acc,strict = True, flip = True))
        for (old_idx, old_gen),(new_idx,_) in collisions:
            if new_idx in dead:
                continue
            delta = centers[old_idx] - centers[new_idx]
            if delta.dot(delta) <= (effective_radii[old_gen] + radii[i])**2:
                dead.add((new_idx,i))
            
        acc.merge(new, filter = lambda x: x not in dead)
        
    return acc
    
circles = prune_circles(points, radii, keepout, space)


# In[56]:


doc = ezdxf.new(dxfversion="R2010")
msp = doc.modelspace()

for index, generation in circles.elements():
    msp.add_circle(points[index], radius = radii[generation])
        
doc.saveas("test_small.dxf")


# In[37]:


@dataclass
class Quote:
    s : str
        
def write_sexp(sexp,fp):
    if isinstance(sexp, Quote):
        f.write('"' + sexp.s + '"')
    elif isinstance(sexp, list) or inspect.isgenerator(sexp):
        head = None
        f.write('(')
        for x in sexp:
            if head is None:
                head = x
            else:
                f.write(' ')
            write_sexp(x,f)
        f.write(')')
        if head == 'xy':
            f.write('\n')
    else:
        f.write(str(sexp))


def circle_to_tolerance(center, radius, error, minimum = 8):
    x,y = center
    n = max(minimum, math.ceil(math.pi / math.acos(abs(radius - error) / radius)))
    for t in np.linspace(0, 2 * math.pi, n):
        yield x + radius * math.cos(t), y + radius * math.sin(t)

def circle_as_square(center, radius):
    x,y = center
    phase = 2 * math.pi * np.random.rand(1)
    for t in phase + np.linspace(0,2 * math.pi, 5):
        yield x + radius * math.cos(t), y + radius * math.sin(t)

        
def kicad_polygon(layer, points):
    yield 'fp_poly'
    def sub():
        yield 'pts'
        for x,y in points:
            yield ['xy',x,y]
    yield sub()
    yield ['layer',layer]
    yield ['width',0]
    
def kicad_module(name, layers, objects, description = '', tags = None):
    yield 'module'
    yield name
    yield ['layer'] + layers
    yield ["tedit", hex(int(time.time()))[2:]]
    yield ['attr', 'virtual']
    if description:
        yield ["descr", Quote(description)],
    if tags:
        yield ['tags'] + tags
        
    yield from objects
    
layer = 'B.Cu'
module_name = 'starfield-calibrator-back'
error = 0.001
objects = (kicad_polygon(layer,circle_as_square(points[idx],radii[gen])) for idx, gen in circles.elements())

with open(module_name + ".kicad_mod", "w") as f:
    write_sexp(kicad_module(module_name,[layer], objects), f)


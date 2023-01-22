#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
get_ipython().run_line_magic('matplotlib', 'inline')
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.tri as tri
from numpy.random import rand
import math
from collections import defaultdict
import heapq
matplotlib.rcParams['figure.figsize'] = [14.0,14.0]
import triangle
import inspect
import ezdxf
from scipy.spatial import KDTree
from dataclasses import dataclass
import time
#from cvxopt import spmatrix, solvers, matrix
#from hhg import HHG


# In[2]:


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


# In[3]:


def inside_box(dimension, radius):
    def criteria(pt):
        x,y = pt
        
        if x < radius or y < radius:
            return False
        
        if x > (dimension - radius) or y > (dimension - radius):
            return False
        
        return True
    return criteria


# In[4]:


dimension = 25 # Overall size that we should fill with the pattern
trace = 0.003 * 25.4 # Minimum feature size we can fabricate
space = 0.003 * 25.4 # Minimum distance between features we can fabricate
fraction = 0.25      # Scale between poisson disc spacing and circle radius
keepout = 1.1
starting_feature = 1 # The radius of the largest disc in the pattern
exponent = 0.75 # Each time we iterate, how much do we shrink everything?

points = [np.array((0.5 * dimension,0.5 * dimension))]
scale, generation, generations =  starting_feature, 0, {0: {0}}
scales = [scale]

while True:
    # Calculate the new feature size, check if it's too small to make
    scale *= exponent
    if 2 * scale < trace:
        break
    scales.append(scale)
    # Compute the new points
    old_count = len(points)
    pdsample(dimension, dimension, scale / fraction, points, criteria = inside_box(dimension, scale))
    # Record them in generation log
    generation += 1
    generations[generation] =set(range(old_count, len(points)))

points = np.array(points)
scales = np.array(scales)


# In[5]:


for i in range(len(generations) - 1):
    # Build a k-d of this generation...
    idx = list(generations[i])
    tree = KDTree(points[idx])
    #...then filter all the other generations with it with a given radius.
    effective_radius = max(keepout * scales[i], space + scales[i])
    
    for j in range(i+1, len(generations)):
        generations[j] = {k for k in generations[j] if tree.query_ball_point(points[k],effective_radius + scales[j]) == []}


# In[6]:


for _,g in generations.items():
    l = list(g)
    plt.scatter(points[l,0],points[l,1])


# In[7]:


doc = ezdxf.new(dxfversion="R2010")
msp = doc.modelspace()

for i,v in generations.items():
    for k in v:
        msp.add_circle((points[k,0],points[k,1]), radius = scales[i])
        
doc.saveas("test_small.dxf")


# In[ ]:





# In[ ]:





# In[8]:


@dataclass
class Quote:
    s : str
        
def format_circle(center, radius, error, layer):
    x,y = center
    n = max(20,math.ceil(math.pi / math.acos(abs(radius - error) / radius)))
    yield 'fp_poly'
    yield ['pts'] + list(['xy', x + radius * math.cos(t), y + radius * math.sin(t)] for t in np.linspace(0, 2 * math.pi, n+1)) 
    yield ['layer', layer]
    yield ['width', 0]


# In[9]:


layer = 'F.SilkS'
module_name = 'starfield-calibrator-front'
circles = []
n = 0

for i,v in generations.items():
    for k in v:
        circles.append(format_circle(points[k],scales[i], 0.001, layer))
        

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
    
        
header = ["module", module_name, ["layer", layer],["tedit", hex(int(time.time()))[2:]],
         ["attr", "virtual"],
         ["descr", Quote("Starfield calibrator target")],
         ["tags", "starfield"]]
    
expr = header + circles
    
with open(module_name + ".kicad_mod", "w") as f:
    write_sexp(expr,f)


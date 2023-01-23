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
matplotlib.rcParams['figure.figsize'] = [14.0,14.0]
import inspect
import ezdxf
from dataclasses import dataclass
import time
from hhg import HHG


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


get_ipython().run_cell_magic('time', '', "dimension = 25 # Overall size that we should fill with the pattern\ntrace = 0.003 * 25.4 # Minimum feature size we can fabricate\nspace = 0.003 * 25.4 # Minimum distance between features we can fabricate\nfraction = 0.25      # Scale between poisson disc spacing and circle radius\nkeepout = 1.1\nstarting_feature = 1 # The radius of the largest disc in the pattern\nexponent = 0.75 # Each time we iterate, how much do we shrink everything?\n\npoints = [np.array((0.5 * dimension,0.5 * dimension))]\nscale, generation, generations =  starting_feature, 0, {0: {0}}\nscales = [scale]\n\nwhile True:\n    # Calculate the new feature size, check if it's too small to make\n    scale *= exponent\n    if 2 * scale < trace:\n        break\n    scales.append(scale)\n    # Compute the new points\n    old_count = len(points)\n    pdsample(dimension, dimension, scale / fraction, points, criteria = inside_box(dimension, scale))\n    # Record them in generation log\n    generation += 1\n    generations[generation] =set(range(old_count, len(points)))\n\npoints = np.array(points)\nscales = np.array(scales)")


# In[ ]:





# In[ ]:





# In[ ]:





# In[7]:


get_ipython().run_cell_magic('time', '', '                \ndef candidates(old, new, dead):\n    for a,b in old.check_upwards(new):\n        if b in dead:\n            continue\n        yield a,b\n    for b,a in new.check_upwards(old,skip_identical = True):\n        if b in dead:\n            continue\n        yield a,b\n                \ndef check_collision(points,prev_radii, generation, radius):\n    def check(old, new):\n        delta = points[old] - points[new]\n        return delta.dot(delta) < (prev_radii[generation[old]] + radius)**2\n    return check\n\ngeneration = dict()\neffective_radii = np.maximum(keepout * scales, space + scales)\nbounding_box = np.min(points, axis = 0) - max(effective_radii), np.max(points, axis = 0) + max(effective_radii)\n#Build an empty hierarchical hash grid that we will then merge everything into.\nprev = HHG(bounding_box)\n\nfor i in range(len(generations)):\n    # Index this generation\n    new = HHG(bounding_box)\n    r = effective_radii[i]\n    for j in generations[i]:\n        new.insert_bbox(points[j] - r, points[j] + r, j)\n    # Then filter this generation against the previous ones\n    dead = set()\n    check = check_collision(points, effective_radii, generation, scales[i])\n\n    for old_element, new_element in candidates(prev,new,dead):\n        if check(old_element, new_element):\n            dead.add(new_element)\n    # Add this back in, and then set the generation for the surviving points\n    prev.merge(new, filter = lambda x: x not in dead)\n    for p in generations[i]:\n        if p not in dead:\n            generation[p] = i')


# In[ ]:





# In[8]:


doc = ezdxf.new(dxfversion="R2010")
msp = doc.modelspace()

for _,grid in prev.grids.items():
    for _,v in grid.items():
        for k in v:
            msp.add_circle((points[k,0],points[k,1]), radius = scales[generation[k]])
        
doc.saveas("test_small.dxf")


# In[ ]:





# In[ ]:





# In[ ]:


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


# In[ ]:


layer = 'F.SilkS'
module_name = 'starfield-calibrator-front'
circles = []
n = 0
        
for _,grid in prev.grids.items():
    for _,v in grid.items():
        for k in v:
            circles.append(format_circle(points[k],scales[generation[k]], 0.001, layer))

        
        

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


# In[ ]:





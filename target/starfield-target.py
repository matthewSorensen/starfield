import numpy as np
from numpy.random import rand
import math
from collections import defaultdict
from hhg import HHG
from itertools import chain
import click

from kicad_output import write_squares

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


def inside_box(dimension, radius):
    def criteria(pt):
        x,y = pt
        
        if x < radius or y < radius:
            return False
        
        if x > (dimension - radius) or y > (dimension - radius):
            return False
        
        return True
    return criteria

def multiscale_poisson(dimension, starting_radius, ending_radius, fraction = 0.25, iterations = 10):
    points = [np.array((0.5 * dimension,0.5 * dimension))]
    radii = np.exp(np.linspace(math.log(starting_radius), math.log(ending_radius), iterations))
    generations = {0: {0}}
    for generation,r in enumerate(radii):
        old_count = len(points)
        pdsample(dimension, dimension, r / fraction, points, criteria = inside_box(dimension, r))
        generations[generation] = set(range(old_count, len(points)))

    
    return radii, np.array(points), generations


def prune_circles(centers, radii, generations, keepout, clearance):
    effective_radii = np.maximum(keepout * radii, clearance + radii)
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


@click.command()
@click.option('--size', default = 25.4, type = float, help = "Target size in mm")
@click.option('--trace', default = 0.006 * 25.4, type = float, help = "Smallest positive feature to create.")
@click.option('--space', default = 0.006 * 25.4, type = float, help = "Smallest distance between features to create.")
@click.option('--initial-size', default = 1, type = float, help = "Largest feature to create.")
@click.option('--layer', default = "B.Cu", type = str, help = "KiCAD layer for features.")
@click.option('--module', default = "starfield-target", type = str, help = "KiCAD module name - also determines output path.")

def write_target(*args,**kwargs):

    keepout= 1.1
    
    radii, points, generations = multiscale_poisson(kwargs['size'], kwargs['initial_size'], 0.5 * kwargs['trace'] , iterations = 10)
    circles = prune_circles(points, radii, generations, keepout, kwargs['space'])
    squares = ((points[idx], radii[gen]) for idx, gen in circles.elements())
    write_squares(squares, kwargs['layer'], kwargs['module'], kwargs['module'] + '.kicad_mod')

if __name__ == '__main__':
    write_target()


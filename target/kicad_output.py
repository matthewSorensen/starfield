import inspect
import ezdxf
from dataclasses import dataclass
import time
import math
import numpy as np

@dataclass
class Quote:
    s : str
        
def write_sexp(sexp,f):
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

def square(center, radius):
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



def write_squares(squares, layer, module_name, path):

    objects = (kicad_polygon(layer, square(*s)) for s in squares)

    with open(path, "w") as f:
        write_sexp(kicad_module(module_name, [layer], objects),f)

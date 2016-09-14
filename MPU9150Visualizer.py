#!/usr/bin/python

import pygame
import urllib
from OpenGL.GL import *
from OpenGL.GLU import *
from math import radians
from pygame.locals import *
import serial
from math import *
import numpy
import math

SCREEN_SIZE = (800, 600)
SCALAR = .5
SCALAR2 = 0.2

port = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=3.0)

def unit_vector(data, axis=None, out=None):
    """Return ndarray normalized by length, i.e. Euclidean norm, along axis.

    >>> v0 = numpy.random.random(3)
    >>> v1 = unit_vector(v0)
    >>> numpy.allclose(v1, v0 / numpy.linalg.norm(v0))
    True
    >>> v0 = numpy.random.rand(5, 4, 3)
    >>> v1 = unit_vector(v0, axis=-1)
    >>> v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0*v0, axis=2)), 2)
    >>> numpy.allclose(v1, v2)
    True
    >>> v1 = unit_vector(v0, axis=1)
    >>> v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0*v0, axis=1)), 1)
    >>> numpy.allclose(v1, v2)
    True
    >>> v1 = numpy.empty((5, 4, 3))
    >>> unit_vector(v0, axis=1, out=v1)
    >>> numpy.allclose(v1, v2)
    True
    >>> list(unit_vector([]))
    []
    >>> list(unit_vector([1]))
    [1.0]

    """
    if out is None:
        data = numpy.array(data, dtype=numpy.float64, copy=True)
        if data.ndim == 1:
            data /= math.sqrt(numpy.dot(data, data))
            return data
    else:
        if out is not data:
            out[:] = numpy.array(data, copy=False)
        data = out
    length = numpy.atleast_1d(numpy.sum(data*data, axis))
    numpy.sqrt(length, length)
    if axis is not None:
        length = numpy.expand_dims(length, axis)
    data /= length
    if out is None:
        return data

def resize(width, height):
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(width) / height, 0.001, 10.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(0.0, 1.0, -5.0,
              0.0, 0.0, 0.0,
              0.0, 1.0, 0.0)

def init():
    glEnable(GL_DEPTH_TEST)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glShadeModel(GL_SMOOTH)
    glEnable(GL_BLEND)
    glEnable(GL_POLYGON_SMOOTH)
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST)
    glEnable(GL_COLOR_MATERIAL)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glLightfv(GL_LIGHT0, GL_AMBIENT, (0.3, 0.3, 0.3, 1.0));

def readlineCR(port):
    rv = ""
    i = 0
    while True:
        ch = port.read()
        i = i + 1
        print i
        rv += ch
        if (i > 50):
            ch = ''
            break;
        if ch=='\n':
            i = 0
            return rv

def read_values():
    port.write("w")
    tmp = readlineCR(port)
    #print (tmp)
    return tmp.split(" ")
    #return "0 0 45".split(" ")

def q_to_mat4(q):
    w, x, y, z = q
    return numpy.array(
        [[1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w, 0],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w, 0],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y, 0],
        [0, 0, 0, 1] ],'f')

def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v

def quaternion_conjugate(quaternion):
    """Return conjugate of quaternion.

    >>> q0 = random_quaternion()
    >>> q1 = quaternion_conjugate(q0)
    >>> q1[0] == q0[0] and all(q1[1:] == -q0[1:])
    True

    """
    q = numpy.array(quaternion, dtype=numpy.float64, copy=True)
    q[1] = -q[1]
    q[2] = -q[2]
    q[3] = -q[3]

    return q

def quaternion_multiply(q1, q2):
    ''' Multiply two quaternions

    Parameters
    ----------
    q1 : 4 element sequence
    q2 : 4 element sequence

    Returns
    -------
    q12 : shape (4,) array

    Notes
    -----
    See : http://en.wikipedia.org/wiki/Quaternions#Hamilton_product
    '''
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 + y1*w2 + z1*x2 - x1*z2
    z = w1*z2 + z1*w2 + x1*y2 - y1*x2
    return numpy.array([w, x, y, z])

def qv_mult(q1, v1):
    #v1 = unit_vector(v1)
    q2 = [0, v1[1], v1[2], v1[3]]
    return quaternion_multiply(
        quaternion_multiply(q1, q2),
        quaternion_conjugate(q1)
    )[1:]

def run():
    pygame.init()
    screen = pygame.display.set_mode(SCREEN_SIZE, HWSURFACE | OPENGL | DOUBLEBUF)
    resize(*SCREEN_SIZE)
    init()
    clock = pygame.time.Clock()
    cube = Cube((0.0, 0.0, 0.0), (.5, .5, .7))
    angle = 0

    while True:
        then = pygame.time.get_ticks()
        for event in pygame.event.get():
            if event.type == QUIT:
                return
            if event.type == KEYUP and event.key == K_ESCAPE:
                return

        values = read_values()
        q = (float(values[0]), float(values[2]), float(values[1]), float(values[3]))

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glColor((1.,1.,1.))
        glLineWidth(1)
        glBegin(GL_LINES)

        for x in range(-20, 22, 2):
            glVertex3f(x/10.,-1,-1)
            glVertex3f(x/10.,-1,1)

        for x in range(-20, 22, 2):
            glVertex3f(x/10.,-1, 1)
            glVertex3f(x/10., 1, 1)

        for z in range(-10, 12, 2):
            glVertex3f(-2, -1, z/10.)
            glVertex3f( 2, -1, z/10.)

        for z in range(-10, 12, 2):
            glVertex3f(-2, -1, z/10.)
            glVertex3f(-2,  1, z/10.)

        for z in range(-10, 12, 2):
            glVertex3f( 2, -1, z/10.)
            glVertex3f( 2,  1, z/10.)

        for y in range(-10, 12, 2):
            glVertex3f(-2, y/10., 1)
            glVertex3f( 2, y/10., 1)

        for y in range(-10, 12, 2):
            glVertex3f(-2, y/10., 1)
            glVertex3f(-2, y/10., -1)

        for y in range(-10, 12, 2):
            glVertex3f(2, y/10., 1)
            glVertex3f(2, y/10., -1)

        glEnd()
        glPushMatrix()

        glRotate(90, 1, 0, 0)
        glRotate(210, 0, 0, 1)

        glMultMatrixf(q_to_mat4(q))
        pos = cube.apply_q(q)

        cube.render()

        glPopMatrix()

        glPushMatrix()

        glRotate(90, 1, 0, 0)
        glRotate(210, 0, 0, 1)

        glColor((1.,0.,1.))
        glBegin(GL_LINES)
        glVertex3fv((0,0,0))

        #print pos
        glVertex3fv((pos[0], pos[1], pos[2]))
        glEnd()

        glPopMatrix()

        pygame.display.flip()
        pygame.time.wait(10)

def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
  return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

class Cube(object):

    def __init__(self, position, color):
        self.position = position
        self.color = color

    # Cube information
    num_faces = 6

    origin = [ 0.0, 0.0, 1.0 ]
    direction = [ 0.0, -1.0, 0.0, 1.0 ]

    vertices = [ (-1.0, -0.05, 0.5),
                 (1.0, -0.05, 0.5),
                 (1.0, 0.05, 0.5),
                 (-1.0, 0.05, 0.5),
                 (-1.0, -0.05, -0.5),
                 (1.0, -0.05, -0.5),
                 (1.0, 0.05, -0.5),
                 (-1.0, 0.05, -0.5) ]

    normals = [ (0.0, 0.0, +1.0),  # front
                (0.0, 0.0, -1.0),  # back
                (+1.0, 0.0, 0.0),  # right
                (-1.0, 0.0, 0.0),  # left
                (0.0, +1.0, 0.0),  # top
                (0.0, -1.0, 0.0) ]  # bottom

    vertex_indices = [ (0, 1, 2, 3),  # front
                       (4, 5, 6, 7),  # back
                       (1, 5, 6, 2),  # right
                       (0, 4, 7, 3),  # left
                       (3, 2, 6, 7),  # top
                       (0, 1, 5, 4) ]  # bottom

    def apply_q(self, q):

        r11 = 2 * q[0] * q[0] - 1 + 2 * q[1] * q[1];
        r12 = 2 * (q[1] * q[2] + q[0] * q[3]);
        r13 = 2 * (q[1] * q[3] - q[0] * q[2]);
        r21 = 2 * (q[1] * q[2] - q[0] * q[3]);
        r22 = 2 * q[0] * q[0] - 1 + 2 * q[2] * q[2];
        r23 = 2 * (q[2] * q[3] + q[0] * q[1]);
        r31 = 2 * (q[1] * q[3] + q[0] * q[2]);
        r32 = 2 * (q[2] * q[3] - q[0] * q[1]);
        r33 = 2 * q[0] * q[0] - 1 + 2 * q[3] * q[3];

        currentModelView = [[r11, r12, r13],
                            [r21, r22, r23],
                            [r31, r32, r33]]

        mx = numpy.matrix(currentModelView)
        my = numpy.matrix(self.direction[0:3])
        tmp2 = numpy.matmul( mx , numpy.transpose(my))

        return tmp2[0:3]

    def render(self):
        then = pygame.time.get_ticks()
        glColor(self.color)

        vertices = self.vertices

        # Draw all 6 faces of the cube
        glBegin(GL_QUADS)

        for face_no in xrange(self.num_faces):
            glNormal3dv(self.normals[face_no])
            v1, v2, v3, v4 = self.vertex_indices[face_no]
            glVertex(vertices[v1])
            glVertex(vertices[v2])
            glVertex(vertices[v3])
            glVertex(vertices[v4])
        glEnd()

if __name__ == "__main__":
    run()

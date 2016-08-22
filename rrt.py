#/usr/local/bin
## 
## Implementation of RT-RRT* algorithm
##
## Original paper presented by Kourosh Naderi * Joose Rajamaki * Perttu Hamalainen
##
## @author Aswathi Radhakrishnan (UNCC ID: 800936796)
##

from math import sqrt, atan2, cos, sin, pi
from Node import Node
from Object import Object
import sys, pygame
import random

BROWN =   60,  60,  60
WHITE =  195, 195, 195
GREEN =   20, 255,  20 
RED   =  255, 120, 134
PEACH =  255, 218, 185

INFINITY    = float("inf")
EPSILON     = 10
MAX_NODES   = 1000
RADIUS      = 15 #log(MAX_NODES)/MAX_NODES
XDIM        = 500
YDIM        = 480

speed       = [1,1]                             # translates object by (x,y) pixels
robotRadius = 50

qinit = Node(50,20)
qgoal = Node(400,300)

nodes = []
edges = []
path  = []

size   = XDIM, YDIM
screen = pygame.display.set_mode(size)

background = pygame.Surface(screen.get_size())
background = background.convert()
background.fill(BROWN)
screen.fill(BROWN)

objects = []
moving_objects = []
image = pygame.image.load('round.png').convert_alpha()


def display():
    fname = "obstacles.txt"    
    with open(fname) as f:                                                  
        for line in f.readlines():                              # read obstacles' position from file                            
            content = line.split(" ")
            string = [int(i) for i in content]
            button = Object(image, string[0], string[1], string[2], string[3])
            objects.append(button)
            screen.blit(image, [button.x,button.y])             # display obstacles on screen
             
#
# distance
# @param q1,q2: two points in C-free
# returns: distance between q1 and q2
#  
def distance(q1,q2):
    return sqrt(pow(q2.x - q1.x, 2) + pow(q2.y - q1.y, 2))      # finds distance between two points

#
# getNearest
# @param qrand: random node in C-free
# returns: node in G nearest to the random node
#
def getNearest(qrand):
    qnearest = qinit
    for node in nodes:
        if distance(node,qrand) < distance(qnearest,qrand):	
            qnearest = node
    return qnearest

#
# getNew
# @param qnearest,qrand
# returns: qnew - a new node in C-free steered towards qrand at a distance EPSILON = 10 from qnearest
#
def getNew(qnearest, qrand, EPSILON = 10):	
    if distance(qnearest, qrand) < EPSILON:						
        qnew = qrand													
    else:
        theta = atan2(qrand.y - qnearest.y, qrand.x - qnearest.x)				# computes angle towards qrand 
        qnew = Node(qnearest.x + EPSILON * cos(theta), qnearest.y + EPSILON * sin(theta))	# find a new node in the direction of qrand
    return qnew

#
# cost
# @param a node q
# returns: shortest distance of node q from start node
#
def cost(q):
    return q.distance

#
# isInCFree: detects collision between points
# @param a point q2
# returns: true if q2 is in C-free
#	   false otherwise
#
def isInCFree(q2):
    for obj in objects:
        try:
            if (q2.x > obj.x) and (q2.x + robotRadius < (obj.x + obj.width + robotRadius)) and (q2.y > obj.y) and (q2.y + robotRadius < (obj.y + obj.height + robotRadius)):
                return True
        except AttributeError:
            if (q2[0] > obj.x) and (q2[0] + robotRadius < (obj.x + obj.width + robotRadius)) and (q2[1] > obj.y) and (q2[1] + robotRadius < (obj.y + obj.height + robotRadius)):
                return True
    return False

#
# drawSolutionPath
# @param start node, end node, color of path
#
def drawSolutionPath(qinit,color=None):
    if color == None:
        color = RED
    node = qgoal  
    parent = qgoal.predecessor
    if parent == None:
        sys.exit("Unable to find a path to goal")
    else:
        while parent != qinit:
            path.insert(0, parent)
            try:
                pygame.draw.line(screen,color,[node.x,node.y],[parent.x,parent.y],3)  
                pygame.display.update()
                node = parent
                parent = node.predecessor
            except:
                pass
            
#
# reDrawSolutionPath: recomputes solution path when an obstacle is found in the original path computed using RRT*
# @param start node, end node
#
def reDrawSolutionPath(qinit,qgoal):
    parent = qinit
    for node in path:								
        if isInCFree(node):							# when an obstacle is found in the path computed
            path.remove(node)							# 	remove the node at which the obstacle resides from path
            node.predecessor = None						# 	clear all data related to the node
            parent.adj.remove(node)
            qinit = node
            xmin = round(abs(node.x - 100))					# 	compute a new boundary for choosing qrand in C-free
            ymin = round(abs(node.y - 100))					# 	around the obstacle
            xmax = round(node.x + 100)
            ymax = round(node.y + 100)
            rrt(qinit, xmin, ymin, xmax, ymax)					# 	expands RRT tree within the specified boundary
            drawSolutionPath(qinit, GREEN)        			# 	draw new solution path
        else:									# else if no collision
            pygame.draw.line(screen,RED,[parent.x,parent.y],[node.x,node.y],3)  #	draw solution path from start to end
            pygame.display.update()
            parent = node
                    
#
# getPoints
# @param count
# returns: a set of points within a radius MAX_R
#
def get_points(count):
    points = []
    MAX_R = 50
    for i in range(count):
        theta = 2 * pi
        #radius = MAX_R * sqrt(random.random())
        x = MAX_R * cos(theta)							# find x,y coordinates of new points within max radius MAX_R
        y = MAX_R * sin(theta)
        points.append((x,y))
    return points

#
# rrt : implementation of RRT
# @param start node, dimension of the screen - XMIN, YMIN, XMAX, YMAX
# returns: a collision free path from start node to end node
#
def rrt(qinit = None, XMIN = 0, YMIN = 0, XMAX = 0, YMAX = 0):
    if XMAX == 0 or YMAX == 0:
        XMAX = size[0]
        YMAX = size[1]
        
    ball = pygame.image.load("round.png")
    ball.set_colorkey((255, 255, 255))
    ballrect = ball.get_rect()
    clearsurf = background.subsurface(ballrect)
    
    for i in range(MAX_NODES):
        #print i
        
        x = ballrect.centerx
        y = ballrect.centery
        button = Object(ball, x, y, 25, 25)
        moving_objects.append(button)
        screen.blit(clearsurf, ballrect)
        ballrect = ballrect.move(speed)
        ballrect.left = 250
        if ballrect.bottom > YDIM or ballrect.top < 0:
            speed[1] = -speed[1]
        screen.blit(ball, ballrect)
        pygame.display.flip()
        pygame.time.delay(10)
            
        
        qrand = Node(random.randrange(XMIN,XMAX), random.randrange(YMIN,YMAX))
        qnearest = getNearest(qrand)
        qnew = getNew(qnearest, qrand)
        if not isInCFree(qnew):
            nodes.append(qnew)          
            Xnear = []
            for node in nodes:
                if distance(qnew, node) < RADIUS and node != qnearest and node != qnew:
                    Xnear.append(node)
            qmin = qnearest
            cmin = cost(qnearest) + distance(qnearest, qnew)
            for qnear in Xnear:
                if not isInCFree(qnew) and cost(qnear)+distance(qnear,qnew) < cmin:
                    qmin = qnear
                    cmin = cost(qnear) + distance(qnear,qnew)
            edges.append((qmin,qnew))
            qmin.adj.append(qnew)
            qnew.predecessor = qmin
            qnew.distance = cmin
            if distance(qnew,qgoal) < 10:
                edges.append((qmin,qgoal))
                print "Reached Goal"
                qgoal.predecessor = qmin
                qmin.adj.append(qgoal)
            qparent = qmin
            for qnear in Xnear:
                if not isInCFree(qnear) and cost(qnew)+distance(qnew,qnear) < cost(qnear):
                    qparent = qnear.predecessor
                    try:
                        index = edges.index((qparent,qnear))
                        del edges[index]
                    except ValueError:
                        pass
                    edges.append((qnew,qnear))
                    qnear.predecessor = qnew
                    qnew.adj.append(qnear)
                    qnear.distance = cost(qnew) + distance(qnew,qnear)
            pygame.draw.aaline(screen,WHITE,[qnearest.x,qnearest.y],[qnew.x,qnew.y],1)
            pygame.display.update()

            
        for e in pygame.event.get():
            if e.type == pygame.QUIT or (e.type == pygame.KEYUP and e.key == pygame.K_ESCAPE):
                sys.exit("Bye!")
        
def main():
    pygame.init()
    pygame.display.set_caption('RT-RRT*')
    nodes.append(qinit)
    display()
    rrt()

     
if __name__ == '__main__': 
    main()
    pygame.display.update()   
    drawSolutionPath(qinit)
    pygame.time.delay(100)

    x = path[10].x
    y = path[10].y
    button1 = Object(image, x-20, y-20, 25, 25)
    objects.append(button1)
    screen.blit(image, [button1.x, button1.y])
    pygame.display.update()

    reDrawSolutionPath(qinit, qgoal)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
   
    

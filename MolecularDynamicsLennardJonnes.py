import pygame
import random
import numpy as np
import matplotlib.pyplot as plt
import math

background_colour = (255,255,255)
width, height = 500, 500
aafac = 2 # anti-aliasing factor screen to off-screen image

number_of_particles = 20
my_particles = []

sigma = 10
sigma2 = sigma**2
e = 5
dt = 1 # simulation time interval between frames
timesteps = 10 # intermediate invisible steps of length dt/timesteps
def r(p1,p2):
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    angle = 0.5 * math.pi - math.atan2(dy, dx)
    dist = np.hypot(dx, dy)
    return dist

def collide(p1, p2):
    dx = p1.x - p2.x
    dy = p1.y - p2.y

    dist = np.hypot(dx, dy)
    if dist < (p1.size + p2.size):
        tan = math.atan2(dy, dx)
        angle = 0.5 * np.pi + tan

        angle1 = 2*tan - p1.angle
        angle2 = 2*tan - p2.angle
        speed1 = p2.speed
        speed2 = p1.speed
        (p1.angle, p1.speed) = (angle1, speed1)
        (p2.angle, p2.speed) = (angle2, speed2)

        overlap = 0.5*(p1.size + p2.size - dist+1)
        p1.x += np.sin(angle) * overlap
        p1.y -= np.cos(angle) * overlap
        p2.x -= np.sin(angle) * overlap
        p2.y += np.cos(angle) * overlap

def LJ_force(p1,p2):
    rx = p1.x - p2.x
    ry = p1.y - p2.y

    r2 = rx**2 + ry**2

    r2s = r2/sigma2+1
    r6s = r2s*r2s*r2s
    f = 24*e*( 2/(r6s**2) - 1/(r6s) )

    p1.ax += f*(rx/r2)
    p1.ay += f*(ry/r2)
    p2.ax -= f*(rx/r2)
    p2.ay -= f*(ry/r2)

def Verlet_step(particles, h):
    for p in particles:
        p.verlet1_update_vx(h);
        p.bounce()
    #t += h;
    for i, p1 in enumerate(particles):
        for p2 in particles[i+1:]:
            LJ_force(p1,p2);
            collide(p1,p2)
    for p in particles:
        p.verlet2_update_v(h);

class Particle():
    def __init__(self, x,y, vx,vy, size):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.size = size
        self.colour = (0, 0, 255)
        self.thickness = 2
        self.ax = 0
        self.ay = 0
        self.speed = 0
        self.angle = 0

    def verlet1_update_vx(self,h):
        self.vx += self.ax*h/2
        self.vy += self.ay*h/2
        self.x += self.vx*h
        self.y += self.vy*h
        self.ax = 0
        self.ay = 0

    def verlet2_update_v(self,h):
        self.vx += self.ax*h/2
        self.vy += self.ay*h/2

    def display(self,screen, aa):
        pygame.draw.circle(screen, self.colour, (int(aa*self.x+0.5), int(aa*self.y+0.5)), aa*self.size, aa*self.thickness)

    def bounce(self):
        if self.x > width - self.size:
            self.x = 2*(width - self.size) - self.x
            self.vx = - self.vx
            self.angle = - self.angle

        elif self.x < self.size:
            self.x = 2*self.size - self.x
            self.vx = - self.vx
            self.angle = - self.angle

        if self.y > height - self.size:
            self.y = 2*(height - self.size) - self.y
            self.vy = - self.vy
            self.angle = np.pi - self.angle

        elif self.y < self.size:
            self.y = 2*self.size - self.y
            self.vy = - self.vy
            self.angle = np.pi - self.angle

#------------ end class particle ------------
#------------ start main program ------------

for n in range(number_of_particles):
    x = 1.0*random.randint(15, width-15)
    y = 1.0*random.randint(15, height-15)
    vx, vy = 0., 0.
    for k in range(6):
        vx += random.randint(-10, 10)/2.
        vy += random.randint(-10, 10)/2.

    particle = Particle(x,y,vx,vy, 10)

    my_particles.append(particle)

#--------- pygame event loop ----------
screen = pygame.display.set_mode((width, height))
offscreen = pygame.Surface((aafac*width, aafac*height))

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    offscreen.fill(background_colour)

    for k in range(timesteps):
        Verlet_step(my_particles, dt/timesteps)

    for particle in my_particles:
        particle.display(offscreen, aafac)

    pygame.transform.smoothscale(offscreen, (width,height), screen)
    pygame.display.flip()
pygame.quit()

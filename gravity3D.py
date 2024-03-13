## Kurt Hahn
## 23 November 2023
## PHYS 225 Introduction to Computational Physics and Programming
## Final Project - Gravity in 3D


import pygame
import math
import numpy as np


TITLE = "Gravity in 3D"

WINDOWWIDTH, WINDOWHEIGHT = 1200, 650
TIMEDELAY = 1 #milliseconds
DELTAT = 0.0001

ACCELERATIONFACTOR = 1e17
COLLISIONS = True
RESTITUTIONCOEFF = -0.999
EXPONENT = 2
RESETPATHS = True

WHITE = (255, 255, 255)
RED = (255, 0, 0)
ORANGE = (255, 127, 0)
YELLOW = (240, 240, 0)
GREEN = (0, 255, 0)
BLUE = (0, 100, 255)
PURPLE = (75, 0, 130)
BLACK = (0, 0, 0)
MAGENTA = (180, 29, 245)

CENTER = (WINDOWWIDTH // 2, WINDOWHEIGHT // 2)
YMARGIN = 50
XMARGIN = 50
ZPLANE = 10
VISUALEXPONENT = 0.5

G = 6.674e-11*ACCELERATIONFACTOR

class Vector():

    def magnitude(vector):
        
        result = 0

        for value in vector:
            result += value**2

        return math.sqrt(result)
    
    def scale(scale, vector):
        
        result = []

        for i, value in enumerate(vector):
            result.append(scale * vector[i])

        return result

    def add(v1, v2):

        result = []

        for i, value in enumerate(v1):
            result.append(v1[i] + v2[i])

        return result
    
    def subtract(v1, v2):

        result = []

        for i, value in enumerate(v1):
            result.append(v1[i] - v2[i])

        return result
    
    def dot_product(v1, v2):

        result = 0

        for i, value in enumerate(v1):
            result += v1[i] * v2[i]

        return result


class Ball():

    def __init__(self, window, pos, vel, MASS, RADIUS, COLOR):

        self.WINDOW= window
        self.COLOR = COLOR

        self.pos = pos

        self.vel = vel

        self.MASS = MASS
        self.RADIUS = RADIUS


class Simulation():

    def __init__(self, ballset):

        self.ballset = ballset
        self.zPlane = ZPLANE
        self.polarAngle = 0
        self.azimuthalAngle = 0

    def return_distance(self, ball1, ball2):

        sum = 0

        for i, value in enumerate(ball1.pos):
            sum += (value - ball2.pos[i])**2

        return math.sqrt(sum)
    
    def return_acceleration(self, ball1):

        acc = []

        for i, value in enumerate(ball1.pos):
            acc.append(0)

        for ball2 in self.ballset:

            if ball1 == ball2:
                continue

            radiusVec = Vector.subtract(ball2.pos, ball1.pos)
            radiusMag = Vector.magnitude(radiusVec)
            radiusHat = Vector.scale(1/radiusMag, radiusVec)

            accelerationMag = G * ball2.MASS / (radiusMag**EXPONENT)

            for i, value in enumerate(acc):
                acc[i] += accelerationMag * radiusHat[i]

        return acc

    def set_next_frame_velocities(self, ball):

        acc = self.return_acceleration(ball)

        for i, value in enumerate(ball.vel):

            ball.vel[i] += acc[i] * DELTAT

    def set_next_frame_positions(self, ball):

        for i, value in enumerate(ball.pos):

            ball.pos[i] += ball.vel[i] * DELTAT

    def check_for_collision(self, ball1, ball2):
        return self.return_distance(ball1, ball2) < (ball1.RADIUS + ball2.RADIUS)

    ## I pulled this off stack exchange physics
    ## https://physics.stackexchange.com/questions/681396/elastic-collision-3d-eqaution
    def set_velocities_after_collision(self, ball1, ball2):


        m1 = ball1.MASS
        m2 = ball2.MASS

        r = Vector.subtract(ball1.pos, ball2.pos)
        rHat = Vector.scale(1/Vector.magnitude(r), r)

        vRel = Vector.subtract(ball1.vel, ball2.vel)

      
        ball1dvel = Vector.scale(-(2*m2/(m1+m2))*Vector.dot_product(rHat, vRel), rHat)
        ball2dvel = Vector.scale((2*m1/(m1+m2))*Vector.dot_product(rHat, vRel), rHat)
        ball1.vel = Vector.add(ball1.vel, ball1dvel)
        ball2.vel = Vector.add(ball2.vel, ball2dvel)


        # contact_vector = Vector.subtract(ball2.pos, ball1.pos)
        # contact_vector_magnitude = Vector.magnitude(contact_vector)
        # contact_normal = Vector.scale(1/contact_vector_magnitude, contact_vector)
        
        # effective_mass = 1/(1/m1 + 1/m2)

        # impact_speed = Vector.dot_product(contact_normal, Vector.subtract(ball1.vel, ball2.vel))

        # impulse_magnitude = (1 + RESTITUTIONCOEFF) * effective_mass * impact_speed

        # ball1dvel = Vector.scale(-impulse_magnitude/m1, contact_normal)
        # ball2dvel = Vector.scale(impulse_magnitude/m2, contact_normal)

        # ball1.vel = Vector.add(ball1.vel, ball1dvel)
        # ball2.vel = Vector.add(ball2.vel, ball2dvel)      

    def set_positions_after_collision(self, ball1, ball2):
        
        distance = self.return_distance(ball1, ball2)

        overlap = (ball1.RADIUS + ball2.RADIUS) - distance

        angle = math.atan2(ball2.pos[1] - ball1.pos[1], ball2.pos[0] - ball1.pos[0])

        ball1.pos[0] -= (overlap / 2) * math.cos(angle)
        ball1.pos[1] -= (overlap / 2) * math.sin(angle)

        ball2.pos[0] += (overlap / 2) * math.cos(angle)
        ball2.pos[1] += (overlap / 2) * math.sin(angle)

    def draw_balls(self):

        ## Sort balls by farthest to closest
        for index1, ball1 in enumerate(self.ballset):

            for index2, ball2 in enumerate(self.ballset):

                if index1 >= index2:
                    continue

                if ball2.pos[2] < ball1.pos[2]:
                    store = ball1
                    self.ballset[index1] = ball2
                    self.ballset[index2] = store

        # for ball in self.ballset:

        #     r = math.sqrt(ball.pos[0]**2 + ball.pos[1]**2 + ball.pos[2]**2)

        #     zPosImage = r * math.cos(self.azimuthalAngle) * math.cos(self.polarAngle)
        #     print(self.zPlane - zPosImage)

        #     if self.zPlane - zPosImage <= 0:
        #         continue

        #     xPosImage = r * math.cos(self.azimuthalAngle) * math.sin(self.polarAngle)
        #     yPosImage = r * math.sin(self.azimuthalAngle)

        #     radiusImage = ball.RADIUS / ((self.zPlane - zPosImage)**VISUALEXPONENT)

        #     print(radiusImage)

        #     pygame.draw.circle(ball.WINDOW, ball.COLOR, (xPosImage, WINDOWHEIGHT - yPosImage), radiusImage)

        ## Draw farthest first
        ## Drawing with depth perseption
        for ball in self.ballset:

            ## This is an approximation for the effective radius at a distance from z=0
            imageRadius = ball.RADIUS / ((self.zPlane - ball.pos[2])**VISUALEXPONENT)

            imageX = ball.pos[0]
            imageY = WINDOWHEIGHT - ball.pos[1]

            pygame.draw.circle(ball.WINDOW, ball.COLOR, (imageX, imageY), imageRadius)

    def evolve(self):
             
        for index1, ball1 in enumerate(self.ballset):

            ## Collision mechanics
            if COLLISIONS:
                for index2, ball2 in enumerate(self.ballset):
                    if index1 <= index2:
                        continue

                    if not self.check_for_collision(ball1, ball2):
                        continue

                    self.set_velocities_after_collision(ball1, ball2)
                    
            ## Gravity Mechanics
            self.set_next_frame_velocities(ball1)

            for ball in self.ballset:
                self.set_next_frame_positions(ball)

            if COLLISIONS:
                ## Rectify overlaps
                for index2, ball2 in enumerate(self.ballset):
                    if index1 <= index2:
                        continue

                    if not self.check_for_collision(ball1, ball2):
                        continue

                    self.set_positions_after_collision(ball1, ball2)

            self.draw_balls()

class Main():

    def main():

        pygame.init()

        WINDOW = pygame.display.set_mode((WINDOWWIDTH, WINDOWHEIGHT))
        pygame.display.set_caption(TITLE)

        icon = pygame.image.load('image1.png')
        pygame.display.set_icon(icon)

        WINDOW.fill(BLACK)

        ball1 = Ball(WINDOW, [600, 500, -50], [-20, 20, 200], 5, 25, RED)
        ball2 = Ball(WINDOW, [500, 400, -50], [20, -20, -200], 5, 25, BLUE)

        ball3 = Ball(WINDOW, [600, 325, -500], [0, 0, 50], 5, 20, RED)

        ballset = [ball1, ball2]
        # ballset.append(ball3)
    

        simulation = Simulation(ballset)

        run = True
        while run:
            pygame.time.delay(TIMEDELAY)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    run = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_p:
                        simulation.zPlane -= 1
                    if event.key == pygame.K_m:
                        simulation.zPlane += 1
                    if event.key == pygame.K_LEFT:
                        simulation.polarAngle -= 0.01
                    if event.key == pygame.K_RIGHT:
                        simulation.polarAngle += 0.01
                    if event.key == pygame.K_UP:
                        simulation.azimuthalAngle -= 0.01
                    if event.key == pygame.K_DOWN:
                        simulation.azimuthalAngle += 0.01

            if RESETPATHS: WINDOW.fill(BLACK)

            simulation.evolve()


            pygame.display.update()

        pygame.quit()


if __name__ == "__main__":
    Main.main()
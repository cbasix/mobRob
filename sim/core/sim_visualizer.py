import os
import sys
import time
import tkinter as tk
from tkinter import filedialog
import dill
import pygame

from sim.core.base import Simulation
from sim.core.obstacles import Obstacle
from sim.core.vector import Vector2D

black = 0, 0, 0
red = 255, 0, 0
green = 0, 155, 0
grey = 150, 150, 150
white = 255, 255, 255

NEW_OBSTACLE_SIZE = 10


# all angles are in radians !
def loop(simulation):

    # pygame stuff
    zoom = 2  # only 1, 2 or 4
    screen = pygame.display.set_mode(simulation.size*zoom)
    pause = True

    root = tk.Tk()
    root.withdraw()

    try:
        while True:
            if not pause:
                simulation.step()

            screen.fill(black)

            for thing in simulation.things:
                try:
                    type = thing.type
                except AttributeError:
                    continue

                if type == "robot":
                    pygame.draw.circle(screen, red, thing.position.data.astype(int)*zoom, thing.radius*zoom, 2*zoom)
                    pygame.draw.line(
                        screen,
                        green,
                        thing.position.data.astype(int) * zoom,
                        (thing.position.data + Vector2D(thing.radius, 0).get_rotated(thing.angle)).astype(int) * zoom
                    )

                    for sensor in thing.sensors:
                        pygame.draw.circle(screen, white, sensor.get_position().astype(int)*zoom, 1*zoom, 1)

                elif type == "field_source":
                    pygame.draw.circle(screen, green, thing.position.data.astype(int)*zoom, thing.radius*zoom, 2*zoom)

                elif type == "obstacle":
                    pygame.draw.circle(screen, grey, thing.position.data.astype(int) * zoom, thing.radius * zoom, 2 * zoom)

            pygame.display.flip()

            time.sleep(0.01)

            for event in pygame.event.get():
                if event.type == pygame.QUIT: sys.exit()

                if event.type == pygame.MOUSEBUTTONDOWN:
                    pos = pygame.mouse.get_pos()
                    simulation.things.append(
                        Obstacle(radius=NEW_OBSTACLE_SIZE, position=Vector2D(pos[0] / zoom, pos[1] / zoom)))

                if event.type == pygame.KEYUP and event.key == pygame.K_p:
                    pause = not pause

                if event.type == pygame.KEYUP and event.key == pygame.K_s:
                    # save simulation
                    simulation.save()

                if event.type == pygame.KEYUP and event.key == pygame.K_l:
                    # load simulation from selectable file
                    simulation = Simulation.loadFrom(None)

    except KeyboardInterrupt:
        print("Stopped")





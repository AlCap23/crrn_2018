#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This file contains the main class of a pybullet server.

Copyright:

    Julius Martensen, University of Bremen, 2018
"""

import pybullet
import numpy
import threading
import logging
import pandas
from time import sleep

# Define a simple logger

logging.basicConfig(level=logging.DEBUG)


class simulationServer(object):
    """A pybullet physic client able to directly import robots, fmus and other.
    
    Arguments:
        mode, string : Either GUI or other. Starts the server with or without a needed GUI.

    Returns:
        None
    """

    def __init__(self, mode="GUI", shader = 'OPENGL3'):
        # Init of the physics client:
        if mode == "GUI":
            if shader == 'OPENGL3':
                self.client_id = pybullet.connect(pybullet.GUI)
            else:
                self.client_id = pybullet.connect(pybullet.GUI, options = "--opengl2")
            self.mode = True
        else:
            self.mode = False
            # Try to connect to a shared memory server
            self.client_id = pybullet.connect(pybullet.SHARED_MEMORY)
            if self.client_id < 0:
                # Make a new, non-visual server
                self.client_id = pybullet.connect(pybullet.DIRECT)

        self._initialize_()

        # Start the simulation thread
        self.thread_running = True
        self.simulation_running = False
        self.simulation_thread = threading.Thread(target=self.simulate)
        self.simulation_thread.start()
        self.lock = threading.Lock()

    def reset(self, loadState=False, state_id=None, filename=None):
        """Reset the bullet physic client.
        """

        if self.simulation_running:
            self.stopSimulation()

        while self.simulation_running:
            sleep(1e-3)

        self._initialize_()

        if loadState:
            self.loadStatus(state_id, filename)
        else:
            pybullet.rese

    def _initialize_(self):

        # Set the gravity
        self.setGravity(numpy.array([0, 0, 0]))

        # Current relative time
        self.time = 0

        # Get the time step
        self.setStepsize(pybullet.getPhysicsEngineParameters()["fixedTimeStep"])

        # Set the visualization time
        self.sleep_time = 1e-3

        # Store the simulation data
        self.measurements = pandas.DataFrame()
        self._measurements = pandas.DataFrame()

    def saveStatus(self, save_state=False, on_disk=False, filename=None):
        """Save the current status of the simulation. 
        
        Either stores 
        - an initial configuration (save_state = False ,on_disk = False) with a sufficient filename
        - current state in memory (save_state = True, on_disk = False)
        - current state on disk (save_state = True, on_disk = True) with a sufficient filename

        Is a wrapper for pybullet.saveWorld, saveState, saveBullet

        Returns:
            state_id, filename
        """

        if save_state and on_disk:
            if not filename:
                print("Supply a valid filename!")
                return
            pybullet.saveBullet(filename)
            return filename

        if save_state and not on_disk:
            if not hasattr(self, "stored_states"):
                self.stored_states = []

            self.stored_states.append(pybullet.saveState())
            return self.stored_states[-1]

        if not save_state and not on_disk:
            if not filename:
                print("Supply a valid filename!")
                return

            if not hasattr(self, "stored_states"):
                self.stored_states = []

            self.stored_states.append(
                pybullet.saveWorld(fileName=filename, clientServerId=self.client_id)
            )

            return self.stored_states[-1], filename

    def loadStatus(self, state_id=None, filename=None):
        """Loads a previous stored configuration of the server with a sufficient state_id or filename.
        """
        if not (state_id is not None or filename is not None):
            print("Please provide valid state id or filename.")
            return

        # Stop current simulation
        if self.thread_running:
            self.stopSimulation()
        # Wait for stop
        while self.thread_running:
            sleep(1e-3)
        # Load
        if state_id is not None:
            pybullet.restoreState(stateId=state_id)
        elif filename is not None:
            pybullet.restoreState(fileName=filename)

    def setStepsize(self, step_size=None):
        """Set the stepsize of a simulation.

        Arguments:
            step_size, float : optional, Step size in seconds. Default is 240 Hz
        """
        if step_size:
            self.step_size = step_size

        pybullet.setTimeStep(self.step_size, self.client_id)

    def setGravity(self, gravity=None):
        """Set the gravity according to the given vector.

        Arguments:
            gravity, array of floats: optional, X,Y,Z component of the gravitational acceleration.
        """

        if gravity is not None:
            self.gravity = gravity

        pybullet.setGravity(
            self.gravity[0], self.gravity[1], self.gravity[2], self.client_id
        )

    def startSimulation(self, stop_time=None):
        """Start the simulation and simulates until an optional stop time is reached.

        """
        if stop_time:
            self.stop_time = stop_time
        else:
            self.stop_time = None
        if self.stop_time is None or self.stop_time > self.time:
            self.simulation_running = True

    def stopSimulation(self):
        """Stops the simulation immediately.
        """
        self.simulation_running = False

    def simulate(self):
        """Simulate in a new thread.
        """
        while self.thread_running:
            if self.simulation_running:

                # Step the simulation
                with self.lock:
                    pybullet.stepSimulation()

                # Add the new time
                self.time += self.step_size
                # TODO Have a look a thread safety
                self._measurements = self._measurements.append(
                    {"time": self.time}, ignore_index=True
                )
                # Collect all measurements
                if hasattr(self, "robots"):
                    for robot in self.robots.values():
                        robot.update()
                if hasattr(self, "objects"):
                    for simObject in self.objects.values():
                        simObject.update()

                self.measurements = self.measurements.append(
                    self._measurements, ignore_index=True
                )
                self._measurements = pandas.DataFrame()

                # Check for gui -> slow down sim
                if self.mode:
                    sleep(self.sleep_time)
                # Stop if stop time is given or exceeded
                if (
                    self.stop_time is not None
                    and self.time + self.step_size >= self.stop_time
                ):
                    self.stopSimulation()
            else:
                sleep(self.sleep_time)

    def addObject(self, SimulationObject):
        """Adds an object to the simulation
        """

        if not hasattr(self, "objects"):
            self.objects = {}

        uid = SimulationObject.id

        self.objects.update({uid: SimulationObject})

    def addRobot(self, SimulationObject):
        """Adds a robot object ( collects measurements)
        """

        if not hasattr(self, "robots"):
            self.robots = {}

        uid = SimulationObject.id

        self.robots.update({uid: SimulationObject})


### This is Gabe Johnson's completed project for the Localization course in Udacity's Self-Driving Car Nanodegree.


## Localization
This project implements a 2 dimensional particle filter in C++, which is run in a simulator.  A "particle" is basically an imaginary version of the vehicle with its own location and velocity direction.  A particle filter creates a collection of particles, weeds out the particles that are least likely to accurately reflect the vehicle, and hones in on the true vehicle's true location and heading.  The simulator provides an initial (noisy) GPS estimate of starting location, a map of known locations with their coordinates, and noisy sensor measurements at each time step.  Code for interacting with the simulator, calculating error, etc was provided by Udacity, and my assignment was to code the particle filter contained in `particle_filter.cpp`. 

The particle filter starts by initializing a specified number of particles distributed around the initial GPS estimate with noise introduced to create variation in location and velocity direction.  At each time step, the location of each particle is updated with an estimated new location based on the vehicle's velocity and time step interval.  The noisy sensor data for the landmarks at each time step is received by the vehicle and given in coordinates with the vehicle as the origin.  That sensor data then undergoes a translation and rotation for each unique particle to give it in terms of the map's origin.  Each particle's sensed landmarks in map coordinates are compared to the map's landmarks and each sensed landmark is associated with the nearest map landmark.  If a particle's sensor landmark locations closely match the map's landmark locations, then the particle is likely to be a close representation of the actual vehicle.  This probability, or weight, is calculated for each particle using a Multivariate-Gaussian probability density function using the distances between the pairs of sensed landmarks and map landmarks.  Then the particles are resampled using a discrete distribution that favors those with higher calculated weight (those with little discrepancy between pairs) and a this new set of particles which more accurately reflect the true vehicle replaces the old set of particles.  Then after another time step, the new location and heading of each particle is calculated with noise introduced so that duplicated particles will diverge.  The process continues of continually evaluating the accuracy of the particles and weeding out the unlikely ones until the particle filter has a realistic representation of the vehicle. 

When running the simulator, you will see a vehicle surrounded by plotted map landmarks.  You will also see a purple circle with an arrow inside which represents the particle with the highest weight.  The green lines that come from the vehicle represent the sensor measurements.  The blue lines that come from the particle represent the particle's estimation of the landmarks based on its location and heading.  The red lines (very short) that connect the end of each green line to the end of each corresponding blue line represent the error between the two.  As the simulator runs and the vehicle moves, you will see that the landmarks come in and out of the senor range, and you will see that the most likely particle's purple circle and the true vehicle location correlate very closely.

[![ParticleFilterSimulation](video/KidnappedVehicleProjectSimulatio.gif =250x)](https://youtu.be/8LK1TXaFEWI)


## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux (install_ubuntu.sh) or Mac (install_mac.sh) systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.


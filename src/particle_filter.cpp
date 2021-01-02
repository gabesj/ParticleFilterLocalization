/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 * Assignment Completed by Gabe Johnson (see TODO sections)
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
	num_particles = 100;  
	/// prepare to generate x,y, and theta values normally distributed around the initial GPS inputs///
	std::default_random_engine gen;
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);
	for ( int i=0; i < num_particles; ++i){
		Particle initial_particle;
		initial_particle.id = i+1;
		initial_particle.x = dist_x(gen);
		initial_particle.y = dist_y(gen);
		initial_particle.theta = dist_theta(gen);
		initial_particle.weight = 1.0;
		particles.push_back(initial_particle);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
	std::default_random_engine gen;
	for ( int i=0; i < num_particles; ++i){
		std::normal_distribution<double> delta_x((cos(particles[i].theta) * velocity * delta_t) , std_pos[0]);
		particles[i].x += delta_x(gen);
		std::normal_distribution<double> delta_y((sin(particles[i].theta) * velocity * delta_t) , std_pos[1]);
		particles[i].y += delta_y(gen);
		std::normal_distribution<double> delta_theta((yaw_rate * delta_t), std_pos[2]);
		particles[i].theta += delta_theta(gen);
	} 
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                    vector<LandmarkObs>& observations) { 
   /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
	/// Not used ///
}



void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
   
   /// Normalization term for probability calculation
   double gauss_norm = 1/(2*M_PI*std_landmark[0]*std_landmark[1]);

   for (int i=0; i<particles.size(); ++i){
	particles[i].associations.clear();
	particles[i].sense_x.clear();
	particles[i].sense_y.clear();
	/// The vehicle gets sensor measurements of landmarks from its point of view, but its point of view is not the map's origin.
   	/// So perform a translation and rotation to get the sensed landmarks' coordinates in relation to the map's origin.
	for (int j=0; j<observations.size(); ++j){ 
		LandmarkObs particle_observations;
		particle_observations.x = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
		particle_observations.y = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y);
		particles[i].sense_x.push_back(particle_observations.x);
		particles[i].sense_y.push_back(particle_observations.y);		
   	}
	/// Then take the coordinates of those sensed landmarks and compare them to known landmark positions in the map.
	/// Associate each sensed landmark with the nearest known map landmark coordinate.  (or if none, use assume first landmark)
	double particle_weight = 0;
	for (int l=0; l<particles[i].sense_x.size(); ++l){
		double distance = sensor_range;
       		int landmark_index = 0;
		double particle_observation_weight = 0;
       		for (int m=0; m<map_landmarks.landmark_list.size(); ++m){
           		double temp_distance = dist(particles[i].sense_x[l], particles[i].sense_y[l],
						map_landmarks.landmark_list[m].x_f, map_landmarks.landmark_list[m].y_f);
	   		if (temp_distance < distance) {
				distance = temp_distance;
				landmark_index = m;
	  		}
		}
		particles[i].associations.push_back(landmark_index+1);
		/// For all the sensed landmark / map landmark pairs and use a Multivariate-Gaussian probability density function
		/// to calculate the probability (weight) that this is in fact a correct pair of sensed/map landmarks.
		double exponent = (pow((particles[i].sense_x[l]-map_landmarks.landmark_list[(particles[i].associations[l])-1].x_f),2.0)/(2.0*pow(std_landmark[0], 2.0)))
				+ (pow((particles[i].sense_y[l]-map_landmarks.landmark_list[(particles[i].associations[l])-1].y_f),2.0)/(2.0*pow(std_landmark[1], 2.0)));
		particle_observation_weight = gauss_norm * exp(-exponent);
		particle_weight += particle_observation_weight;
   			
	}
	particles[i].weight = particle_weight; 
   }     

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
	/// Random generator
	std::random_device rd;
    	std::mt19937 gen(rd());

	/// Create a vector of the particle weights
	std::vector<double> weights;
	weights.clear();
	for (int i=0; i<particles.size(); ++i){
		weights.push_back(particles[i].weight);
	}
	/// Create a discrete distribution (distr) with those weights
	std::discrete_distribution<> distr(weights.begin(), weights.end());
	
	/// Create an empty vector of particles which will hold the resampled particles
	vector<Particle> resampled_particles;
	resampled_particles.clear();
	
	/// Sample from the discrete_distribution and create a new array of particles.  
	/// This selects a sample with probability proportional to its weight
	Particle sampled_particle;
	int particle_index;
	for (int j=0; j<particles.size(); ++j){
		particle_index = distr(gen);
		resampled_particles.push_back(particles[particle_index]);
	}
	
	/// Replace the old particles with the new resampled particles
	particles = std::move(resampled_particles);

	
		


}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
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
  num_particles = 100;  // TODO: Set the number of particles
  /// prepare to generate x,y, and theta values normally distributed around the initial GPS inputs.
  std::default_random_engine gen;
  ///std::cout << "std[]= " << std[0] << " " << std[1] << " " << std[2] << " " << x << " " << y <<std::endl;
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
	///std::cout << "initial_particle " << initial_particle.x << " " << initial_particle.y << std::endl;/////for debug
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
      particles[i].y += delta_theta(gen);
      ///std::cout << "prediction " << i << " " << particles[i].x << " " << particles[i].y << " " << particles[i].theta << std::endl;
}
   
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                    vector<LandmarkObs>& observations) { 
///void ParticleFilter::dataAssociation(vector<LandmarkObs> &map_transformed_particle_observations, Map map_landmarks){
///need to update particle_filter.h////////
   /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

   /*
   for (int l; l<map_transformed_particle_observations.size(), ++l){
       double distance = 999999.9;
       int landmark_index;
       for (int m; m<map_landmarks.size(), ++m){
//	   double temp_distance = sqrt( pow((map_transformed_particle_observations[l].x - map_landmarks[m].x), 2.0) + pow((map_transformed_particle_observations[l].y - map_landmarks[m]), 2.0));
           double temp_distance = dist(map_transformed_particle_observations[l].x, map_transformed_particle_observations[l].y, map_landmarks[m].x, map_landmarks[m].y);
	   if temp_distance < distance {
		distance = temp_distance;
		landmark_index = m;
	   }
       particle_observation_prob = 
	   
	   map_transformed_particle_observations[l].id = 
       }
   }*/
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
   ///std::cout << "sensor_range " << sensor_range << " " << "std_landmark[] " << std_landmark[0] << " " << std_landmark[1] << " " << std_landmark[2] << std::endl;
   
   /// Here's what I need to do: 
   /// For each particle, create a vector of the observations from its point of view based on the particle's position/heading.
   /// Then for each particle, call the dataAssociation() function to compare the predicted particle's observations to the car's observations and associate the car's observations with the particles' closest observation.
   /// Then update the weight of the particle
   /// Then move on to the next particle   


    
   /// Take the landmark observations from the vehicle's point of view in the vehicle coordinate system 
   /// and apply them to each particle's position/heading
   /// and transform them into the map's point of view in the map coordinate system


   double gauss_norm = 1/(2*M_PI*std_landmark[0]*std_landmark[1]);///normalization term for probability calculation
   for (int i=0; i<particles.size(); ++i){
	///vector<LandmarkObs> map_transformed_particle_observations;
   	///map_transformed_particle_observations.clear();
	particles[i].associations.clear();
	particles[i].sense_x.clear();
	particles[i].sense_y.clear();
	///map the car's observations onto the particle's coordinates/heading and transform to map coordinates///
	for (int j=0; j<observations.size(); ++j){ 
		LandmarkObs particle_observations;
		particle_observations.x = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
		particle_observations.y = particles[i].y + (sin(particles[i].theta) * observations[j].x) - (cos(particles[i].theta) * observations[j].y);
		///map_transformed_particle_observations.push_back(particle_observations);
		particles[i].sense_x.push_back(particle_observations.x);
		particles[i].sense_y.push_back(particle_observations.y);
   	}
	/// for each particle observation, find the closest map landmark (or if none, use the first landmark) and calculate the probability 
	///for (int l; l<map_transformed_particle_observations.size(), ++l){
	double particle_weight = 0;
	for (int l; l<particles[i].sense_x.size(); ++l){
		double distance = sensor_range;
       		int landmark_index = 0;
		double particle_observation_weight = 0;
       		for (int m; m<map_landmarks.landmark_list.size(); ++m){
//	        	double temp_distance = sqrt( pow((map_transformed_particle_observations[l].x - map_landmarks.landmark_list[m].x_f), 2.0) + 					       pow((map_transformed_particle_observations[l].y - map_landmarks.landmark_list[m].y_f), 2.0));
           		double temp_distance = dist(particles[i].sense_x[l], particles[i].sense_y[l],
						map_landmarks.landmark_list[m].x_f, map_landmarks.landmark_list[m].y_f);
	   		if (temp_distance < distance) {
				distance = temp_distance;
				landmark_index = m;
	  		}
		particles[i].associations.push_back(landmark_index);

		/// maybe put this in another for loop, maybe not ///
       		//double exponent = (pow((map_transformed_particle_observations[l].x-map_landmarks[landmark_index].x),2.0)/(2.0*pow(std_landmark[0], 2.0)))
		//		+ (pow((map_transformed_particle_observations[l].y-map_landmarks[landmark_index].y),2.0)/(2.0*pow(std_landmark[1], 2.0)));
		double exponent = (pow((particles[i].sense_x[l]-map_landmarks.landmark_list[(particles[i].associations[l])].x_f),2.0)/(2.0*pow(std_landmark[0], 2.0)))
				+ (pow((particles[i].sense_y[l]-map_landmarks.landmark_list[(particles[i].associations[l])].y_f),2.0)/(2.0*pow(std_landmark[1], 2.0)));
		particle_observation_weight = gauss_norm * exp(-exponent);
		particle_weight += particle_observation_weight;
   		}	
	}
	/// save the particle's weight to the particle array ///
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

/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  std::default_random_engine generator;
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  num_particles = 100;
  
  for(int i = 0; i < num_particles; i++)
  {
    double temp_weight = 1.0/num_particles;

    Particle temp_particle;
    temp_particle.id = i;
    temp_particle.x = dist_x(generator);
    temp_particle.y = dist_y(generator);
    temp_particle.theta = dist_theta(generator);
    temp_particle.weight = temp_weight;
    particles.push_back(temp_particle);

    weights.push_back(temp_weight);
  }

  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  std::default_random_engine generator;
  std::normal_distribution<double> dist_x(0.0, std_pos[0]);
  std::normal_distribution<double> dist_y(0.0, std_pos[1]);
  std::normal_distribution<double> dist_theta(0.0, std_pos[2]);

  double x_diff = 0.0;
  double y_diff = 0.0;
  double theta_diff = 0.0;
  double x_noise = 0.0;
  double y_noise = 0.0;
  double theta_noise = 0.0;

  double yaw    = 0.0;

  for(int i = 0; i < particles.size(); i++)
  {
    yaw = particles[i].theta;
    if(yaw_rate < 0.0001)
    {
      x_diff = velocity * delta_t * std::cos(yaw);
      y_diff = velocity * delta_t * std::sin(yaw);
      theta_diff = 0.0;
    }
    else
    {
      x_diff = (velocity/yaw_rate) * (std::sin(yaw_rate * delta_t + yaw) - std::sin(yaw));
      y_diff = (velocity/yaw_rate) * (std::cos(yaw) - std::cos(yaw_rate * delta_t + yaw));
      theta_diff = yaw_rate * delta_t;
    }

    x_noise = 0.5 * delta_t * delta_t * dist_x(generator);
    y_noise = 0.5 * delta_t * delta_t * dist_y(generator);
    theta_noise = 0.5 * delta_t * delta_t * dist_theta(generator);

    particles[i].x = particles[i].x + x_diff + x_noise;
    particles[i].y = particles[i].y + y_diff + y_noise;
    particles[i].theta = particles[i].theta + theta_diff + theta_noise;
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html



  /* For each Particle do the following */
  for(int i = 0; i < particles.size(); i++)
  {
    double theta = particles[i].theta;
    /* Part 1 : Transform observations from Particle's Coordinate system to MAP's coordinate system */
    double rot_mat[2][2] = { { std::cos(theta), -std::sin(theta)},
                             { std::sin(theta),  std::cos(theta)}};

    std::vector<LandmarkObs> trans_obs;

    for(int j = 0; j < observations.size(); j++)
    {
      LandmarkObs temp_obs = { observations[i].id, observations[i].x, observations[i].y };
      temp_obs.x = temp_obs.x * rot_mat[0][0] + temp_obs.y * rot_mat[1][0];
      temp_obs.y = temp_obs.x * rot_mat[0][1] + temp_obs.y * rot_mat[1][1];
      temp_obs.x += particles[i].x;
      temp_obs.y += particles[i].y;
      trans_obs.push_back(temp_obs);

    }


    /* Part 1.5 : Identify for each observation which landmark it corresponds to, and combine overlapping observations*/

    /* Part 2 : For each observation/landmark pair, Find the belief */

    /* Part 3 : Multiply the beliefs of all the landmarks to get the un-normalized weight for this particle */
  }

  /* After processing all particles, normalize all the weights*/
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution


}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}

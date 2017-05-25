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
#include <limits>
#include <unordered_map>
#include <chrono>
#include "particle_filter.h"

#define M_PIl          3.141592653589793238462643383279502884L /* pi */
#define USE_DIST_PRED_TO_REAL
#define USE_DISCRETE_DISTRIBUTION
enum debug_mode_e {
  DEBUG_MODE_OFF,
  DEBUG_MODE_LOW,
  DEBUG_MODE_HIGH,
  NUM_DEBUG_MODES
};

debug_mode_e debug_mode = DEBUG_MODE_OFF;

void printParticles(std::vector<Particle> particles)
{
  for(unsigned int i = 0; i < particles.size(); i++)
  {
    if(DEBUG_MODE_OFF != debug_mode)
    {
      std::cout<<"Particle["<<i<<"] : x = "<<particles[i].x<<", y = "<<particles[i].y<<", theta = "<<particles[i].theta
        <<", weight = "<<particles[i].weight<<std::endl;
    }
  }
}

std::default_random_engine global_generator;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  num_particles = 20;

  for(int i = 0; i < num_particles; i++)
  {
    //double temp_weight = 1.0/num_particles;
    double temp_weight = 1.0;

    Particle temp_particle;
    temp_particle.id = i;
    temp_particle.x = dist_x(global_generator);
    temp_particle.y = dist_y(global_generator);
    temp_particle.theta = dist_theta(global_generator);
    temp_particle.weight = temp_weight;
    particles.push_back(temp_particle);

    weights.push_back(temp_weight);
  }
  std::cout<<"Initializing"<<std::endl;
  printParticles(particles);

  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  if(DEBUG_MODE_OFF != debug_mode)
  {
    std::cout<<"Called Prediction : dt = "<<delta_t<<", std_pos = {"<<std_pos[0]<<","<<std_pos[1]<<","<<std_pos[2]<<"}, velocity = "
      <<velocity<<", yaw_rate = "<<yaw_rate<<std::endl;
  }
  std::normal_distribution<long double> dist_x(0.0, std_pos[0]);
  std::normal_distribution<long double> dist_y(0.0, std_pos[1]);
  std::normal_distribution<long double> dist_theta(0.0, std_pos[2]);

  long double x_diff = 0.0;
  long double y_diff = 0.0;
  long double theta_diff = 0.0;
  long double x_noise = 0.0;
  long double y_noise = 0.0;
  long double theta_noise = 0.0;
  long double velocity_l = velocity;
  long double yaw_rate_l = yaw_rate;

  long double yaw    = 0.0;
  long double delta_t_l = delta_t;

  for(unsigned int i = 0; i < particles.size(); i++)
  {
    yaw = particles[i].theta;
    if(yaw_rate < 0.0000001)
    {
      x_diff = velocity * delta_t * std::cos(yaw);
      y_diff = velocity * delta_t * std::sin(yaw);
      theta_diff = yaw_rate * delta_t;
    }
    else
    {
      x_diff = (velocity_l/yaw_rate_l) * (std::sin(yaw_rate_l * delta_t_l + yaw) - std::sin(yaw));
      y_diff = (velocity_l/yaw_rate_l) * (std::cos(yaw) - std::cos(yaw_rate_l * delta_t_l + yaw));
      theta_diff = yaw_rate_l * delta_t_l;
    }
    x_noise = dist_x(global_generator);
    y_noise = dist_y(global_generator);
    theta_noise = dist_theta(global_generator);
    particles[i].x = particles[i].x + x_diff + x_noise;
    particles[i].y = particles[i].y + y_diff + y_noise;
    particles[i].theta = particles[i].theta + theta_diff + theta_noise;
  }
  printParticles(particles);
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted,
    std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

  for(unsigned int i = 0; i < observations.size(); i++)
  {
    double min_dist = std::numeric_limits<double>::max();
    double obs_x = observations[i].x;
    double obs_y = observations[i].y;

    for(unsigned int j = 0; j < predicted.size(); j++)
    {
      double x_diff = predicted[j].x - obs_x;
      double y_diff = predicted[j].y - obs_y;
      double dist = (x_diff * x_diff) + (y_diff * y_diff);
      if(dist < min_dist)
      {
        min_dist = dist;
        observations[i].id = j;
      }
    }
  }
}

/**
 * Perform a linear transform on the input measuremen
 */
LandmarkObs transform(Particle p, LandmarkObs input)
{
  long double theta = p.theta;
  long double rot_mat[3][3] =
  {
    {  std::cos(theta), -std::sin(theta), p.x },
    {  std::sin(theta),  std::cos(theta), p.y },
    {  0              ,  0              , 1   }
  };
  LandmarkObs result;
  result.x = (rot_mat[0][0] * input.x) + (rot_mat[0][1] * input.y) + rot_mat[0][2];
  result.y = (rot_mat[1][0] * input.x) + (rot_mat[1][1] * input.y) + rot_mat[1][2];

  return result;
}

/**
 * Calculate the predicted observation for a landmark
 */
LandmarkObs landMarkToObs( Map::single_landmark_s s)
{
  LandmarkObs result;
  result.id = s.id_i;
  result.x  = s.x_f ;//- p.x;
  result.y  = s.y_f ;//- p.y;
  return result;
}


/**
 * Calculate the belief for a given point
 */
long double calculateBelief(double val[], double mean[], double std_dev[])
{
  long double var[2] = { std_dev[0] * std_dev[0], std_dev[1] * std_dev[1]};
  long double det = var[0] * var[1];

  long double inv_cov_mat[2][2] =
  {
    { var[0]/det, 0          },
    { 0         , var[0]/det }
  };

  long double diff[2] = { val[0] - mean[0], val[1] - mean[1]};

  long double power = -0.5 * (  (diff[0] * diff[0])*(inv_cov_mat[0][0] + inv_cov_mat[1][0]) +
                         (diff[1] * diff[1])*(inv_cov_mat[0][1] + inv_cov_mat[1][1]) );

  long double result = std::exp(power)/ std::sqrt( fabsl(M_PIl * 2.0 * det));
  if(DEBUG_MODE_HIGH == debug_mode)
  {
    std::cout<<"Calc Belief : val : {"<<val[0]<<","<<val[1]<<"}, mean : {"<<mean[0]<<","<<mean[1]<<"}, std : {"<<std_dev[0]<<","<<std_dev[1]<<"}, ";
    std::cout<<"Belief : "<<result<<std::endl;
  }

  return result;
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

  long double total_weight = 0.0;
  long double temp_weights[particles.size()];

  /* For each Particle do the following */
  for(unsigned int i = 0; i < particles.size(); i++)
  {

    std::vector<LandmarkObs> trans_obs;

    for(unsigned int j = 0; j < observations.size(); j++)
    {
      trans_obs.push_back(transform(particles[i], observations[j]));
    }

    /* Part 1.1 : Generate list of map landmarks and predicted observations corresponding to them */
    std::vector<LandmarkObs> pred_obs;

    for(unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++)
    {
      double distance_to_landmark = dist(particles[i].x,
                                         particles[i].y,
                                         map_landmarks.landmark_list[j].x_f,
                                         map_landmarks.landmark_list[j].y_f
                                         );
      if(distance_to_landmark < sensor_range)
      {
        pred_obs.push_back(landMarkToObs( map_landmarks.landmark_list[j]));
      }
    }

    /* Part 1.5 : Identify for each observation which landmark it corresponds to, and combine overlapping observations*/
    dataAssociation(pred_obs, trans_obs);

    /* Part 2 : For each observation/landmark pair, Find the belief */
    long double belief = 1.0;

    std::unordered_map<int, LandmarkObs> pred_trans_obs_map;

    for(unsigned int j = 0; j < trans_obs.size(); j++)
    {
#ifdef USE_DIST_PRED_TO_REAL
      double dist_pred_trans = dist(pred_obs[trans_obs[j].id].x,
                                    pred_obs[trans_obs[j].id].y,
                                    trans_obs[j].x,
                                    trans_obs[j].y);
#else
      double dist_pred_trans = dist(particles[i].x,
                                    particles[i].y,
                                    trans_obs[j].x,
                                    trans_obs[j].y);
#endif

      if(0 == pred_trans_obs_map.count(trans_obs[j].id))
      {
        pred_trans_obs_map[trans_obs[j].id] = trans_obs[j];
      }
      else
      {
#ifdef USE_DIST_PRED_TO_REAL
        double prev_dist = dist(pred_obs[trans_obs[j].id].x,
                                pred_obs[trans_obs[j].id].y,
                                pred_trans_obs_map[trans_obs[j].id].x,
                                pred_trans_obs_map[trans_obs[j].id].y);
#else
        double prev_dist = dist(particles[i].x,
                                particles[i].y,
                                pred_trans_obs_map[trans_obs[j].id].x,
                                pred_trans_obs_map[trans_obs[j].id].y);
#endif
        if(dist_pred_trans < prev_dist)
        {
          pred_trans_obs_map[trans_obs[j].id] = trans_obs[j];
        }
      }
    }

    for(unsigned int j = 0; j < pred_obs.size(); j++)
    {
      if(0 != pred_trans_obs_map.count(j))
      {
        LandmarkObs temp_trans_obs = pred_trans_obs_map[j];
        double val[2] = { temp_trans_obs.x, temp_trans_obs.y  };
        double mean[2] = { pred_obs[j].x, pred_obs[j].y };
        belief *= calculateBelief(val, mean, std_landmark);
      }
    }
    /* Part 3 : Multiply the beliefs of all the landmarks to get the un-normalized weight for this particle */
    temp_weights[i] = belief;

    total_weight += belief;
    if(DEBUG_MODE_HIGH == debug_mode)
    {
      std::cout<<"Updating weight for particle : "<<i<<", Belief = "<<belief<<", Total weight = "<<total_weight<<std::endl;
    }
  }

  /* After processing all particles, normalize all the weights*/
  for(unsigned int i = 0; i < particles.size(); i++)
  {
    particles[i].weight = (double)((long double)num_particles * (long double)temp_weights[i] / total_weight);
    weights[i] = particles[i].weight;
    if(DEBUG_MODE_OFF != debug_mode)
    {
      std::cout<<"Particle["<<i<<"].weight : "<<particles[i].weight<<std::endl;
    }
  }

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  std::discrete_distribution<> distribution(weights.begin(), weights.end());
  std::vector<Particle> new_particles;
  for(int i = 0; i < num_particles; i++)
  {
    int weight_index = distribution(global_generator);
    new_particles.push_back(particles[weight_index]);
  }
  particles = new_particles;
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

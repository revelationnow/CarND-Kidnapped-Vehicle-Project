/*
 * particle_filter_test.h
 *
 * 2D particle filter class.
 *  Created on: May 2, 2017
 *      Author: Anoop Ramakrishna
 */

#ifndef PARTICLE_FILTER_TEST_H
#define PARTICLE_FILTER_TEST_H

#include "particle_filter.h"
#include <array>
#include <iostream>
#include <cmath>

class ParticleFilterTest {
  private:
    ParticleFilter pf_;
  public:

    bool compare_float(double f1, double f2, double thresh)
    {
      if(std::fabs(f2 - f1) < std::fabs(thresh))
        return true;
      return false;
    }

    void PrintParticle(ParticleFilter pf)
    {
      std::cout<<"List of Particles : "<<std::endl;
      for(unsigned int i = 0; i < pf.particles.size(); i++)
      {
        std::cout<<"Particle : "<<pf.particles[i].id<<" -> x : "<<pf.particles[i].x<<", y : "<<pf.particles[i].y<<", theta : "<<pf.particles[i].theta<<", weight : "<<pf.particles[i].weight<<std::endl;
      }
    }

    void Test_2_belief()
    {
      double std_dev[2] = {4.25, 4.25};
      double mean[2] = {5,9};
      double val[2] = {1,3};

      long double result = calculateBelief(val, mean, std_dev);

      std::cout<<"Belief = "<<result<<std::endl;
    }

    void Test_1_rotate()
    {
      Particle p1;
      LandmarkObs l1, l2, l3;
      LandmarkObs l1_out, l2_out, l3_out;
      bool result = true;

      p1.x = 4;
      p1.y = 5;
      p1.theta = -3.14159/2;

      l1.x = 2;
      l1.y = 2;
      l1_out.x = 6;
      l1_out.y = 3;

      l2.x = 3;
      l2.y = -2;
      l2_out.x = 2;
      l2_out.y = 2;

      l3.x = 0;
      l3.y = -4;
      l3_out.x = 0;
      l3_out.y = 5;

      LandmarkObs l1_trans = transform(p1, l1);
      LandmarkObs l2_trans = transform(p1, l2);
      LandmarkObs l3_trans = transform(p1, l3);

      if( (compare_float(l1_out.x, l1_trans.x, 1e-5) && compare_float(l1_out.y, l1_trans.y, 1e-5)) != true )
      {
        std::cout<<"Transform failed for ("<<l1.x<<","<<l1.y<<"), expected : (";
        std::cout<<l1_out.x<<","<<l1_out.y<<"), received ("<<l1_trans.x<<","<<l1_trans.y<<")"<<std::endl;
        result = false;
      }

      if( (compare_float(l2_out.x, l2_trans.x, 1e-5) && compare_float(l2_out.y, l2_trans.y, 1e-5)) != true )
      {
        std::cout<<"Transform failed for ("<<l2.x<<","<<l2.y<<"), expected : (";
        std::cout<<l2_out.x<<","<<l2_out.y<<"), received ("<<l2_trans.x<<","<<l2_trans.y<<")"<<std::endl;
        result = false;
      }

      if( (compare_float(l3_out.x, l3_trans.x, 1e-5) && compare_float(l3_out.y, l3_trans.y, 1e-5)) != true )
      {
        std::cout<<"Transform failed for ("<<l3.x<<","<<l3.y<<"), expected : (";
        std::cout<<l3_out.x<<","<<l3_out.y<<"), received ("<<l3_trans.x<<","<<l3_trans.y<<")"<<std::endl;
        result = false;
      }

      if(true == result)
      {
        std::cout<<"Transform test passed"<<std::endl;
      }
    }

    void Test_0_init()
    {
      /* Set up inputs */
      double inp_x = 0.0, inp_y = 0.0, inp_theta = 0.0;
      double inp_std_x = 2.0, inp_std_y = 2.0, inp_std_theta = 0.05;
      std::array<double, 3> std { { inp_std_x, inp_std_y, inp_std_theta  } };

      /* Run function */
      pf_.init(inp_x, inp_y , inp_theta, std.data());

      /* Check mean and std */
      /* Check mean for x, y, theta */
      double sum_x = 0.0;
      double sum_y = 0.0;
      double sum_theta = 0.0;

      for(unsigned int i = 0; i < pf_.particles.size(); i++)
      {
        sum_x += pf_.particles[i].x;
        sum_y += pf_.particles[i].y;
        sum_theta += pf_.particles[i].theta;

      }

      double mean_x = sum_x / pf_.particles.size();
      double mean_y = sum_y / pf_.particles.size();
      double mean_theta = sum_theta / pf_.particles.size();

      if((fabs(mean_x - inp_x) > 0.5) ||
         (fabs(mean_y - inp_y) > 0.5) ||
         (fabs(mean_theta - inp_theta) > 0.5)
        )

      {
        std::cout<<"Mean x : "<<mean_x<<", Mean y : "<<mean_y<<", Mean theta : "<<mean_theta<<std::endl;
        std::cout<<"Inp x : "<<inp_x<<", Inp y : "<<inp_y<<", Inp theta : "<<inp_theta<<std::endl;
      }
      else
      {
        std::cout<<"Mean value test passed"<<std::endl;
      }


      double variance_sum_x = 0.0;
      double variance_sum_y = 0.0;
      double variance_sum_theta = 0.0;
      /* Check std for x, y, theta */
      for(unsigned int i = 0; i < pf_.particles.size(); i++)
      {
        variance_sum_x += (pf_.particles[i].x - mean_x) * (pf_.particles[i].x - mean_x);
        variance_sum_y += (pf_.particles[i].y - mean_y) * (pf_.particles[i].y - mean_y);
        variance_sum_theta += (pf_.particles[i].theta - mean_theta) * (pf_.particles[i].theta - mean_theta);
      }

      double std_x = std::sqrt(variance_sum_x / pf_.particles.size());
      double std_y = std::sqrt(variance_sum_y / pf_.particles.size());
      double std_theta = std::sqrt(variance_sum_theta / pf_.particles.size());

      if(
          (fabs(std_x - inp_std_x) > 0.1) ||
          (fabs(std_y - inp_std_y) > 0.1) ||
          (fabs(std_theta - inp_std_theta) > 0.1)
        )
      {
        std::cout<<"Std x : "<<std_x<<", Std y : "<<std_y<<"< Std theta : "<<std_theta<<std::endl;
        std::cout<<"Inp Std x : "<<inp_std_x<<", Inp y : "<<inp_std_y<<", Inp theta : "<<inp_std_theta<<std::endl;
      }
      else
      {
        std::cout<<"Std value test passed"<<std::endl;
      }

    }

};

#endif//PARTICLE_FILTER_TEST_H

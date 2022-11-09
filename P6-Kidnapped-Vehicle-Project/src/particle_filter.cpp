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
#include <limits>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
    /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */

    // TODO: Set the flag
    is_initialized = true;

    // TODO: Set the number of particles
    num_particles = 50;
    particles.reserve(num_particles);

    // TODO: Set standard deviations for x, y, and theta
    double std_x = std[0];
    double std_y = std[1];
    double std_theta = std[2];

    // TODO: Create normal distributions for x, y and theta
    std::normal_distribution<double> dist_x(x, std_x);
    std::normal_distribution<double> dist_y(y, std_y);
    std::normal_distribution<double> dist_theta(theta, std_theta);
    std::default_random_engine gen;

    for (int i = 0; i < num_particles; ++i)
    {
        // TODO: Sample from these normal distributions like this:
        //   sample_x = dist_x(gen);
        //   where "gen" is the random engine initialized earlier.
        Particle sample;
        sample.x = dist_x(gen);
        sample.y = dist_y(gen);
        sample.theta = dist_theta(gen);
        sample.weight = 1;

        particles.push_back(sample);

        // Print your samples to the terminal.
        //std::cout << "Sample " << i + 1 << " " << sample.x << " " << sample.y << " "
        //          << sample.theta << std::endl;
    }

    return;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate)
{
    /**
     * TODO: Add measurements to each particle and add random Gaussian noise.
     * NOTE: When adding noise you may find std::normal_distribution 
     *   and std::default_random_engine useful.
     *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
     *  http://www.cplusplus.com/reference/random/default_random_engine/
     */

    // TODO: Set standard deviations for x, y, and theta
    double std_x = std_pos[0];
    double std_y = std_pos[1];
    double std_theta = std_pos[2];

    // TODO: Create normal distributions for x, y and theta
    std::normal_distribution<double> dist_x(0, std_x);
    std::normal_distribution<double> dist_y(0, std_y);
    std::normal_distribution<double> dist_theta(0, std_theta);
    std::default_random_engine gen;

    for (unsigned int i = 0; i < particles.size(); ++i)
    {

        // TODO: Perform prediction. This should handle division by zero.
        double theta0 = particles[i].theta;
        if(yaw_rate == 0) {
            particles[i].x += velocity * cos(theta0) * delta_t;
            particles[i].y += velocity * sin(theta0) * delta_t;

        } else {
            particles[i].x += velocity / yaw_rate * (sin(theta0 + yaw_rate * delta_t) - sin(theta0));
            particles[i].y += velocity / yaw_rate * (cos(theta0) - cos(theta0 + yaw_rate * delta_t));
            particles[i].theta += yaw_rate * delta_t;
        }

        // Angle Normalization. This is not a must.
        while (particles[i].theta > 2 * M_PI)
            particles[i].theta -= 2 * M_PI;
        while (particles[i].theta <= 0)
            particles[i].theta += 2 * M_PI;
        // add prediction noise
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted,
                                     std::vector<LandmarkObs> &observations)
{
    /**
     * TODO: Find the predicted measurement that is closest to each 
     *   observed measurement and assign the observed measurement to this 
     *   particular landmark.
     * NOTE: this method will NOT be called by the grading code. But you will 
     *   probably find it useful to implement this method and use it as a helper 
     *   during the updateWeights phase.
     */

    // for each observations
    for (unsigned int i = 0; i < observations.size(); ++i)
    {
        double xo = observations[i].x;
        double yo = observations[i].y;

        double dist_min = std::numeric_limits<double>::infinity();

        //calculate the distance and associate the nearest landmark
        for (unsigned int j = 0; j < predicted.size(); ++j)
        {
            double xl = predicted[j].x;
            double yl = predicted[j].y;
            double dist = pow(xl - xo, 2) + pow(yl - yo, 2);

            if (dist_min > dist)
            {
                // TODO: associate id here
                dist_min = dist;
                observations[i].id = predicted[j].id;
            }
        }
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{
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
    double std_x = std_landmark[0];
    double std_y = std_landmark[1];
    vector<LandmarkObs> valid_landmarks;
    valid_landmarks.reserve(map_landmarks.landmark_list.size());

    // for each particle, compare observations with landmark and evaluate.
    for (unsigned int i = 0; i < particles.size(); ++i)
    {
        const double xp = particles[i].x;
        const double yp = particles[i].y;
        const double theta = particles[i].theta;

        //TODO: filter out landmarks within sensor range
        for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); ++j)
        {
            double xl = map_landmarks.landmark_list[j].x_f;
            double yl = map_landmarks.landmark_list[j].y_f;
            int idl = map_landmarks.landmark_list[j].id_i;

            double dist = sqrt(pow(xl - xp, 2) + pow(yl - yp, 2));
            if (dist <= sensor_range)
            {
                LandmarkObs ld;
                ld.x = xl;
                ld.y = yl;
                ld.id = idl;
                valid_landmarks.push_back(ld);
            }
        }

        //TODO: Transform observation to map coordinate for given particle
        vector<LandmarkObs> transformed_obs;
        transformed_obs.reserve(observations.size());

        for (unsigned int j = 0; j < observations.size(); ++j)
        {
            double xc = observations[j].x;
            double yc = observations[j].y;

            LandmarkObs obj;
            obj.x = xc * cos(theta) - yc * sin(theta) + xp;
            obj.y = xc * sin(theta) + yc * cos(theta) + yp;
            transformed_obs.push_back(obj);
        }

        // TODO: dataAssociation() function will label transformed_obs vector with landmark id
        dataAssociation(valid_landmarks, transformed_obs);

        // TODO: calculate importance weight
        double weight = 1;
        for (unsigned int j = 0; j < transformed_obs.size(); ++j)
        {
            for (unsigned int k = 0; k < valid_landmarks.size(); ++k)
            {
                // find associated landmark
                if (transformed_obs[j].id == valid_landmarks[k].id)
                {
                    // calculate probability
                    double xo = transformed_obs[j].x;
                    double yo = transformed_obs[j].y;

                    double xl = valid_landmarks[k].x;
                    double yl = valid_landmarks[k].y;

                    // product probabiltiy
                    double exponent = -(pow(xo - xl, 2) / (2 * pow(std_x, 2)) + pow(yo - yl, 2) / (2 * pow(std_y, 2)));
                    weight *= 1.0 / (2 * M_PI * std_x * std_y) * exp(exponent);

                    break;
                }
            }
        }

        //TODO: Save weights to the particle
        particles[i].weight = weight;
    }
}

void ParticleFilter::resample()
{
    /**
     * TODO: Resample particles with replacement with probability proportional 
     *   to their weight. 
     * NOTE: You may find std::discrete_distribution helpful here.
     *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
     */

    // TODO: get maximum value of weight
    double weight_max = 0;
    for (unsigned int i = 0; i < particles.size(); ++i)
    {
        if (weight_max < particles[i].weight)
        {
            weight_max = particles[i].weight;
        }
    }

    // TODO: resampling wheel
    std::uniform_real_distribution<double> dist_weight(0, weight_max);
    std::uniform_int_distribution<int> dist_index(0, particles.size() - 1);
    std::default_random_engine gen;

    std::vector<Particle> resampled_particles;
    resampled_particles.reserve(particles.size());

    double beta = 0.0;
    int index = dist_index(gen);

    // for (number of particles) times
    for (unsigned int i = 0; i < particles.size(); ++i)
    {
        // pick one
        beta += 2.0 * dist_weight(gen);
        while (beta > particles[index].weight)
        {
            beta -= particles[index].weight;
            index = (index + 1) % particles.size();
        }

        resampled_particles.push_back(particles[index]);
    }

    // replace the vector
    particles = resampled_particles;
 
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y)
{
    // particle: the particle to which assign each listed association,
    //   and association's (x,y) world coordinates mapping
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1); // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
    vector<double> v;

    if (coord == "X")
    {
        v = best.sense_x;
    }
    else
    {
        v = best.sense_y;
    }

    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1); // get rid of the trailing space
    return s;
}
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <Eigen/Core>
#include "particle/particle_filter.h"
#include "particle/helper_functions.h"

using namespace std;

static default_random_engine gen;



/**
* Initializes particle filter by randomly distributing the particles 
* around the map.
* @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
*   standard deviation of yaw [rad]]
* @param nParticles Number of particles used by the algorithm
*/
void ParticleFilter::init_random(double std[], int nParticles) {
    normal_distribution<double> noise_dist_x(0, std[0]);
    normal_distribution<double> noise_dist_y(0, std[1]);
    normal_distribution<double> noise_dist_theta(0, std[2]);

    std::uniform_real_distribution<> dist_x(map_x_boundaries.first, map_x_boundaries.second);
    std::uniform_real_distribution<> dist_y(map_y_boundaries.first, map_y_boundaries.second);
    std::uniform_real_distribution<> dist_theta(-M_PI, M_PI);

    for (int i=0; i<nParticles; ++i) {
        particles.emplace_back(
            dist_x(gen) + noise_dist_x(gen),
            dist_y(gen) + noise_dist_y(gen),
            dist_theta(gen) + noise_dist_theta(gen)
        );
    }

    is_initialized = true;
}

/*
* TODO
* This function initialize the particles using an initial guess
* Input:
*  x,y,theta - position and orientation
*  std - noise that might be added to the position
*  nParticles - number of particles
*/ 
void ParticleFilter::init(double x, double y, double theta, double std[],int nParticles) {
    num_particles = nParticles;
    normal_distribution<double> dist_x(-std[0], std[0]); //random value between [-noise.x,+noise.x]
    normal_distribution<double> dist_y(-std[1], std[1]);
    normal_distribution<double> dist_theta(-std[2], std[2]);

	//TODO
    
    is_initialized=true;
}

/**
* Predicts the state for the next time step using the process model.
* @param delta_t Time between time step t and t+1 in measurements [s]
* @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
*   standard deviation of yaw [rad]]
* @param velocity Velocity of car from t to t+1 [m/s]
* @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
*/
void ParticleFilter::prediction(double delta_t, double std[], double velocity, double yaw_rate) {
    //for each particle
    normal_distribution<double> noise_dist_x(0, std[0]);
    normal_distribution<double> noise_dist_y(0, std[1]);
    normal_distribution<double> noise_dist_theta(0, std[2]);
    const double displacement = velocity*delta_t;

    auto noise_distribution = MultivariateNormalDistribution<double>(
        noise_dist_x,
        noise_dist_y,
        noise_dist_theta,
        gen
    );

    for (auto &particle : particles) {
        auto pe = particle.eigenize();

        if (fabs(yaw_rate) < 0.00001) {  // Moving forward
            pe += Eigen::Vector3d{cos(pe(2)) * displacement, sin(pe(2)) * displacement, 0};  // Add x, y motion components
        }
        else {  // Turning
            pe += Eigen::Vector3d{
                (velocity / yaw_rate) * (sin(pe(2) + yaw_rate * delta_t) - sin(pe(2))),
                (velocity / yaw_rate) * (cos(pe(2)) - cos(pe(2) + yaw_rate * delta_t)),
                yaw_rate*delta_t
            };
        }

        pe += noise_distribution.get_rand();  // Add noise
        particle = pe;
    }
}

/**
* Finds which observations correspond to which landmarks (likely by using
* a nearest-neighbors data association).
* @param predicted Vector of predicted landmark observations
* @param observations Vector of landmark observations
*/
void dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    Eigen::MatrixXd predicted_matrix(predicted.size(), 2);

    for (size_t i = 0; i < predicted.size(); ++i) {
        predicted_matrix(i, 0) = predicted[i].x;
        predicted_matrix(i, 1) = predicted[i].y;
    }

    for (auto &observation : observations) {
        Eigen::Vector2d observation_vector(observation.x, observation.y);
        Eigen::VectorXd squared_distances = (predicted_matrix.rowwise() - observation_vector.transpose()).rowwise().squaredNorm();

        Eigen::Index nearest_index;
        squared_distances.minCoeff(&nearest_index);

        observation.id = predicted[nearest_index].id;
    }

    // A KD Tree can be used to speedup nn search
}


/*
* Transforms a local (vehicle) observation into a global (map) coordinates observation.
* Input:
*  observation   - A single landmark observation
*  p             - A single particle
* Output:
*  local         - transformation of the observation from local coordinates to global
*/
LandmarkObs transformation(LandmarkObs observation, Particle p){
    LandmarkObs global;

    // Homogeneous transformation
    Eigen::Matrix3d T;
    T << cos(p.theta), -sin(p.theta), p.x,
         sin(p.theta),  cos(p.theta), p.y,
         0,           0,           1;

    Eigen::Vector3d local_obs{observation.x, observation.y, 1.0};
    auto global_obs = T * local_obs;

    return {observation.id, global_obs(0), global_obs(1)};
}

/*
* TODO
* This function updates the weights of each particle
* Input:
*  std_landmark   - Sensor noise
*  observations   - Sensor measurements
*  map_landmarks  - Map with the landmarks
* Output:
*  Updated particle's weight (particles[i].weight *= w)
*/
void ParticleFilter::updateWeights(
    double std_landmark[], 
	std::vector<LandmarkObs> observations,
    Map map_landmarks) {

    // Creates a vector that stores the map (this part can be improved)
    std::vector<LandmarkObs> mapLandmark;
    for(int j=0;j<map_landmarks.landmark_list.size();j++){
        mapLandmark.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i,map_landmarks.landmark_list[j].x_f,map_landmarks.landmark_list[j].y_f});
    }

    for(auto &particle : particles){
        std::vector<LandmarkObs> transformed_observations;
        for (const auto &observation : observations) {
            transformed_observations.push_back(transformation(observation, particle));
        }
        
        dataAssociation(mapLandmark, transformed_observations);
        
        particle.weight = 1.0;

        // Compute the probability
		//The particles final weight can be represented as the product of each measurement’s Multivariate-Gaussian probability density
		//We compute basically the distance between the observed landmarks and the landmarks in range from the position of the particle
        for(int k=0;k<transformed_observations.size();k++){
            double obs_x,obs_y,l_x,l_y;
            obs_x = transformed_observations[k].x;
            obs_y = transformed_observations[k].y;

            //get the associated landmark 
            for (int p = 0; p < mapLandmark.size(); p++) {
                if (transformed_observations[k].id == mapLandmark[p].id) {
                    l_x = mapLandmark[p].x;
                    l_y = mapLandmark[p].y;
                }
            }

			// How likely a set of landmarks measurements are, given a prediction state of the car 
            double w = exp( -( pow(l_x-obs_x,2)/(2*pow(std_landmark[0],2)) + pow(l_y-obs_y,2)/(2*pow(std_landmark[1],2)) ) ) / ( 2*M_PI*std_landmark[0]*std_landmark[1] );
            particles[i].weight *= w;
        }

    }    
}

/*
* TODO
* This function resamples the set of particles by repopulating the particles using the weight as metric
*/
void ParticleFilter::resample() {
    
    uniform_int_distribution<int> dist_distribution(0,num_particles-1);
    double beta  = 0.0;
    vector<double> weights;
    int index = dist_distribution(gen);
    vector<Particle> new_particles;

    for(int i=0;i<num_particles;i++)
        weights.push_back(particles[i].weight);
																
    float max_w = *max_element(weights.begin(), weights.end());
    uniform_real_distribution<double> uni_dist(0.0, max_w);

    //TODO write here the resampling technique (feel free to use the above variables)
}



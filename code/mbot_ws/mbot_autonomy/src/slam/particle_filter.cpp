#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>


static bool comparisonByWeight(mbot_lcm_msgs::particle_t a, mbot_lcm_msgs::particle_t b) {
    return a.weight > b.weight;
}

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation_(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1),
  sample_percentage_(50./100.)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose2D_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0 / kNumParticles_;  //uniform distribution
    posteriorPose_ = pose;

    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<> dist(0.0, 0.01);
    
    for(auto& p : posterior_){
        p.pose.x = posteriorPose_.x + dist(generator);
        p.pose.y = posteriorPose_.y + dist(generator);
        p.pose.theta = wrap_to_pi(posteriorPose_.theta + dist(generator));
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
    }
}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0 / kNumParticles_;
    
    for(auto& p : posterior_){
        p = randomPoseGen_.get_particle(map);
    }
}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose2D_t& odometry)
{   // given
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose2D_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{   //given
    bool RobotMovedflag = actionModel_.updateAction(odometry);

    if (RobotMovedflag)
    {           
        /// TODO: Add reinvigoration step
        // auto prior = resamplePosteriorDistribution(map);
        auto prior = resamplePosteriorDistributionAndReinvigorate(laser, map);
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}

mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose2D_t& odometry)
{   //given
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;

    return posteriorPose_;
}


mbot_lcm_msgs::pose2D_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid& map,
                                                           const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    ParticleList prior = lowVarianceSample(kNumParticles_, posterior_);
    
    return prior; 
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    ParticleList prior;
    if(keep_best){
        ParticleList sorted_posterior = posterior_;
        std::sort(sorted_posterior.begin(), sorted_posterior.end(), 
                [](const mbot_lcm_msgs::particle_t& a, const mbot_lcm_msgs::particle_t& b){return a.weight > b.weight;});
    
        int num_particle = static_cast<int>(0.4 * sorted_posterior.size());
        for(int i = 0; i < num_particle; i++){
            prior.push_back(sorted_posterior[i]);
        }
        
        ParticleList resample_particles = lowVarianceSample((kNumParticles_-num_particle), posterior_);
        prior.insert(prior.end(), resample_particles.begin(), resample_particles.end());   
    }
    else{
        prior = lowVarianceSample(kNumParticles_, posterior_);
    }

    if(reinvigorate){
        reinvigoratePriorDistribution(prior);
    }

    
    return prior;   
}


ParticleList ParticleFilter::resamplePosteriorDistributionAndReinvigorate(const mbot_lcm_msgs::lidar_t& laser, const OccupancyGrid& map,
                                                           const bool keep_best,
                                                           bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    ParticleList prior = posterior_;
    prior = lowVarianceSample(kNumParticles_, prior);

    // std::cout<<"Resampling and reinvigoration"<<std::endl;

    int total_reinvigorate = (int)kNumParticles_*quality_reinvigoration_percentage;
    std::sort(prior.begin(),prior.end(), comparisonByWeight);
    double bot_avg = 0;
    for (int i = kNumParticles_-total_reinvigorate; i < kNumParticles_; i++){
        bot_avg += prior[i].weight;
    }
    bot_avg/=total_reinvigorate;
    
    double top_avg = 0;
    int num_best = (int)prior.size()*sample_percentage_;
    for (int j = 0; j < num_best; j++){
        top_avg += prior[j].weight;
    }
    top_avg/=num_best;

    double dist_var;
    double ang_var;
    int reinvigorate_start;
    int reinvigorate_end;

    //reuse poorer particles
    if (bot_avg/prior[0].weight<0.25){
        std::cout<<"reinvigorating: reuse garbage particles\n"<<std::endl;
        dist_var = 0.07;
        ang_var = 0.0872665;
        reinvigorate_start = kNumParticles_-total_reinvigorate;
        reinvigorate_end = kNumParticles_;
        reinvigorate = true;
    }
    //homogenous exploration
    else if (bot_avg/prior[0].weight > 0.95 ){
        std::cout<<"reinvigorating: homogenous exploration\n"<<std::endl;
        dist_var = 0.05;
        ang_var = 0.0872665;
        reinvigorate_start = kNumParticles_*6.0/10.0;
        reinvigorate_end = kNumParticles_;
        reinvigorate = true;
    }

    //reuse poorer particles|| homogenous exploration || jerk_filter
    if (reinvigorate){
            std::random_device rd;
            std::mt19937 e2(rd());
            std::normal_distribution<> dist(0, dist_var);
            std::normal_distribution<> angle_dist(0, ang_var);

            for (int i = reinvigorate_start; i < reinvigorate_end; i++)
            {
                // posterior_[i] = randomPoseGen_.get_particle();
                prior[i].pose.x = posteriorPose_.x+dist(e2);
                prior[i].pose.y = posteriorPose_.y+dist(e2);
                prior[i].pose.theta = posteriorPose_.theta+angle_dist(e2);
                // printf("e2: %f x:%f y%f\n", (dist(e2)), posterior_[i].pose.x ,posterior_[i].pose.y);
                // posterior_[i].weight = 1.0/kNumParticles_;
                prior[i].weight = sensorModel_.likelihood(prior[i], laser, map);
            }

        }
        
    prior = lowVarianceSample(kNumParticles_, prior);
        // for (auto i : posterior_){
        //         std::cout<<i.weight<<" ";
        //     }
        // std::cout<<std::endl<<std::endl;

    // }
    return prior;
}


void ParticleFilter::reinvigoratePriorDistribution(ParticleList& prior)
{
    // Augmentation: if sensor model suspects an average particle quality of
    //      less than 15%, invigorate
    distribution_quality /= kNumParticles_;

    if (distribution_quality < 0.15)  // TODO: make 0.15 a parameter
    {
        int count = 0;
        int max_count = floor(quality_reinvigoration_percentage * prior.size());

        std::random_device rd;
        std::default_random_engine generator(rd());
        auto ud01 = std::uniform_real_distribution<double>(0.0, 1.0);
        int step = std::max<int>(1, floor(ud01(generator) * prior.size() / max_count));

        for (int i = 0; i < max_count; i++)
        {
            prior[i*step] = randomPoseGen_.get_particle();
        }

    }

    // // Augmentation: randomize any unreasonable samples
    // if(map != nullptr)
    // {
    //     for (int i = 0; i < prior.size(); i++)
    //     {
    //         const auto& p = prior[i].pose;
    //         if(!map->isCellInGrid(p.x, p.y) ||
    //           (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0)))
    //         {
    //             std::cout << "\tinvalid sample!!" << ", "
    //                 << !map->isCellInGrid(p.x, p.y) << ", "
    //                 << (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0.0))
    //                 << std::endl;


    //             prior[i] = randomPoseGen_.get_particle();
    //         }
    //     }
    // }
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;
    for(auto& p: prior){
        proposal.push_back(actionModel_.applyAction(p));
    }
    
    return proposal;
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    
    ParticleList posterior;
    double sumWeights = 0.0;

    for(auto& p: proposal){
        mbot_lcm_msgs::particle_t weighted = p;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        sumWeights += weighted.weight;
        posterior.push_back(weighted);
    }

    for(auto& p: posterior){
        p.weight /= sumWeights;
    }
    
    return posterior;    
}


mbot_lcm_msgs::pose2D_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    // Figure out which pose to take for the posterior pose
    // Weighted average is simple, but could be very bad
    // Maybe only take the best x% and then average.


    mbot_lcm_msgs::pose2D_t pose;
    
    ParticleList posterior_sorted = posterior;
    std::sort(posterior_sorted.begin(), posterior_sorted.end(), 
                [](const mbot_lcm_msgs::particle_t& a, const mbot_lcm_msgs::particle_t& b){return a.weight > b.weight;});
    
    int num_particle = static_cast<int>(0.2 * posterior_sorted.size());
    ParticleList selected_particles(posterior_sorted.begin(), posterior_sorted.begin()+num_particle);

    pose = computeParticlesAverage(selected_particles);

    return pose;

}

mbot_lcm_msgs::pose2D_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    mbot_lcm_msgs::pose2D_t avg_pose;
    avg_pose.x = 0.0;
    avg_pose.y = 0.0;
    avg_pose.theta = 0.0;
    double sum_weight = 0.0;

    // Aux variables to compute theta average
    double theta_x = 0.0;
    double theta_y = 0.0;
    for (auto &&p : particles_to_average)
    {
        avg_pose.x += p.weight * p.pose.x;
        avg_pose.y += p.weight * p.pose.y;
        theta_x += p.weight * std::cos(p.pose.theta);
        theta_y += p.weight * std::sin(p.pose.theta);

        sum_weight += p.weight;
    }
    avg_pose.x /= sum_weight;
    avg_pose.y /= sum_weight;
    theta_x /= sum_weight;
    theta_y /= sum_weight;
    avg_pose.theta = std::atan2(theta_y, theta_x);

    return avg_pose;

    std::cout << "avg_pose: " << avg_pose.x << " " << avg_pose.y << " " << avg_pose.theta << std::endl;

}

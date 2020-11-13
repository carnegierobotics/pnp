#pragma once
/** \file    parameters.h
 *
 * \brief    This header contains parameter set for the pnp methods
 *
 * \remark
 *  - header only
 *  - c++11
 *  - no dependencies
 * \todo
 *  - improve structure
 *
 * The purpose of sharing one set of parameters for all pnp ransac variants is to reduce the amount of ifdeffing required to include the ip code
 *
 * \author   Mikael Persson
 * \date     2015-10-01
 * \note MIT licence
 *
 *
 *
 ******************************************************************************/

#include <cmath>


namespace cvl{

/**
 * @brief The PNPParams class
 * common PNP ransac parameters
 */
class PnpParams {
public:

    PnpParams(double threshold=0.001 /*=pixel_threshold/K(0,0), approx 1-3 pixels*/):threshold(threshold){}


    //  parameters
    /// initial inlier ratio estimate, good value = expected inlier ratio/2
    /// should really be estimated online.
    /// double inlier_ratio=0.25;

    /// minimum probability for never findning a single full inlier set
    double min_probability=0.99999; // effectively capped by the max_iterations too...

    /// pixeldist/focallength
    double threshold=0;

    /// perform maximum likelihood sampling consensus,
    //bool MLESAC=false;// not supported at this time... its not better in any of my cases anyways...

    /// break early in the ransac loop if a sufficiently good solution is found
    bool early_exit=false;
    uint early_exit_min_iterations =5;
    double early_exit_inlier_ratio=0.8;

    /// if random eval is used, this is the number of samples drawn
    //uint max_support_eval=500; not used...


    /// a maximum number of iterations to perform to limit the amount of time spent
    uint max_iterations=1000;
    /// the minimum number of iterations to perform to ensure the required number of iterations is correctly computed
    uint min_iterations=100;





    /**
     * @brief get_iterations
     * @param estimated_inliers, start with a conservative guess, say expected_inliers/2.0
     * @param p_inlier_given_noise_and_gt
     * @return
     */
    int get_iterations(double confidence, double err_prob, int modelpoints, int maxIters){
        err_prob = std::max(err_prob, 0.);
        err_prob = std::min(err_prob, 1.);

        double num = std::max(1.0 - confidence, __DBL_MIN__);
        double denom = 1.0 - std::pow(1.0 - err_prob, modelpoints);
        if(denom < __DBL_MIN__){
            return 0;
        }
        num = std::log(num);
        denom = std::log(denom);

        return denom >= 0 || -num >= maxIters*(-denom) ? maxIters : round(num/denom);
    }


};



}

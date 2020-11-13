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

PnpParams(double threshold, double min_probability, uint max_iterations):
            threshold(threshold), min_probability(min_probability), max_iterations(max_iterations){}

double min_probability;
double threshold;
uint max_iterations;

/**
* @brief get_iterations
* @param confidence probability of returning a valid result
* @param err_prob estimated portion of points that are outliers
* @param modelpoints min number of correspondances needed to construct a pose
* @param maxIters maximum number of iterations
* @return the number of iterations to run RANSAC
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

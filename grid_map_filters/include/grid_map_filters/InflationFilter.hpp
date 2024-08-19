/*
 * InflationFilter.hpp
 *
 *  Created on: August 19, 2024
 *      Author: Kean Hao
 */

#ifndef GRID_MAP_FILTERS__INFLATIONFILTER_HPP_
#define GRID_MAP_FILTERS__INFLATIONFILTER_HPP_

#include <filters/filter_base.hpp>

#include <vector>
#include <string>

namespace grid_map
{

/*!
 * Filter class to apply exponential decay to the values inside a radius.
 */
template<typename T>
class InflationFilter : public filters::FilterBase<T>
{
public:
  /*!
   * Constructor
   */
  InflationFilter();

  /*!
   * Destructor.
   */
  virtual ~InflationFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  bool configure() override;

  /*!
   * Computes for each value in the input layer the exponentially weighted mean of all values in a radius around it
   * Saves this result in an additional output layer.
   * @param mapIn grid map containing the input layer.
   * @param mapOut grid map containing the layers of the input map and the new layer.
   */
  bool update(const T & mapIn, T & mapOut) override;

private:
  //! Radius to apply the decay.
  double radius_;

  //! Force field decay parameter.
  double decay_;

  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;
};

}  // namespace grid_map
#endif  // GRID_MAP_FILTERS__INFLATIONFILTER_HPP_

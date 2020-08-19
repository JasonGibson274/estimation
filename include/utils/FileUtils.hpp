//
// Created by jason on 8/18/20.
//

#ifndef STATEESTIMATOR_FILEUTILS_HPP
#define STATEESTIMATOR_FILEUTILS_HPP

#include <math.h>
#include <map>
#include <string>
#include <vector>
#include <iomanip>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace estimation_utils {
  template <class T1, class T2 = T1>
  inline T2 get(const std::string &key, YAML::Node map, T2 default_val) {
    if(!map[key]) {
      std::cout << "on parameter = " << key << " using default value" << std::endl;
    }
    return map[key] ? map[key].as<T2>() : default_val;
  }
}

#endif //STATEESTIMATOR_FILEUTILS_HPP

#ifndef CIUDAD_H
#define CIUDAD_H
#include "string"
#include <list>
#include <unordered_map>
#include <vector>

struct City{
  std::string name;
  std::string city;
  std::string country;
  std::string airport_id;
  std::vector<std::string> destinations;
  double longitude;
  double latitude;
};
#endif 

#include <limits>

#include "simple_stat.h"

SimpleStat::SimpleStat ()
{
  sigma_x = 0.0;
  sigma_x2 = 0.0;
  n = 0;
  min_x = std::numeric_limits<double>::max();
  max_x = std::numeric_limits<double>::min();
  mean = 0.0;
  std_deviation = 0.0;
}

SimpleStat::~SimpleStat ()
{

}

void SimpleStat::add_data (double data)
{
  sigma_x += data;
  sigma_x2 += data*data;
  n++;
  mean = sigma_x/n;
  if (data < min_x)
    min_x = data;
  if (data > max_x)
    max_x = data;
}


#ifndef _SIMPLE_STAT_
#define _SIMPLE_STAT_

class SimpleStat
{

  public:
    SimpleStat ();
    ~SimpleStat ();

    void add_data (double data);

    double sigma_x;
    double sigma_x2;
    int n;
    double min_x;
    double max_x;
    double mean;
    double std_deviation;

};

#endif

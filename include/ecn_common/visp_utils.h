#ifndef vispUtils_H
#define vispUtils_H
#include <visp/vpSubMatrix.h>
#include <visp/vpSubColVector.h>


namespace ecn
{

// Utility functions for varying weights
double weight(double s, double s_act, double s_max)
{
    if(s < s_act)
        return 0;
    return (s-s_act)/(s_max-s);
}

double weightBothSigns(double s, double s_act, double s_max)
{
    return weight(s, s_act, s_max) + weight(-s, s_act, s_max);
}



// ViSP vpMatrix utilities to put a matrix inside another
void putAt(vpMatrix &_J, const vpMatrix &_Jsub, const unsigned int r, const unsigned int c)
{
    vpSubMatrix Js(_J, r, c, _Jsub.getRows(), _Jsub.getCols());
    Js = _Jsub;
}

// put a vector inside another
void putAt(vpColVector &_e, const vpColVector &_esub, const unsigned int r)
{
    vpSubColVector es(_e, r, _esub.getRows());
    es = _esub;
}

}


#endif // vispUtils_H

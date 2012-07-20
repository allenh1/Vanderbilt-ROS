#include <iostream>
#include <math.h>

using namespace std;
const double N = 1000;

double f(double x){ return pow(x, 2); }//f(x) = x^2

double integrate(double lower, double upper)
{
    double x = (upper - lower) / N;
    double c = lower + x;
    double area = 0;

    for(double i = lower; i < upper; i += x)
    {
        c += x;
        area += f(c) * x;
    }

    return area;
}//function for integrating.

int main()
{
    double upper, lower;

    cout<<"f(x) = x^2 \n";
    cout<<"Please enter the lower bound: ";
    cin>>lower;
    cout<<"\nPlease enter the upper bound: ";
    cin>>upper;

    double integrated = integrate(lower, upper);
    double actual = pow(upper, 3)/3 - pow(lower, 3)/3;

    cout<<"\nReturned by integrator: "<<integrated;
    cout<<"\nActual value of the integrand: "<<actual;
}

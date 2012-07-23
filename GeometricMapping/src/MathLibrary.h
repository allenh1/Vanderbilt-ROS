/** This file holds all the math methods that the Geometric Mapping utilizes. 
  * It holds functions for average value, stdDev, etc. 
**/
#include<math.h>
#include<qt4/QtCore/QList>
#include<map>
#include<iostream>
#include<qt4/QtCore/QList>
#include<sstream>
#include<fstream>

#define Z 1
#define getMin(a, b) a < b ? a:b
#define lessThan(a, b) a<b?1:0
#define distance(a, b, c, d) sqrt(pow(b - a, 2) + pow(d - c, 2))
#define PI 3.1415926542

inline double getDistance(pcl::PointXYZ a, pcl::PointXYZ b)
{
    return distance(a.x, b.x, a.y, b.y);
}//get distance between two points.

inline int getMindex(QList<double> list)
{
    int mindex = 0;

    for (int x = 1; x < list.size(); x++)
        if (list.at(x) < list.at(mindex))
            mindex = x;

    return mindex;
}//get the min value

inline double getSlope(pcl::PointXYZ a, pcl::PointXYZ b)
{ return (a.y + b.y) / (a.x + b.x); }

inline double getPerp(double m)
{ return ( - 1.0) / (m); }

inline double getAverage(int a, int b, QList<double> list)
{
    double sum = 0;

    for (int x = a; x < b; ++x)
        sum += list.at(x);

    return sum / (b - a);
}//get the average of the above.

inline double reformat(double x)
{
	int temp = x * 10000;
	return (double) temp / 10000.0;
}//truncate decimal to 4 places. 

inline double getStdDev(QList<double> list)
{
	double mean = getAverage(0, list.size() - 1, list);
	double sum  = 0; 

    for (int x = 0; x < list.size(); x++)
		sum += pow(list.at(x) - mean, 2);
	
	return sqrt(sum / (list.size() - 1));
}//get the standard deviation of a list. 

void outputData(QList<double> data, QList<double> data2)
{
	std::ofstream dataOut;
	dataOut.open("runData.txt");
    dataOut << "Scores: \n" ;

    for (int x = 0; x < data.size(); x++)
	{ 
		std::ostringstream stm;
		stm << reformat(data.at(x));

		if (x != 0 && x % 9 == 0)
			dataOut<<"\n";
		dataOut<<stm.str();
		dataOut<<"\t";
	}//store the data 
    dataOut<<"\n";
    dataOut<<"------------------------------------------------------------------\n";
    dataOut<<"Angles (in degrees): \n";

    for (int y = 0; y < data2.size(); y++)
    {
        std::ostringstream stm;
        stm << reformat(data2.at(y));

        if (y != 0 && y % 9 == 0)
            dataOut<<"\n";
        dataOut<<stm.str();
        dataOut<<"\t";
    }//store angle difference
    dataOut<<"\n\n";

	std::ostringstream stm;
	stm << getAverage(0, data.size() - 1, data);
	dataOut<<"Average: ";
	dataOut<<stm.str();
	std::ostringstream stm2;
	stm2<<pow(getStdDev(data), 2);
	dataOut<<"\nVariance: ";
	dataOut<<stm2.str();
	std::ostringstream stm3;
	dataOut<<"\nStandard Deviation: ";
	stm3 << getStdDev(data); 
	dataOut<<stm3.str(); 
}//output a run's data. 

#include "Shape.h"
namespace data_server {


pcl::PointXYZ toPCLPoint(geometry_msgs::Point32 in)
{
    pcl::PointXYZ out;
    out.x = in.x;
    out.y = in.y;
    out.z = Z;

    return out;
}//converts the input geometry messages point to a PCL point.

Shape::Shape(PointCloud object)
{
    shapeType = ORIG;//the default: unmodified pointcloud
    uncorrected = object;
    const int last = uncorrected.points.size() - 1;
    firstX = uncorrected.points.at(0).x;
    finalX = uncorrected.points.at(uncorrected.points.size() - 1).x;

    phiBound = acos(sqrt(pow(uncorrected.points.at(0).x, 2) + pow(uncorrected.points.at(0).y, 2)) /
                    sqrt(pow(uncorrected.points.at(last).x, 2) + pow(uncorrected.points.at(last).y, 2)));

    correct_circle();
    correct_Bezier();
    correct_segment();
}

double Shape::bestMatch()
{     
    if (match_circle() > match_segment() && match_circle() > match_bezier())
    {   shapeType = CIRCLE; return match_circle(); }
    else if (match_segment() > match_circle() && match_segment() > match_bezier())
    {   shapeType = SEGMENT; return match_segment(); }
    else
    {   shapeType = BEZIER; return match_bezier(); }
}//end double

const SHAPE_TYPE & Shape::getType()
{
    return shapeType;
}//return a constant reference to the shape.

double Shape::getAverageSlope()
{
    QList<double> slopes;

    if (uncorrected.points.size() == 1)
        return 0;
    for (unsigned int x = 0; x < uncorrected.points.size() - 1; x++)
    {
        double ydiff = uncorrected.points.at(x + 1).y - uncorrected.points.at(x).y;
        double xdiff = uncorrected.points.at(x + 1).x - uncorrected.points.at(x).x;

        if (xdiff != 0)
            slopes.push_back(ydiff / xdiff);
    }//end for x.

    return getAverage(0, slopes.size() - 1, slopes);
}//get the average slope. 

void Shape::setSlope(double x)
{
    /**
      * When we set the slope to another number, we need
      * to correct the y-intercept (b) parameter as well!
     **/

    //y = mx + b -> y - mx = b
    double py = correctedSegment.points.at(0).y;
    double px = correctedSegment.points.at(0).x;

    S_m = x;
    S_b = py - S_m * px;
    updateSegment();
}//update the slope!

PointCloud Shape::getCorrections()
{
    //ROS_INFO("C: %i \t S: %i \t N: %i, B: %i", circles, segments, unchanged, curves);
    /*if ((bestMatch() < R2TOLL) ||
            (bestMatch() > R2TOLL2))
    {
        //unchanged++;
        return uncorrected; //if not a shape, don't make one!
    } */

    if (is_circle() && C_r < MAXRAD && C_r > MINRAD)
    {
        //circles++;
        return correctedCircle;
    }

    if (is_segment())
    {
        //segments++;
        return correctedSegment;
    }//end else

    else
    {
        //curves++;
        return correctedCurve;
    }
}//get the correct shape for the cloud handler.

void Shape::correct_circle()
{
    QList<pcl::PointXYZ> newPoints;

    QList<double> yVals;

    for (unsigned int x = 0; x < uncorrected.points.size(); x++)
        yVals.push_back(uncorrected.points.at(x).y);

    unsigned int min = getMindex(yVals);

    float a = uncorrected.points.front().x;
    float b = uncorrected.points.front().y;
    float c = uncorrected.points.at(min).x;
    float d = uncorrected.points.at(min).y;
    float e = uncorrected.points.back().x;
    float f = uncorrected.points.back().y;

    if (min == 0 || min == uncorrected.points.size() - 1)
    {
        //ROS_INFO("WARNING: Middle point is undefined.");
        c = (a + e) / 2.0;
        d = (b + f) / 2.0;
    }//fi

    //get three points from the scan.

    if ((b * (e - c)+ d * (a - e) + f*(c - a)) == 0)
    {
        while ((b * (e - c)+ d * (a - e) + f*(c - a)) == 0 && min < uncorrected.points.size())
        {
            //ROS_INFO("WARNING: NEW POINT BEING SELECTED");
            c = uncorrected.points.at(min).x;
            d = uncorrected.points.at(min).y;
            min++;
        }//end while
    }//end if

    float h = 0;
    float k = 0;

    for (unsigned int i = 0; i < uncorrected.points.size(); i++)
    {
        k += uncorrected.points.at(i).y;
        h += uncorrected.points.at(i).x;
    }//end for

    h /= uncorrected.points.size();
    k /= uncorrected.points.size();

    C_k = k;
    C_h = h;

    float r = sqrt(pow((a - h),2) + pow((b - k),2));
    C_r = r;
    
    for (unsigned int iter = 0; iter < uncorrected.points.size(); ++iter)
    {
        float x = uncorrected.points.at(iter).x;
        float y = uncorrected.points.at(iter).y;

        float possible_a = k + sqrt(pow(r, 2) - pow(x, 2) + 2 * h * x - pow(h, 2));
        float possible_b = k - sqrt(pow(r, 2) - pow(x, 2) + 2 * h * x - pow(h, 2));

        float a = abs(y - possible_a);
        float b = abs(y - possible_b);
        float radical = sqrt(pow(r, 2) - pow(x, 2) + 2 * h * x - pow(h, 2));

        if (a > b)
            radical *= -1;
        pcl::PointXYZ thePoint;
        thePoint.x = x; thePoint.y = k + radical; thePoint.z = Z;
        
        if (sqrt(pow(thePoint.y - uncorrected.points.at(iter).y, 2)) >= DIST_TOL)
            thePoint.y = uncorrected.points.at(iter).y;
        newPoints.push_back(thePoint);
    }//end for

    pcl::PointCloud<pcl::PointXYZ> circlized;
    circlized.header.frame_id = "/cloud";
    circlized.header.stamp = ros::Time::now();

    for (int x = 0; x < newPoints.size(); ++x)
        circlized.points.push_back(newPoints.at(x));

    correctedCircle = circlized;
}//correct scan for a circle case 

inline double factorial(int n)
{
    if (n == 0)
        return 1;
    return n * factorial(n - 1);
}//end double

inline double combination(int n, int i)
{
    double num = factorial(n);
    double den = factorial(i) * factorial(n - i);
    return num / den;
}

void Shape::correct_Bezier()
{
    /** Correct the curve as a Bezier polynomial **/
    QList<pcl::PointXYZ> newPoints;

    int n = uncorrected.points.size() - 1 / 5;
    //const int n = uncorrected.points.size() - 1;

    /** Coefficients stored in a list.
      B(t) = coefficients.at(i)*(1 - t)^(n - i)*t^(i)P(i)

        Here, we use a polonomial of degree points / 5, or all points.
    **/

    for (double t = 0; t < 1.0; t += 1.0 / (uncorrected.points.size() - 1))
    {
        double px = uncorrected.points.at(0).x;
        double py = uncorrected.points.at(0).y;

        for (int i = 0; i < n; i++)
        {
            double nCr = combination(n, i);
            double mult1 = pow (1 - t, n - i);
            double mult2 = pow(t, i);

            double scalar = nCr * mult1 * mult2;

            px += uncorrected.points.at(i).x * scalar;
            py += uncorrected.points.at(i).y * scalar;
        }//end for x

        pcl::PointXYZ toPush;

        unsigned int iter = pow(t, -1);

        if (iter >= uncorrected.points.size())
            iter = uncorrected.points.size() - 1;

        if (sqrt( pow(px - uncorrected.points.at(iter).x, 2) + pow(py - uncorrected.points.at(iter).y, 2)) > DIST_TOL)
        {	px = uncorrected.points.at(iter).x; py = uncorrected.points.at(iter).y; }

        toPush.x = px;
        toPush.y = py;
        newPoints.push_back(toPush);
    }//end for x.

    PointCloud Curved;
    Curved.header.frame_id = "/cloud";
    Curved.header.stamp = ros::Time::now();


    for (int x = 0; x < newPoints.size(); ++x)
        Curved.points.push_back(newPoints.at(x));

    correctedCurve = Curved;
}//correct in a bezier style.

void Shape::correct_segment()
{
    QList<pcl::PointXYZ> newPoints;

    /** Now we have the max and the min on the interval above! **/
    //y = mx + b -> y - mx = b
    double x_S = 0; double y_S = 0;
    double x_SS = 0; double y_SS = 0; double xy_S = 0;
    const double n = uncorrected.points.size();

    const double T = xy_S * n - x_S * y_S;
    const double N1 = x_SS * n - pow(x_S, 2);
    const double N2 = y_SS * n - pow(y_S, 2);

    for (unsigned int x = 0; x < uncorrected.points.size(); x++)
    {
        x_S += uncorrected.points.at(x).x;
        x_SS += pow(uncorrected.points.at(x).x, 2);
        y_S += uncorrected.points.at(x).y;
        y_SS += pow(uncorrected.points.at(x).y, 2);
        xy_S += uncorrected.points.at(x).x * uncorrected.points.at(x).y;
    } //get the sums we need!

    double M = T / N1;
    double q = (y_S - M * x_S) / n;
    double S = T / N2;
    double b = (x_S - S * y_S) / n;

    if (N1 > N2)
    {
        for (unsigned int iter = 0; iter < uncorrected.points.size(); ++iter)
        {
            float px; float py;

            px = uncorrected.points.at(iter).x;
            py = M * px + q;

            pcl::PointXYZ toPush;
            toPush.x = px; toPush.y = py; toPush.z = Z;

            newPoints.push_back(toPush);
        }//end for.
        S_m = M;
        S_b = q;
    }//end if.

    else
    {
        for (unsigned int iter = 0; iter < uncorrected.points.size(); ++iter)
        {
            float px; float py;

            py = uncorrected.points.at(iter).y;
            px = S * py + b;

            pcl::PointXYZ toPush;

            toPush.x = px; toPush.y = py; toPush.z = Z;

            newPoints.push_back(toPush);
        }//end for.

        S_m = S;
        S_b = b;
    }
    pcl::PointXYZ orgin;
    orgin.x = 0; orgin.y = 0; orgin.z = 0;

    PointCloud Segmetized;
    Segmetized.header.frame_id = "/cloud";
    Segmetized.header.stamp = ros::Time::now();


    for (int x = 0; x < newPoints.size(); ++x)
        Segmetized.points.push_back(newPoints.at(x));

    correctedSegment = Segmetized;
}//correct as a segment

void Shape::updateSegment()
{
    QList<pcl::PointXYZ> newPoints;

    for (unsigned int iter = 0; iter < uncorrected.points.size(); ++iter)
    {
        float px = correctedSegment.points.at(iter).x;
        float py = S_m * px + S_b;


        if (sqrt( pow(px - uncorrected.points.at(iter).x, 2) + pow(py - uncorrected.points.at(iter).y, 2)) > DIST_TOL)
        {	px = uncorrected.points.at(iter).x; py = uncorrected.points.at(iter).y; }

        pcl::PointXYZ toPush;
        toPush.x = px; toPush.y = py; toPush.z = Z;
        //Push.rgb = 0xda70d6;
        newPoints.push_back(toPush);
    }//end for.

    PointCloud Segmetized;
    Segmetized.header.frame_id = "/cloud";
    Segmetized.header.stamp = ros::Time::now();


    for (int x = 0; x < newPoints.size(); ++x)
        Segmetized.points.push_back(newPoints.at(x));
}

double Shape::meanY()
{
    double sum = 0;

    for (unsigned int x = 0; x < uncorrected.points.size(); x++)
        sum += uncorrected.points.at(x).y;

    return sum / uncorrected.points.size();
}//get average y value

double Shape::match_circle()
{
    double sum1 = 0;
    double sum2 = 0;
    double yMean = meanY();

    for (unsigned int x = 0; x < uncorrected.points.size(); x++)
        sum1 += pow(uncorrected.points.at(x).y - yMean, 2);

    for (unsigned int y = 0; y < correctedCircle.points.size(); y++)
        sum2 += pow(correctedCircle.points.at(y).y - yMean, 2);

    return 1 - (sum1 / sum2);
}//r^2 value for the circle

double Shape::match_segment()
{
    double sum1 = 0;
    double sum2 = 0;
    double yMean = meanY();

    for (unsigned int i = 0; i < correctedSegment.points.size(); i++)
    {
        const double dist = getDistance(uncorrected.points.at(i), correctedSegment.points.at(i));
        if (dist > max_correct)
            max_correct = dist;
    }//end for i.

    for (unsigned int x = 0; x < uncorrected.points.size(); x++)
        sum1 += pow(uncorrected.points.at(x).y - yMean, 2);

    for (unsigned int y = 0; y < correctedSegment.points.size(); y++)
        sum2 += pow(correctedSegment.points.at(y).y - yMean, 2);

    return 1 - (sum1 / sum2);
}//match for segment

double Shape::match_bezier()
{
    double sum1 = 0;
    double sum2 = 0;
    double yMean = meanY();

    for (unsigned int x = 0; x < uncorrected.points.size(); x++)
        sum1 += pow(uncorrected.points.at(x).y - yMean, 2);

    for (unsigned int y = 0; y < correctedCurve.points.size(); y++)
        sum2 += pow(correctedCurve.points.at(y).y - yMean, 2);

    return 1 - (sum1 / sum2);
}

bool Shape::is_circle()
{ return match_circle() >= match_segment() && match_circle() >= match_bezier(); }

bool Shape::is_segment()
{ return match_segment() > match_circle() && match_segment() > match_bezier(); }

bool Shape::is_bezier()
{ return !is_circle() && !is_segment(); }

double Shape::getSlope()
{ return S_m; }//get the slope

}//end namespace

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <Eigen/Geometry>
#include <math.h>
#include <fstream>
#include <sstream>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <dirent.h>
#include <glob.h>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <stdio.h>
#include <cstdlib>
#include <vector>

using namespace std;

const double PI  = 3.141592653589793238463;

bool arrow = false;
bool cylinder = false;
float cylinderX = 0.0;
float cylinderY = 0.0;
float cylinderZ = 0.0;
float arrowX = 1.0;
float arrowY = 0.0;
float arrowZ = 0.125;
float angle = 0.0; // measured in degrees from the positive y axis


///////////////////////////////////// CHRIS'S ANGLES CODE ///////////////////////////////

vector<double> globalPan(vector<double> cameraCenter, vector<double> focal, vector<double> view_up, double focalLength, double angle_in)
{
    // Camera ceter
  double x0 = cameraCenter[0];
  double y0 = cameraCenter[1];
  double z0 = cameraCenter[2];

    // Focal point
  double x1 = focal[0];
  double y1 = focal[1];
  double z1 = focal[2];

  // cerr << "Focal x is: " << x1 << endl;
  // cerr << "Focal y is: " << y1 << endl;
  // cerr << "Focal z is: " << z1 << endl;

    // Distance vector between focal point and top of fulstrum
    double x_up = view_up[0];
    double y_up = view_up[1];
    double z_up = view_up[2];

    // cerr << "View up x is: " << x_up << endl;
    // cerr << "View up y is: " << y_up << endl;
    // cerr << "View up z is: " << z_up << endl;
    // cerr << endl;

  // (x, y, z) are new focal point. y stays the same for pan.
    double x;
    double y;
    double z;

    // (x_top, y_top, z_top) is the top of the fulstrum (found by following view_up vector starting at focal point)
    double x_top = x1 + x_up;
    double y_top = y1 + y_up;
    double z_top = z1 + z_up;

    // Position vector
    double x_hat = x1 - x0;
    double y_hat = y1 - y0;
    double z_hat = z1 - z0;

    double theta = atan2(x_hat, z_hat);
    theta += angle_in;

    // x = rcos(theta)
    // z = rsin(theta)
    x_hat = focalLength * cos(theta);
    z_hat = focalLength * sin(theta);

    x = x0 + x_hat;
    y = y0 + y_hat;
    z = z0 + z_hat;


    // Change view up vector

    // Position vector for (x_top, y_top, z_top)
    double x_topHat = x_top - x0;
    double y_topHat = y_top - y0;
    double z_topHat = z_top - z0;

    // Euclidean distance between (x_top, y_top, z_top) and camera center
    double dist = sqrt( pow((x_top - x0), 2) + pow((y_top - y0), 2) + pow((z_top + z0), 2) );

    double t_theta = atan2(x_topHat, z_topHat);
    t_theta += angle_in;

    x_topHat = dist * cos(t_theta);
    z_topHat = dist * sin(t_theta);

    x_top = x0 + x_topHat;
    y_top = y0 + y_topHat;
    z_top = z0 + z_topHat;

    //x_up = x_top - x;
    //y_up = y_top - y;
    //x_up = x_top - z;

    vector<double> result(6);
    result[0] = x;
    result[1] = y;
    result[2] = z;
    result[3] = x_up;
    result[4] = y_up;
    result[5] = z_up;

    // cerr << "New Focal x is: " << x << endl;
    // cerr << "New Focal y is: " << y << endl;
    // cerr << "New Focal z is: " << z << endl;

    // cerr << "New View up x is: " << x_up << endl;
    // cerr << "New View up y is: " << y_up << endl;
    // cerr << "New View up z is: " << z_up << endl;
    // cerr << endl;

    return result;
}

vector<double> globalTilt(vector<double> cameraCenter, vector<double> focal, vector<double> view_up, double focalLength, double angle_in)
{
    // Camera ceter
    double x0 = cameraCenter[0];
    double y0 = cameraCenter[1];
    double z0 = cameraCenter[2];

    // Focal point
    double x1 = focal[0];
    double y1 = focal[1];
    double z1 = focal[2];

    // cerr << "Focal x is: " << x1 << endl;
    // cerr << "Focal y is: " << y1 << endl;
    // cerr << "Focal z is: " << z1 << endl;

    // Distance vector between focal point and top of fulstrum
    double x_up = view_up[0];
    double y_up = view_up[1];
    double z_up = view_up[2];

    // cerr << "View up x is: " << x_up << endl;
    // cerr << "View up y is: " << y_up << endl;
    // cerr << "View up z is: " << z_up << endl;
    // cerr << endl;

    // (x, y, z) are new focal point. y stays the same for pan.
    double x;
    double y;
    double z;

    // (x_top, y_top, z_top) is the top of the fulstrum (found by following view_up vector starting at focal point)
    double x_top = x1 + x_up;
    double y_top = y1 + y_up;
    double z_top = z1 + z_up;

    // Position vector
    double x_hat = x1 - x0;
    double y_hat = y1 - y0;
    double z_hat = z1 - z0;

    double theta = atan2(x_hat, y_hat);
    theta += angle_in;

    // x = rcos(theta)
    // y = rsin(theta)
    x_hat = focalLength * cos(theta);
    y_hat = focalLength * sin(theta);

    x = x0 + x_hat;
    y = y0 + y_hat;
    z = z0 + z_hat;


    // Change view up vector

    // Position vector for (x_top, y_top, z_top)
    double x_topHat = x_top - x0;
    double y_topHat = y_top - y0;
    double z_topHat = z_top - z0;

    // Euclidean distance between (x_top, y_top, z_top) and camera center
    double dist = sqrt( pow((x_top - x0), 2) + pow((y_top - y0), 2) + pow((z_top + z0), 2) );

    double t_theta = atan2(x_topHat, y_topHat);
    t_theta += angle_in;

    x_topHat = dist * cos(t_theta);
    y_topHat = dist * sin(t_theta);

    x_top = x0 + x_topHat;
    y_top = y0 + y_topHat;
    z_top = z0 + z_topHat;

    //x_up = x_top - x;
    //y_up = y_top - y;
    //x_up = x_top - z;

    vector<double> result(6);
    result[0] = x;
    result[1] = y;
    result[2] = z;
    result[3] = x_up;
    result[4] = y_up;
    result[5] = z_up;

    // cerr << "New Focal x is: " << x << endl;
    // cerr << "New Focal y is: " << y << endl;
    // cerr << "New Focal z is: " << z << endl;

    // cerr << "New View up x is: " << x_up << endl;
    // cerr << "New View up y is: " << y_up << endl;
    // cerr << "New View up z is: " << z_up << endl;
    // cerr << endl;

    return result;
}

vector<double> globalRoll(vector<double> cameraCenter, vector<double> focal, vector<double> view_up, double focalLength, double angle_in)
{
  // Camera ceter
    double x0 = cameraCenter[0];
    double y0 = cameraCenter[1];
    double z0 = cameraCenter[2];

    // Focal point
    double x1 = focal[0];
    double y1 = focal[1];
    double z1 = focal[2];

    // cerr << "Focal x is: " << x1 << endl;
    // cerr << "Focal y is: " << y1 << endl;
    // cerr << "Focal z is: " << z1 << endl;

    // Distance vector between focal point and top of fulstrum
    double x_up = view_up[0];
    double y_up = view_up[1];
    double z_up = view_up[2];

    // cerr << "View up x is: " << x_up << endl;
    // cerr << "View up y is: " << y_up << endl;
    // cerr << "View up z is: " << z_up << endl;
    // cerr << endl;

    // (x, y, z) are new focal point. y stays the same for pan.
    double x;
    double y;
    double z;

    // (x_top, y_top, z_top) is the top of the fulstrum (found by following view_up vector starting at focal point)
    double x_top = x1 + x_up;
    double y_top = y1 + y_up;
    double z_top = z1 + z_up;

    // Position vector
    double x_hat = x1 - x0;
    double y_hat = y1 - y0;
    double z_hat = z1 - z0;

    double theta = atan2(y_hat, z_hat);
    theta += angle_in;

    // x = rcos(theta)
    // y = rsin(theta)
    y_hat = focalLength * cos(theta);
    z_hat = focalLength * sin(theta);

    x = x0 + x_hat;
    y = y0 + y_hat;
    z = z0 + z_hat;


    // Change view up vector

    // Position vector for (x_top, y_top, z_top)
    double x_topHat = x_top - x0;
    double y_topHat = y_top - y0;
    double z_topHat = z_top - z0;

    // Euclidean distance between (x_top, y_top, z_top) and camera center
    double dist = sqrt( pow((x_top - x0), 2) + pow((y_top - y0), 2) + pow((z_top + z0), 2) );

    double t_theta = atan2(y_topHat, z_topHat);
    t_theta += angle_in;

    y_topHat = dist * cos(t_theta);
    z_topHat = dist * sin(t_theta);

    x_top = x0 + x_topHat;
    y_top = y0 + y_topHat;
    z_top = z0 + z_topHat;

    x_up = x_top - x;
    y_up = y_top - y;
    x_up = x_top - z;

    vector<double> result(6);
    result[0] = x;
    result[1] = y;
    result[2] = z;
    result[3] = x_up;
    result[4] = y_up;
    result[5] = z_up;

    // cerr << "New Focal x is: " << x << endl;
    // cerr << "New Focal y is: " << y << endl;
    // cerr << "New Focal z is: " << z << endl;

    // cerr << "New View up x is: " << x_up << endl;
    // cerr << "New View up y is: " << y_up << endl;
    // cerr << "New View up z is: " << z_up << endl;
    // cerr << endl;

    return result;
}

vector<double> localPan(vector<double> cameraCenter, vector<double> focal, vector<double> view_up, double angle_in)
{
    double focalLength = 1;
    double viewUpMag = 1;

    // Convert incoming angle from degrees to radians
    angle_in = (angle_in/180.0)*PI;

    // Camera ceter
    double x0 = cameraCenter[0];
    double y0 = cameraCenter[1];
    double z0 = cameraCenter[2];

    // Focal point
    double x1 = focal[0];
    double y1 = focal[1];
    double z1 = focal[2];

    //cerr << "Camera Center: " << x0 << " " << y0 << " " << z0 << endl;
    cerr << "Focal Point: " << x1 << " " << y1 << " " << z1 << endl;

    // Distance vector between focal point and top of fulstrum
    double x_up = view_up[0];
    double y_up = view_up[1];
    double z_up = view_up[2];

    cerr << "View Up: " << x_up << " " << y_up << " " << z_up << endl << endl;

    // Unit vector version of view up vector
    //double viewUpMag = sqrt( pow(x_up, 2) + pow(y_up, 2) + pow(z_up, 2) );

    // Position vector
    double x_hat = x1 - x0;
    double y_hat = y1 - y0;
    double z_hat = z1 - z0;

    // Take cross product of position vector and view up vector
    double crossX = (y_hat * z_up) - (z_hat * y_up);
    double crossY = (z_hat * x_up) - (x_hat * z_up);
    double crossZ = (x_hat * y_up) - (y_hat * x_up);

    double crossMag = sqrt( pow(crossX, 2) + pow(crossY, 2) + pow(crossZ, 2) );

    crossX = crossX/ crossMag;
    crossY = crossY/ crossMag;
    crossZ = crossZ/ crossMag;

    double delta_x_pos = x_hat/ focalLength;
    double delta_y_pos = y_hat/ focalLength;
    double delta_z_pos = z_hat/ focalLength;

    // Shift in x,z coordinates in the local frame (z local is cross vector, x local is pos vector)
    // Pan left (increase z) is positive
    delta_x_pos *= (focalLength * (-1 + cos(angle_in)) );
    delta_y_pos *= (focalLength * (-1 + cos(angle_in)) );
    delta_z_pos *= (focalLength * (-1 + cos(angle_in)) );

    crossX *= (focalLength * sin(angle_in));
    crossY *= (focalLength * sin(angle_in));
    crossZ *= (focalLength * sin(angle_in));

    x_hat += (delta_x_pos + crossX);
    y_hat += (delta_y_pos + crossY);
    z_hat += (delta_z_pos + crossZ);

    double x = x_hat + x0;
    double y = y_hat + y0;
    double z = z_hat + z0;

    // cerr << "x1 is: " << x1 << endl;
    // cerr << "y1 is: " << y1 << endl;
    // cerr << "z1 is: " << z1 << endl;
    // cerr << endl;

    // cerr << "x is: " << x << endl;
    // cerr << "y is: " << y << endl;
    // cerr << "z is: " << z << endl;
    // cerr << endl;

    // Up vector doesn't change in pan
    vector<double> result(3);
    result[0] = x;
    result[1] = y;
    result[2] = z;

    return result;



























    // // Camera ceter
    // double x0 = cameraCenter[0];
    // double y0 = cameraCenter[1];
    // double z0 = cameraCenter[2];

    // // Focal point
    // double x1 = focal[0];
    // double y1 = focal[1];
    // double z1 = focal[2];

    // // Distance vector between focal point and top of fulstrum
    // double x_up = view_up[0];
    // double y_up = view_up[1];
    // double z_up = view_up[2];

    // // (x, y, z) are new focal point.
    // double x;
    // double y;
    // double z;

    // // (x_top, y_top, z_top) is the top of the fulstrum (found by following view_up vector starting at focal point)
    // double x_top = x1 + x_up;
    // double y_top = y1 + y_up;
    // double z_top = z1 + z_up;

    // // Unit vector version of view up vector
    // double viewUpMag = sqrt( pow(x_up, 2) + pow(y_up, 2) + pow(z_up, 2) );
    // double unit_xUp = x_up/viewUpMag;
    // double unit_yUp = y_up/viewUpMag;
    // double unit_zUp = z_up/viewUpMag;

    // // Position vector
    // double x_hat = x1 - x0;
    // double y_hat = y1 - y0;
    // double z_hat = z1 - z0;

    // // Unit vector version of position vector
    // double unit_xHat = x_hat/focalLength;
    // double unit_yHat = y_hat/focalLength;
    // double unit_zHat = z_hat/focalLength;

    // // Get local frame coordinates relative to global frame
    // // local x is the position vector (unit vector form)
    // // local y is view up (unit vector)
    // // local z is the cross product of local x and local y (unit vector)
    // vector<double> local_xHat(3);
    // local_xHat[0] = unit_xHat;
    // local_xHat[1] = unit_yHat;
    // local_xHat[2] = unit_zHat;

    // vector<double> local_yHat(3);
    // local_yHat[0] = unit_xUp;
    // local_yHat[1] = unit_yUp;
    // local_yHat[2] = unit_zUp;

    // double crossX = (unit_yHat * unit_zUp) - (unit_zHat * unit_yUp);
    // double crossY = (unit_zHat * unit_xUp) - (unit_xHat * unit_zUp);
    // double crossZ = (unit_xHat * unit_yUp) - (unit_yHat * unit_xUp);
    // vector<double> local_zHat(3);
    // local_zHat[0] = crossX;
    // local_zHat[1] = crossY;
    // local_zHat[2] = crossZ;

    // // Shift in x,z coordinates in the local frame (pan adjusts x,z only)
    // // Pan inward (left) is positive
    // double xPan = focalLength * cos(angle_in);
    // double zPan = focalLength * sin(angle_in);

    // // New coordinates in the global frame
    // // In our adjusted frame, focal point starts at (1, 0, 0)
    // x = x1 - (1 - xPan*local_xHat[0]) + zPan*local_zHat[0];
    // y = y1 - (1 - xPan*local_xHat[1]) + zPan*local_zHat[1];
    // z = z1 - (1 - xPan*local_xHat[2]) + zPan*local_zHat[2];

    // // In a pan, top of fulstram moves the same as focal point does
    // x_top += -(1 - xPan*local_xHat[0]) + zPan*local_zHat[0];
    // y_top += -(1 - xPan*local_xHat[1]) + zPan*local_zHat[1];
    // z_top += -(1 - xPan*local_xHat[2]) + zPan*local_zHat[2];

    // //x_up = x_top - x;
    // //y_up = y_top - y;
    // //x_up = x_top - z;

    // vector<double> result(6);
    // result[0] = x;
    // result[1] = y;
    // result[2] = z;
    // result[3] = x_up;
    // result[4] = y_up;
    // result[5] = z_up;

    // return result;
}

vector<double> localTilt(vector<double> cameraCenter, vector<double> focal, vector<double> view_up, double angle_in)
{
    double focalLength = 1;
    double viewUpMag = 1;

    // Convert incoming angle from degrees to radians
    angle_in = (angle_in/180.0)*PI;

    // Camera ceter
    double x0 = cameraCenter[0];
    double y0 = cameraCenter[1];
    double z0 = cameraCenter[2];

    // Focal point
    double x1 = focal[0];
    double y1 = focal[1];
    double z1 = focal[2];

    //cerr << "Camera Center: " << x0 << " " << y0 << " " << z0 << endl;
    cerr << "Focal Point: " << x1 << " " << y1 << " " << z1 << endl;

    // Distance vector between focal point and top of fulstrum
    double x_up = view_up[0];
    double y_up = view_up[1];
    double z_up = view_up[2];

    cerr << "View Up: " << x_up << " " << y_up << " " << z_up << endl << endl;

    // Unit vector version of view up vector
    //double viewUpMag = sqrt( pow(x_up, 2) + pow(y_up, 2) + pow(z_up, 2) );

    // Position vector
    double x_hat = x1 - x0;
    double y_hat = y1 - y0;
    double z_hat = z1 - z0;

    double posMag = sqrt( pow(x_hat, 2) + pow(y_hat, 2) + pow(z_hat, 2) );

    // Unit vector for position
    double delta_x_pos = x_hat/ posMag;
    double delta_y_pos = y_hat/ posMag;
    double delta_z_pos = z_hat/ posMag;

    // Unit vector for tilt direction (up vector)
    double tiltX = x_up/ viewUpMag;
    double tiltY = y_up/ viewUpMag;
    double tiltZ = z_up/ viewUpMag;

    // Shift in x,z coordinates in the local frame (z local is cross vector, x local is pos vector)
    // Pan left (increase z) is positive
    delta_x_pos *= (focalLength * (-1 + cos(angle_in)) );
    delta_y_pos *= (focalLength * (-1 + cos(angle_in)) );
    delta_z_pos *= (focalLength * (-1 + cos(angle_in)) );

    tiltX *= (focalLength * sin(angle_in));
    tiltY *= (focalLength * sin(angle_in));
    tiltZ *= (focalLength * sin(angle_in));

    x_hat += (delta_x_pos + tiltX);
    y_hat += (delta_y_pos + tiltY);
    z_hat += (delta_z_pos + tiltZ);

    double x = x_hat + x0;
    double y = y_hat + y0;
    double z = z_hat + z0;

    // Up vector changes due to tilting as well

    x_hat = x1 - x0;
    y_hat = y1 - y0;
    z_hat = z1 - z0;

    delta_x_pos = x_hat/ posMag;
    delta_y_pos = y_hat/ posMag;
    delta_z_pos = z_hat/ posMag;

    tiltX = x_up/ viewUpMag;
    tiltY = y_up/ viewUpMag;
    tiltZ = z_up/ viewUpMag;

    delta_x_pos *= (viewUpMag * sin(-angle_in));
    delta_y_pos *= (viewUpMag * sin(-angle_in));
    delta_z_pos *= (viewUpMag * sin(-angle_in));

    tiltX *= (viewUpMag * (-1 + cos(angle_in)) );
    tiltY *= (viewUpMag * (-1 + cos(angle_in)) );
    tiltZ *= (viewUpMag * (-1 + cos(angle_in)) );

    x_up += (delta_x_pos + tiltX);
    y_up += (delta_y_pos + tiltY);
    z_up += (delta_z_pos + tiltZ);
    

    // cerr << "x1 is: " << x1 << endl;
    // cerr << "y1 is: " << y1 << endl;
    // cerr << "z1 is: " << z1 << endl;
    // cerr << endl;

    // cerr << "x is: " << x << endl;
    // cerr << "y is: " << y << endl;
    // cerr << "z is: " << z << endl;
    // cerr << endl;

    // Both pos and up vectors change in tilt
    vector<double> result(6);
    result[0] = x;
    result[1] = y;
    result[2] = z;
    result[3] = x_up;
    result[4] = y_up;
    result[5] = z_up;

    return result;














































    // // Camera ceter
    // double x0 = cameraCenter[0];
    // double y0 = cameraCenter[1];
    // double z0 = cameraCenter[2];

    // // Focal point
    // double x1 = focal[0];
    // double y1 = focal[1];
    // double z1 = focal[2];

    // // Distance vector between focal point and top of fulstrum
    // double x_up = view_up[0];
    // double y_up = view_up[1];
    // double z_up = view_up[2];

    // // (x, y, z) are new focal point.
    // double x;
    // double y;
    // double z;

    // // (x_top, y_top, z_top) is the top of the fulstrum (found by following view_up vector starting at focal point)
    // double x_top = x1 + x_up;
    // double y_top = y1 + y_up;
    // double z_top = z1 + z_up;

    // // Unit vector version of view up vector
    // double viewUpMag = sqrt( pow(x_up, 2) + pow(y_up, 2) + pow(z_up, 2) );
    // double unit_xUp = x_up/viewUpMag;
    // double unit_yUp = y_up/viewUpMag;
    // double unit_zUp = z_up/viewUpMag;

    // // Position vector
    // double x_hat = x1 - x0;
    // double y_hat = y1 - y0;
    // double z_hat = z1 - z0;

    // // Unit vector version of position vector
    // double unit_xHat = x_hat/focalLength;
    // double unit_yHat = y_hat/focalLength;
    // double unit_zHat = z_hat/focalLength;

    // // Get local frame coordinates relative to global frame
    // // local x is the position vector (unit vector form)
    // // local y is view up (unit vector)
    // // local z is the cross product of local x and local y (unit vector)
    // vector<double> local_xHat(3);
    // local_xHat[0] = unit_xHat;
    // local_xHat[1] = unit_yHat;
    // local_xHat[2] = unit_zHat;

    // vector<double> local_yHat(3);
    // local_yHat[0] = unit_xUp;
    // local_yHat[1] = unit_yUp;
    // local_yHat[2] = unit_zUp;

    // double crossX = (unit_yHat * unit_zUp) - (unit_zHat * unit_yUp);
    // double crossY = (unit_zHat * unit_xUp) - (unit_xHat * unit_zUp);
    // double crossZ = (unit_xHat * unit_yUp) - (unit_yHat * unit_xUp);
    // vector<double> local_zHat(3);
    // local_zHat[0] = crossX;
    // local_zHat[1] = crossY;
    // local_zHat[2] = crossZ;

    // // Shift in x,y coordinates in the local frame (tilt adjusts x,y only)
    // // Pan up is positive
    // double xTilt = focalLength * cos(angle_in);
    // double yTilt = focalLength * sin(angle_in);

    // // New coordinates in the global frame
    // // In our adjusted frame, focal point starts at (1, 0, 0)
    // x = x1 - (1 - xTilt*local_xHat[0]) + yTilt*local_yHat[0];
    // y = y1 - (1 - xTilt*local_xHat[1]) + yTilt*local_yHat[1];
    // z = z1 - (1 - xTilt*local_xHat[2]) + yTilt*local_yHat[2];

    // // Adjust view up

    // // Euclidean distance between (x_top, y_top, z_top) and camera center
    // double dist = sqrt( pow((x_top - x0), 2) + pow((y_top - y0), 2) + pow((z_top + z0), 2) );

    // // Starting angle
    // double theta0 = atan2(viewUpMag, focalLength);

    // double xTopTilt = dist * cos(angle_in + theta0);
    // double yTopTilt = dist * sin(angle_in + theta0);

    // // New coordinates in the global frame
    // // In our adjusted frame, top of fulstrum starts at (1, viewUpMag, 0)
    // x_top += - (1 - xTopTilt*local_xHat[0]) - (viewUpMag - yTopTilt*local_yHat[0]);
    // y_top += - (1 - xTopTilt*local_xHat[1]) - (viewUpMag - yTopTilt*local_yHat[1]);
    // z_top += - (1 - xTopTilt*local_xHat[2]) - (viewUpMag - yTopTilt*local_yHat[2]);

    // x_up = x_top - x;
    // y_up = y_top - y;
    // x_up = x_top - z;

    // vector<double> result(6);
    // result[0] = x;
    // result[1] = y;
    // result[2] = z;
    // result[3] = x_up;
    // result[4] = y_up;
    // result[5] = z_up;

    // return result;
}


vector<double> localRoll(vector<double> cameraCenter, vector<double> focal, vector<double> view_up, double angle_in)
{
    double focalLength = 1;
    double viewUpMag = 1;

    // Convert incoming angle from degrees to radians
    angle_in = (angle_in/180.0)*PI;

    // Camera center
    double x0 = cameraCenter[0];
    double y0 = cameraCenter[1];
    double z0 = cameraCenter[2];

    // Focal point
    double x1 = focal[0];
    double y1 = focal[1];
    double z1 = focal[2];

    //cerr << "Camera Center: " << x0 << " " << y0 << " " << z0 << endl;
    cerr << "Focal Point: " << x1 << " " << y1 << " " << z1 << endl;

    // Distance vector between focal point and top of fulstrum
    double x_up = view_up[0];
    double y_up = view_up[1];
    double z_up = view_up[2];

    cerr << "View Up: " << x_up << " " << y_up << " " << z_up << endl << endl;

    // Unit vector version of view up vector
    //double viewUpMag = sqrt( pow(x_up, 2) + pow(y_up, 2) + pow(z_up, 2) );

    // Position vector
    double x_hat = x1 - x0;
    double y_hat = y1 - y0;
    double z_hat = z1 - z0;

    double posMag = sqrt( pow(x_hat, 2) + pow(y_hat, 2) + pow(z_hat, 2) );

    // Take cross product of position vector and view up vector
    double crossX = (y_hat * z_up) - (z_hat * y_up);
    double crossY = (z_hat * x_up) - (x_hat * z_up);
    double crossZ = (x_hat * y_up) - (y_hat * x_up);

    double crossMag = sqrt( pow(crossX, 2) + pow(crossY, 2) + pow(crossZ, 2) );

    crossX = crossX/ crossMag;
    crossY = crossY/ crossMag;
    crossZ = crossZ/ crossMag;

    double delta_x_tilt = x_up/ viewUpMag;
    double delta_y_tilt = y_up/ viewUpMag;
    double delta_z_tilt = z_up/ viewUpMag;

    // Shift in x,z coordinates in the local frame (z local is cross vector, x local is pos vector)                     FIX DOCUMENTATION
    // Pan left (increase z) is positive
    delta_x_tilt *= (viewUpMag * (-1 + cos(angle_in)) );
    delta_y_tilt *= (viewUpMag * (-1 + cos(angle_in)) );
    delta_z_tilt *= (viewUpMag * (-1 + cos(angle_in)) );

    crossX *= (viewUpMag * sin(angle_in));
    crossY *= (viewUpMag * sin(angle_in));
    crossZ *= (viewUpMag * sin(angle_in));

    x_up += (delta_x_tilt + crossX);
    y_up += (delta_y_tilt + crossY);
    z_up += (delta_z_tilt + crossZ);

    // cerr << "x1 is: " << x1 << endl;
    // cerr << "y1 is: " << y1 << endl;
    // cerr << "z1 is: " << z1 << endl;
    // cerr << endl;

    // cerr << "x is: " << x << endl;
    // cerr << "y is: " << y << endl;
    // cerr << "z is: " << z << endl;
    // cerr << endl;

    // Only up vector changes in roll
    vector<double> result(3);
    result[0] = x_up;
    result[1] = y_up;
    result[2] = z_up;

    return result;








































    // // Camera ceter
    // double x0 = cameraCenter[0];
    // double y0 = cameraCenter[1];
    // double z0 = cameraCenter[2];

    // // Focal point
    // double x1 = focal[0];
    // double y1 = focal[1];
    // double z1 = focal[2];

    // // Distance vector between focal point and top of fulstrum
    // double x_up = view_up[0];
    // double y_up = view_up[1];
    // double z_up = view_up[2];

    // // (x, y, z) are new focal point.
    // // (x, y, z) do not change during roll, only view_up vector changes
    // double x = x1;
    // double y = y1;
    // double z = z1;

    // // (x_top, y_top, z_top) is the top of the fulstrum (found by following view_up vector starting at focal point)
    // double x_top = x1 + x_up;
    // double y_top = y1 + y_up;
    // double z_top = z1 + z_up;

    // // Unit vector version of view up vector
    // double viewUpMag = sqrt( pow(x_up, 2) + pow(y_up, 2) + pow(z_up, 2) );
    // double unit_xUp = x_up/viewUpMag;
    // double unit_yUp = y_up/viewUpMag;
    // double unit_zUp = z_up/viewUpMag;

    // // Position vector
    // double x_hat = x1 - x0;
    // double y_hat = y1 - y0;
    // double z_hat = z1 - z0;

    // // Unit vector version of position vector
    // double unit_xHat = x_hat/focalLength;
    // double unit_yHat = y_hat/focalLength;
    // double unit_zHat = z_hat/focalLength;

    // // Get local frame coordinates relative to global frame
    // // local x is the position vector (unit vector form)
    // // local y is view up (unit vector)
    // // local z is the cross product of local x and local y (unit vector)
    // vector<double> local_xHat(3);
    // local_xHat[0] = unit_xHat;
    // local_xHat[1] = unit_yHat;
    // local_xHat[2] = unit_zHat;

    // vector<double> local_yHat(3);
    // local_yHat[0] = unit_xUp;
    // local_yHat[1] = unit_yUp;
    // local_yHat[2] = unit_zUp;

    // double crossX = (unit_yHat * unit_zUp) - (unit_zHat * unit_yUp);
    // double crossY = (unit_zHat * unit_xUp) - (unit_xHat * unit_zUp);
    // double crossZ = (unit_xHat * unit_yUp) - (unit_yHat * unit_xUp);
    // vector<double> local_zHat(3);
    // local_zHat[0] = crossX;
    // local_zHat[1] = crossY;
    // local_zHat[2] = crossZ;

    // // starting angle
    // double theta0 = 90.0;

    // double yTopRoll = viewUpMag * cos(angle_in + theta0);
    // double zTopRoll = viewUpMag * sin(angle_in + theta0);

    // // New coordinates in the global frame
    // // In our adjusted frame, top of fulstrum starts at (1, viewUpMag, 0)
    // x_top += - (viewUpMag - yTopRoll*local_yHat[0]) - (zTopRoll*local_zHat[0]);
    // y_top += - (viewUpMag - yTopRoll*local_yHat[1]) - (zTopRoll*local_zHat[1]);
    // z_top += - (viewUpMag - yTopRoll*local_yHat[2]) - (zTopRoll*local_zHat[2]);

    // x_up = x_top - x;
    // y_up = y_top - y;
    // x_up = x_top - z;

    // vector<double> result(6);
    // result[0] = x;
    // result[1] = y;
    // result[2] = z;
    // result[3] = x_up;
    // result[4] = y_up;
    // result[5] = z_up;

    // return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////


// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr environmentCloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud (environmentCloud, "environment cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "environment cloud");
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
#if 0
  viewer->camera_.clip[0] = 1.75082;
  viewer->camera_.clip[1] = 17.0692;

  viewer->camera_.focal[0] = 0;
  viewer->camera_.focal[1] = 0;
  viewer->camera_.focal[2] = 0;

  viewer->camera_.pos[0] = -6.49882;
  viewer->camera_.pos[1] = -1.14148;
  viewer->camera_.pos[2] = 1.31208;

  viewer->camera_.view[0] = 0.166;
  viewer->camera_.view[1] = 0.015;
  viewer->camera_.view[2] = 0.978916;

  viewer->camera_.window_pos[0] = 66;
  viewer->camera_.window_pos[1] = 52; 

  viewer->camera_.fovy = 0.523599;

  viewer->updateCamera();

  //viewer->spinOnce();
#endif

  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> plyLoader (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, string environmentCloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addModelFromPLYFile (environmentCloud, "environment cloud");
  viewer->initCameraParameters ();
#if 0
  viewer->camera_.clip[0] = 1.75082;
  viewer->camera_.clip[1] = 17.0692;

  viewer->camera_.focal[0] = 0;
  viewer->camera_.focal[1] = 0;
  viewer->camera_.focal[2] = 0;

  viewer->camera_.pos[0] = -6.49882;
  viewer->camera_.pos[1] = -1.14148;
  viewer->camera_.pos[2] = 1.31208;

  viewer->camera_.view[0] = 0.166;
  viewer->camera_.view[1] = 0.015;
  viewer->camera_.view[2] = 0.978916;

  viewer->camera_.window_pos[0] = 66;
  viewer->camera_.window_pos[1] = 52; 

  viewer->camera_.fovy = 0.523599;

  viewer->updateCamera();

  //viewer->spinOnce();
#endif

  return (viewer);
}

// Takes in a PCLVisualizer object and resets the viewpoint to looking through the origin at the point cloud
void resetOriginalViewpoint (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  #if 0
  viewer->camera_.clip[0] = 1.75082;
  viewer->camera_.clip[1] = 17.0692;

  viewer->camera_.focal[0] = 0;
  viewer->camera_.focal[1] = 0;
  viewer->camera_.focal[2] = 0;

  viewer->camera_.pos[0] = -6.49882;
  viewer->camera_.pos[1] = -1.14148;
  viewer->camera_.pos[2] = 1.31208;

  viewer->camera_.view[0] = 0.166;
  viewer->camera_.view[1] = 0.015;
  viewer->camera_.view[2] = 0.978916;

  viewer->camera_.window_pos[0] = 66;
  viewer->camera_.window_pos[1] = 52; 

  viewer->camera_.fovy = 0.523599;
  #endif
#if 0
  // 1.87707,15.4311/0,0,0/-3.91934,-1.10731,5.35461/0.787253,0.129175,0.602948/0.523599/800,600/66,52
  viewer->camera_.clip[0] = 1.87707;
  viewer->camera_.clip[1] = 15.4311;

  viewer->camera_.focal[0] = 0;
  viewer->camera_.focal[1] = 0;
  viewer->camera_.focal[2] = 0;

  viewer->camera_.pos[0] = -3.91934;
  viewer->camera_.pos[1] = -1.10731;
  viewer->camera_.pos[2] = 5.35461;

  viewer->camera_.view[0] = 0.787253;
  viewer->camera_.view[1] = 0.129175;
  viewer->camera_.view[2] = 0.602948;

  viewer->camera_.window_pos[0] = 66;
  viewer->camera_.window_pos[1] = 52; 

  viewer->camera_.fovy = 0.523599;
#endif
  #if 0
  // 11.4677,30.4953/-0.0837911,-3.37767,-0.187972/-10.1371,-3.92142,14.0639/0.813247,0.0781398,0.576648/0.523599/800,600/66,52
  viewer->camera_.clip[0] = 11.4677;
  viewer->camera_.clip[1] = 30.4953;

  viewer->camera_.focal[0] = -0.0837911;
  viewer->camera_.focal[1] = -3.37767;
  viewer->camera_.focal[2] = -0.187972;

  viewer->camera_.pos[0] = -10.1371;
  viewer->camera_.pos[1] = -3.92142;
  viewer->camera_.pos[2] = 14.0639;

  viewer->camera_.view[0] = 0.813247;
  viewer->camera_.view[1] = 0.0781398;
  viewer->camera_.view[2] = 0.576648;

  viewer->camera_.window_pos[0] = 66;
  viewer->camera_.window_pos[1] = 52; 

  viewer->camera_.fovy = 0.523599;
  #endif

  // 6.94083,32.012/0.125376,-3.43583,-0.0426443/-11.6865,-4.8284,8.11236/0.561569,0.0665266,0.824751/0.523599/800,600/66,52
  viewer->camera_.clip[0] = 6.94083;
  viewer->camera_.clip[1] = 32.012;

  viewer->camera_.focal[0] = 0.125376;
  viewer->camera_.focal[1] = -3.43583;
  viewer->camera_.focal[2] = -0.0426443;

  viewer->camera_.pos[0] = -11.6865;
  viewer->camera_.pos[1] = -4.8284;
  viewer->camera_.pos[2] = 8.11236;

  viewer->camera_.view[0] = 0.561569;
  viewer->camera_.view[1] = 0.0665266;
  viewer->camera_.view[2] = 0.824751;

  viewer->camera_.window_pos[0] = 66;
  viewer->camera_.window_pos[1] = 52; 

  viewer->camera_.fovy = 0.523599;

  // 0.00789528,7.89528/0,0,0/-0.0219228,-0.00272186,-0.000408394/-0.0173165,-0.0105385,0.999795/0.523599/800,600/66,52

  // set the viewpoint to approximately the origin looking at the point cloud
  #if 0
  viewer->camera_.clip[0] = 0.00789528;
  viewer->camera_.clip[1] = 7.89528;

  viewer->camera_.focal[0] = 0;
  viewer->camera_.focal[1] = 0;
  viewer->camera_.focal[2] = 0;

  viewer->camera_.pos[0] = -0.0219228;
  viewer->camera_.pos[1] = -0.00272186;
  viewer->camera_.pos[2] = -0.000408394;

  viewer->camera_.view[0] = -0.0173165;
  viewer->camera_.view[1] = -0.0105385;
  viewer->camera_.view[2] = 0.999795;

  viewer->camera_.window_pos[0] = 66;
  viewer->camera_.window_pos[1] = 52; 

  viewer->camera_.fovy = 0.523599;
  #endif

  viewer->updateCamera();
}

// Takes in a PCLVisualizer object and sets the viewpoint to look "up"
void renderViewpointUp (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  //1.36367,19.7585/0,0,0/-6.0945,-1.70323,-2.28368/-0.323506,-0.10351,0.940547/0.523599/800,600/66,52
  //viewer->camera_.clip[0] = 1.75082;
  //viewer->camera_.clip[1] = 17.0692;

  //viewer->camera_.focal[0] = 0;
  //viewer->camera_.focal[1] = 0;
  //viewer->camera_.focal[2] = 0;

  //viewer->camera_.pos[0] = -6.49882;
  //viewer->camera_.pos[1] = -1.14148;
  viewer->camera_.pos[2] -= 3;

  viewer->camera_.view[0] -= 0.4;
  //viewer->camera_.view[1] = 0.015;
  //viewer->camera_.view[2] = 0.978916;

  //viewer->camera_.window_pos[0] = 66;
  //viewer->camera_.window_pos[1] = 52; 

  //viewer->camera_.fovy = 0.523599;

  viewer->updateCamera();
}

// Takes in a PCLVisualizer object and sets the viewpoint to look "down"
void renderViewpointDown (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{

}

// 1.87707,15.4311/0,0,0/-3.91934,-1.10731,5.35461/0.787253,0.129175,0.602948/0.523599/800,600/66,52
// 1.87707,15.4311/0.940986,-0.459172,0.593806/-2.97835,-1.56648,5.94842/0.787253,0.129175,0.602948/0.523599/800,600/66,52
// Takes in a PCLVisualizer object and renders the viewpoint to look at 1 o'clock on a clock face
void renderViewpoint1 (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  viewer->camera_.focal[0] += 0.940986;
  viewer->camera_.focal[1] -= 0.459172;
  viewer->camera_.focal[2] += 0.593806;

  viewer->camera_.pos[0] += 0.940986;
  viewer->camera_.pos[1] -= 0.459172;
  viewer->camera_.pos[2] += 0.593806;

  viewer->updateCamera();
}

// 1.87707,15.4311/0,0,0/-3.91934,-1.10731,5.35461/0.787253,0.129175,0.602948/0.523599/800,600/66,52
// 1.87707,15.4311/1.35394,-2.03166,0.570889/-2.5654,-3.13897,5.9255/0.787253,0.129175,0.602948/0.523599/800,600/66,52
// 1.87707,15.4311/0.667471,-1.1237,0.256183/-3.25187,-2.23101,5.61079/0.787253,0.129175,0.602948/0.523599/800,600/66,52
// Takes in a PCLVisualizer object and renders the viewpoint to look at 2 o'clock on a clock face
void renderViewpoint2 (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  viewer->camera_.focal[0] += 0.667471;
  viewer->camera_.focal[1] -= 1.1237;
  viewer->camera_.focal[2] += 0.256183;

  viewer->camera_.pos[0] += 0.667471;
  viewer->camera_.pos[1] -= 1.1237;
  viewer->camera_.pos[2] += 0.256183;

  viewer->updateCamera();
}

// 0.0156802,15.6802/0,0,0/-4.52794,0.212378,0.752607/0.164139,0.00384902,0.98643/0.523599/800,600/66,52
// 0.0156802,15.6802/-0.0989176,-1.9435,-0.0466858/-4.62686,-1.73112,0.705922/0.164139,0.00384902,0.98643/0.523599/800,600/66,52
// Takes in a PCLVisualizer object and renders the viewpoint to look at 3 o'clock on a clock face
void renderViewpoint3 (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  viewer->camera_.focal[0] -= 0.0640334;
  viewer->camera_.focal[1] -= 1.41583;
  viewer->camera_.focal[2] -= 0.11518;

  viewer->camera_.pos[0] -= 0.06;
  viewer->camera_.pos[1] -= 1.4;
  viewer->camera_.pos[2] -= 0.1;

  viewer->updateCamera();
}

// Takes in a PCLVisualizer object and renders the viewpoint to look at 4 o'clock on a clock face
void renderViewpoint4 (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  viewer->camera_.focal[0] -= 0.545612;
  viewer->camera_.focal[1] -= 1.14657;
  viewer->camera_.focal[2] -= 0.63647;

  viewer->camera_.pos[0] -= 0.545612;
  viewer->camera_.pos[1] -= 1.14657;
  viewer->camera_.pos[2] -= 0.63647;

  viewer->updateCamera();
}

// Takes in a PCLVisualizer object and renders the viewpoint to look at 5 o'clock on a clock face
void renderViewpoint5 (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  viewer->camera_.focal[0] -= 0.891196;
  viewer->camera_.focal[1] -= 0.468204;
  viewer->camera_.focal[2] -= 0.749139;

  viewer->camera_.pos[0] -= 0.891196;
  viewer->camera_.pos[1] -= 0.468204;
  viewer->camera_.pos[2] -= 0.749139;

  viewer->updateCamera();
}

// Takes in a PCLVisualizer object and renders the viewpoint to look at 6 o'clock on a clock face
void renderViewpoint6 (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  viewer->camera_.focal[0] -= 1.77512;
  viewer->camera_.focal[1] -= 0.285192;
  viewer->camera_.focal[2] -= 1.35829;

  viewer->camera_.pos[0] -= 1.77;
  viewer->camera_.pos[1] -= 0.285;
  viewer->camera_.pos[2] -= 1.358;

  viewer->updateCamera();
}

// Takes in a PCLVisualizer object and renders the viewpoint to look at 7 o'clock on a clock face
void renderViewpoint7 (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  viewer->camera_.focal[0] -= 0.940986;
  viewer->camera_.focal[1] += 0.459172;
  viewer->camera_.focal[2] -= 0.593806;

  viewer->camera_.pos[0] -= 0.940986;
  viewer->camera_.pos[1] += 0.459172;
  viewer->camera_.pos[2] -= 0.593806;

  viewer->updateCamera();
}

// Takes in a PCLVisualizer object and renders the viewpoint to look at 8 o'clock on a clock face
void renderViewpoint8 (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  viewer->camera_.focal[0] -= 0.667471;
  viewer->camera_.focal[1] += 1.1237;
  viewer->camera_.focal[2] -= 0.256183;

  viewer->camera_.pos[0] -= 0.667471;
  viewer->camera_.pos[1] += 1.1237;
  viewer->camera_.pos[2] -= 0.256183;

  viewer->updateCamera();
}

// 0.0121771,12.1771/0,0,0/-1.4618,0.0617238,0.0539453/0.0344611,-0.0559496,0.997839/0.523599/800,600/66,52
// 0.0121771,12.1771/0.0640334,1.41583,0.11518/-1.39777,1.47756,0.169126/0.0344611,-0.0559496,0.997839/0.523599/800,600/66,52
// Takes in a PCLVisualizer object and renders the viewpoint to look at 9 o'clock on a clock face
void renderViewpoint9 (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  viewer->camera_.focal[0] += 0.0640334;
  viewer->camera_.focal[1] += 1.41583;
  viewer->camera_.focal[2] += 0.11518;

  viewer->camera_.pos[0] += 0.06;
  viewer->camera_.pos[1] += 1.4;
  viewer->camera_.pos[2] += 0.1;

  viewer->updateCamera();
}

// 1.87707,15.4311/0,0,0/-3.91934,-1.10731,5.35461/0.787253,0.129175,0.602948/0.523599/800,600/66,52
// 1.87707,15.4311/0.545612,1.14657,0.63647/-3.37373,0.0392615,5.99108/0.787253,0.129175,0.602948/0.523599/800,600/66,52
// Takes in a PCLVisualizer object and renders the viewpoint to look at 10 o'clock on a clock face
void renderViewpoint10 (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  viewer->camera_.focal[0] += 0.545612;
  viewer->camera_.focal[1] += 1.14657;
  viewer->camera_.focal[2] += 0.63647;

  viewer->camera_.pos[0] += 0.545612;
  viewer->camera_.pos[1] += 1.14657;
  viewer->camera_.pos[2] += 0.63647;

  viewer->updateCamera();
}

// 1.87707,15.4311/0,0,0/-3.91934,-1.10731,5.35461/0.787253,0.129175,0.602948/0.523599/800,600/66,52
// 1.87707,15.4311/0.891196,0.468204,0.749139/-3.02814,-0.639106,6.10375/0.787253,0.129175,0.602948/0.523599/800,600/66,52
// Takes in a PCLVisualizer object and renders the viewpoint to look at 11 o'clock on a clock face
void renderViewpoint11 (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  viewer->camera_.focal[0] += 0.891196;
  viewer->camera_.focal[1] += 0.468204;
  viewer->camera_.focal[2] += 0.749139;

  viewer->camera_.pos[0] += 0.891196;
  viewer->camera_.pos[1] += 0.468204;
  viewer->camera_.pos[2] += 0.749139;

  viewer->updateCamera();
}

// 1.87707,15.4311/0,0,0/-3.91934,-1.10731,5.35461/0.787253,0.129175,0.602948/0.523599/800,600/66,52
// 1.87707,15.4311/1.77512,0.285192,1.35829/-2.14422,-0.822118,6.7129/0.787253,0.129175,0.602948/0.523599/800,600/66,52
// Takes in a PCLVisualizer object and renders the viewpoint to look at 12 o'clock on a clock face
void renderViewpoint12 (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  viewer->camera_.focal[0] += 1.77512;
  viewer->camera_.focal[1] += 0.285192;
  viewer->camera_.focal[2] += 1.35829;

  viewer->camera_.pos[0] += 1.77;
  viewer->camera_.pos[1] += 0.285;
  viewer->camera_.pos[2] += 1.358;

  viewer->updateCamera();
}

 unsigned int text_id = 0;
// void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
//                             void* viewer_void)
// {
//   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
//   #if 0
//   if (event.getKeySym() == "t" && event.keyDown()) {
//     arrow = true;
//     viewer->addArrow(pcl::PointXYZ(arrowX, arrowY, arrowZ), pcl::PointXYZ(arrowX, arrowY-1.0, arrowZ), 0.662, 0.662, 0.662, false, "arrow");
//     cout << "t was pressed... adding an arrow" << endl;
//   }
//   #endif
//   if (event.getKeySym() == "c" && event.keyDown()) {
//     //Eigen::Vector3f pt_on_axis, axis_direction;
//     //float radius;
//     cylinder = true;
//     pcl::ModelCoefficients cylinder_coeff;
//     cylinder_coeff.values.resize (7); // We need 7 values
//     cylinder_coeff.values[0] = cylinderX;//pt_on_axis.x ();
//     cylinder_coeff.values[1] = cylinderY;//pt_on_axis.y ();
//     cylinder_coeff.values[2] = cylinderZ;//pt_on_axis.z ();
//     cylinder_coeff.values[3] = 0;//axis_direction.x ();
//     cylinder_coeff.values[4] = 0;//axis_direction.y ();
//     cylinder_coeff.values[5] = 0.25;//axis_direction.z ();
//     cylinder_coeff.values[6] = 0.5;//radius;
//     viewer->addCylinder (cylinder_coeff, "cylinder");

//     //viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0001, "cylinder");

//     viewer->setRepresentationToSurfaceForAllActors();

//     viewer->addArrow(pcl::PointXYZ(arrowX, arrowY, arrowZ), pcl::PointXYZ(cylinderX, cylinderY, cylinderZ+0.125), 1, 1, 1, false, "arrow");

//     cout << "c was pressed... adding a cylinder" << endl;
//   }
//   #if 0
//   if (event.getKeySym() == "w" && event.keyDown() && arrow == true)
//   {
//     viewer->removeShape("arrow");
//     viewer->addArrow(pcl::PointXYZ(0,3,0.125), pcl::PointXYZ(0,2.5,0.125), 1, 1, 1, false, "arrow");
//   }
//   #endif

//   // move in the positive y direction
//   if (event.getKeySym() == "Up" && event.keyDown() && cylinder == true) {
//     Eigen::Affine3f newPose;
//     cylinderY += 0.1;
//     arrowY += 0.1;
//     Eigen::Translation<float,3> translation = Eigen::Translation<float,3>(cylinderX, cylinderY, cylinderZ);
//     newPose = translation;
//     viewer->updateShapePose("cylinder", newPose);
//     viewer->removeShape("arrow");
//     viewer->addArrow(pcl::PointXYZ(arrowX, arrowY, arrowZ), pcl::PointXYZ(cylinderX, cylinderY, cylinderZ+0.125), 1, 1, 1, false, "arrow");
//     cout << "up arrow pressed (+y)" << endl;
//   }
  
//   // move in the positive x direction
//   if (event.getKeySym() == "Right" && event.keyDown() && cylinder == true) {
//     Eigen::Affine3f newPose;
//     cylinderX += 0.1;
//     arrowX += 0.1;
//     Eigen::Translation<float,3> translation = Eigen::Translation<float,3>(cylinderX, cylinderY, cylinderZ);
//     newPose = translation;
//     viewer->updateShapePose("cylinder", newPose);
//     viewer->removeShape("arrow");
//     viewer->addArrow(pcl::PointXYZ(arrowX, arrowY, arrowZ), pcl::PointXYZ(cylinderX, cylinderY, cylinderZ+0.125), 1, 1, 1, false, "arrow");
//     cout << "right arrow pressed (+x)" << endl;
//   }

//   // move in the negative y direction
//   if (event.getKeySym() == "Down" && event.keyDown() && cylinder == true) {
//     Eigen::Affine3f newPose;
//     cylinderY -= 0.1;
//     arrowY -= 0.1;
//     Eigen::Translation<float,3> translation = Eigen::Translation<float,3>(cylinderX, cylinderY, cylinderZ);
//     newPose = translation;
//     viewer->updateShapePose("cylinder", newPose);
//     viewer->removeShape("arrow");
//     viewer->addArrow(pcl::PointXYZ(arrowX, arrowY, arrowZ), pcl::PointXYZ(cylinderX, cylinderY, cylinderZ+0.125), 1, 1, 1, false, "arrow");
//     cout << "down arrow pressed (-y)" << endl;
//   }
  
//   // move in the negative x direction
//   if (event.getKeySym() == "Left" && event.keyDown() && cylinder == true) {
//     Eigen::Affine3f newPose;
//     cylinderX -= 0.1;
//     arrowX -= 0.1;
//     Eigen::Translation<float,3> translation = Eigen::Translation<float,3>(cylinderX, cylinderY, cylinderZ);
//     newPose = translation;
//     viewer->updateShapePose("cylinder", newPose);
//     viewer->removeShape("arrow");
//     viewer->addArrow(pcl::PointXYZ(arrowX, arrowY, arrowZ), pcl::PointXYZ(cylinderX, cylinderY, cylinderZ+0.125), 1, 1, 1, false, "arrow");
//     cout << "left arrow pressed (-x)" << endl;
//   }

//   // move in the positive z direction
//   if (event.getKeySym() == "a" && event.keyDown() && cylinder == true) {
//     Eigen::Affine3f newPose;
//     cylinderZ += 0.1;
//     arrowZ += 0.1;
//     Eigen::Translation<float,3> translation = Eigen::Translation<float,3>(cylinderX, cylinderY, cylinderZ);
//     newPose = translation;
//     viewer->updateShapePose("cylinder", newPose);
//     viewer->removeShape("arrow");
//     viewer->addArrow(pcl::PointXYZ(arrowX, arrowY, arrowZ), pcl::PointXYZ(cylinderX, cylinderY, cylinderZ+0.125), 1, 1, 1, false, "arrow");
//     cout << "a arrow pressed (+z)" << endl;
//   }

//   // move in the negative z direction
//   if (event.getKeySym() == "z" && event.keyDown() && cylinder == true) {
//     Eigen::Affine3f newPose;
//     cylinderZ -= 0.1;
//     arrowZ -= 0.1;
//     Eigen::Translation<float,3> translation = Eigen::Translation<float,3>(cylinderX, cylinderY, cylinderZ);
//     newPose = translation;
//     viewer->updateShapePose("cylinder", newPose);
//     viewer->removeShape("arrow");
//     viewer->addArrow(pcl::PointXYZ(arrowX, arrowY, arrowZ), pcl::PointXYZ(cylinderX, cylinderY, cylinderZ+0.125), 1, 1, 1, false, "arrow");
//     cout << "z arrow pressed (-z)" << endl;
//   }

//   // rotate in the positive theta direction
//   if (event.getKeySym() == "m" && event.keyDown() && cylinder == true) {
//     Eigen::Affine3f newPose;
//     angle += 10.0;
//     arrowX = cylinderX + sin(angle*3.14159/180.0);
//     arrowY = cylinderY + cos(angle*3.14159/180.0);
//     Eigen::Translation<float,3> translation = Eigen::Translation<float,3>(cylinderX, cylinderY, cylinderZ);
//     newPose = translation;
//     viewer->updateShapePose("cylinder", newPose);
//     viewer->removeShape("arrow");
//     viewer->addArrow(pcl::PointXYZ(arrowX, arrowY, arrowZ), pcl::PointXYZ(cylinderX, cylinderY, cylinderZ+0.125), 1, 1, 1, false, "arrow");
//     cout << "m arrow pressed (+theta)" << endl;
//   }

//   // rotate in the negative theta direction
//   if (event.getKeySym() == "n" && event.keyDown() && cylinder == true) {
//     Eigen::Affine3f newPose;
//     angle -= 10.0;
//     arrowX = cylinderX + sin(angle*3.14159/180.0);
//     arrowY = cylinderY + cos(angle*3.14159/180.0);
//     Eigen::Translation<float,3> translation = Eigen::Translation<float,3>(cylinderX, cylinderY, cylinderZ);
//     newPose = translation;
//     viewer->updateShapePose("cylinder", newPose);
//     viewer->removeShape("arrow");
//     viewer->addArrow(pcl::PointXYZ(arrowX, arrowY, arrowZ), pcl::PointXYZ(cylinderX, cylinderY, cylinderZ+0.125), 1, 1, 1, false, "arrow");
//     cout << "n arrow pressed (-theta)" << endl;
//   }

//   // Use keyboad presses to change the camera view
//   // move the camera angle up more
//   if (event.getKeySym() == "u" && event.keyDown()) {
//     viewer->camera_.pos[2] -= 1;
//     //viewer->camera_.view[0] -= 0.5;
//     viewer->updateCamera();
//   }

//   // move the camera angle down more
//   if (event.getKeySym() == "i" && event.keyDown()) {
//     //viewer->camera_.pos[2] += 1;
//     //viewer->updateCamera();
//     renderViewpoint9(viewer);
//   }

//   // move the camera angle left more
//   if (event.getKeySym() == "t" && event.keyDown()) {
//     //viewer->camera_.pos[1] -= 1;
//     //viewer->updateCamera();
//     renderViewpoint3(viewer);
//   }

//   // move the camera angle right more
//   if (event.getKeySym() == "y" && event.keyDown()) {
//     //viewer->camera_.pos[1] += 1;
//     //viewer->updateCamera();
//     renderViewpoint12(viewer);
//   }

//   // move the camera angle
//   if (event.getKeySym() == "x" && event.keyDown()) {
//     //viewer->camera_.pos[0] -= 1;
//     //viewer->updateCamera();
//     renderViewpoint6(viewer);
//   }

//   // move the camera angle
//   if (event.getKeySym() == "v" && event.keyDown()) {
//     viewer->camera_.pos[0] += 1;
//     viewer->updateCamera();
//   }

//   // camera translations
//   if (event.getKeySym() == "s" && event.keyDown()) {
//     //viewer->camera_.clip[0] -= 0.05;
//     //viewer->camera_.clip[1] -= 0.5;
//     //viewer->camera_.pos[0] += 1;
//     //viewer->camera_.pos[1] += 0.5;
//     //viewer->camera_.pos[2] -= 0.5;
//     renderViewpoint4(viewer);
//   }

//   if (event.getKeySym() == "d" && event.keyDown()) {
//     //viewer->camera_.clip[0] += 5;
//     //viewer->camera_.clip[1] += 3;
//     //viewer->camera_.pos[0] -= 5;
//     //viewer->camera_.pos[1] -= 1;
//     //viewer->camera_.pos[2] += 1;
//     //viewer->updateCamera();
//     renderViewpoint10(viewer);
//   }

//   if (event.getKeySym() == "f" && event.keyDown()) {
//     // 0.00789528,7.89528/0,0,0/-0.0219228,-0.00272186,-0.000408394/-0.0173165,-0.0105385,0.999795/0.523599/800,600/66,52
//     // 0.00791333,7.91333/-0.0511144,0.410011,0.0112097/-0.0832116,0.406026,0.0106118/-0.0173165,-0.0105385,0.999795/0.523599/800,600/66,52
//     viewer->camera_.focal[0] -= 0.05;
//     viewer->camera_.focal[1] += 0.5;
//     viewer->camera_.focal[2] += 0.01;
//     // viewer->camera_.pos[0] 
//     viewer->updateCamera();
//   }
// }

/////////////////////////////////// KEYBOARD TESTING ///////////////////////////////////////////

// find values for these later
// double posX = 0;
// double posY = 0;
// double posZ = 0;
// double viewX = 0.3;
// double viewY = 0;
// double viewZ = 0.3;
// double upX = 0.1;
// double upY = 0.1;
// double upZ = 0.1;

// void keyboardEventOccurred2 (const pcl::visualization::KeyboardEvent &event,
//                             void* viewer_void)
// {
//   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

//   //viewer->setCameraPosition(0.0, 0.0, 0.0, 3.0, 0.0, 3.0);//, 1.0, 1.0, 1.0);
//   //viewer->setCameraPosition(posX, posY, posZ, viewX, viewY, viewZ);

//   viewer->setCameraPose(posX, posY, posZ, viewX, viewY, viewZ, upX, upY, upZ);
//   //viewer->updateCamera();

//   // increase posX
//   if (event.getKeySym() == "X" && event.keyDown()) {
//     posX += 0.1;
//   }

//   // decrease posX
//   if (event.getKeySym() == "x" && event.keyDown()) {
//     posX -= 0.1;
//   }

//   // increase posY
//   if (event.getKeySym() == "Y" && event.keyDown()) {
//     posY += 0.1;
//   }

//   // decrease posY
//   if (event.getKeySym() == "y" && event.keyDown()) {
//     posY -= 0.1;
//   }

//   // increase posZ
//   if (event.getKeySym() == "Z" && event.keyDown()) {
//     posZ += 0.1;
//   }

//   // decrease posZ
//   if (event.getKeySym() == "z" && event.keyDown()) {
//     posZ -= 0.1;
//   }

//   // increase viewX
//   if (event.getKeySym() == "S" && event.keyDown()) {
//     viewX += 0.1;
//   }

//   // decrease viewX
//   if (event.getKeySym() == "s" && event.keyDown()) {
//     viewX -= 0.1;
//   }

//   // increase viewY
//   if (event.getKeySym() == "F" && event.keyDown()) {
//     viewY += 0.1;
//   }

//   // decrease viewY
//   if (event.getKeySym() == "f" && event.keyDown()) {
//     viewY -= 0.1;
//   }

//   // increase viewZ
//   if (event.getKeySym() == "T" && event.keyDown()) {
//     viewZ += 0.1;
//   }

//   // decrease viewZ
//   if (event.getKeySym() == "t" && event.keyDown()) {
//     viewZ -= 0.1;
//   }

//   // increase upX
//   if (event.getKeySym() == "V" && event.keyDown()) {
//     upX += 0.1;
//   }

//   // decrease upX
//   if (event.getKeySym() == "v" && event.keyDown()) {
//     upX -= 0.1;
//   }

//   // increase upY
//   if (event.getKeySym() == "D" && event.keyDown()) {
//     upY += 0.1;
//   }

//   // decrease upY
//   if (event.getKeySym() == "d" && event.keyDown()) {
//     upY -= 0.1;
//   }

//   // increase upZ
//   if (event.getKeySym() == "I" && event.keyDown()) {
//     upZ += 0.1;
//   }

//   // decrease upZ
//   if (event.getKeySym() == "i" && event.keyDown()) {
//     upZ -= 0.1;
//   }

// }


void keyboardEventOccurred3 (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

  //viewer->setCameraPosition(0.0, 0.0, 0.0, 3.0, 0.0, 3.0);//, 1.0, 1.0, 1.0);
  //viewer->setCameraPosition(posX, posY, posZ, viewX, viewY, viewZ);

  //viewer->updateCamera();
  
  //(vector<double> cameraCenter, vector<double> focal, vector<double> view_up, double focalLength, double angle)


  // vector<double> posCor(6);
  // vector<double> cameraCenter(3);
  // vector<double> focal(3);
  // vector<double> view_up(3);
  // double focalLength;

  

  // focalLength = sqrt(pow(cameraCenter[0]-focal[0], 2) + pow(cameraCenter[1]-focal[1], 2) + pow(cameraCenter[2]-focal[2], 2));


  if (event.getKeySym() == "n" && event.keyDown()) {
    viewer->saveScreenshot("/home/robotics/PCP/build/screenshots/!!!.png");
  }





  ///////////////////////

  // Initialize starting view point (move 1 unit in positive z, then roll 90 degrees, tilt 90 degrees)

  if (event.getKeySym() == "i" && event.keyDown()) {

      //viewer->camera_.pos[2] += -1;
      //viewer->camera_.focal[2] += -1;
      //viewer->updateCamera();

      vector<double> posCor(6);
      vector<double> cameraCenter(3);
      vector<double> focal(3);
      vector<double> view_up(3);

      double angle = 90;

      cameraCenter[0] = viewer->camera_.pos[0];
      cameraCenter[1] = viewer->camera_.pos[1];
      cameraCenter[2] = viewer->camera_.pos[2];

      focal[0] = viewer->camera_.focal[0];
      focal[1] = viewer->camera_.focal[1];
      focal[2] = viewer->camera_.focal[2];

      double focalLength = sqrt(pow(cameraCenter[0]-focal[0], 2) + pow(cameraCenter[1]-focal[1], 2) + pow(cameraCenter[2]-focal[2], 2));

      view_up[0] = viewer->camera_.view[0];
      view_up[1] = viewer->camera_.view[1];
      view_up[2] = viewer->camera_.view[2];

      posCor = localRoll(cameraCenter, focal, view_up, angle);

      viewer->camera_.view[0] = posCor[0];
      viewer->camera_.view[1] = posCor[1];
      viewer->camera_.view[2] = posCor[2];

      viewer->updateCamera();


      cameraCenter[0] = viewer->camera_.pos[0];
      cameraCenter[1] = viewer->camera_.pos[1];
      cameraCenter[2] = viewer->camera_.pos[2];

      focal[0] = viewer->camera_.focal[0];
      focal[1] = viewer->camera_.focal[1];
      focal[2] = viewer->camera_.focal[2];

      //focalLength = sqrt(pow(cameraCenter[0]-focal[0], 2) + pow(cameraCenter[1]-focal[1], 2) + pow(cameraCenter[2]-focal[2], 2));

      view_up[0] = viewer->camera_.view[0];
      view_up[1] = viewer->camera_.view[1];
      view_up[2] = viewer->camera_.view[2];

      posCor = localTilt(cameraCenter, focal, view_up, angle);

      viewer->camera_.focal[0] = posCor[0];
      viewer->camera_.focal[1] = posCor[1];
      viewer->camera_.focal[2] = posCor[2];

      viewer->camera_.view[0] = posCor[3];
      viewer->camera_.view[1] = posCor[4];
      viewer->camera_.view[2] = posCor[5];

      viewer->updateCamera();

      viewer->camera_.pos[2] += -1;
      viewer->camera_.focal[2] += -1;
      viewer->updateCamera();

    viewer->updateCamera();
    }

////////////////////////

if (event.getKeySym() == "I" && event.keyDown()) {

    vector<double> posCor(6);
    vector<double> cameraCenter(3);
    vector<double> focal(3);
    vector<double> view_up(3);

    cameraCenter[0] = viewer->camera_.pos[0];
    cameraCenter[1] = viewer->camera_.pos[1];
    cameraCenter[2] = viewer->camera_.pos[2];

    focal[0] = viewer->camera_.focal[0];
    focal[1] = viewer->camera_.focal[1];
    focal[2] = viewer->camera_.focal[2];

    double angle = 90;
    double focalLength = sqrt(pow(cameraCenter[0]-focal[0], 2) + pow(cameraCenter[1]-focal[1], 2) + pow(cameraCenter[2]-focal[2], 2));

    view_up[0] = viewer->camera_.view[0];
    view_up[1] = viewer->camera_.view[1];
    view_up[2] = viewer->camera_.view[2];

    posCor = localTilt(cameraCenter, focal, view_up, angle);

    viewer->camera_.focal[0] = posCor[0];
    viewer->camera_.focal[1] = posCor[1];
    viewer->camera_.focal[2] = posCor[2];

    viewer->camera_.view[0] = posCor[3];
    viewer->camera_.view[1] = posCor[4];
    viewer->camera_.view[2] = posCor[5];

    viewer->updateCamera();
}

if (event.getKeySym() == "k" && event.keyDown()) {
    viewer->camera_.pos[2] += -2;
    viewer->camera_.focal[2] += -2;
    viewer->updateCamera();

    viewer->updateCamera();
}

if (event.getKeySym() == "K" && event.keyDown()) {
    viewer->camera_.pos[2] += .5;
    viewer->camera_.focal[2] += .5;
    viewer->updateCamera();

    viewer->updateCamera();
}



  // print camera data members
  if (event.getKeySym() == "m" && event.keyDown()) {
    cerr << "Camera Center x: " << viewer->camera_.pos[0] << endl;
    cerr << "Camera Center y: " << viewer->camera_.pos[1] << endl;
    cerr << "Camera Center z: " << viewer->camera_.pos[2] << endl << endl;

    cerr << "Focal x: " << viewer->camera_.focal[0] << endl;
    cerr << "Focal y: " << viewer->camera_.focal[1] << endl;
    cerr << "Focal z: " << viewer->camera_.focal[2] << endl << endl;

    cerr << "View x: " << viewer->camera_.view[0] << endl;
    cerr << "View y: " << viewer->camera_.view[1] << endl;
    cerr << "View z: " << viewer->camera_.view[2] << endl << endl;
  }


  // translate x
  if (event.getKeySym() == "x" && event.keyDown()) {
    viewer->camera_.pos[0] += .5;
    viewer->camera_.focal[0] += .5;
    viewer->updateCamera();
  }

  // translate -x
  if (event.getKeySym() == "X" && event.keyDown()) {
    viewer->camera_.pos[0] -= .5;
    viewer->camera_.focal[0] -= .5;
    viewer->updateCamera();
  }

  // translate y
  if (event.getKeySym() == "y" && event.keyDown()) {
    viewer->camera_.pos[1] += .5;
    viewer->camera_.focal[1] += .5;
    viewer->updateCamera();
  }

  // translate -y
  if (event.getKeySym() == "Y" && event.keyDown()) {
    viewer->camera_.pos[1] -= .5;
    viewer->camera_.focal[1] -= .5;
    viewer->updateCamera();
  }

  // translate z
  if (event.getKeySym() == "z" && event.keyDown()) {
    viewer->camera_.pos[2] -= .5;
    viewer->camera_.focal[2] -= .5;
    viewer->updateCamera();
  }

  // translate -z
  if (event.getKeySym() == "Z" && event.keyDown()) {
    viewer->camera_.pos[2] += .5;
    viewer->camera_.focal[2] += .5;
    viewer->updateCamera();
  }

  

  if (event.getKeySym() == "p" && event.keyDown()) {

    vector<double> posCor(3);
    vector<double> cameraCenter(3);
    vector<double> focal(3);
    vector<double> view_up(3);

    double angle = 5;

    cameraCenter[0] = viewer->camera_.pos[0];
    cameraCenter[1] = viewer->camera_.pos[1];
    cameraCenter[2] = viewer->camera_.pos[2];

    focal[0] = viewer->camera_.focal[0];
    focal[1] = viewer->camera_.focal[1];
    focal[2] = viewer->camera_.focal[2];

    double focalLength = sqrt(pow(cameraCenter[0]-focal[0], 2) + pow(cameraCenter[1]-focal[1], 2) + pow(cameraCenter[2]-focal[2], 2));

    view_up[0] = viewer->camera_.view[0];
    view_up[1] = viewer->camera_.view[1];
    view_up[2] = viewer->camera_.view[2];

    // cerr << "On the outside: focal x is: " << focal[0] << endl;
    // cerr << "On the outside: focal y is: " << focal[1] << endl;
    // cerr << "On the outside: focal z is: " << focal[2] << endl;
    // cerr << endl;

    posCor = localPan(cameraCenter, focal, view_up, angle);

    // cerr << "Now on the outside: focal x is: " << posCor[0] << endl;
    // cerr << "Now on the outside: focal y is: " << posCor[1] << endl;
    // cerr << "Now on the outside: focal z is: " << posCor[2] << endl;
    // cerr << endl;

    viewer->camera_.focal[0] = posCor[0];
    viewer->camera_.focal[1] = posCor[1];
    viewer->camera_.focal[2] = posCor[2];

    viewer->updateCamera();
  }

  if (event.getKeySym() == "P" && event.keyDown()) {

    vector<double> posCor(3);
    vector<double> cameraCenter(3);
    vector<double> focal(3);
    vector<double> view_up(3);

    double angle = -5;

    cameraCenter[0] = viewer->camera_.pos[0];
    cameraCenter[1] = viewer->camera_.pos[1];
    cameraCenter[2] = viewer->camera_.pos[2];

    focal[0] = viewer->camera_.focal[0];
    focal[1] = viewer->camera_.focal[1];
    focal[2] = viewer->camera_.focal[2];

    double focalLength = sqrt(pow(cameraCenter[0]-focal[0], 2) + pow(cameraCenter[1]-focal[1], 2) + pow(cameraCenter[2]-focal[2], 2));

    view_up[0] = viewer->camera_.view[0];
    view_up[1] = viewer->camera_.view[1];
    view_up[2] = viewer->camera_.view[2];

    // cerr << "On the outside: focal x is: " << focal[0] << endl;
    // cerr << "On the outside: focal y is: " << focal[1] << endl;
    // cerr << "On the outside: focal z is: " << focal[2] << endl;
    // cerr << endl;

    posCor = localPan(cameraCenter, focal, view_up, angle);

    // cerr << "Now on the outside: focal x is: " << posCor[0] << endl;
    // cerr << "Now on the outside: focal y is: " << posCor[1] << endl;
    // cerr << "Now on the outside: focal z is: " << posCor[2] << endl;
    // cerr << endl;

    viewer->camera_.focal[0] = posCor[0];
    viewer->camera_.focal[1] = posCor[1];
    viewer->camera_.focal[2] = posCor[2];

    viewer->updateCamera();
  }

  if (event.getKeySym() == "t" && event.keyDown()) {

    vector<double> posCor(6);
    vector<double> cameraCenter(3);
    vector<double> focal(3);
    vector<double> view_up(3);

    double angle = 5;

    cameraCenter[0] = viewer->camera_.pos[0];
    cameraCenter[1] = viewer->camera_.pos[1];
    cameraCenter[2] = viewer->camera_.pos[2];

    focal[0] = viewer->camera_.focal[0];
    focal[1] = viewer->camera_.focal[1];
    focal[2] = viewer->camera_.focal[2];

    double focalLength = sqrt(pow(cameraCenter[0]-focal[0], 2) + pow(cameraCenter[1]-focal[1], 2) + pow(cameraCenter[2]-focal[2], 2));

    view_up[0] = viewer->camera_.view[0];
    view_up[1] = viewer->camera_.view[1];
    view_up[2] = viewer->camera_.view[2];

    // cerr << "On the outside: focal x is: " << focal[0] << endl;
    // cerr << "On the outside: focal y is: " << focal[1] << endl;
    // cerr << "On the outside: focal z is: " << focal[2] << endl;
    // cerr << endl;

    posCor = localTilt(cameraCenter, focal, view_up, angle);

    // cerr << "Now on the outside: focal x is: " << posCor[0] << endl;
    // cerr << "Now on the outside: focal y is: " << posCor[1] << endl;
    // cerr << "Now on the outside: focal z is: " << posCor[2] << endl;
    // cerr << endl;

    viewer->camera_.focal[0] = posCor[0];
    viewer->camera_.focal[1] = posCor[1];
    viewer->camera_.focal[2] = posCor[2];

    viewer->camera_.view[0] = posCor[3];
    viewer->camera_.view[1] = posCor[4];
    viewer->camera_.view[2] = posCor[5];

    viewer->updateCamera();
  }

  if (event.getKeySym() == "T" && event.keyDown()) {

    vector<double> posCor(6);
    vector<double> cameraCenter(3);
    vector<double> focal(3);
    vector<double> view_up(3);

    double angle = -5;

    cameraCenter[0] = viewer->camera_.pos[0];
    cameraCenter[1] = viewer->camera_.pos[1];
    cameraCenter[2] = viewer->camera_.pos[2];

    focal[0] = viewer->camera_.focal[0];
    focal[1] = viewer->camera_.focal[1];
    focal[2] = viewer->camera_.focal[2];

    double focalLength = sqrt(pow(cameraCenter[0]-focal[0], 2) + pow(cameraCenter[1]-focal[1], 2) + pow(cameraCenter[2]-focal[2], 2));

    view_up[0] = viewer->camera_.view[0];
    view_up[1] = viewer->camera_.view[1];
    view_up[2] = viewer->camera_.view[2];

    // cerr << "On the outside: focal x is: " << focal[0] << endl;
    // cerr << "On the outside: focal y is: " << focal[1] << endl;
    // cerr << "On the outside: focal z is: " << focal[2] << endl;
    // cerr << endl;

    posCor = localTilt(cameraCenter, focal, view_up, angle);

    // cerr << "Now on the outside: focal x is: " << posCor[0] << endl;
    // cerr << "Now on the outside: focal y is: " << posCor[1] << endl;
    // cerr << "Now on the outside: focal z is: " << posCor[2] << endl;
    // cerr << endl;

    viewer->camera_.focal[0] = posCor[0];
    viewer->camera_.focal[1] = posCor[1];
    viewer->camera_.focal[2] = posCor[2];

    viewer->camera_.view[0] = posCor[3];
    viewer->camera_.view[1] = posCor[4];
    viewer->camera_.view[2] = posCor[5];

    viewer->updateCamera();
  }

  if (event.getKeySym() == "r" && event.keyDown()) {

    vector<double> posCor(3);
    vector<double> cameraCenter(3);
    vector<double> focal(3);
    vector<double> view_up(3);

    double angle = 5;

    cameraCenter[0] = viewer->camera_.pos[0];
    cameraCenter[1] = viewer->camera_.pos[1];
    cameraCenter[2] = viewer->camera_.pos[2];

    focal[0] = viewer->camera_.focal[0];
    focal[1] = viewer->camera_.focal[1];
    focal[2] = viewer->camera_.focal[2];

    double focalLength = sqrt(pow(cameraCenter[0]-focal[0], 2) + pow(cameraCenter[1]-focal[1], 2) + pow(cameraCenter[2]-focal[2], 2));

    view_up[0] = viewer->camera_.view[0];
    view_up[1] = viewer->camera_.view[1];
    view_up[2] = viewer->camera_.view[2];

    // cerr << "On the outside: focal x is: " << focal[0] << endl;
    // cerr << "On the outside: focal y is: " << focal[1] << endl;
    // cerr << "On the outside: focal z is: " << focal[2] << endl;
    // cerr << endl;

    posCor = localRoll(cameraCenter, focal, view_up, angle);

    // cerr << "Now on the outside: focal x is: " << posCor[0] << endl;
    // cerr << "Now on the outside: focal y is: " << posCor[1] << endl;
    // cerr << "Now on the outside: focal z is: " << posCor[2] << endl;
    // cerr << endl;

    viewer->camera_.view[0] = posCor[0];
    viewer->camera_.view[1] = posCor[1];
    viewer->camera_.view[2] = posCor[2];

    viewer->updateCamera();
  }

  if (event.getKeySym() == "R" && event.keyDown()) {

    vector<double> posCor(3);
    vector<double> cameraCenter(3);
    vector<double> focal(3);
    vector<double> view_up(3);

    double angle = -5;

    cameraCenter[0] = viewer->camera_.pos[0];
    cameraCenter[1] = viewer->camera_.pos[1];
    cameraCenter[2] = viewer->camera_.pos[2];

    focal[0] = viewer->camera_.focal[0];
    focal[1] = viewer->camera_.focal[1];
    focal[2] = viewer->camera_.focal[2];

    double focalLength = sqrt(pow(cameraCenter[0]-focal[0], 2) + pow(cameraCenter[1]-focal[1], 2) + pow(cameraCenter[2]-focal[2], 2));

    view_up[0] = viewer->camera_.view[0];
    view_up[1] = viewer->camera_.view[1];
    view_up[2] = viewer->camera_.view[2];

    // cerr << "On the outside: focal x is: " << focal[0] << endl;
    // cerr << "On the outside: focal y is: " << focal[1] << endl;
    // cerr << "On the outside: focal z is: " << focal[2] << endl;
    // cerr << endl;

    posCor = localRoll(cameraCenter, focal, view_up, angle);

    // cerr << "Now on the outside: focal x is: " << posCor[0] << endl;
    // cerr << "Now on the outside: focal y is: " << posCor[1] << endl;
    // cerr << "Now on the outside: focal z is: " << posCor[2] << endl;
    // cerr << endl;

    viewer->camera_.view[0] = posCor[0];
    viewer->camera_.view[1] = posCor[1];
    viewer->camera_.view[2] = posCor[2];

    viewer->updateCamera();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////

// 1.75082,17.0692/0,0,0/-6.49882,-1.14148,1.31208/0.167169,0.0151057,0.985812/0.523599/800,600/66,52
// 61.5209,76.4805/0,0,0/-64.0116,-11.2433,12.9236/0.167169,0.0151057,0.985812/0.523599/800,600/66,52

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

bool stringCompare( const string &left, const string &right ){
   for( string::const_iterator lit = left.begin(), rit = right.begin(); lit != left.end() && rit != right.end(); ++lit, ++rit )
      if( tolower( *lit ) < tolower( *rit ) )
         return true;
      else if( tolower( *lit ) > tolower( *rit ) )
         return false;
   if( left.size() < right.size() )
      return true;
   return false;
}

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
  // Read from a config.txt
  // The first line is the path to the point cloud you want
  // The second line is the text file containing the transformation matrix
  // The third line is the directory path where we want to save the screenshots to
  ifstream infile;
  ifstream in_stream;
  vector<string> fileNames;
  string line;
  in_stream.open("/home/robotics/PCP/build/config.txt");

  while(!in_stream.eof()) {
      //cout << in_stream << endl;
      in_stream >> line;
      fileNames.push_back(line);
  }

  in_stream.close();

  string envCloud = fileNames[0];
  string positions = fileNames[1];
  string screenshot = fileNames[2];
  string posDirectory = fileNames[3];

  const char * pos;
  pos = positions.c_str();

  // Read the text file with camera positions in it
  ifstream infile2;
  ifstream in_stream2;
  vector<string> fileNames2;
  string line2;
  in_stream2.open(pos);

  while(!in_stream2.eof()) {
      //cout << in_stream2 << endl;
      in_stream2 >> line2;
      fileNames2.push_back(line2);
  }

  in_stream2.close();

#if 0
/////////////////// GET POSITIONS FROM FILES //////////////////////
  // Use the text file to get the new x, y, and z coordinates
  string X = fileNames2[0];
  string Y = fileNames2[1];
  string Z = fileNames2[2];

  float newX = ::atof(X.c_str());
  float newY = ::atof(Y.c_str());
  float newZ = ::atof(Z.c_str());

  string Rx = fileNames2[3];
  float xRot = ::atof(Rx.c_str());
  angle = xRot;
#endif
//#if 0
  // loading in a point cloud

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr environmentCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PCDReader reader;

  //reader.read(envCloud, *environmentCloud);

#if 0
  /////////////////////// FILTERING /////////////////////////
  sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2 ());
  sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2 ());

  reader.read(envCloud, *cloud);

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (.1f, .1f, .1f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

  pcl::PCDWriter writer;
  writer.write ("downsampled2.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
  #endif

  reader.read("downsampled.pcd", *environmentCloud);
//#endif

  /////////////////// END FILTERING /////////////////////

  // ------------------------------------
  // -----Create example point cloud-----
  // ------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  //std::cout << "Generating example point clouds.\n\n";
  // We're going to make an ellipse extruded along the z-axis. The colour for
  // the XYZRGB cloud will gradually go from red to green to blue.
  uint8_t r(255), g(15), b(15);
  for (float z(-1.0); z <= 1.0; z += 0.05)
  {
    for (float angle(0.0); angle <= 360.0; angle += 5.0)
    {
      pcl::PointXYZ basic_point;
      basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
      basic_point.y = sinf (pcl::deg2rad(angle));
      basic_point.z = z;
      basic_cloud_ptr->points.push_back(basic_point);

      pcl::PointXYZRGB point;
      point.x = basic_point.x;
      point.y = basic_point.y;
      point.z = basic_point.z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;

  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (point_cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.05);
  ne.compute (*cloud_normals1);

  // ---------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.1-----
  // ---------------------------------------------------------------
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.1);
  ne.compute (*cloud_normals2);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  viewer = rgbVis(point_cloud_ptr, environmentCloud);

  //viewer = plyLoader(point_cloud_ptr, envCloud);

  // Render different viewpoints and save screenshots
  // TimeStamp
          time_t t = time(0);   // get time now
          struct tm * now = localtime( & t );
          string timeStamp;

          string month = static_cast<ostringstream*>( &(ostringstream() << now->tm_mon + 1) )->str(); // months go from 0-11
          if (month.length() < 2)
              month = "0" + month;
          timeStamp += month;
          timeStamp += "_";
          string date = static_cast<ostringstream*>( &(ostringstream() << now->tm_mday) )->str();
          if (date.length() < 2)
              date = "0" + date;
          timeStamp += date;
          timeStamp += "_";\
          string year = static_cast<ostringstream*>( &(ostringstream() << now->tm_year + 1900) )->str(); // years since 1900
          timeStamp += year;
          timeStamp += ":";
          string hour = static_cast<ostringstream*>( &(ostringstream() << now->tm_hour) )->str();
          if (hour.length() < 2)
              hour = "0" + hour;
          timeStamp += hour;
          timeStamp += ":";
          string minute = static_cast<ostringstream*>( &(ostringstream() << now->tm_min) )->str();
          if (minute.length() < 2)
              minute = "0" + minute;
          timeStamp += minute;
          timeStamp += ":";
          string second = static_cast<ostringstream*>( &(ostringstream() << now->tm_sec) )->str();
          if (second.length() < 2)
              second = "0" + second;
          timeStamp += second;

          // create new directory to hold date's images
          string dirPath = screenshot;

          dirPath += "/";

          string visualFileName = dirPath;
          visualFileName += timeStamp;
          visualFileName += ".png";


  //resetOriginalViewpoint(viewer);
  //renderViewpoint1(viewer);
  //viewer->saveScreenshot(visualFileName);
  //resetOriginalViewpoint(viewer);

  //viewer->addArrow(pcl::PointXYZ(arrowX, arrowY, arrowZ), pcl::PointXYZ(cylinderX, cylinderY, cylinderZ+0.125), 1, 1, 1, false, "arrow");

  //Eigen::Vector3f pt_on_axis, axis_direction;
  //float radius;
  pcl::ModelCoefficients cylinder_coeff;
  cylinder_coeff.values.resize (7); // We need 7 values
  cylinder_coeff.values[0] = 0;//pt_on_axis.x ();
  cylinder_coeff.values[1] = 0;//pt_on_axis.y ();
  cylinder_coeff.values[2] = 0;//pt_on_axis.z ();
  cylinder_coeff.values[3] = 0;//axis_direction.x ();
  cylinder_coeff.values[4] = 0;//axis_direction.y ();
  cylinder_coeff.values[5] = 0.25;//axis_direction.z ();
  cylinder_coeff.values[6] = 0.5;//radius;

  //viewer->addCylinder (cylinder_coeff, "cylinder");

  viewer->setRepresentationToSurfaceForAllActors();





   ///////////////////////

    // Initialize starting view point (move 1 unit in positive z, then roll 90 degrees, tilt 90 degrees)

  // viewer->camera_.pos[2] += -1;
  // viewer->camera_.focal[2] += -1;
  // viewer->updateCamera();

  // vector<double> posCor(6);
  // vector<double> cameraCenter(3);
  // vector<double> focal(3);
  // vector<double> view_up(3);

  // double angle = 90;

  // cameraCenter[0] = viewer->camera_.pos[0];
  // cameraCenter[1] = viewer->camera_.pos[1];
  // cameraCenter[2] = viewer->camera_.pos[2];

  // focal[0] = viewer->camera_.focal[0];
  // focal[1] = viewer->camera_.focal[1];
  // focal[2] = viewer->camera_.focal[2];

  // double focalLength = sqrt(pow(cameraCenter[0]-focal[0], 2) + pow(cameraCenter[1]-focal[1], 2) + pow(cameraCenter[2]-focal[2], 2));

  // view_up[0] = viewer->camera_.view[0];
  // view_up[1] = viewer->camera_.view[1];
  // view_up[2] = viewer->camera_.view[2];

  // posCor = localRoll(cameraCenter, focal, view_up, angle);

  // viewer->camera_.view[0] = posCor[0];
  // viewer->camera_.view[1] = posCor[1];
  // viewer->camera_.view[2] = posCor[2];

  // viewer->updateCamera();


  // cameraCenter[0] = viewer->camera_.pos[0];
  // cameraCenter[1] = viewer->camera_.pos[1];
  // cameraCenter[2] = viewer->camera_.pos[2];

  // focal[0] = viewer->camera_.focal[0];
  // focal[1] = viewer->camera_.focal[1];
  // focal[2] = viewer->camera_.focal[2];

  // focalLength = sqrt(pow(cameraCenter[0]-focal[0], 2) + pow(cameraCenter[1]-focal[1], 2) + pow(cameraCenter[2]-focal[2], 2));

  // view_up[0] = viewer->camera_.view[0];
  // view_up[1] = viewer->camera_.view[1];
  // view_up[2] = viewer->camera_.view[2];

  // posCor = localTilt(cameraCenter, focal, view_up, angle);

  // viewer->camera_.focal[0] = posCor[0];
  // viewer->camera_.focal[1] = posCor[1];
  // viewer->camera_.focal[2] = posCor[2];

  // viewer->camera_.view[0] = posCor[3];
  // viewer->camera_.view[1] = posCor[4];
  // viewer->camera_.view[2] = posCor[5];

  // viewer->updateCamera();

  ////////////////////////


  

  //6.94083,32.012/0.125376,-3.43583,-0.0426443/-11.6865,-4.8284,8.11236/0.561569,0.0665266,0.824751/0.523599/840,525/66,52
#if 0
  viewer->camera_.clip[0] = 6.94083;
  viewer->camera_.clip[1] = 13;

  viewer->camera_.focal[0] = 0;
  viewer->camera_.focal[1] = 0;
  viewer->camera_.focal[2] = 0;

  viewer->camera_.pos[0] = -8;
  viewer->camera_.pos[1] = 0;
  viewer->camera_.pos[2] = 0;

  viewer->camera_.view[0] = 0;
  viewer->camera_.view[1] = 1.55;
  viewer->camera_.view[2] = 40;

  viewer->camera_.fovy = 0.523599;

  //Kristina lost some numbers here

  viewer->camera_.window_pos[0] = 66;
  viewer->camera_.window_pos[1] = 52;

  viewer->updateCamera();

  //viewer->setCameraPosition(0,0,0,0,0,0);
#endif
//////////////// Animation /////////////////

#if 0
  for (size_t i=0; i != fileNames2.size(); ++i) {
    cout << i << endl;
    const char * pos2;
    pos2 = fileNames2[i].c_str();
    ifstream infile3;
    ifstream in_stream3;
    vector<string> fileNames3;
    string line3;
    in_stream3.open(pos2);

    while(!in_stream3.eof()) {
      //cout << in_stream << endl;
      in_stream3 >> line3;
      fileNames3.push_back(line3);
    }

    in_stream3.close();

    string X = fileNames3[1];
    string Y = fileNames3[2];
    string Z = fileNames3[3];

    float newX = ::atof(X.c_str());
    float newY = ::atof(Y.c_str());
    float newZ = ::atof(Z.c_str());

    string Rx = fileNames3[4];
    float xRot = ::atof(Rx.c_str());
    angle = xRot;

    Eigen::Affine3f newPose;
    arrowY = sin(-angle);
    arrowX = cos(-angle);
    Eigen::Translation<float,3> translation = Eigen::Translation<float,3>(newX, newY, newZ);
    newPose = translation;
    viewer->updateShapePose("cylinder", newPose);
    viewer->removeShape("arrow");
    viewer->addArrow(pcl::PointXYZ(newX+arrowX, newY+arrowY, newZ+0.125), pcl::PointXYZ(newX, newY, newZ+0.125), i*50+100%255, i*200%255, i*300%255, false, "arrow");

    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (1000000));
  }
  #endif

  // 1.08623,17.0692/0,0,0/-6.49882,-1.14148,1.31208/0.202432,-0.0272943,0.978916/0.523599/800,600/66,52

  // 3.80149,15.3999/0,0,0/-6.57809,0.108406,1.40579/0.209568,0.0636295,0.975722/0.523599/800,600/66,52

  // 1.75082,13.072/0,0,0/-4.50044,-0.194277,0.468674/0.104939,-0.0331702,0.993925/0.523599/800,600/66,52

  // 3.80149,17.0692/0,0,0/-6.49882,0.108406,1.31208/0.202507,0.979281,0/0.523599/800,600/66,52

  

  viewer->registerKeyboardCallback (keyboardEventOccurred3, (void*)&viewer);

  //viewer->setCameraPose(0.1, 0.1, 0.1, 1, 1, 1, 1, 1, 1);

  #if 0

  viewer->addArrow(pcl::PointXYZ(arrowX, arrowY, arrowZ), pcl::PointXYZ(cylinderX, cylinderY, cylinderZ+0.125), 1, 1, 1, false, "arrow");

  //Eigen::Vector3f pt_on_axis, axis_direction;
  //float radius;
  pcl::ModelCoefficients cylinder_coeff;
  cylinder_coeff.values.resize (7); // We need 7 values
  cylinder_coeff.values[0] = 0;//pt_on_axis.x ();
  cylinder_coeff.values[1] = 0;//pt_on_axis.y ();
  cylinder_coeff.values[2] = 0;//pt_on_axis.z ();
  cylinder_coeff.values[3] = 0;//axis_direction.x ();
  cylinder_coeff.values[4] = 0;//axis_direction.y ();
  cylinder_coeff.values[5] = 0.25;//axis_direction.z ();
  cylinder_coeff.values[6] = 0.5;//radius;
  viewer->addCylinder (cylinder_coeff, "cylinder");

  viewer->setRepresentationToSurfaceForAllActors();

  Eigen::Affine3f newPose;
  arrowY = sin(angle*2*3.14159/180.0);
  arrowX = cos(angle*2*3.14159/180.0);
  Eigen::Translation<float,3> translation = Eigen::Translation<float,3>(newX, newY, newZ);
  newPose = translation;
  viewer->updateShapePose("cylinder", newPose);
  viewer->removeShape("arrow");
  viewer->addArrow(pcl::PointXYZ(arrowX, arrowY, newZ+0.125), pcl::PointXYZ(newX, newY, newZ+0.125), 1, 1, 1, true, "arrow");

  #endif

  

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
#include "frame.h"

std::vector<double> llh2ecef(std::vector<double> llh)
{
  // http://mathforum.org/library/drmath/view/51832.html
  // http://what-when-how.com/gps-with-high-rate-sensors/ecef-coordinate-systems-gps/
  std::vector<double> out;

  double lat = DEF_D2R(llh[0]); // latitude [Deg]
  double lon = DEF_D2R(llh[1]); // longitude [Deg]
  double h = llh[2];   // height [m]

  double a = 6378137.0;                 // the equatorial earth radius
  double b = 6356752.31424518;        // the polar cross-section earth radius
  double f = (a - b) / a;             // the "flattening" parameter
  double e = sqrt( (2*f) - (f*f)); // eccentricity e of the figure of the earth

  double C = 1 / sqrt( 1 - ((e*e) * (sin(lat)*sin(lat))) );
  double S = C * ( (1-f)*(1-f) );

  double x = (a*C + h) * cos(lat) * cos(lon);
  double y = (a*C + h) * cos(lat) * sin(lon);
  double z = (a*S + h) * sin(lat);

  out.push_back(x);
  out.push_back(y);
  out.push_back(z);

  std::cout<<"ECEF "<<x<<" "<<y<<" "<<z<<std::endl;

  return out;
}


std::vector<double> ecef2llh(std::vector<double> ecef)
{
  // LLH = Latitude[rad], Longitude[rad], Height[m]
  double x = ecef[0];
  double y = ecef[1];
  double z = ecef[2];

  double a = 6378137;                    // the equatorial earth radius
  double b = 6356752.31424518;           // the polar cross - section earth radius
  double f = (a - b) / a;                // the "flattening" parameter
  double e = sqrt(2.0f * f - pow(f, 2)); // eccentricity e of the figure of the earth

  double e_b = sqrt((pow(a, 2) - pow(b, 2)) / pow(b, 2));
  double p = sqrt(pow(x, 2) + pow(y, 2));
  double seta = atan2(z * a, p * b);

  double lon = atan2(y, x);
  double lat = atan2(z + pow(e_b, 2) * b * pow(sin(seta), 3), p - pow(e, 2) * a * pow(cos(seta), 3));
  double h = (p / cos(lat)) - (a / sqrt(1 - pow(e, 2) * pow(sin(lat), 2)));

  std::vector<double> llh{lat, lon, h};
  return llh;
}

std::vector<double> ecef2enu(std::vector<double> ecef,
                            std::vector<double> ori_ecef)
{
  std::vector<double> out;

  // ori_ecef is criterion point
  std::vector<double> ori_llh = ecef2llh(ori_ecef);
  double seta = ori_llh[0] - DEF_PI / 2; // Latitude[rad]
  double lam = -ori_llh[1] - DEF_PI / 2; // Longitude [rad]
  std::vector<double> diff;
  diff.push_back(ecef[0]-ori_ecef[0]);
  diff.push_back(ecef[1]-ori_ecef[1]);
  diff.push_back(ecef[2]-ori_ecef[2]);

  double sL = sin(seta);
  double cL = cos(seta);
  double sB = sin(lam);
  double cB = cos(lam);
  double x = diff[0];
  double y = diff[1];
  double z = diff[2];

  double e = (cB * x) + (-sB * y);
  double n = (cL * sB * x) + (cL * cB * y) + (-sL * z);
  double u = (sL * sB * x) + (sL * cB * y) + (cL * z);

  out.push_back(e);
  out.push_back(n);
  out.push_back(u);

  std::cout<<"ENU "<<e<<" "<<n<<" "<<u<<std::endl;

  return out;
}

std::vector<double> ecef2ned(std::vector<double> ecef,
                            std::vector<double> ori_ecef)
{
  std::vector<double> enu = ecef2enu(ecef, ori_ecef);
  std::vector<double> ned(3);
  ned[0] = enu[1];
  ned[1] = enu[0];
  ned[2] = -enu[2];
  
  std::cout<<"NED "<<ned[0]<<" "<<ned[1]<<" "<<ned[2]<<std::endl;

  return ned;
}

std::vector<double> ecef2en(std::vector<double> ecef,
                            std::vector<double> ori_ecef)
{
  std::vector<double> out;

  // ori_ecef is criterion point
  std::vector<double> ori_llh = ecef2llh(ori_ecef);
  double seta = ori_llh[0] - DEF_PI / 2; // Latitude[rad]
  double lam = -ori_llh[1] - DEF_PI / 2; // Longitude [rad]
  std::vector<double> diff;
  diff.push_back(ecef[0]-ori_ecef[0]);
  diff.push_back(ecef[1]-ori_ecef[1]);
  diff.push_back(ecef[2]-ori_ecef[2]);

  double sL = sin(seta);
  double cL = cos(seta);
  double sB = sin(lam);
  double cB = cos(lam);
  double x = diff[0];
  double y = diff[1];
  double z = diff[2];

  double e = (cB * x) + (-sB * y);
  double n = (cL * sB * x) + (cL * cB * y) + (-sL * z);
  double u = (sL * sB * x) + (sL * cB * y) + (cL * z);

  out.push_back(e);
  out.push_back(n);
  //out.push_back(u);
  std::cout<<"ENU "<<e<<" "<<n<<" "<<u<<std::endl;

  return out;
}


      
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <cmath>


constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

// Grid granularity for rounding UTM coordinates to generate MapXY.
constexpr double grid_size = 100000.0;  ///< 100 km grid

// WGS84 Parameters
#define WGS84_A 6378137.0         ///< major axis
#define WGS84_B 6356752.31424518  ///< minor axis
#define WGS84_F 0.003352810664747 /// 0.0033528107      ///< ellipsoid flattening
#define WGS84_E 0.0818191908      ///< first eccentricity
#define WGS84_EP 0.0820944379     ///< second eccentricity

// UTM Parameters
#define UTM_K0 0.9996                    ///< scale factor
#define UTM_FE 500000.0                  ///< false easting
#define UTM_FN_N 0.0                     ///< false northing, northern hemisphere
#define UTM_FN_S 10000000.0              ///< false northing, southern hemisphere
#define UTM_E2 (WGS84_E * WGS84_E)       ///< e^2
#define UTM_E4 (UTM_E2 * UTM_E2)         ///< e^4
#define UTM_E6 (UTM_E4 * UTM_E2)         ///< e^6
#define UTM_EP2 (UTM_E2 / (1 - UTM_E2))  ///< e'^2

// clang-format off
constexpr double f = WGS84_F;
constexpr double a = WGS84_A;
constexpr double n = f/(2.0-f);
constexpr double A = a/(1.0+n) * (1.0+ n*n/4.0 + n*n*n*n/64.0);
constexpr double alpha1 = 1.0/2.0*n - 2.0/3.0*n*n + 5.0/16.0*n*n*n;
constexpr double alpha2 = 13.0/48.0*n*n - 3.0/5.0*n*n*n;
constexpr double alpha3 = 61.0/240.0*n*n*n;
constexpr double beta1 = 1.0/2.0*n - 2.0/3.0*n*n + 37.0/96.0*n*n*n;
constexpr double beta2 = 1.0/48.0*n*n + 1.0/15.0*n*n*n;
constexpr double beta3 = 17.0/480.0*n*n*n;
constexpr double delta1 = 2.0*n - 2.0/3.0*n*n - 2.0*n*n*n;
constexpr double delta2 = 7.0/3.0*n*n - 8.0/5.0*n*n*n;
constexpr double delta3 = 56.0/15.0*n*n*n;
constexpr double E0 = UTM_FE;
constexpr double k0 = UTM_K0;
// clang-format on

static inline char utmLetterDesignator(double Lat) {
  char LetterDesignator;

  // clang-format off
  if     ((84 >= Lat) && (Lat >= 72))  LetterDesignator = 'X';
  else if ((72 > Lat) && (Lat >= 64))  LetterDesignator = 'W';
  else if ((64 > Lat) && (Lat >= 56))  LetterDesignator = 'V';
  else if ((56 > Lat) && (Lat >= 48))  LetterDesignator = 'U';
  else if ((48 > Lat) && (Lat >= 40))  LetterDesignator = 'T';
  else if ((40 > Lat) && (Lat >= 32))  LetterDesignator = 'S';
  else if ((32 > Lat) && (Lat >= 24))  LetterDesignator = 'R';
  else if ((24 > Lat) && (Lat >= 16))  LetterDesignator = 'Q';
  else if ((16 > Lat) && (Lat >= 8))   LetterDesignator = 'P';
  else if (( 8 > Lat) && (Lat >= 0))   LetterDesignator = 'N';
  else if (( 0 > Lat) && (Lat >= -8))  LetterDesignator = 'M';
  else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
  else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
  else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
  else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
  else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
  else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
  else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
  else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
  else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
  // 'Z' is an error flag, the Latitude is outside the UTM limits
  else LetterDesignator = 'Z';
  return LetterDesignator;
  // clang-format on
}

static inline void Wgs84toUtm(const double lon, const double lat, double* E, double* N, double* k,
                              double* gamma, char* zone) {
  double N0 = (lat < 0) ? UTM_FN_S:UTM_FN_N;
  double phi = lat;
  double lambda = lon;

  int zone_number;
  zone_number = int((lon * RAD_TO_DEG + 180) / 6) + 1;

  // +3 puts origin in middle of zone
  double lambda0 = ((zone_number - 1) * 6 - 180 + 3) * DEG_TO_RAD;

  // compute the UTM Zone from the latitude and longitude
  if (zone != nullptr) {
    sprintf(zone, "%d%c", zone_number, utmLetterDesignator(lat * RAD_TO_DEG));
  }

  // calc the intermediate value
  // clang-format off
  double t = sinh( atanh(sin(phi)) -  2.0*sqrt(n)/(1.0+n) * atanh( 2.0*sqrt(n)/(1.0+n) * sin(phi) ));
  double xi_quote = atan(t/cos(lambda-lambda0));
  double eta_quote = atanh(sin(lambda-lambda0)/sqrt(1.0+t*t));
  double sigma = 1.0 + 2.0*alpha1*cos(2.0*xi_quote)*cosh(2.0*eta_quote)+
                       4.0*alpha2*cos(4.0*xi_quote)*cosh(4.0*eta_quote)+
                       6.0*alpha3*cos(6.0*xi_quote)*cosh(6.0*eta_quote);
  double tao = 2.0*alpha1*sin(2.0*xi_quote)*sinh(2.0*eta_quote)+
               4.0*alpha2*sin(4.0*xi_quote)*sinh(4.0*eta_quote)+
               6.0*alpha3*sin(6.0*xi_quote)*sinh(6.0*eta_quote);

  if(E != nullptr){
    *E = E0 + k0*A*(eta_quote + alpha1*cos(2.0*xi_quote)*sinh(2.0*eta_quote)
                              + alpha2*cos(4.0*xi_quote)*sinh(4.0*eta_quote)
                              + alpha3*cos(6.0*xi_quote)*sinh(6.0*eta_quote)
                              );
  }

  if(N != nullptr){
    *N = N0 + k0*A*(xi_quote + alpha1*sin(2.0*xi_quote)*cosh(2.0*eta_quote)
                             + alpha2*sin(4.0*xi_quote)*cosh(4.0*eta_quote)
                             + alpha3*sin(6.0*xi_quote)*cosh(6.0*eta_quote)
                             );
  }

  if(k != nullptr){
    *k = k0*A/a*sqrt((1.0 + ((1.0-n)/(1.0+n)*tan(phi))*((1.0-n)/(1.0+n)*tan(phi))) * (sigma*sigma+tao*tao)/(t*t + cos(lambda-lambda0)*cos(lambda-lambda0)));
  }

  if(gamma != nullptr){
    *gamma = atan( (tao*sqrt(1.0+t*t) + sigma*t*tan(lambda-lambda0))/(sigma*sqrt(1.0+t*t) - tao*t*tan(lambda-lambda0)));
  }
  // clang-format on
}

static inline void UtmtoWgs84(const double E, const double N, const char* zone, double* lon,
                              double* lat, double* k, double* gamma) {
  char* zone_letter;
  int zone_number = strtoul(zone, &zone_letter, 10);

  // clang-format off
  double hemi = ((*zone_letter - 'N')) < 0 ? -1.0 : 1.0;
  double N0 = ((*zone_letter - 'N') < 0) ? UTM_FN_S : UTM_FN_N;
  double lambda0 = ((zone_number - 1) * 6 - 180 + 3) * DEG_TO_RAD;

  double xi = (N - N0) / (k0 * A);
  double eta = (E-E0)/(k0*A);

  double xi_quote = xi - beta1*sin(2.0*xi)*cosh(2.0*eta)-
                         beta2*sin(4.0*xi)*cosh(4.0*eta)-
                         beta3*sin(6.0*xi)*cosh(6.0*eta);

  double eta_quote = eta - beta1*cos(2.0*xi)*sinh(2.0*eta)-
                           beta2*cos(4.0*xi)*sinh(4.0*eta)-
                           beta3*cos(6.0*xi)*sinh(6.0*eta);

  double sigma_quote = 1.0 - 2.0*beta1*cos(2.0*xi)*cosh(2.0*eta)-
                             4.0*beta2*cos(4.0*xi)*cosh(4.0*eta)-
                             6.0*beta3*cos(6.0*xi)*cosh(6.0*eta);

  double tao_quote = 2.0*beta1*sin(2.0*xi)*sinh(2.0*eta)+
                     4.0*beta2*sin(4.0*xi)*sinh(4.0*eta)+
                     6.0*beta3*sin(6.0*xi)*sinh(6.0*eta);

  double chi = asin(sin(xi_quote)/cosh(eta_quote));

  double phi = chi + delta1*sin(2.0*chi)+
                     delta2*sin(4.0*chi)+
                     delta3*sin(6.0*chi);

  if(lat != nullptr){
    *lat = phi;
  }

  if(lon != nullptr){
    double lambda = lambda0 + atan(sinh(eta_quote)/cos(xi_quote));
    *lon = lambda;
  }

  if(k != nullptr){
    *k = k0*A/a*sqrt( (1.0 + ((1.0-n)/(1.0+n)*tan(phi))*((1.0-n)/(1.0+n)*tan(phi)) )*((cos(xi_quote)*cos(xi_quote) + sinh(eta_quote)*sinh(eta_quote))/(sigma_quote*sigma_quote + tao_quote*tao_quote)) );
  }

  if(gamma != nullptr){
    *gamma = hemi * atan((tao_quote + sigma_quote*tan(xi_quote)*tanh(eta_quote))/(sigma_quote - tao_quote*tan(xi_quote)*tanh(eta_quote)));
  }
  // clang-format on
}

    
/**************************************************************************
 * Module: vincenty (direct).
 *
 * Calculate WGS 84 destination given starting lat/long (degrees), 
 * bearing (degrees) & distance (Meters).
 *
 * from: Vincenty direct formula - T Vincenty, "Direct and Inverse 
 * Solutions of Geodesics on the Ellipsoid with application of 
 * nested equations", Survey Review, vol XXII no 176, 1975
 *       http://www.ngs.noaa.gov/PUBS_LIB/inverse.pdf
 *
 * Ported from web java script implementation.  Code standard is a bit
 * odd, but it's efficient and closely parallels the alg equations.
 *
 * Doug Summers		Nov 2010
 *
 * 
 * Vincenty’s formula used in the code:
 * 
 * a, b = major & minor semiaxes of the ellipsoid 	 
 * f = flattening (a−b)/a 	 
 * φ1, φ2 = geodetic latitude 	 
 * s = length of the geodesic 	 
 * α1, α2 = azimuths of the geodesic (initial/final bearing) 	 
 *   	 
 * tanU1 = (1−f).tanφ1 (U is ‘reduced latitude’) 	 
 * cosU1 = 1/√(1+tan²U1), sinU1 = tanU1.cosU1 (trig identities; §6) 	 
 * σ1 = atan2(tanU1, cosα1) 	                                (1)
 * sinα = cosU1.sinα1 	                                        (2)
 * cos²α = 1 − sin²α                            (trig identity; §6) 	 
 * u² = cos²α.(a²−b²)/b² 	 
 * A = 1+u²/16384.{4096+u².[−768+u².(320−175.u²)]} 	        (3)
 * B = u²/1024.{256+u².[−128+u².(74−47.u²)]} 	                (4)
 *   	 
 * σ = s / b.A (1st approximation), σ′ = 2π 	 
 * while fabs(σ−σ′) > 10-12 { (i.e. 0.06mm) 	 
 *      	cos2σm = cos(2.σ1 + σ) 	                        (5)
 *   	Δσ = B.sinσ.{cos2σm + B/4.[cosσ.(−1 + 2.cos²2σm) − 
 *           B/6.cos2σm.(−3 + 4.sin²σ).(−3 + 4.cos²2σm)]} 	(6)
 *   	σ′ = σ 	 
 *   	σ = s / b.A + Δσ 	                                (7)
 * } 	  	 
 * φ2 = atan2(sinU1.cosσ + cosU1.sinσ.cosα1, 
 *            (1−f).√[sin²α + (sinU1.sinσ − cosU1.cosσ.cosα1)²])(8)
 * λ = atan2(sinσ.sinα1, cosU1.cosσ − sinU1.sinσ.cosα1) 	(9)
 * C = f/16.cos²α.[4+f.(4−3.cos²α)] 	                        (10)
 * L = λ − (1−C).f.sinα.{
 *     σ+C.sinσ.[cos2σm + C.cosσ.(−1 + 2.cos²2σm)]} (delta lon)	(11)
 * α2 = atan2(sinα, −sinU1.sinσ + cosU1.cosσ.cosα1) (reverse az)(12)
 * p2 = (φ2, λ1+L) 	 
 */
  	 
#include <stdio.h>
#include <math.h>
#include <cmath>
#include "slamac.h"
#include "Vincenty.h"

using namespace std;

Vincenty::Vincenty()
{                     
}

// Inputs (and outputs) are in Degrees, Meters
void Vincenty::destVincenty(double lat1, double lon1, double bearing, double dist, double *lat2out, double *lon2out) 
{
  cout << "destVincenty" << endl;
  /* local variable definitions */
  
  // WGS-84 ellipsiod
  double a=6378137.0, b=6356752.3142, f=1/298.257223563;
  double alpha1,sinAlpha, sinAlpha1, cosAlpha1, cosSqAlpha;
  double sigma, sigma1, cos2SigmaM, sinSigma, cosSigma, deltaSigma, sigmaP;
  double tanU1, cosU1, sinU1, uSq;
  double A, B, C, L, lambda;
  double tmp, lat2;
  //double revAz;   /* unused but retained for alg completeness */

  /* code body */
  cout << "code body" << endl;
  alpha1 = bearing*DD2R;
  sinAlpha1 = sin(alpha1);
  cosAlpha1 = cos(alpha1);

  tanU1 = (1-f) * tan(lat1*DD2R);
  cosU1 = 1 / sqrt((1 + tanU1*tanU1));
  sinU1 = tanU1*cosU1;
  sigma1 = atan2(tanU1, cosAlpha1);
  sinAlpha = cosU1 * sinAlpha1;
  cosSqAlpha = 1 - sinAlpha*sinAlpha;
  uSq = cosSqAlpha * (a*a - b*b) / (b*b);
  A = 1 + uSq/16384*(4096+uSq*(-768+uSq*(320-175*uSq)));
  B = uSq/1024 * (256+uSq*(-128+uSq*(74-47*uSq)));

  sigma = dist / (b*A);
  sigmaP = 2*DPI;
  while (fabs(sigma-sigmaP) > 1e-12) {
    cos2SigmaM = cos(2*sigma1 + sigma);
    sinSigma = sin(sigma);
    cosSigma = cos(sigma);
    deltaSigma = B*sinSigma*(cos2SigmaM+B/4*(cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)-B/6*cos2SigmaM*(-3+4*sinSigma*sinSigma)*(-3+4*cos2SigmaM*cos2SigmaM)));
    sigmaP = sigma;
    sigma = dist / (b*A) + deltaSigma;
  }

  tmp = sinU1*sinSigma - cosU1*cosSigma*cosAlpha1;
  lat2 = atan2(sinU1*cosSigma + cosU1*sinSigma*cosAlpha1, 
      (1-f)*sqrt(sinAlpha*sinAlpha + tmp*tmp));
  lambda = atan2(sinSigma*sinAlpha1, 
                 cosU1*cosSigma - sinU1*sinSigma*cosAlpha1);
  C = f/16*cosSqAlpha*(4+f*(4-3*cosSqAlpha));
  L = lambda - (1-C)*f*sinAlpha*(sigma+C*sinSigma*(cos2SigmaM+C*cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)));

  // final bearing 
  // revAz = atan2(sinAlpha, -tmp);

  // algorithm convention uses Deg outputs 
  *lat2out = lat2*DR2D;
  *lon2out = lon1+(L*DR2D);
  //*lat2out = lat2;
  //*lon2out = lon1*DD2R+L;
}

double Vincenty::distVincenty(double lat1, double lon1, double lat2, double lon2)
{
    // WGS-84 ellipsiod
    double a=6378137.0, b=6356752.3142, f=1/298.257223563;
    lat1 = lat1*DD2R;
    lon1 = lon1*DD2R;
    lat2 = lat2*DD2R;
    lon2 = lon2*DD2R;

    //a, b = major & minor semiaxes of the ellipsoid
    //f = flattening (a-b)/a
    double L = lon2 - lon1;
    double u1 = atan((1-f) * tan(lat1));
    double u2 = atan((1-f) * tan(lat2));
    double sin_u1 = sin(u1);
    double cos_u1 = cos(u1);
    double sin_u2 = sin(u2);
    double cos_u2 = cos(u2);
    double lambda = L;
    double lambda_pi = 2*DPI;

    double sin_lambda, cos_lambda, sin_sigma, cos_sigma, sigma, alpha, cos_sq_alpha, cos2sigma_m, cc; 
    while (abs(lambda-lambda_pi) > 1e-12){
        sin_lambda = sin(lambda);
        cos_lambda = cos(lambda);
        sin_sigma = sqrt((cos_u2 * sin_lambda) * (cos_u2*sin_lambda) + 
            (cos_u1*sin_u2-sin_u1*cos_u2*cos_lambda) * (cos_u1*sin_u2-sin_u1*cos_u2*cos_lambda));
        cos_sigma = sin_u1*sin_u2 + cos_u1*cos_u2*cos_lambda;
        sigma = atan2(sin_sigma, cos_sigma);
        alpha = asin(cos_u1 * cos_u2 * sin_lambda / sin_sigma);
        cos_sq_alpha = cos(alpha) * cos(alpha);
        cos2sigma_m = cos_sigma - 2*sin_u1*sin_u2/cos_sq_alpha;
        cc = f/16*cos_sq_alpha*(4+f*(4-3*cos_sq_alpha));
        lambda_pi = lambda;
        lambda = L + (1-cc) * f * sin(alpha) *
            (sigma + cc*sin_sigma*(cos2sigma_m+cc*cos_sigma*(-1+2*cos2sigma_m*cos2sigma_m)));
    }
    double usq = cos_sq_alpha*(a*a-b*b)/(b*b);
    double aa = 1 + usq/16384*(4096+usq*(-768+usq*(320-175*usq)));
    double bb = usq/1024 * (256+usq*(-128+usq*(74-47*usq)));
    double delta_sigma = bb*sin_sigma*(cos2sigma_m+bb/4*(cos_sigma*(-1+2*cos2sigma_m*cos2sigma_m)-
      bb/6*cos2sigma_m*(-3+4*sin_sigma*sin_sigma)*(-3+4*cos2sigma_m*cos2sigma_m)));
    double c = b*aa*(sigma-delta_sigma);
    return c;
}

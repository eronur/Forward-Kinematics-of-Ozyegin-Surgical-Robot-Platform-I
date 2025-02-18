#Forward Kinematics of Ozyegin Surgical Robot Platform I by Onur Ersoy-17.09.2018 

#include "FKin.h"
#include "FKin_data.h"
#include "FKin_initialize.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/*
 * forward kinematic functions
 * link lengths
 * Arguments    : double q1,q2,q3,q4,q5;
 *                double *x,*y,*z,*phi,*theta,*ksi;
 * Return Type  : void
 */
void FKin(double q1, double q2, double q3, double q4, double q5, double *x,
          double *y, double *z, double *phi, double *theta, double *ksi)
{
  static const double dv[360] = { 0.0, 0.0175, 0.035, 0.052500000000000005, 0.07,
    0.087500000000000008, 0.10500000000000001, 0.12250000000000001, 0.14,
    0.15750000000000003, 0.17500000000000002, 0.1925, 0.21000000000000002,
    0.22750000000000004, 0.24500000000000002, 0.2625, 0.28, 0.29750000000000004,
    0.31500000000000006, 0.3325, 0.35000000000000003, 0.36750000000000005, 0.385,
    0.4025, 0.42000000000000004, 0.43750000000000006, 0.45500000000000007,
    0.47250000000000003, 0.49000000000000005, 0.50750000000000006, 0.525,
    0.54250000000000009, 0.56, 0.5775, 0.59500000000000008, 0.6125,
    0.63000000000000012, 0.64750000000000008, 0.665, 0.68250000000000011,
    0.70000000000000007, 0.7175, 0.7350000000000001, 0.75250000000000006, 0.77,
    0.78750000000000009, 0.805, 0.82250000000000012, 0.84000000000000008, 0.8575,
    0.87500000000000011, 0.89250000000000007, 0.91000000000000014,
    0.9275000000000001, 0.94500000000000006, 0.96250000000000013,
    0.98000000000000009, 0.9975, 1.0150000000000001, 1.0325000000000002, 1.05,
    1.0675000000000001, 1.0850000000000002, 1.1025, 1.12, 1.1375000000000002,
    1.155, 1.1725, 1.1900000000000002, 1.2075, 1.225, 1.2425000000000002,
    1.2600000000000002, 1.2775, 1.2950000000000002, 1.3125000000000002, 1.33,
    1.3475000000000001, 1.3650000000000002, 1.3825, 1.4000000000000001,
    1.4175000000000002, 1.435, 1.4525000000000001, 1.4700000000000002, 1.4875,
    1.5050000000000001, 1.5225000000000002, 1.54, 1.5575, 1.5750000000000002,
    1.5925000000000002, 1.61, 1.6275000000000002, 1.6450000000000002, 1.6625,
    1.6800000000000002, 1.6975000000000002, 1.715, 1.7325000000000002,
    1.7500000000000002, 1.7675, 1.7850000000000001, 1.8025000000000002,
    1.8200000000000003, 1.8375000000000001, 1.8550000000000002,
    1.8725000000000003, 1.8900000000000001, 1.9075000000000002,
    1.9250000000000003, 1.9425000000000001, 1.9600000000000002,
    1.9775000000000003, 1.995, 2.0125, 2.0300000000000002, 2.0475000000000003,
    2.0650000000000004, 2.0825, 2.1, 2.1175, 2.1350000000000002,
    2.1525000000000003, 2.1700000000000004, 2.1875, 2.205, 2.2225, 2.24,
    2.2575000000000003, 2.2750000000000004, 2.2925000000000004, 2.31, 2.3275,
    2.345, 2.3625000000000003, 2.3800000000000003, 2.3975000000000004, 2.415,
    2.4325, 2.45, 2.4675000000000002, 2.4850000000000003, 2.5025000000000004,
    2.5200000000000005, 2.5375, 2.555, 2.5725000000000002, 2.5900000000000003,
    2.6075000000000004, 2.6250000000000004, 2.6425, 2.66, 2.6775,
    2.6950000000000003, 2.7125000000000004, 2.7300000000000004, 2.7475, 2.765,
    2.7825, 2.8000000000000003, 2.8175000000000003, 2.8350000000000004,
    2.8525000000000005, 2.87, 2.8875, 2.9050000000000002, 2.9225000000000003,
    2.9400000000000004, 2.9575000000000005, 2.975, 2.9925, 3.0100000000000002,
    3.0275000000000003, 3.0450000000000004, 3.0625000000000004, 3.08, 3.0975,
    3.115, 3.1325000000000003, 3.1500000000000004, 3.1675000000000004,
    3.1850000000000005, 3.2025000000000006, 3.22, 3.2375000000000003,
    3.2550000000000003, 3.2725000000000004, 3.2900000000000005,
    3.3075000000000006, 3.325, 3.3425000000000002, 3.3600000000000003,
    3.3775000000000004, 3.3950000000000005, 3.4125000000000005, 3.43,
    3.4475000000000002, 3.4650000000000003, 3.4825000000000004,
    3.5000000000000004, 3.5175000000000005, 3.5350000000000006, 3.5525,
    3.5700000000000003, 3.5875000000000004, 3.6050000000000004,
    3.6225000000000005, 3.6400000000000006, 3.6575, 3.6750000000000003,
    3.6925000000000003, 3.7100000000000004, 3.7275000000000005,
    3.7450000000000006, 3.7625, 3.7800000000000002, 3.7975000000000003,
    3.8150000000000004, 3.8325000000000005, 3.8500000000000005,
    3.8675000000000006, 3.8850000000000002, 3.9025000000000003,
    3.9200000000000004, 3.9375000000000004, 3.9550000000000005,
    3.9725000000000006, 3.99, 4.0075, 4.025, 4.0425, 4.0600000000000005,
    4.0775000000000006, 4.0950000000000006, 4.1125000000000007,
    4.1300000000000008, 4.1475000000000009, 4.1650000000000009,
    4.182500000000001, 4.2000000000000011, 4.2175, 4.235, 4.2525,
    4.2700000000000005, 4.2875000000000005, 4.3050000000000006,
    4.3225000000000007, 4.3400000000000007, 4.3575, 4.375, 4.3925, 4.41, 4.4275,
    4.445, 4.4625, 4.48, 4.4975000000000005, 4.5150000000000006,
    4.5325000000000006, 4.5500000000000007, 4.5675000000000008,
    4.5850000000000009, 4.6025000000000009, 4.620000000000001, 4.6375, 4.655,
    4.6725, 4.69, 4.7075000000000005, 4.7250000000000005, 4.7425000000000006,
    4.7600000000000007, 4.7775000000000007, 4.7950000000000008, 4.8125, 4.83,
    4.8475, 4.865, 4.8825, 4.9, 4.9175, 4.9350000000000005, 4.9525000000000006,
    4.9700000000000006, 4.9875000000000007, 5.0050000000000008,
    5.0225000000000009, 5.0400000000000009, 5.057500000000001,
    5.0750000000000011, 5.0925, 5.11, 5.1275, 5.1450000000000005,
    5.1625000000000005, 5.1800000000000006, 5.1975000000000007,
    5.2150000000000007, 5.2325000000000008, 5.25, 5.2675, 5.285, 5.3025, 5.32,
    5.3375, 5.355, 5.3725000000000005, 5.3900000000000006, 5.4075000000000006,
    5.4250000000000007, 5.4425000000000008, 5.4600000000000009,
    5.4775000000000009, 5.495000000000001, 5.5125000000000011, 5.53, 5.5475,
    5.565, 5.5825000000000005, 5.6000000000000005, 5.6175000000000006,
    5.6350000000000007, 5.6525000000000007, 5.6700000000000008,
    5.6875000000000009, 5.705000000000001, 5.7225, 5.74, 5.7575, 5.775, 5.7925,
    5.8100000000000005, 5.8275000000000006, 5.8450000000000006,
    5.8625000000000007, 5.8800000000000008, 5.8975000000000009,
    5.9150000000000009, 5.932500000000001, 5.9500000000000011, 5.9675, 5.985,
    6.0025, 6.0200000000000005, 6.0375000000000005, 6.0550000000000006,
    6.0725000000000007, 6.0900000000000007, 6.1075000000000008,
    6.1250000000000009, 6.142500000000001, 6.160000000000001, 6.1775, 6.195,
    6.2125, 6.23, 6.2475000000000005, 6.2650000000000006, 6.2825000000000006 };

  static const signed char b_b[9] = { -1, 0, 0, 0, -1, 0, 0, 0, 1 };

  static const signed char c_b[9] = { -1, 0, 0, 0, 1, 0, 0, 0, -1 };

  static const signed char d_b[3] = { 1, 0, 0 };

  double V_rotper[1080],x2[360],y2[360],z2[360],X1[3],Y1[3],Z1[3],V_per_idx_1;
  double alpha,mag,q,s,t,x_tmp,x_up,y_tmp,y_up,z_up;
  int b_i;
  int i;
  int k;
  int partialTrueCount;
  int trueCount;
  short tmp_data[360];
  signed char cond4[360];
  boolean_T b;
  boolean_T b1;
  
  if (!isInitialized_FKin) {
    FKin_initialize();
  }

  /* FK OSR Forward Kinematics */
  /* mm */
  /* mm */
  /* mm */
  /* mm */
  /* mm */
  /* mm constant between 2 stages */
  /* mm endowrist length from the joint */
  /* 0:false, 1:true %Origin base'de ise 1 seç */
  /*  forward kinematic functions */
  s = (255.0 * sin(q1 - 3.1415926535897931) + 255.0 * sin(6.2831853071795862 -
        q2)) + 103.0 * sin(q2 - 3.9269908169872414);
  z_up = s * sin(q3);
  y_up = s * cos(q3);

  /* (2*l*cos(pi/4)+r) offsettir, bu base'deki bir origin icin cikarilabilir */
  x_up = (255.0 * cos(q1 - 3.1415926535897931) - 255.0 * cos(6.2831853071795862
           - q2)) + 103.0 * cos(q2 - 3.9269908169872414);
  q = sqrt(63625.0 - 29400.0 * cos(q4 - 3.1415926535897931));
  mag = q4 - q5;
  s = sin(mag);
  t = 60.0 * sin(6.2831853071795862 - q4) / s;
  V_per_idx_1 = sin(q5 - 3.1415926535897931);
  s = 60.0 * V_per_idx_1 / s;
  s = sqrt(((t + 245.0) * (t + 245.0) + (s + 245.0) * (s + 245.0)) - 2.0 * (t +
            245.0) * (s + 245.0) * cos(mag));
  mag = s * s;
  alpha = (3.1415926535897931 - acos((mag - 120050.0) / -120050.0)) / 2.0;
  s = acos(((q * q - 60025.0) - mag) / (-490.0 * s));

  /* y position of jl3 */
  /* x position of jl3 */
  /* endoscope direction */
  mag = ((3.9269908169872414 - q5) + alpha) + s;
  q = (((245.0 * cos(q5 - 3.1415926535897931) + 30.0) - 245.0 * cos
        (((3.1415926535897931 - q5) + alpha) + s)) - 125.0 * cos(mag)) - x_up;

  /* (x-x1)/xd=(y-y1)/yd=(z-z1)/zd */
  s = ((245.0 * V_per_idx_1 + 245.0 * sin((alpha + s) - (q5 - 3.1415926535897931)))
       + 125.0 * sin(mag)) - y_up;
  mag = sqrt((q * q + s * s) + (-282.5 - z_up) * (-282.5 - z_up));

  /*  endoscope_u=[xd/mag yd/mag zd/mag] %unit vector of endoscope */
  x_tmp = q / mag;
  *x = x_tmp * 540.0 + x_up;
  y_tmp = s / mag;
  *y = y_tmp * 540.0 + y_up;
  y_up = (-282.5 - z_up) / mag;
  *z = y_up * 540.0 + z_up;

  /* Origin Frame */
  /* unit vector of Z1 */
  Z1[0] = x_tmp;
  Z1[1] = y_tmp;
  Z1[2] = y_up;

  /* Creating a vector that is perpedicular to Y1 */
  /*  [x2,y2,z2] = vpasolve(0 == x1*x2+y1*y2+z1*z2, 1 == sqrt(x2^2+y2^2+z2^2)); */
  /* Random vector that lies on the plane that perpendicular to surgical tool */
  /* direction */
  /*  V = rand(1,3); */
  X1[0] = y_tmp * 0.33374025054106188 - y_up * 0.66723546418745339;
  X1[1] = y_up * 0.6658935954785441 - x_tmp * 0.33374025054106188;
  X1[2] = x_tmp * 0.66723546418745339 - y_tmp * 0.6658935954785441;
  q = -y_tmp * X1[2] - -y_up * X1[1];
  V_per_idx_1 = -y_up * X1[0] - -x_tmp * X1[2];
  x_up = -x_tmp * X1[1] - -y_tmp * X1[0];
  s = 3.3121686421112381E-170;
  mag = fabs(q);
  if (mag > 3.3121686421112381E-170) {
    alpha = 1.0;
    s = mag;
  } else {
    t = mag / 3.3121686421112381E-170;
    alpha = t * t;
  }

  mag = fabs(V_per_idx_1);
  if (mag > s) {
    t = s / mag;
    alpha = alpha * t * t + 1.0;
    s = mag;
  } else {
    t = mag / s;
    alpha += t * t;
  }

  mag = fabs(x_up);
  if (mag > s) {
    t = s / mag;
    alpha = alpha * t * t + 1.0;
    s = mag;
  } else {
    t = mag / s;
    alpha += t * t;
  }

  alpha = s * sqrt(alpha);
  q /= alpha;
  V_per_idx_1 /= alpha;
  x_up /= alpha;
  memset(&V_rotper[0], 0, 1080U * sizeof(double));

  /* this loop eliminates the error that occurs due to residual difference */
  /* between 1 and norm of the V_rotper */
  for (i = 0; i < 360; i++) {
    alpha = dv[i];
    mag = cos(alpha);
    s = sin(alpha);
    V_rotper[i] = mag * q + s * (y_tmp * x_up - y_up * V_per_idx_1);
    V_rotper[i + 360] = mag * V_per_idx_1 + s * (y_up * q - x_tmp * x_up);
    V_rotper[i + 720] = mag * x_up + s * (x_tmp * V_per_idx_1 - y_tmp * q);
    memcpy(&x2[0], &V_rotper[0], 360U * sizeof(double));
    memcpy(&y2[0], &V_rotper[360], 360U * sizeof(double));
    memcpy(&z2[0], &V_rotper[720], 360U * sizeof(double));

    /*  x2=V_rotper(:,1);y2=V_rotper(:,2);z2=V_rotper(:,3); */
    cond4[i] = 0;
  }

  /*  cond4 = 1==x2.*x2+y2.*y2+z2.*z2; */
  /*  cond4 = Tol(1,x2.*x2+y2.*y2+z2.*z2,2.5e-02); */
  /*  cond4 = ismembertol(sqrt(x2.*x2+y2.*y2+z2.*z2),1,10e-2); */
  trueCount = 0;
  partialTrueCount = 0;
  for (i = 0; i < 360; i++) {
    alpha = x2[i];
    s = y2[i];
    mag = z2[i];
    if (0.0025 > 1.0 - ((alpha * alpha + s * s) + mag * mag)) {
      cond4[i] = 1;
    }

    b = ((-1.0 < alpha) && (alpha < 1.0) && ((-1.0 < s) && (s < 1.0)) && ((-1.0 <
           mag) && (mag < 1.0)));
    b1 = (cond4[i] != 0);
    if (b && b1) {
      trueCount++;
      tmp_data[partialTrueCount] = (short)(i + 1);
      partialTrueCount++;
    }
  }

  for (i = 0; i < trueCount; i++) {
    V_rotper[i] = x2[tmp_data[i] - 1];
  }

  for (i = 0; i < trueCount; i++) {
    V_rotper[i + trueCount] = y2[tmp_data[i] - 1];
  }

  for (i = 0; i < trueCount; i++) {
    V_rotper[i + trueCount * 2] = z2[tmp_data[i] - 1];
  }

  if (trueCount > 3) {
    i = trueCount;
  } else {
    i = 3;
  }

  if (trueCount == 0) {
    partialTrueCount = 0;
  } else {
    partialTrueCount = i;
  }

  /*  X1(1) = max(x2(ix));X1(2) = max(y2(ix));X1(3) = max(z2(ix)); */
  /*  x2 = X1(1);y2 = X1(2);z2 = X1(3); */
  /*  % X1 = eval([x2(1),y2(1),z2(1)]); */
  /* Creating a vector that is perpendicular to both X1 & Y1  */
  s = y_tmp * z2[0] - y_up * y2[0];
  q = y_up * x2[0] - x_tmp * z2[0];
  mag = x_tmp * y2[0] - y_tmp * x2[0];

  /*   */
  alpha = 0.0;
  for (k = 0; k < 3; k++) {
    i = 3 * k + 1;
    b_i = 3 * k + 2;
    Y1[k] = (s * (double)b_b[3 * k] + q * (double)b_b[i]) + mag * (double)
      b_b[b_i];
    X1[k] = (V_rotper[partialTrueCount - 1] * (double)c_b[3 * k] + V_rotper
             [(partialTrueCount + trueCount) - 1] * (double)c_b[i]) + V_rotper
      [(partialTrueCount + trueCount * 2) - 1] * (double)c_b[b_i];
    alpha += Z1[k] * (double)d_b[k];
  }

  q = (X1[0] * 0.0 + X1[1]) + X1[2] * 0.0;
  mag = (Y1[0] * 0.0 + Y1[1]) + Y1[2] * 0.0;
  s = (x_tmp * 0.0 + y_tmp * 0.0) + y_up;

  /*  eul = rotm2eul(R,'ZYZ') */
  /*  phi= eul(1);theta = eul(2);ksi = eul(3); */
  if (s < 1.0) {
    if (s > -1.0) {
      *theta = acos(s);
      *phi = rt_atan2d_snf((x_tmp * 0.0 + y_tmp) + y_up * 0.0, alpha);
      *ksi = rt_atan2d_snf((Y1[0] * 0.0 + Y1[1] * 0.0) + Y1[2], -((X1[0] * 0.0 +
        X1[1] * 0.0) + X1[2]));
    } else {
      /* R(3,3)= -1 not a unique solution */
      *theta = 3.1415926535897931;
      *phi = -rt_atan2d_snf(q, mag);
      *ksi = 0.0;
    }
  } else {
    /*  R(3,3) = 1 not a unique solution */
    *theta = 0.0;
    *phi = -rt_atan2d_snf(q, mag);
    *ksi = 0.0;
  }

  /*  up  */
  /*                       .                             */
  /*                 x    |z|                             */
  /*                ------ -                             */
  /*                      |                                 */
  /*                      |y                                */
  /*                      |                                 */
  /*                                                        */
  /*         =========================                                       */
  /*          theta1-pi   -\    2pi-theta2                                                         */
  /*                    --  -\                                                                */
  /*                  /-      -\                                                              */
  /*                /-          -\                                                            */
  /*              /-              -\   l                                                      */
  /*      l     /-                  -\                   ---------------------------          */
  /*           /                      -\                theta2-pi  -\                        */
  /*         /-                         -\                         --\                        */
  /*       /-                             -\                        --\                     */
  /*     /-                                 -\                      -/ \                   */
  /*   /-theta1-pi                 2pi-theta2 --                  -=/   \                 */
  /*  -----------                ------------  /                  |   pi-(2pi-theta2)-(theta1-pi)=theta2-theta1 */
  /*   -\   2pi-theta2           theta1-pi   /-                   |                           */
  /*     -\                                /-                     |                           */
  /*       -\                            /-                     theta2-pi-(theta2-theta1)+pi/4=theta1-pi+pi/4 */
  /*         -\                   l     /                                                     */
  /*           -\                     /-                                                      */
  /*      l      -\                 /-                                                        */
  /*               -\              /                                                          */
  /*                 -\          /-                                                           */
  /*                   -\      /-                                                             */
  /*                     -\  /-                                                               */
  /*                       --                                                                 */
  /*                       | 3pi/4                                                             */
  /*             r = 103mm |                                                                  */
  /*                       |               */
  /* low                 x----.z                                       */
  /*                          |                                         */
  /*                         y|                                         */
  /*                                                                    */
  /*                  -------m45-------                                 */
  /*       2pi-theta5/      theta4-pi/-                                  */
  /*                 /   -\       /-  -\                                */
  /*                /--    --\  /-      \                               */
  /*               /          /-        --\                             */
  /*      l       /         /-  --\        \                            */
  /*             /        /-       -\       -\                          */
  /*            /       /-           --\      \    l                    */
  /*          /-      /-             q  -\     \                        */
  /*         /      /- p                  -\    -\                      */
  /*        /     /-                        --\   \                     */
  /*       /    /-                           - -\  -\                   */
  /*      /   /-                                 --\ \                  */
  /*     /  /-             n                gamma  -\-\                */
  /*    / /-                                -------------               */
  /*   //- psi     -----------------------/  alpha   -/                */
  /*  -------------/                                -/                  */
  /*    -\  alpha                                  /                    */
  /*      -\                                     -/                     */
  /*       --\                                 -/                       */
  /*          -\-                            -/                         */
  /*            -\                          /                           */
  /*              -\                      -/  l                         */
  /*            l   -\                  -/                              */
  /*                  -\               /                                */
  /*                    -\  beta     -/                                 */
  /*                      -\       -/                                   */
  /*                        -\   -/                                     */
  /*                          --/                                       */
  /*                    3pi/4  ||                             */
  /*                           ||                                       */
  /*                           || r=120mm                                       */
  /*  textik.com               ||       */
}


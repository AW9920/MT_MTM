void KinematicCalc_L(int *enc_val[], Quaternion q, int hall_val, double Q[]) { //Signs of q1, q3 and L3 are inverted in comparision to right PSM Kinematic calculation
  q_mtm[0] = -double(*enc_val[0]) * res_mag_enc * PI / 180.0;   //Convert 2 rad
  q_mtm[1] = -double(*enc_val[2]) * res_mag_enc * PI / 180.0;  //Convert 2 rad
  q_mtm[2] = double(*enc_val[1]) * res_mag_enc * PI / 180.0;  //Convert 2 rad

  //-----------------------Compute planned trajectory---------------------------------
  // To-Do: Compute desired position of PSM in task space
  x_m = L1 * cos(q_mtm[0]) + L2 * cos(q_mtm[0]) * cos(q_mtm[1]) + L3 * cos(q_mtm[0]) * sin(q_mtm[1]) - H1 * sin(q_mtm[0]) * sin(q_mtm[2] - PI / 2) + H1 * cos(q_mtm[0]) * cos(q_mtm[1]) * cos(q_mtm[2] - PI / 2);
  y_m = L1 * sin(q_mtm[0]) + L2 * cos(q_mtm[1]) * sin(q_mtm[0]) + H1 * cos(q_mtm[0]) * sin(q_mtm[2] - PI / 2) + L3 * sin(q_mtm[0]) * sin(q_mtm[1]) + H1 * cos(q_mtm[1]) * cos(q_mtm[2] - PI / 2) * sin(q_mtm[0]);
  z_m = -L3 * cos(q_mtm[1]) + L2 * sin(q_mtm[1]) + H1 * cos(q_mtm[2] - PI / 2) * sin(q_mtm[1]);
  C_mtm[0] = x_m;
  C_mtm[1] = y_m;
  C_mtm[2] = z_m;

  //Compute trajectory
  double d_x = x_m - x_m_init;
  double d_y = y_m - y_m_init;
  double d_z = z_m - z_m_init;
  x_p = x_p_init + motion_scaler_x * d_x;
  y_p = y_p_init + motion_scaler_y * d_y;
  z_p = z_p_init + motion_scaler_z * d_z;

  //-------------------Compute desired joint values 1, 2 & 3 for PSM---------------------------
  // To-Do: Conduct inverse Kinematics
  int sign = 0;
  if (y_p < 0) {
    sign = 1;
  } else if (y_p >= 0) {
    sign = -1;
  }
  //Compute desired q3 of PSM
  q3_p = sign * sqrt(sq(x_p) + sq(y_p) + sq(z_p)) - d0;
  //Compute desired q2 of PSM
  double s2 = -x_p / (d0 + q3_p);
  double c2 = sqrt(1 - sq(s2));
  q2_p = atan2(s2, c2);
  if ((q2_p <= (-PI / 2)) || (q2_p >= (PI / 2))) {
    q2_p = atan2(s2, -c2);
  }
  q2_p *= (180 / PI);
  //Compute desired q1 of PSM
  double b = c2 * (q3_p + d0);
  double a = -b;
  double c = y_p + z_p;
  q1_p = atan2(b, a) + atan2(sqrt(sq(a) + sq(b) - sq(c)), c);
  if ((q1_p <= (-PI / 2)) || (q1_p >= (PI / 2))) {
    q1_p = atan2(b, a) - atan2(sqrt(sq(a) + sq(b) - sq(c)), c);
  }
  q1_p *= (180 / PI);

  //-------------------Compute desired joint values 4, 5 & 6 for PSM---------------------------
  //Right MTM
  double s5, c5, s4, c4, s6, c6;
  quaternionToArray(q, quatArray);
  quatToRotMat(quatArray, rotMat);
  arrayToMatrix(rotMat, mat);
  matrixMult(mat1, mat, result);
  s5 = result[2][1];
  c5 = sqrt(sq(result[0][1]) + sq(result[1][1]));
  q5_m = atan2(s5, c5);
  if ((q5_m <= (-PI / 2)) || (q5_m >= (PI / 2))) {
    q5_m = atan2(s5, -c5);
  }
  q5_m *= (180 / PI);

  if (q5_m != 90.0) {
    s4 = -result[0][1] / c5;
    c4 = result[1][1] / c5;
    q4_m = atan2(s4, c4);
    q4_m *= (180.0 / PI);
  } else {
    q4_m = 0.0;
  }

  if (q5_m == 90.0) {
    s6 = result[1][0];
    c6 = result[1][2];
    q6_m = atan2(s6, c6);
    q6_m *= (180.0 / PI);
  } else if (q5_m == -90.0) {
    s6 = result[1][0];
    c6 = result[1][2];
    q6_m = -atan2(s6, c6);
    q6_m *= (180.0 / PI);
  } else {
    s6 = -result[2][0] / c5;
    c6 = -result[2][2] / c5;
    q6_m = atan2(s6, c6);
    q6_m *= (180.0 / PI);
  }

  q7_m = 1.261157 + (53481730 - 1.261157) / (1 + pow((HallR / 84.42502), 8.110327));

  // Remap values to PSM
  q4_p = q6_m;   //PSM Roll
  q5_p = q4_m;   //PSM Pitch
  q6_p = -q5_m;  //PSM Yaw
  q7_p = -2 * (q7_m - 1.37);

  Q[0] = q1_p;
  Q[1] = q2_p;
  Q[2] = q3_p;
  Q[3] = q4_p;
  Q[4] = q5_p;
  Q[5] = q6_p;
  Q[6] = q7_p;
}
using namespace swarm_msgs;

  enum VerticalLogic
  {
    /*!
     - Set the control-mode to control the vertical
       speed of UAV, upward is positive
     - Limit: -5 to 5 m/s
     */
    VERTICAL_VELOCITY = 0x00,
    /*!
     - Set the control-mode to control the height of UAV
     - Limit: 0 to 120 m
     */
    VERTICAL_POSITION = 0x10,
    /*!
     - Set the control-mode to directly control the thrust
     - Range: 0% to 100%
     */
    VERTICAL_THRUST = 0x20,
  };

  /*! @brief bit 7:6 of the 8-bit (7:0) CtrlData.flag
   *
   *  @note
   *        - Only when the GPS signal is good (health_flag >=3)，horizontal
   * position control (HORIZONTAL_POSITION) related control modes can be used.
   *        - Only when GPS signal is good (health_flag >=3)，or when AdvancedSensing
   * system is working properly with Autopilot，
   *          horizontal velocity control（HORIZONTAL_VELOCITY）related control
   * modes can be used.
   */
  enum HorizontalLogic
  {
    /*!
     - Set the control-mode to control pitch & roll
     angle of the vehicle.
     - Need to be referenced to either the ground or
     body frame by HorizontalCoordinate setting.
     - Limit: 35 degree
     */
    HORIZONTAL_ANGLE = 0x00,
    /*!
     - Set the control-mode to control horizontal
     vehicle velocities.
     - Need to be referenced to either the ground
     or body frame by HorizontalCoordinate setting.
     - Limit: 30 m/s
     */
    HORIZONTAL_VELOCITY = 0x40,
    /*!
     - Set the control-mode to control position
     offsets of pitch & roll directions
     - Need to be referenced to either the ground
     or body frame by HorizontalCoordinate setting.
     - Limit: N/A
     */
    HORIZONTAL_POSITION = 0x80,
    /*!
     - Set the control-mode to control rate of
     change of the vehicle's attitude
     - Need to be referenced to either the ground
     or body frame by HorizontalCoordinate setting.
     - Limit: 150.0 deg/s
     */
    HORIZONTAL_ANGULAR_RATE = 0xC0
  };
  /*! @brief bit 3 of the 8-bit (7:0) CtrlData.flag
   */
  enum YawLogic
  {
    /*!
     - Set the control-mode to control yaw angle.
     - Yaw angle is referenced to the ground frame.
     - In this control mode, Ground frame is enforeced in Autopilot.
     */
    YAW_ANGLE = 0x00,
    /*!
     - Set the control-mode to control yaw angular velocity.
     - Same reference frame as YAW_ANGLE.
     - Limite: 150 deg/s
     */
    YAW_RATE = 0x08
  };

  /*! @brief bit 2:1 of the 8-bit (7:0) CtrlData.flag
   */
  enum HorizontalCoordinate
  {
    /*! Set the x-y of ground frame as the horizontal frame (NEU) */
    HORIZONTAL_GROUND = 0x00,
    /*! Set the x-y of body frame as the horizontal frame (FRU) */
    HORIZONTAL_BODY = 0x02
  };

  /*!
   * @brief bit 0 of the 8-bit (7:0) CtrlData.flag.
   *
   * Drone will try to hold at current position if enable
   */
  enum StableMode
  {
    STABLE_DISABLE = 0x00, /*!< Disable the stable mode */
    STABLE_ENABLE  = 0x01  /*!< Enable the stable mode */
  };

Eigen::Vector3d quat_to_pry(Eigen::Quaterniond q) {
    q.normalize();
    // Set up convenience variables

    double w = q.w(); 
    double x = q.x(); 
    double y = q.y(); 
    double z = q.z();
    double w2 = w*w; 
    double x2 = x*x; 
    double y2 = y*y; 
    double z2 = z*z;
    double xy = x*y; 
    double xz = x*z; 
    double yz = y*z;
    double wx = w*x; 
    double wy = w*y; 
    double wz = w*z;
    Eigen::Matrix3d R = q.toRotationMatrix();
    // R << w2+x2-y2-z2 , 2*(xy - wz) , 2*(wy + xz) ,
        //  2*(wz + xy) , w2-x2+y2-z2 , 2*(yz - wx) ,
        //  2*(xz - wy) , 2*(wx + yz) , w2-x2-y2+z2;

    // std::cout << R << std::endl;

    Eigen::Vector3d pry;      
    pry.z() = atan2(-R(0,1),R(1,1));//zxy(1)
    pry.y() = asin(R(2,1));//zxy(2)
    pry.x() = atan2(-R(2,0),R(2,2));//zxy(3)
  
    // printf("q:%4.3f %4.3f %4.3f %4.3f\n", q.w(), q.x(), q.y(), q.z());
    // printf("pry %4.3f %4.3f %4.3f\n", pry.x(), pry.y(), pry.z());

    return pry;//R.eulerAngles(1, 0, 2);;
}

inline Eigen::Vector3d lowpass_filter(Eigen::Vector3d input, double fc, Eigen::Vector3d outputlast, double dt) {
	double RC = 1.0 / (fc *2 * M_PI);
	double alpha = dt / (RC + dt);
	return outputlast + (alpha* (input - outputlast));
}

double constrainAngle(double x){
    x = fmod(x + 180,360);
    if (x < 0)
        x += 360;
    return x - 180;
}

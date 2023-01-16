补全代码：

UpdateOdomEstimation: 用imu信息更新先验。和第六章作业一样。区别是我们只是用这个函数的输出来建构process公式，而不是直接用积分出来的值当作状态量。
```
void ErrorStateKalmanFilter::UpdateOdomEstimation(
    Eigen::Vector3d &linear_acc_mid, Eigen::Vector3d &angular_vel_mid) {
  //
  // TODO: this is one possible solution to previous chapter, IMU Navigation,
  // assignment
  //
  // get deltas:
  Eigen::Vector3d angular_delta, vel_delta;
  Eigen::Matrix3d R_p, R_c;
  double dt;
  angular_delta.setZero();
  vel_delta.setZero();
  R_p.setZero();
  R_c.setZero();

  GetAngularDelta(1, 0, angular_delta, angular_vel_mid);

  // update orientation:
  UpdateOrientation(angular_delta, R_c, R_p);

  // get velocity delta:
  GetVelocityDelta(1, 0, R_c, R_p, dt, vel_delta, linear_acc_mid);

  // update position:
  UpdatePosition(dt, vel_delta);
}
```

SetProcessEquation：用IMU积分出来的值架构线性化的状态更新方程
```
void ErrorStateKalmanFilter::SetProcessEquation(const Eigen::Matrix3d &C_nb,
                                                const Eigen::Vector3d &f_n,
                                                const Eigen::Vector3d &w_b) {
  // TODO: set process / system equation:
  // a. set process equation for delta vel:
  F_.block<3, 3>(kIndexErrorVel, kIndexErrorOri) = -C_nb * Sophus::SO3d::hat(f_n).matrix();
  F_.block<3, 3>(kIndexErrorVel, kIndexErrorAccel) = -C_nb;
  B_.block<3, 3>(kIndexErrorVel, kIndexErrorPos) = C_nb;

  // b. set process equation for delta ori:
  F_.block<3, 3>(kIndexErrorOri, kIndexErrorOri) = -Sophus::SO3d::hat(w_b).matrix();
}
```

UpdateErrorEstimation： 使用SetProcessEquation给出的公式来更新状态量，因为是离散时间所以要乘以时间差
```
void ErrorStateKalmanFilter::UpdateErrorEstimation(
    const double &T, const Eigen::Vector3d &linear_acc_mid,
    const Eigen::Vector3d &angular_vel_mid) {
  static MatrixF F_1st;
  static MatrixF F_2nd;
  // TODO: update process equation:
  UpdateProcessEquation(linear_acc_mid, angular_vel_mid);
  // TODO: get discretized process equations:
  F_1st = F_ * T;
  F_2nd = MatrixF::Identity() + F_1st;

  B_.block<3, 3>(kIndexErrorVel, kIndexNoiseAccel) *= T;
  B_.block<3, 3>(kIndexErrorOri, kIndexNoiseGyro) *= T;
  B_.block<3, 3>(kIndexErrorAccel, kIndexNoiseBiasAccel) *= sqrt(T);
  B_.block<3, 3>(kIndexErrorGyro, kIndexNoiseBiasGyro) *= sqrt(T);
  // TODO: perform Kalman prediction
  X_ = F_2nd * X_;
  P_ = F_2nd * P_ * F_2nd.transpose() + B_ * Q_ * B_.transpose();
}
```

CorrectErrorEstimationPose:利用观测结果计算观测与预测之差，并且计算出卡尔曼增量
```
void ErrorStateKalmanFilter::CorrectErrorEstimationPose(
    const Eigen::Matrix4d &T_nb, Eigen::VectorXd &Y, Eigen::MatrixXd &G,
    Eigen::MatrixXd &K) {
  //
  // TODO: set measurement:
  //
  Eigen::Vector3d dx = pose_.block<3,1>(0,3) - T_nb.block<3,1>(0,3);
  Eigen::Matrix3d dR = T_nb.block<3,3>(0,0).transpose() * pose_.block<3, 3>(0,0);
  Eigen::Vector3d dtheta = Sophus::SO3d::vee(dR - Eigen::Matrix3d::Identity()); 

  YPose_.head(3) = dx;
  YPose_.tail(3) = dtheta;
  Y = YPose_;

  // TODO: set measurement equation:
  G = GPose_;

  // TODO: set Kalman gain:              
  K.setZero();
  K = P_ * G.transpose() * (G * P_ * G.transpose() + CPose_ * RPose_ * CPose_.transpose()).inverse();
}
```

CorrectErrorEstimation：利用CorrectErrorEstimationPose给出的卡尔曼增量和误差，更新后验状态量
```
void ErrorStateKalmanFilter::CorrectErrorEstimation(
    const MeasurementType &measurement_type, const Measurement &measurement) {
  //
  // TODO: understand ESKF correct workflow
  //
  Eigen::VectorXd Y;
  Eigen::MatrixXd G, K;
  switch (measurement_type) {
  case MeasurementType::POSE:
    CorrectErrorEstimationPose(measurement.T_nb, Y, G, K);
    break;
  default:
    break;
  }

  // TODO: perform Kalman correct:
  X_ += K * (Y - G * X_);
  P_ = (MatrixP::Identity() - K * G) * P_;
}
```

EliminateError：由于我们用的是error state KF，计算出的状态量为状态更新量，这一步目的为更新状态量。由于我们课件定义方式，我们需要减去更新量。之后在ResetState函数中将更新量清零
```
void ErrorStateKalmanFilter::EliminateError(void) {
  //
  // TODO: correct state estimation using the state of ESKF
  //
  // a. position:
  // do it!
  pose_.block<3,1>(0,3) -= X_.block<3,1>(kIndexErrorPos, 0);
  // b. velocity:
  // do it!
  vel_ -= X_.block<3,1>(kIndexErrorVel, 0);
  // c. orientation:
  // do it!
  Eigen::Quaterniond temp_quat(pose_.block<3,3>(0,0) * (Eigen::Matrix3d::Identity() - Sophus::SO3d::hat((X_.block<3, 1>(kIndexErrorOri, 0)))));
  temp_quat.normalize();
  pose_.block<3,3>(0,0) = temp_quat.toRotationMatrix();

  // d. gyro bias:
  if (IsCovStable(kIndexErrorGyro)) {
    gyro_bias_ -= X_.block<3, 1>(kIndexErrorGyro, 0);
  }

  // e. accel bias:
  if (IsCovStable(kIndexErrorAccel)) {
    accl_bias_ -= X_.block<3, 1>(kIndexErrorAccel, 0);
  }
}
```

结果：
原始参数：

<table>
  <td> <img src="imgs/mid_point.png" width="500" height="400" />
  Mid-point Method
  </td> 
  <td> <img src="imgs/euler.png" width="500" height="400" />
  Euler's Method
  </td> 
</table>

可以很明显的看出在这个轨迹中中值法比欧拉法好很多。

gnss_imu_sim:
首先需要将reference trajectory存入rosbag内,根据recorder_node_allan_variance_analysis.py：
将reference trajectory 取出
```
    for i, (gyro, accel, ref_pos, ref_att_quat) in enumerate(
        zip(
            # a. gyro
            sim.dmgr.get_data_all('gyro').data[0], 
            # b. accel
            sim.dmgr.get_data_all('accel').data[0],
            sim.dmgr.get_data_all('ref_pos').data,
            sim.dmgr.get_data_all('ref_att_quat').data
        )
    ):
        yield {
            'stamp': i * step_size,
            'data': {
                # a. gyro:
                'gyro_x': gyro[0],
                'gyro_y': gyro[1],
                'gyro_z': gyro[2],
                # b. accel:
                'accel_x': accel[0],
                'accel_y': accel[1],
                'accel_z': accel[2],
                # c. ref_position
                'pos_x': ref_pos[0] - origin_x,
                'pos_y': ref_pos[1] - origin_y,
                'pos_z': ref_pos[2] - origin_z,
                # d. quaternion
                'quat_w': ref_att_quat[0],
                'quat_x': ref_att_quat[1],
                'quat_y': ref_att_quat[2],
                'quat_z': ref_att_quat[3],
            }
        }
```

将reference trajectory存入/pose/ground_truth

```
            # Groundtruth Odom
            odom_msg = Odometry()
            odom_msg.header.frame_id = 'inertial'
            odom_msg.header.stamp = msg.header.stamp

            odom_msg.pose.pose.position.x =  measurement['data']['pos_x']   
            odom_msg.pose.pose.position.y =  measurement['data']['pos_y']  
            odom_msg.pose.pose.position.z =  measurement['data']['pos_z']   

            odom_msg.pose.pose.orientation.w = measurement['data']['quat_w']
            odom_msg.pose.pose.orientation.x = measurement['data']['quat_x']
            odom_msg.pose.pose.orientation.y = measurement['data']['quat_y']
            odom_msg.pose.pose.orientation.z = measurement['data']['quat_z']

            # write:
            bag.write(topic_name_imu, msg, msg.header.stamp)
            bag.write(topic_name_odom, odom_msg, odom_msg.header.stamp)
```

用gnss_imu_sim由于噪音比较大的imu数据导致可视化和效果可能更多是由于噪音造成的，而不是由算法的误差造成的，所以将噪音设置得比较小。

```
imu_err = {
        # 1. gyro:
        # a. random noise:
        # gyro angle random walk, deg/rt-hr
        'gyro_arw': np.array([0.05, 0.05, 0.05]),
        # gyro bias instability, deg/hr
        'gyro_b_stability': np.array([1.0, 1.0, 1.0]),
        # gyro bias isntability correlation time, sec
        'gyro_b_corr': np.array([10.0, 10.0, 10.0]),
        # b. deterministic error:
        'gyro_b': np.array([0.0, 0.0, 0.0]),
        'gyro_k': np.array([1, 1, 1]),
        'gyro_s': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        # 2. accel:
        # a. random noise:
        # accel velocity random walk, m/s/rt-hr
        'accel_vrw': np.array([0.00005, 0.00005, 0.00005]),
        # accel bias instability, m/s2
        'accel_b_stability': np.array([2.0e-7, 2.0e-7, 2.0e-7]),
        # accel bias isntability correlation time, sec
        'accel_b_corr': np.array([100.0, 100.0, 100.0]),
        # b. deterministic error:
        'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
        'accel_k': np.array([1.0, 1.0, 1.0]),
        'accel_s': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        # 3. mag:
        'mag_si': np.eye(3) + np.random.randn(3, 3)*0.0, 
        'mag_hi': np.array([10.0, 10.0, 10.0])*0.0,
        'mag_std': np.array([0.1, 0.1, 0.1])
    }
```

以下是用gnss_imu_sim生成的匀加速轨迹：

<table>
  <td> <img src="imgs/mid_point_hold.png" width="500" height="300" />
  Mid-point Method
  </td> 
  <td> <img src="imgs/euler_hold.png" width="500" height="300" />
  Euler's Method
  </td> 
</table>

以下是用gnss_imu_sim生成的静止轨迹：

<table>
  <td> <img src="imgs/mid_point_vx_001.png" width="500" height="300" />
  Mid-point Method
  </td> 
  <td> <img src="imgs/euler_vx_001.png" width="500" height="300" />
  Euler's Method
  </td> 
</table>

可以看出在轨迹加速度的导数变化不大时，两种方法并没有很大的差距

以下是用gnss_imu_sim生成的200米跑道的形状：

<table>
  <td> <img src="imgs/mid_point_loop.png" width="500" height="300" />
  Mid-point Method
  </td> 
  <td> <img src="imgs/euler_loop.png" width="500" height="300" />
  Euler's Method
  </td> 
</table>

以下是用gnss_imu_sim生成的和上面差不多的输入，但是在掉头时给的时间很短导致角速度变化较大：

<table>
  <td> <img src="imgs/mid_point_sharper_loop.png" width="500" height="300" />
  Mid-point Method
  </td> 
  <td> <img src="imgs/euler_sharper_loop.png" width="500" height="300" />
  Euler's Method
  </td> 
</table>

可以看出在第一组图里，欧拉法反倒是更好的效果，具体原因也不是很清楚。
但是在第二组图里，由于角速度变化过快，中值法明显优于欧拉法。

最后我将gnss_imu_sim的噪声改大了一些：

```
imu_err = {
        # 1. gyro:
        # a. random noise:
        # gyro angle random walk, deg/rt-hr
        'gyro_arw': np.array([0.1, 0.1, 0.1]),
        # gyro bias instability, deg/hr
        'gyro_b_stability': np.array([1.0, 1.0, 1.0]),
        # gyro bias isntability correlation time, sec
        'gyro_b_corr': np.array([10.0, 10.0, 10.0]),
        # b. deterministic error:
        'gyro_b': np.array([0.0, 0.0, 0.0]),
        'gyro_k': np.array([1, 1, 1]),
        'gyro_s': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        # 2. accel:
        # a. random noise:
        # accel velocity random walk, m/s/rt-hr
        'accel_vrw': np.array([0.005, 0.005, 0.005]),
        # accel bias instability, m/s2
        'accel_b_stability': np.array([2.0e-6, 2.0e-6, 2.0e-6]),
        # accel bias isntability correlation time, sec
        'accel_b_corr': np.array([100.0, 100.0, 100.0]),
        # b. deterministic error:
        'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
        'accel_k': np.array([1.0, 1.0, 1.0]),
        'accel_s': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        # 3. mag:
        'mag_si': np.eye(3) + np.random.randn(3, 3)*0.0, 
        'mag_hi': np.array([10.0, 10.0, 10.0])*0.0,
        'mag_std': np.array([0.1, 0.1, 0.1])
    }
```

<table>
  <td> <img src="imgs/mid_point_sharper_noisier.png" width="500" height="300" />
  Mid-point Method
  </td> 
  <td> <img src="imgs/euler_sharper_loop_noisier.png" width="500" height="300" />
  Euler's Method
  </td> 
</table>

由于噪音的变化两种算法都受到影响更大，但是哪种算法收到噪音的影响更大还需要更深一步的探索。但是由于中值法有一个平均的步骤，相当于一个长度为2的movmean filter所以感觉上中值法会比欧拉法受到的噪音更小一些

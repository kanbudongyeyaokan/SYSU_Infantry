我现在需要把 bmi088 驱动库中的ekf部分进行解耦，把ekf相关的数据结构体，以及算法预测、更新的函数进行抽取解耦开来
比如，保留原来的 Bmi088_error_e Bmi088_ekf_update(Bmi088_device_t* bmi088) 函数接口不变，但将其内部实现与其他模块解耦
比如下面部分的接口是和BMI驱动无关的，可以在别的文件模块实现？然后通过头文件包含进行引入
    // 转换为数组格式
    float acc[3] = {acc_data->x, acc_data->y, acc_data->z};
    float gyro[3] = {gyro_data->roll * EKF_DEG_TO_RAD, 
                     gyro_data->pitch * EKF_DEG_TO_RAD, 
                     gyro_data->yaw * EKF_DEG_TO_RAD};
    
    // 静态检测和零偏校准
    if (detect_static_state(bmi088, acc, gyro)) {
        // 在静态状态下更新零偏
        Ekf_state_t* ekf = &bmi088->data.ekf_state;
        for (int i = 0; i < 3; i++) {
            ekf->gyro_bias[i] = 0.95f * ekf->gyro_bias[i] + 0.05f * gyro[i];
        }
    }
    
    // EKF预测步骤
    ekf_predict_step(bmi088, gyro);
    
    // EKF更新步骤（使用加速度计数据）
    ekf_update_step(bmi088, acc);
    
    // 更新欧拉角
    quaternion_to_euler(&bmi088->data.ekf_state.quaternion, 
                        &bmi088->data.ekf_state.euler);


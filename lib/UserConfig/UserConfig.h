#ifndef UserConfg_h
#define UserConfg_h

/* Phone WiFi config (unused by the first balance test). */
#define USER_SSID "BalanceCar"
#define USER_PASSWORD "123456789"
#define STATIC_IP_MODE 0
#define YOUR_IP 229

/* Inner motor velocity loop, already verified by the wheel test. */
#define VEL_Kp 0.09f
#define VEL_Ki 3.0f

/* Reserved current-loop settings. Current sensing is disabled for now. */
#define C_Kp 0.6f
#define C_Ki 0.8f
#define C_LF 0.1f

/* Outer wheel-speed loop: output is the requested lean angle in degrees. */
#define VELOCITY_Kp 0.15f
#define VELOCITY_Ki 0.0f
#define VELOCITY_Kd 0.0f
#define VELOCITY_LIMIT 3.0f

/* Upright angle loop: output is the motor target velocity in rad/s. */
#define UPRIGHT_Kp 1.0f
#define UPRIGHT_Ki 0.0f
#define UPRIGHT_Kd 0.02f
#define UPRIGHT_LIMIT 12.0f

#define TARGET_VEL_LIMIT 5.0f
#define STR_LIMIT 0.0f
#define VEL_PID_UPDATE 10
#define UPRIGHT_PID_UPDATE 1

/* Flydigi BLE gamepad limits. Keep these low for the first suspended-wheel test. */
#define GAMEPAD_DEADZONE 0.12f
#define GAMEPAD_FORWARD_LIMIT 1.5f
#define GAMEPAD_STEER_LIMIT 1.0f
#define GAMEPAD_FORWARD_SIGN -1.0f
#define GAMEPAD_STEER_SIGN 1.0f
#define GAMEPAD_REPORT_TIMEOUT_MS 300UL
#define GAMEPAD_DEBUG_INTERVAL_MS 250UL

/* Conservative first-test limits. */
#define DRIVER_SUPPLY_VOLTAGE 8.4f
#define MOTOR_VOLTAGE_LIMIT 1.5f
#define MOTOR_VELOCITY_LIMIT 12.0f
#define ARM_ANGLE_LIMIT 10.0f
#define ARM_GYRO_LIMIT 20.0f
#define FALL_ANGLE_LIMIT 25.0f
#define TELEMETRY_INTERVAL_MS 250UL

/* Change to -1.0f if the wheels accelerate in the falling direction. */
#define BALANCE_DIRECTION 1.0f

/* MPU6050 complementary and gyro low-pass filters. */
#define GYRO_COEF 0.975f
#define GYRO_COEFX 0.85f
#define PRE_GYRO_COEFX 0.0f

#define MONITOR_MODE 0

#endif

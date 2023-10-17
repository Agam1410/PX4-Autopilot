uint64 timestamp		# time since system start (microseconds)

float32 roll_p		# body angle in NED frame (can be NaN for FW)
float32 pitch_p		# body angle in NED frame (can be NaN for FW)
float32 yaw_p		# body angle in NED frame (can be NaN for FW)

float32 roll_i		# body angle in NED frame (can be NaN for FW)
float32 pitch_i		# body angle in NED frame (can be NaN for FW)
float32 yaw_i		# body angle in NED frame (can be NaN for FW)

float32 roll_d		# body angle in NED frame (can be NaN for FW)
float32 pitch_d		# body angle in NED frame (can be NaN for FW)
float32 yaw_d		# body angle in NED frame (can be NaN for FW)

float32 roll_f_hat	# body angle in NED frame (can be NaN for FW)
float32 pitch_f_hat	# body angle in NED frame (can be NaN for FW)
float32 yaw_f_hat	# body angle in NED frame (can be NaN for FW)

float32 roll_sp_der	# body angle in NED frame (can be NaN for FW)
float32 pitch_sp_der	# body angle in NED frame (can be NaN for FW)
float32 yaw_sp_der	# body angle in NED frame (can be NaN for FW)

float32 roll_rate_sp	# body angle in NED frame (can be NaN for FW)
float32 pitch_rate_sp	# body angle in NED frame (can be NaN for FW)
float32 yaw_rate_sp	# body angle in NED frame (can be NaN for FW)

float32 roll_rate	# body angle in NED frame (can be NaN for FW)
float32 pitch_rate	# body angle in NED frame (can be NaN for FW)
float32 yaw_rate	# body angle in NED frame (can be NaN for FW)



float32 dt

float32[3] torque

# TOPICS pid_values

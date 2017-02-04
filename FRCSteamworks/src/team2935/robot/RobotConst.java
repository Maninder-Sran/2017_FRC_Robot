package team2935.robot;

import edu.wpi.first.wpilibj.Preferences;

public class RobotConst {
	//Robot Properties
	public static final double MAX_DRIVE_SPEED = 0;
	public static final double MAX_FINAL_SHOOTER_SPEED = 0;
	public static final double DRIVE_ENCODER_COUNTS_PER_FT = 0;
	public static final double DRIVE_LOW_SHIFT_THRESHOLD = Preferences.getInstance().getDouble("Auto Low Shift Threshold", 10);
	public static final double DRIVE_HIGH_SHIFT_THRESHOLD = Preferences.getInstance().getDouble("Auto High Shift Threshold", 6);
	
	//Drive PID Control Variables
	public static final double DRIVE_PID_P = Preferences.getInstance().getDouble("PID_p", 0);
	public static final double DRIVE_PID_D = Preferences.getInstance().getDouble("PID_d", 0);
	public static final double DRIVE_PID_POWER_DROP = 0.1;
	
	//Gyro PID Control Variables
	public static final double GYRO_PROPORTIONAL_GAIN = Preferences.getInstance().getDouble("Gyro_p", 0);
}

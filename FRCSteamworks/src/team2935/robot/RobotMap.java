package team2935.robot;


public class RobotMap {
	//PWM Ports for motors
	public static final int DRIVE_LEFT_MOTOR1 = 0;
	public static final int DRIVE_LEFT_MOTOR2 = 1;
	public static final int DRIVE_LEFT_MOTOR3 = 2;
	public static final int DRIVE_RIGHT_MOTOR1 = 3;
	public static final int DRIVE_RIGHT_MOTOR2 = 4;
	public static final int DRIVE_RIGHT_MOTOR3 = 5;
	public static final int SHOOTER_SHOOT_MOTOR = 6;
	public static final int SHOOTER_FEEDER_MOTOR = 7;
	public static final int INTAKE_MOTOR = 8;
	public static final int CLIMBER_MOTOR = 9;
	
	//DIO Ports for encoders
	public static final int DRIVE_LEFT_ENCODER_A = 0;
	public static final int DRIVE_LEFT_ENCODER_B = 1;
	public static final int DRIVE_RIGHT_ENCODER_A = 2;
	public static final int DRIVE_RIGHT_ENCODER_B = 3;
	
	//Analog Input Ports 
	public static final int SOLENOID_SHIFTER_HIGH = 0;
	public static final int SOLENOID_SHIFTER_LOW = 0;
	public static final int SOLENOID_OPEN_CLAW = 0;
	public static final int SOLENOID_CLOSE_CLAW = 0;
	public static final int SOLENOID_PUSH_ARM = 0;
	public static final int SOLENOID_PULL_ARM = 0;

	//USB ports for joy-sticks
	public static final int DRIVE_CONTROLLER = 0;
	
}

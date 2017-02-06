package team2935.robot;


public class RobotMap {
	//PWM Ports for motors
	public static final int DRIVE_LEFT_MOTOR1 = 1;
	public static final int DRIVE_LEFT_MOTOR2 = 2;
	public static final int DRIVE_LEFT_MOTOR3 = 3;
	public static final int SHOOTER_FEEDER_MOTOR = 4;
	public static final int INTAKE_MOTOR = 5;
	public static final int DRIVE_RIGHT_MOTOR1 = 6;
	public static final int DRIVE_RIGHT_MOTOR2 = 7;
	public static final int DRIVE_RIGHT_MOTOR3 = 8;
	public static final int CLIMBER_MOTOR = 9;
	public static final int SHOOTER_SHOOT_MOTOR = 10;
	
	//DIO Ports for encoders
	public static final int DRIVE_LEFT_ENCODER_A = 0;
	public static final int DRIVE_LEFT_ENCODER_B = 1;
	public static final int DRIVE_RIGHT_ENCODER_A = 2;
	public static final int DRIVE_RIGHT_ENCODER_B = 3;
	
	//Analog Input Ports 
	public static final int SOLENOID_SHIFTER_HIGH = 0;
	public static final int SOLENOID_SHIFTER_LOW = 1;
	public static final int SOLENOID_OPEN_CLAW = 2;
	public static final int SOLENOID_CLOSE_CLAW = 3;
	public static final int SOLENOID_PUSH_ARM = 4;
	public static final int SOLENOID_PULL_ARM = 5;

	//USB ports for joy-sticks
	public static final int DRIVE_CONTROLLER = 0;
	
}

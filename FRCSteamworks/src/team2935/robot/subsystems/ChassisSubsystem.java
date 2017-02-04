package team2935.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.toronto.subsystems.T_Subsystem;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;
import team2935.robot.RobotConst;
import team2935.robot.RobotMap;
import team2935.robot.commands.drive.GameControllerDriveCommand;
import team2935.utils.DrivePIDController;

public class ChassisSubsystem extends T_Subsystem {
	//Definition of the motors used on the drive_train subsystem
	private VictorSP leftMotor1 = new VictorSP(RobotMap.DRIVE_LEFT_MOTOR1);
	private VictorSP leftMotor2 = new VictorSP(RobotMap.DRIVE_LEFT_MOTOR2);
	private VictorSP leftMotor3 = new VictorSP(RobotMap.DRIVE_LEFT_MOTOR3);
	private VictorSP rightMotor1 = new VictorSP(RobotMap.DRIVE_RIGHT_MOTOR1);
	private VictorSP rightMotor2 = new VictorSP(RobotMap.DRIVE_RIGHT_MOTOR1);
	private VictorSP rightMotor3 = new VictorSP(RobotMap.DRIVE_RIGHT_MOTOR1);
	private Solenoid shifterHigh = new Solenoid(RobotMap.SOLENOID_SHIFTER_HIGH);
	private Solenoid shifterLow  = new Solenoid(RobotMap.SOLENOID_SHIFTER_LOW);
	
	@Override
	public void updatePeriodic() {
		// TODO Auto-generated method stub
		
	}
	//Definition of the sensors used on the drive_train subsystem
	private Encoder leftEncoder = new Encoder(RobotMap.DRIVE_LEFT_ENCODER_A,RobotMap.DRIVE_LEFT_ENCODER_B);
	private Encoder rightEncoder = new Encoder(RobotMap.DRIVE_RIGHT_ENCODER_A,RobotMap.DRIVE_RIGHT_ENCODER_B);
	private DrivePIDController pidController = new DrivePIDController();
	private AHRS gyro = new AHRS(SerialPort.Port.kUSB);

	//Declaration of variables used to track the current state the robot is in
	private States shiftedState;
	private States transmissionState;
	
	//Declaration of variables that need to be tracked for the AUTO SHIFTER controller
	private int prev_distance;
	
	public enum States{
		LOW,HIGH,HAS_SHIFTED,NOT_SHIFTED;
	}
	public void initDefaultCommand() {
		setDefaultCommand(new GameControllerDriveCommand());
	}
	public void robotInit(){
		prev_distance = 0;
		leftMotor1.setInverted(true);
		leftMotor2.setInverted(true);
		leftMotor3.setInverted(true);
		shiftedState = States.NOT_SHIFTED;
		transmissionState = States.LOW;
		gyro.reset();
		resetEncoders();
	}
	public void resetEncoders(){
		leftEncoder.reset();
		rightEncoder.reset();
	}
	public double getAngle(){
		return gyro.getAngle() % 360;
	}
	public double getVelocity(){
		return (((prev_distance + getEncoderDistance()) - prev_distance)/RobotConst.DRIVE_ENCODER_COUNTS_PER_FT)/ 0.0027;
	}
	public double getEncoderDistance(){
		return (leftEncoder.getDistance() + rightEncoder.getDistance())/2;
	}
	public void speedController(double speed){
		if(shiftedState.equals(States.NOT_SHIFTED)){
			if(getVelocity() > RobotConst.DRIVE_LOW_SHIFT_THRESHOLD && transmissionState.equals(States.LOW)){
				setAllMotorSpeeds(speed - RobotConst.DRIVE_PID_POWER_DROP);
				shiftHigh();
				transmissionState = States.HIGH;
				shiftedState = States.HAS_SHIFTED;
			}else if(getVelocity() < RobotConst.DRIVE_HIGH_SHIFT_THRESHOLD && transmissionState.equals(States.HIGH)){
				setAllMotorSpeeds(speed - RobotConst.DRIVE_PID_POWER_DROP);
				shiftLow();
				transmissionState = States.LOW;
				shiftedState = States.HAS_SHIFTED;
			}
			setAllMotorSpeeds(speed);
		}else if(shiftedState.equals(States.HAS_SHIFTED)){
			setAllMotorSpeeds(speed);
			shiftedState = States.NOT_SHIFTED;
		}
	}
	public void setAllMotorSpeeds(double speed){
		setLeftMotorSpeeds(pidController.calcPIDValue(speed, leftEncoder.getRate()));
		setRightMotorSpeeds(pidController.calcPIDValue(speed, rightEncoder.getRate()));
	}
	public void setDifferentMotorSpeeds(double leftSpeed, double rightSpeed){
		setLeftMotorSpeeds(pidController.calcPIDValue(leftSpeed, leftEncoder.getRate()));
		setRightMotorSpeeds(pidController.calcPIDValue(rightSpeed, rightEncoder.getRate()));
		
	}
	public void setLeftMotorSpeeds(double speed){
		leftMotor1.set(speed);
		leftMotor2.set(speed);
		leftMotor3.set(speed);
	}
	public void setRightMotorSpeeds(double speed){
		rightMotor1.set(speed);
		rightMotor2.set(speed);
		rightMotor3.set(speed);
	}
	public void shiftHigh(){
		shifterHigh.set(true);
		shifterLow.set(false);
	}
	public void shiftLow(){
		shifterHigh.set(false);
		shifterLow.set(true);
	}
	
}

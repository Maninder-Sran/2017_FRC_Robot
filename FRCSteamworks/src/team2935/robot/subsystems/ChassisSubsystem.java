package team2935.robot.subsystems;

import java.text.DecimalFormat;

import com.kauailabs.navx.frc.AHRS;
import com.toronto.subsystems.T_Subsystem;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team2935.robot.RobotConst;
import team2935.robot.RobotMap;
import team2935.robot.commands.drive.GameControllerDriveCommand;
import team2935.utils.DrivePIDController;
import team2935.utils.GyroPIDController;

public class ChassisSubsystem extends T_Subsystem {
	//Definition of the motors used on the drive_train subsystem
	private VictorSP leftMotor1 = new VictorSP(RobotMap.DRIVE_LEFT_MOTOR1);
	private VictorSP leftMotor2 = new VictorSP(RobotMap.DRIVE_LEFT_MOTOR2);
	private VictorSP leftMotor3 = new VictorSP(RobotMap.DRIVE_LEFT_MOTOR3);
	private VictorSP rightMotor1 = new VictorSP(RobotMap.DRIVE_RIGHT_MOTOR1);
	private VictorSP rightMotor2 = new VictorSP(RobotMap.DRIVE_RIGHT_MOTOR2);
	private VictorSP rightMotor3 = new VictorSP(RobotMap.DRIVE_RIGHT_MOTOR3);
	private Solenoid shifterHigh = new Solenoid(RobotMap.SOLENOID_SHIFTER_HIGH);
	private Solenoid shifterLow  = new Solenoid(RobotMap.SOLENOID_SHIFTER_LOW);
	
	//Definition of the sensors used on the drive_train subsystem
	private Encoder leftEncoder = new Encoder(RobotMap.DRIVE_LEFT_ENCODER_A,RobotMap.DRIVE_LEFT_ENCODER_B,true);
	private Encoder rightEncoder = new Encoder(RobotMap.DRIVE_RIGHT_ENCODER_A,RobotMap.DRIVE_RIGHT_ENCODER_B);
	private AHRS gyro = new AHRS(SerialPort.Port.kUSB);
	
	//Definition of PID controllers used for drive train control
	private DrivePIDController pidController = new DrivePIDController();
	private GyroPIDController gyroController = new GyroPIDController(0.2);

	//Declaration of variables used to track the current state the robot is in
	private States shiftedState;
	private States transmissionState;
	public Motions motion;
	
	//Declaration of variables that need to be tracked for the AUTO SHIFTER controller
	public int prev_distance;
	private double powerInput;
	
	public enum States{
		LOW,HIGH,HAS_SHIFTED,NOT_SHIFTED;
	}
	public enum Motions{
		STRAIGHT,TURN;
	}
	public void initDefaultCommand() {
		setDefaultCommand(new GameControllerDriveCommand());
	}
	public void robotInit(){
		prev_distance = 0;
		powerInput = 0;
		leftMotor1.setInverted(true);
		leftMotor2.setInverted(true);
		leftMotor3.setInverted(true);
		shiftedState = States.NOT_SHIFTED;
		transmissionState = States.LOW;
		motion = Motions.STRAIGHT;
		gyro.reset();
		resetEncoders();
	}
	public void resetEncoders(){
		leftEncoder.reset();
		rightEncoder.reset();
	}
	public void resetGyro(){
		gyro.reset();
	}
	public double getAngle(){
		return gyro.getAngle() % 360;
	}
	public double getEncoderDistance(){
		return (getLeftEncoderDistance() + getRightEncoderDistance())/2;
	}
	public double getLeftEncoderDistance(){
		return leftEncoder.getDistance();
	}
	public double getRightEncoderDistance(){
		return rightEncoder.getDistance();
	}
	public double getVelocity(){
		double velocity = ((getEncoderDistance() - prev_distance)/RobotConst.DRIVE_ENCODER_COUNTS_PER_FT)/ 0.027;
		DecimalFormat df = new DecimalFormat("#.0");
		return Double.valueOf(df.format(velocity));
	}
	public void speedController(double speed){
		if(shiftedState.compareTo(States.NOT_SHIFTED) == 0){
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
		}else if(shiftedState.compareTo(States.HAS_SHIFTED) == 0){
			setAllMotorSpeeds(speed);
			shiftedState = States.NOT_SHIFTED;
		}
	}
	public void setAllMotorSpeeds(double speed){
		setLeftMotorSpeeds(speed);
		setRightMotorSpeeds(speed);
	}
	public void setDifferentMotorSpeeds(double leftSpeed, double rightSpeed){
		setLeftMotorSpeeds(leftSpeed);
		setRightMotorSpeeds(rightSpeed);
	}
	public void setLeftMotorSpeeds(double speed){
		powerInput = pidController.calcPIDValue(speed, getVelocity(),powerInput);
		leftMotor1.set(powerInput);
		leftMotor2.set(powerInput);
		leftMotor3.set(powerInput);
	}
	public void setRightMotorSpeeds(double speed){
		powerInput = pidController.calcPIDValue(speed, getVelocity(),powerInput);
		rightMotor1.set(powerInput);
		rightMotor2.set(powerInput);
		rightMotor3.set(powerInput);
	}
	public void shiftHigh(){
		shifterHigh.set(true);
		shifterLow.set(false);
	}
	public void shiftLow(){
		shifterHigh.set(false);
		shifterLow.set(true);
	}
	@Override
	public void updatePeriodic() {
		SmartDashboard.putData("Left Encoder",leftEncoder);
    	SmartDashboard.putData("Right Encoder",rightEncoder);
    	SmartDashboard.putData("Gyro", gyro);
    	SmartDashboard.putNumber("Gyro", gyro.getAngle() % 360);
    	SmartDashboard.putNumber("Velocity",getVelocity());
	}
}

package team2935.utils;

import team2935.robot.Robot;
import team2935.robot.RobotConst;

public class GyroPIDController {
	private double setSpeed;
	
	public GyroPIDController(double speed){
		setSpeed = speed;
	}
	public void adjustAngle(double targetAngle, double currentAngle){
		double angleError = targetAngle - currentAngle;
    	
    	if (angleError > 180.0)  { angleError -= 360.0; }
    	if (angleError < -180.0) { angleError += 360.0; }
    	
		double leftSpeed  = setSpeed;
		double rightSpeed = setSpeed;

		// Slow down one motor based on the error.
		if (angleError > 0) {
    		rightSpeed -= angleError * RobotConst.GYRO_PROPORTIONAL_GAIN * setSpeed;
    		if (rightSpeed < -setSpeed) {
    			 rightSpeed = -setSpeed;
    		}
    	}
    	else {
    		leftSpeed -=  -angleError * RobotConst.GYRO_PROPORTIONAL_GAIN * setSpeed;
    		if (leftSpeed < -setSpeed) {
    			leftSpeed = -setSpeed;
    		}
    	}
		//Robot.chassisSubsystem.setDifferentMotorSpeeds(leftSpeed, rightSpeed);
	}
}

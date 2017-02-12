package team2935.utils;

import team2935.robot.Robot;
import team2935.robot.RobotConst;
import team2935.robot.subsystems.ChassisSubsystem.Motions;

public class DrivePIDController {
	//Declaration of variables that need to be tracked in the PID controller
		private double prev_error;
		private double delta_error;
		//private double integral_error;
		
	private double calcPValue(double error){
		return RobotConst.DRIVE_PID_P * error;
	}
	private double calcDValue(double error){
		delta_error = prev_error - error;
		return delta_error * RobotConst.DRIVE_PID_D;
	}
	public double calcPIDValue(double setPoint, double feedback,double powerInput){
    
		double error;
		if(Robot.chassisSubsystem.motion.compareTo(Motions.STRAIGHT) == 0)
			error = (setPoint - feedback)/RobotConst.MAX_LOW_DRIVE_SPEED;
		else
			error = (setPoint - feedback)/RobotConst.MAX_LOW_TURN_SPEED;
		
		double p_out = calcPValue(error);
    	double d_out = calcDValue(error);
     
    	double output = powerInput + p_out - d_out;
    	
    	double normalizedOutput  = output;
    	normalizedOutput = (normalizedOutput > 1.0) ? 1.0 : normalizedOutput;
    	normalizedOutput = (normalizedOutput < -1.0) ? -1.0 : normalizedOutput;
    	prev_error = error;
    	
    	return normalizedOutput;
	}
}

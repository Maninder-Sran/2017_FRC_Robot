package team2935.utils;

import team2935.robot.RobotConst;

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
	private double calcIValue(){
		return 0;
	}
	public double calcPIDValue(double setPoint, double feedback){
    	double normalizedFeedback = feedback/RobotConst.MAX_DRIVE_SPEED;
    	normalizedFeedback = (normalizedFeedback > 1.0) ? 1.0 : normalizedFeedback;  
    	normalizedFeedback = (normalizedFeedback < -1.0) ? -1.0 : normalizedFeedback;
    	
    	double error = setPoint - normalizedFeedback;
    	
    	double p_out = calcPValue(error);
    	double d_out = calcDValue(error);
     
    	double output = setPoint + p_out - d_out;
    	
    	double normalizedOutput  = output;
    	normalizedOutput = (normalizedOutput > 1.0) ? 1.0 : normalizedOutput;
    	normalizedOutput = (normalizedOutput < -1.0) ? -1.0 : normalizedOutput;
    	prev_error = error;
    	
    	return normalizedOutput;
    }
}

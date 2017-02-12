package team2935.oi;

import com.toronto.oi.T_Axis;
import com.toronto.oi.T_Button;
import com.toronto.oi.T_GameController;
import com.toronto.oi.T_Logitech_GameController;
import com.toronto.oi.T_Stick;

import team2935.robot.RobotMap;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 * 
 * Operator Controller (Game Controller)
 * -------------------------------------
 * Joysticks
 * ---------
 * Left:        Turn - X Axis
 * Right:       Speed - Y Axis
 * 
 * Triggers
 * ---------
 * Left:
 * Right:
 * 
 * Buttons
 * ---------
 * A:
 * B:
 * X:
 * Y:			Drive Straight Command
 * 
 * 
 * 
 * 
 * LBumper:
 * RBumper: 	Rumble (test) the Driver Controller
 * Start:		
 * Back:		Toggle (test)
 * 
 * POV:       	Go Straight Using Gyro Command
 *
 */

public class OI {
	public T_GameController driverController = new T_Logitech_GameController(RobotMap.DRIVE_CONTROLLER);
	 public boolean isDriverAction() {
		 return driverController.isControllerActivated();
	 } 
	 public double getDriveSpeed(){
	    	return driverController.getAxis(T_Stick.LEFT, T_Axis.Y);
	 }
	 public double getTurnSpeed(){
		return driverController.getAxis(T_Stick.RIGHT, T_Axis.X);
	 }
	 public boolean shifterEngaged(){
		 return driverController.getButton(T_Button.RIGHT_BUMPER);
	 }
	 public boolean grabGear(){
	//	 return driverController.getButton(T_Button.A);
		 return false;
	 }
	 public boolean moveGearMechanism(){
		 return driverController.getButton(T_Button.B);
	 }
	 public boolean shootFuel(){
		 return driverController.getButton(T_Button.X);
	 }
	 public boolean intakeFuel(){
		 return driverController.getButton(T_Button.Y);
	 }
	 public boolean driveStraightButton(){
		 return driverController.getButton(T_Button.A);
	 }
}

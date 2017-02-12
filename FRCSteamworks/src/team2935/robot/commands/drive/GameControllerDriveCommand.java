package team2935.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import team2935.robot.Robot;
import team2935.robot.RobotConst;
import team2935.robot.subsystems.ChassisSubsystem.Motions;


public class GameControllerDriveCommand extends Command {
	
	enum ButtonState { PRESSED, RELEASED };
	
	ButtonState driveStraightState = ButtonState.RELEASED;
	
	public GameControllerDriveCommand() {
		requires(Robot.chassisSubsystem);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double joystickDriveSpeedInput = Robot.oi.getDriveSpeed(); 
		double joystickTurnSpeedInput = Robot.oi.getTurnSpeed(); 
		double speed =  joystickDriveSpeedInput * RobotConst.MAX_LOW_DRIVE_SPEED;
		double turn =  joystickTurnSpeedInput * RobotConst.MAX_LOW_TURN_SPEED;
		/*switch(driveStraightState){
			case RELEASED:
				if(Robot.oi.driveStraightButton()){
					Scheduler.getInstance().add(new DriveToDistanceOnHeading(5, 0.5, 5));
					driveStraightState = ButtonState.PRESSED;
		    		return;
				}	
			case PRESSED:
				if (! Robot.oi.driveStraightButton()) {
	    			driveStraightState = ButtonState.RELEASED;
	    		}
	    		break;
		}*/
		if(Robot.oi.driveStraightButton()){
			Scheduler.getInstance().add(new TurnToAngle(90,2));
			return;
		}
		if(Math.abs(joystickTurnSpeedInput) < 0.1 && Math.abs(joystickDriveSpeedInput)> 0.1){
			Robot.chassisSubsystem.motion = Motions.STRAIGHT;
			Robot.chassisSubsystem.setAllMotorSpeeds(speed);
		}else{
			Robot.chassisSubsystem.motion = Motions.TURN;
			if(Math.abs(joystickDriveSpeedInput) > 0.1 && joystickTurnSpeedInput > 0.1){
				Robot.chassisSubsystem.setDifferentMotorSpeeds(speed, 0);
			}else if(Math.abs(joystickDriveSpeedInput) > 0.1 && joystickTurnSpeedInput < -0.1){
				Robot.chassisSubsystem.setDifferentMotorSpeeds(0, speed);
			}else{
				Robot.chassisSubsystem.setDifferentMotorSpeeds(turn, -turn);
			}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		Robot.chassisSubsystem.prev_distance = (int)Robot.chassisSubsystem.getEncoderDistance();
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}

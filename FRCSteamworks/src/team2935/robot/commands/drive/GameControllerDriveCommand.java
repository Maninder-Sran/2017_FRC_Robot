package team2935.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import team2935.robot.Robot;


public class GameControllerDriveCommand extends Command {
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
		double speed = Robot.oi.getDriveSpeed();
		double turn = Robot.oi.getTurnSpeed();
		if(Math.abs(turn) < 0.05 && Math.abs(speed)> 0.05){
			//Robot.chassisSubsystem.speedController(speed);
			Robot.chassisSubsystem.setAllMotorSpeeds(speed);
		}else{
			if(Math.abs(speed) > 0.05 && turn > 0.05){
				Robot.chassisSubsystem.setDifferentMotorSpeeds(speed, 0);
			}else if(Math.abs(speed) > 0.05 && turn < 0.05){
				Robot.chassisSubsystem.setDifferentMotorSpeeds(0, speed);
			}else{
				Robot.chassisSubsystem.setDifferentMotorSpeeds(turn, -turn);
			}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
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

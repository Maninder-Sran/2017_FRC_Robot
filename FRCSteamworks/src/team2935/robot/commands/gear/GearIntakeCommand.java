package team2935.robot.commands.gear;

import edu.wpi.first.wpilibj.command.Command;
import team2935.robot.Robot;

public class GearIntakeCommand extends Command {

    public GearIntakeCommand() {
       // requires(Robot.gearSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}

package team2935.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import team2935.robot.RobotMap;
import team2935.robot.commands.gear.GearIntakeCommand;

public class GearSubsystem extends 	Subsystem {

	Solenoid openClaw = new Solenoid(RobotMap.SOLENOID_OPEN_CLAW);
	Solenoid closeClaw = new Solenoid(RobotMap.SOLENOID_CLOSE_CLAW);
	Solenoid pushArm = new Solenoid(RobotMap.SOLENOID_PUSH_ARM);
	Solenoid pullArm = new Solenoid(RobotMap.SOLENOID_PULL_ARM);
    public void initDefaultCommand() {
        setDefaultCommand(new GearIntakeCommand());
    }
    public void closeClaw(){
    	closeClaw.set(true);
    	openClaw.set(false);
    }
    public void openClaw(){
    	openClaw.set(true);
    	closeClaw.set(false);
    }
    public void extendArm(){
    	pushArm.set(true);
    	pullArm.set(false);
    }
    public void retractArm(){
    	pullArm.set(true);
    	pushArm.set(false);
    }
}


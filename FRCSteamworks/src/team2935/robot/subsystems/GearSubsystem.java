package team2935.robot.subsystems;

import com.toronto.subsystems.T_Subsystem;

import edu.wpi.first.wpilibj.Solenoid;
import team2935.robot.RobotMap;
import team2935.robot.commands.gear.GearIntakeCommand;

public class GearSubsystem extends T_Subsystem {
	@Override
	public void robotInit() {
		// TODO Auto-generated method stub
		
	}
	@Override
	public void updatePeriodic() {
		// TODO Auto-generated method stub
		
	}
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


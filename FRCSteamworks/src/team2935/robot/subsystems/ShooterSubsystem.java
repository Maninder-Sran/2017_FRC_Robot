package team2935.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import team2935.robot.RobotMap;
import team2935.robot.commands.shooter.ShootFuelCommand;

public class ShooterSubsystem extends Subsystem {
	
	private VictorSP shooter = new VictorSP(RobotMap.SHOOTER_SHOOT_MOTOR);
	private VictorSP feeder = new VictorSP(RobotMap.SHOOTER_FEEDER_MOTOR);

	public void initDefaultCommand() {
        setDefaultCommand(new ShootFuelCommand());
    }
	public void shootFuel(double speed){
		shooter.set(speed);
	}
	public void feedFuel(double speed){
		feeder.set(speed);
	}

}


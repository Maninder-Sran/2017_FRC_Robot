package team2935.robot.subsystems;

import com.toronto.subsystems.T_Subsystem;

import edu.wpi.first.wpilibj.VictorSP;
import team2935.robot.RobotMap;
import team2935.robot.commands.shooter.ShootFuelCommand;

public class ShooterSubsystem extends T_Subsystem {
	
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
	@Override
	public void robotInit() {
		// TODO Auto-generated method stub
		
	}
	@Override
	public void updatePeriodic() {
		// TODO Auto-generated method stub
		
	}
}


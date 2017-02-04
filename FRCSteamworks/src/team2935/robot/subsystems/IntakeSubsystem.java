package team2935.robot.subsystems;

import com.toronto.subsystems.T_Subsystem;

import team2935.robot.commands.intake.FuelIntakeCommand;

public class IntakeSubsystem extends T_Subsystem {

    public void initDefaultCommand() {
        setDefaultCommand(new FuelIntakeCommand());
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


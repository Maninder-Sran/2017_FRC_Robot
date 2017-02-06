package team2935.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import team2935.robot.commands.intake.FuelIntakeCommand;

public class IntakeSubsystem extends Subsystem {

    public void initDefaultCommand() {
        setDefaultCommand(new FuelIntakeCommand());
    }
}


package frc.robot.subsystems.intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeCommands {

    public static Command stowIntakeCommand(IntakeSubsystem intake) {
        return Commands.runOnce(() -> intake.stowIntake(), intake);
    }

    public static Command deployIntakeCommand(IntakeSubsystem intake) {
        return Commands.runOnce(() -> intake.deployIntake(), intake);
    }
    
}

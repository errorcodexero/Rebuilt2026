package frc.robot.subsystems.intake;
import edu.wpi.first.wpilibj2.command.Command;

public class StowCmd extends Command {
    public final IntakeSubsystem intake_;

    public StowCmd(IntakeSubsystem intake) {
        intake_= intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake_.stowIntake();
    }

    public void end(boolean interrupted) {
    }

    public boolean isFinished() {
        return false;
    }
}

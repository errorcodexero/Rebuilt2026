package frc.robot.subsystems.intake;
import edu.wpi.first.wpilibj2.command.Command;

public class DeployCmd extends Command {
    public final IntakeSubsystem intake_;
    public DeployCmd(IntakeSubsystem intake) {
        intake_= intake;
        addRequirements(intake);
    }
    @Override
    public void execute() {
        intake_.deployIntake();
    }
    public void end(boolean interrupted) {
    }
    public boolean isFinished() {
        return false;
    }
    
}

package frc.robot.subsystems.intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.Voltage;


public class CollectCmd extends Command {
    public final IntakeSubsystem intake_;
    public final Voltage rollerVoltage_= IntakeConstants.rollerCollectVoltage;
    
    public CollectCmd(IntakeSubsystem intake) {
        intake_= intake;
        addRequirements(intake);
    }
    @Override
    public void execute() {
        intake_.deployIntake();
        intake_.setRollerVoltage(rollerVoltage_);
    }
    public void end(boolean interrupted) {
        intake_.stopRoller();
        intake_.stowIntake();
    }
    public boolean isFinished() {
        return false;
    }
}

package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveIntakeCmd extends Command {
    private final IntakeSubsystem intake;
    private int index ;
    private final Angle[] intakeAngles ;
    private Angle current ;
    private final Time delay;
    private final Timer delayTimer;
    private boolean waitingForDelay;

    public MoveIntakeCmd(IntakeSubsystem intake, Angle[] shootAngles, Time delay) {
        this.intake = intake;
        this.intakeAngles = shootAngles;
        this.delay = delay;
        this.delayTimer = new Timer();
        this.waitingForDelay = false;
    }

    @Override
    public void initialize() {
        index = 0 ;
        current = intakeAngles[index];
        intake.setPivotAngle(current);
        waitingForDelay = false;
    }

    @Override
    public void execute() {
        if (waitingForDelay) {
            // Waiting for delay to complete
            if (delayTimer.hasElapsed(delay)) {
                // Delay complete, move to next angle
                waitingForDelay = false;
                index = (index + 1) % intakeAngles.length;
                current = intakeAngles[index];
                intake.setPivotAngle(current);
            }
        } else {
            // Waiting for angle to be reached
            if (intake.isPivotAtAngle(current)) {
                // Angle reached, start delay
                waitingForDelay = true;
                delayTimer.restart();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

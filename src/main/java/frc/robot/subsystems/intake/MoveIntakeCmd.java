package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveIntakeCmd extends Command {
    private final IntakeSubsystem intake;

    private final Timer delayTimer;
    private final Time delay;

    private final Angle lower;
    private final Angle divisionSize;

    private int index;
    private boolean waitingForDelay;
    private boolean isUp;

    public MoveIntakeCmd(IntakeSubsystem intake, Angle lower, Angle upper, Time delay) {
        this.intake = intake;
        this.lower = lower;
        this.delay = delay;

        divisionSize = lower.minus(upper).div(IntakeConstants.shakeDivisions);

        delayTimer = new Timer();
        waitingForDelay = false;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        index = 1;
        isUp = true;

        intake.setPivotAngle(lower);
        intake.setRollerVelocity(IntakeConstants.rollerShootVelocity);

        waitingForDelay = false;
    }

    @Override
    public void execute() {
        if (waitingForDelay) {
            // Waiting for delay to complete
            if (delayTimer.hasElapsed(delay)) {
                // Delay complete, move to next angle
                waitingForDelay = false;
                if (isUp) {
                    intake.setPivotAngle(lower.minus(divisionSize.times(index)));
                    index = Math.min(index + 1, IntakeConstants.shakeDivisions);
                    isUp = false;
                } else {
                    intake.setPivotAngle(lower);
                    isUp = true;
                }
            }
        } else {
            // Waiting for angle to be reached
            if (intake.isPivotAtSetpoint()) {
                // Angle reached, start delay
                waitingForDelay = true;
                delayTimer.restart();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveIntakeCmd extends Command {
    private final IntakeSubsystem intake;

    private final Timer delayTimer;
    private final Time delayUp;
    private final Time delayDown;

    private final Angle lower;
    private final Angle divisionSize;

    private int index;
    private boolean waitingForDelay;
    private boolean upNext;

    public MoveIntakeCmd(IntakeSubsystem intake, Angle lower, Angle upper, Time delayUp, Time delayDown) {
        this.intake = intake;
        this.lower = lower;
        this.delayUp = delayUp;
        this.delayDown = delayDown;

        divisionSize = lower.minus(upper).div(IntakeConstants.shakeDivisions);

        delayTimer = new Timer();
        waitingForDelay = false;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        index = 1;
        upNext = true;

        intake.setPivotAngle(lower);
        intake.setRollerVelocity(IntakeConstants.rollerShootVelocity);

        waitingForDelay = false;
    }

    @Override
    public void execute() {
        if (waitingForDelay) {
            // Waiting for delay to complete
            if (delayTimer.hasElapsed(upNext ? delayDown : delayUp)) {
                // Delay complete, move to next angle
                waitingForDelay = false;
                if (upNext) {
                    var setpoint = lower.minus(divisionSize.times(index));
                    intake.setPivotAngle(setpoint);
                    index = Math.min(index + 1, IntakeConstants.shakeDivisions);
                    upNext = false;
                } else {
                    var setpoint = intake.getPivotAngle().plus(Degrees.of(10));
                    intake.setPivotAngle(setpoint);
                    upNext = true;
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
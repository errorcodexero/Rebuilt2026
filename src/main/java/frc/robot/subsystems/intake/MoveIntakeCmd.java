package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveIntakeCmd extends Command {
    private final IntakeSubsystem intake;

    private final Timer delayTimer;
    private final Time delayWhenUp;
    private final Time delayWhenDown;

    private final Angle lower;
    private final Angle upper;

    private boolean waitingForDelay;
    private boolean upNext;

    public MoveIntakeCmd(IntakeSubsystem intake, Angle lower, Angle upper, Time delayUp, Time delayDown) {
        this.intake = intake;
        this.lower = lower;
        this.delayWhenUp = delayUp;
        this.delayWhenDown = delayDown;
        this.upper = upper;

        delayTimer = new Timer();
        waitingForDelay = false;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        upNext = true;

        intake.setPivotAngle(lower);
        waitingForDelay = false;
    }

    @Override
    public void execute() {
        if (waitingForDelay) {
            // Waiting for delay to complete
            if (delayTimer.hasElapsed(upNext ? delayWhenDown : delayWhenUp)) {
                // Delay complete, move to next angle
                waitingForDelay = false;
                if (upNext) {
                    //Move to upper angle
                    var setpoint = lower.minus(upper);
                    intake.setPivotAngle(setpoint);
                    intake.setRollerVelocity(IntakeConstants.rollerShootVelocity);
                    upNext = false;
                } else {
                    var setpoint = lower;
                    intake.setPivotAngle(setpoint);
                    intake.stopRoller();
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
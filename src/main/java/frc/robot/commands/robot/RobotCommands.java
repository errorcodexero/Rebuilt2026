package frc.robot.commands.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class RobotCommands {
    /**
     * Shoots into the hub.
     * @param shooter
     * @param hopper
     * @param drive
     * @return
     */
    private static Command shootHub(Shooter shooter, Hopper hopper, IntakeSubsystem intake, Drive drive, Trigger shakeTrigger) {
        BooleanSupplier aimedAtHub =
            () -> drive.rotationIsNear(RobotState.rotationToHub(), ShooterConstants.aimingTolerance);

        return Commands.parallel(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> 0,
                () -> 0, 
                () -> RobotState.rotationToHub()
            ).finallyDo(drive::stopWithX),
            Commands.repeatingSequence(
                Commands.waitUntil(aimedAtHub).deadlineFor(shooter.spinUpForDistance(RobotState::hubDistance)),
                shooter.shootAtDistance(RobotState::hubDistance, hopper, intake, shakeTrigger)
                    .until(() -> !aimedAtHub.getAsBoolean())
            )
        );
    }

    /**
     * Ferrys into a target in your alliance zone. Allows movement.
     * @param shooter
     * @param hopper
     * @param drive
     * @return
     */
    private static Command ferry(Shooter shooter, Hopper hopper, IntakeSubsystem intake, Drive drive, Trigger shakeTrigger) {
        Translation2d rightTarget =
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? ShooterConstants.Positions.blueTargetRight
                : ShooterConstants.Positions.redTargetRight;

        Translation2d leftTarget =
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? ShooterConstants.Positions.blueTargetLeft
                : ShooterConstants.Positions.redTargetLeft;

        Supplier<Translation2d> target =
            () -> drive.getPose().getY() < ShooterConstants.Positions.centerLineY
                ? rightTarget
                : leftTarget;

        Supplier<Distance> targetDistance =
            () -> Meters.of(target.get().getDistance(drive.getPose().getTranslation()));

        Supplier<Rotation2d> targetingAngle = 
            () -> {
                var botToHub = target.get().minus(drive.getPose().getTranslation());
                return new Rotation2d(botToHub.getX(), botToHub.getY());
            };

        return DriveCommands.joystickDriveAtAngle(targetingAngle)
            .alongWith(shooter.shootAtDistance(targetDistance, hopper, intake, shakeTrigger));
    }

    /**
     * Shoots at either the hub, or ferrying targets based on the current robot position.
     * @param shooter
     * @param hopper
     * @param drive
     * @return
     */
    public static Command shoot(Shooter shooter, Hopper hopper, IntakeSubsystem intake, Drive drive, Trigger shakeTrigger) {
        return Commands.either(
            shootHub(shooter, hopper, intake, drive, shakeTrigger),
            ferry(shooter, hopper, intake, drive, shakeTrigger),
            RobotState::inAllianceZone
        );
    }

    /**
     * Ejects balls from the shooter at a low velocity to get them out of the shooter without shooting them towards the target.
     * @param shooter
     * @return
     */
    public static Command ejectUp(Shooter shooter, Hopper hopper) {
        return shooter.ejectUp().alongWith(hopper.ejectUp());
    }

    /**
     * The full sequence for intaking balls, with scrambler movement. For intake while shooting, or
     * any situation where we want to intake while the hopper is already required, just use intake.intakeSequence().
     * 
     * For most cases, this command is preferred.
     * @param intake
     * @param hopper
     * @return
     */
    public static Command intake(IntakeSubsystem intake, Hopper hopper) {
        return intake.intakeSequence().alongWith(hopper.collectScrambler());
    }
}

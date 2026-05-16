package frc.robot.commands.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.AprilTagVision;

public class RobotCommands {
    /**
     * Shoots into the hub while also managing the drivebase angle to aim into the hub.
     * This is the main shooting command for most use cases.
     * @param shooter
     * @param hopper
     * @param drive
     * @return
     */
    public static Command shootHub(Shooter shooter, Hopper hopper, IntakeSubsystem intake, Drive drive) {
        BooleanSupplier shouldXWheels =
            () -> drive.rotationIsNear(RobotState.rotationToHub(), ShooterConstants.xWheelTolerance);

        BooleanSupplier shouldRotateAgain =
            () -> !drive.rotationIsNear(RobotState.rotationToHub(), ShooterConstants.unXWheelTolerance);

        return shootHubNoAim(shooter, hopper, intake, drive).alongWith(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> 0,
                () -> 0, 
                RobotState::rotationToHub
            )
            .until(shouldXWheels)
            .andThen(drive.stopWithXCmd(), drive.idle().until(shouldRotateAgain))
            .repeatedly()
        );
    }

    public static Command shootAtTagDemo(Shooter shooter, Hopper hopper, IntakeSubsystem intake, Drive drive, AprilTagVision vision) {
        Supplier<Rotation2d> target =
            () -> drive.getRotation().minus(Rotation2d.fromDegrees(vision.getCameraData(0).simpleX));

        Supplier<Distance> distance =
            () -> Meters.of(vision.getCameraData(0).poseEstimate.averageDist()).times(0.55);

        BooleanSupplier shouldXWheels =
            () -> drive.rotationIsNear(target.get(), ShooterConstants.xWheelTolerance);

        BooleanSupplier shouldRotateAgain =
            () -> !drive.rotationIsNear(target.get(), ShooterConstants.unXWheelTolerance);

        BooleanSupplier shooting =
            () -> vision.getCameraData(0).simpleID == 25;

        return shootNoAim(shooter, hopper, intake, drive, distance, target)
            .beforeStarting(Commands.waitUntil(shooting))
            .onlyWhile(shooting)
            .repeatedly()
            .alongWith(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> 0,
                    () -> 0, 
                    target
                )
                .until(shouldXWheels)
                .andThen(drive.stopWithXCmd(), drive.idle().until(shouldRotateAgain))
                .repeatedly()
            );
    }

    /**
     * Shoots at a specified distance without aiming. This should not be used in most situations,
     * and only when you need additional flexibility like trying to squeeze
     * the most time out of auto.
     * @param shooter
     * @param hopper
     * @param intake
     * @param drive
     * @param distance
     * @return
     */
    public static Command shootNoAim(Shooter shooter, Hopper hopper, IntakeSubsystem intake, Drive drive, Supplier<Distance> distance, Supplier<Rotation2d> angle) {
        BooleanSupplier shouldShoot =
            () -> drive.rotationIsNear(angle.get(), ShooterConstants.aimingTolerance);

        BooleanSupplier shouldStopShooting =
            () -> !drive.rotationIsNear(angle.get(), ShooterConstants.defenseTolerance);

        BooleanSupplier upToSpeedAndAimed =
            () -> shouldShoot.getAsBoolean()
                && shooter.isShooterReady()
                && shooter.getShooterVelocity().gt(RotationsPerSecond.of(10));

        return Commands.repeatingSequence(
            Commands.waitUntil(upToSpeedAndAimed),
            hopper.feedForShooting(shouldShoot, intake)
                .until(shouldStopShooting)
        ).alongWith(
            shooter.shootAtDistance(distance)
        );
    }

    /**
     * Shoots into the hub, without aiming. This should not be used in most situations,
     * and only when you need additional flexibility like trying to squeeze
     * the most time out of auto.
     * @param shooter
     * @param hopper
     * @param intake
     * @param drive
     * @return
     */
    public static Command shootHubNoAim(Shooter shooter, Hopper hopper, IntakeSubsystem intake, Drive drive) {
        return shootNoAim(shooter, hopper, intake, drive, RobotState::hubDistance, RobotState::rotationToHub);
    }

    /**
     * Ferrys into a target in your alliance zone. Allows movement.
     * @param shooter
     * @param hopper
     * @param drive
     * @return
     */
    public static Command ferry(Shooter shooter, Hopper hopper, IntakeSubsystem intake, Drive drive) {
        return Commands.defer(() -> {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            Translation2d rightTarget;
            Translation2d leftTarget;

            if (RobotState.inOpposingAllianceZone()) {
                rightTarget =
                    alliance == Alliance.Blue
                        ? ShooterConstants.Positions.blueBumpTargetRight
                        : ShooterConstants.Positions.redBumpTargetRight;

                leftTarget =
                    alliance == Alliance.Blue
                        ? ShooterConstants.Positions.blueBumpTargetLeft
                        : ShooterConstants.Positions.redBumpTargetLeft;
            } else {
                rightTarget =
                    alliance == Alliance.Blue
                        ? ShooterConstants.Positions.blueTargetRight
                        : ShooterConstants.Positions.redTargetRight;

                leftTarget =
                    alliance == Alliance.Blue
                        ? ShooterConstants.Positions.blueTargetLeft
                        : ShooterConstants.Positions.redTargetLeft;
            }

            Supplier<Translation2d> target =
                () -> drive.getPose().getY() < ShooterConstants.Positions.centerLineY
                    ? rightTarget
                    : leftTarget;

            Supplier<Distance> targetDistance = () -> Meters.of(target.get().getDistance(drive.getPose().getTranslation()));

            Supplier<Rotation2d> targetingAngle = () -> {
                var botToTarget = target.get().minus(drive.getPose().getTranslation());
                return new Rotation2d(botToTarget.getX(), botToTarget.getY());
            };

            BooleanSupplier ready = () ->
                drive.rotationIsNear(targetingAngle.get(), ShooterConstants.aimingTolerance)
                && shooter.isShooterReady()
                && shooter.getShooterVelocity().gt(RotationsPerSecond.of(10));

            return DriveCommands.joystickDriveAtAngle(targetingAngle)
                .alongWith(
                    shooter.shootAtDistance(targetDistance),

                    Commands.waitUntil(ready)
                        .andThen(hopper.feedForShooting(() -> true, intake)),

                    Commands.runOnce(() -> {
                        Logger.recordOutput("Ferry/Target", target.get());
                        Logger.recordOutput("Ferry/IsFerrying", true);
                    })
                ).finallyDo(i -> Logger.recordOutput("Ferry/IsFerrying", false));
        }, Set.of(shooter, hopper, intake, drive));
    }

    /**
     * Shoots at either the hub, or ferrying targets based on the current robot position.
     * @param shooter
     * @param hopper
     * @param drive
     * @return
     */
    public static Command shoot(Shooter shooter, Hopper hopper, IntakeSubsystem intake, Drive drive) {
        return Commands.either(
            shootHub(shooter, hopper, intake, drive),
            ferry(shooter, hopper, intake, drive),
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

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
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
    private static Command shootHub(Shooter shooter, Hopper hopper, Drive drive) {
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
                Commands.waitUntil(aimedAtHub),
                shooter.shootAtDistance(RobotState::hubDistance, hopper)
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
    private static Command ferry(Shooter shooter, Hopper hopper, Drive drive) {
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
            .alongWith(shooter.shootAtDistance(targetDistance, hopper));
    }

    /**
     * Shoots at either the hub, or ferrying targets based on the current robot position.
     * @param shooter
     * @param hopper
     * @param drive
     * @return
     */
    public static Command shoot(Shooter shooter, Hopper hopper, Drive drive, CommandXboxController gamepad, boolean shootOnMove) {
        return Commands.parallel(
                DriveCommands.pointAtShootingTarget(drive, gamepad, shootOnMove),
                Commands.sequence(
                    Commands.waitTime(ShooterConstants.dbRotationDelay), 
                    shooter.shoot(drive, hopper, gamepad, shootOnMove)
                )
            );
    }
}

package frc.robot.commands.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
                shooter.shoot(() -> drive.getPose(), hopper)
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
        return Commands.parallel(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> drive.getChassisSpeeds().vxMetersPerSecond,
                () -> drive.getChassisSpeeds().vyMetersPerSecond, 
                () -> {

                    var rotation = new Rotation2d();

                    var targetTranslation = new Translation2d();
                    
                    Translation2d rightTarget =
                            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                            ? ShooterConstants.Positions.blueTargetRight
                            : ShooterConstants.Positions.redTargetRight;

                    Translation2d leftTarget =
                            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                            ? ShooterConstants.Positions.blueTargetLeft
                            : ShooterConstants.Positions.redTargetLeft;

                    if (drive.getPose().getY() < ShooterConstants.Positions.centerLineY) {
                        targetTranslation = rightTarget.minus(drive.getPose().getTranslation());
                    }
                    else {
                        targetTranslation = leftTarget.minus(drive.getPose().getTranslation());
                    }
                    rotation = new Rotation2d(targetTranslation.getX(), targetTranslation.getY());

                    return rotation;
                }   
            ),
            Commands.waitUntil(() -> Math.abs(drive.getChassisSpeeds().omegaRadiansPerSecond) < .001).andThen(shooter.shoot(() -> drive.getPose(), hopper))
        );
    }

    public static Command shoot(Shooter shooter, Hopper hopper, Drive drive) {
        return Commands.either(
            shootHub(shooter, hopper, drive),
            ferry(shooter, hopper, drive),
            RobotState::inAllianceZone
        );
    }
}

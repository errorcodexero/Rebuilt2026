package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;

public class RobotCommands {
    public static Command shoot(Supplier<Pose2d> pose, Shooter shooter, Hopper hopper, Drive drive) {
        return new ConditionalCommand(Commands.parallel(DriveCommands.joystickDriveAtAngle(drive,
                () -> 0,
                () -> 0, 
                () -> {
                    Translation2d hub =
                            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                            ? ShooterConstants.Positions.blueHubPose
                            : ShooterConstants.Positions.redHubPose;
                    
                    var hubTranslation = hub.minus(drive.getPose().getTranslation());
                    var rotation = new Rotation2d(hubTranslation.getX(), hubTranslation.getY());

                    return rotation;
                }).andThen(drive.stopWithXCmd()),
                Commands.waitUntil(() -> Math.abs(drive.getChassisSpeeds().omegaRadiansPerSecond) < .001).andThen(shooter.shoot(() -> drive.getPose(), hopper))
            ),

            Commands.parallel(DriveCommands.joystickDriveAtAngle(drive,
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
                }),
                Commands.waitUntil(() -> Math.abs(drive.getChassisSpeeds().omegaRadiansPerSecond) < .001).andThen(shooter.shoot(() -> drive.getPose(), hopper))
            ),

            () -> { 

                Pose2d robotPose = pose.get();

                var zone =
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? ShooterConstants.Positions.blueAllianceWall
                    : ShooterConstants.Positions.redAllianceWall;
                
                if (Math.abs(robotPose.getX() - zone.magnitude()) < ShooterConstants.Positions.spinUpZone.magnitude()) {
                    return true;
                }
                return false;
            }
        );
    }
}

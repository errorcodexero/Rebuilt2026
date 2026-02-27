package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class AutoCommands {


    public static Command NeutralZoneTrenchToTrench(Drive drive, IntakeSubsystem intake, Hopper hopper, Shooter shooter){
        return Commands.sequence(
            DriveCommands.initialFollowPathCommand(drive,"TopToBottom Trench").deadlineFor(intake.intakeSequence()),

            Commands.waitSeconds(.2).andThen(shooter.shoot( () -> drive.getPose(), hopper)).withTimeout(3.8).deadlineFor(DriveCommands.joystickDriveAtAngle(drive, () -> 0,  () -> 0, () -> {
                Translation2d hub =
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? ShooterConstants.Positions.blueHubPose
                    : ShooterConstants.Positions.redHubPose;
                    
                var hubTranslation = hub.minus(drive.getPose().getTranslation());
                var rotation = new Rotation2d(hubTranslation.getX(), hubTranslation.getY());

                return rotation;
            })),
        
            DriveCommands.followPathCommand("BottomToTop").deadlineFor(intake.intakeSequence()),

            Commands.waitSeconds(.2).andThen(shooter.shoot(() -> drive.getPose(), hopper).withTimeout(3.8)).deadlineFor(DriveCommands.joystickDriveAtAngle(drive, () -> 0,  () -> 0, () -> {
                Translation2d hub =
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? ShooterConstants.Positions.blueHubPose
                    : ShooterConstants.Positions.redHubPose;
                    
                var hubTranslation = hub.minus(drive.getPose().getTranslation());
                var rotation = new Rotation2d(hubTranslation.getX(), hubTranslation.getY());

                return rotation;
            }))
        );
    }
    
}
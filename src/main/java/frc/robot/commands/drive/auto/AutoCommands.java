package frc.robot.commands.drive.auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.DriveCommands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.thriftyclimb.ThriftyClimb;
import frc.robot.subsystems.vision.AprilTagVision;
import edu.wpi.first.units.measure.Voltage;

public class AutoCommands {
    public static Command DepotShootClimbAuto(Drive drive, IntakeSubsystem intake, Hopper hopper, Shooter shooter, ThriftyClimb climber, boolean fieldside){
        return Commands.sequence(
            Commands.runOnce(() -> drive.resetGyroCmd()),
            Commands.parallel(
                DriveCommands.initialFollowPathCommand(drive,"GoToDepot"),
                intake.deployCmd()
            ),
            Commands.runOnce(()-> System.err.println("DRIVE POSE AFTER RESET:" + drive.getPose())),
            DriveCommands.followPathCommand("Collect").deadlineWith(intake.intakeSequence()),

            DriveCommands.followPathCommand("GoToShoot"),

            shooter.shootCmd(hopper).withTimeout(2.5),

            shooter.stopCmd(),

            DriveCommands.followPathCommand("ShootToClimb")
        );
    }
    
}

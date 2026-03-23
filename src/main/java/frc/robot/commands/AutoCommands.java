package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.robot.RobotCommands;
import frc.robot.commands.robot.StartupCmd;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;

public class AutoCommands {
    public static Command a1TrenchToTrench(Drive drive, IntakeSubsystem intake, Hopper hopper, Shooter shooter, boolean mirroredX) {
        return Commands.sequence(
            Commands.deadline(
                DriveCommands.initialFollowPathCommand(drive,"A1_TopToBottomTrench", mirroredX),
                new StartupCmd(intake, hopper, shooter).beforeStarting(Commands.waitTime(Seconds.of(0.8)))
            ),

            // Added: timeout on shoot
            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(5.0)),
            DriveCommands.followPathCommand("A1_BottomToTopTrench", mirroredX)
                .deadlineFor(RobotCommands.intake(intake, hopper)),

            // Added: timeout on shoot
            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(5.0))
        );
    }

    public static Command a2NZCollectAuto(Drive drive, Shooter shooter, IntakeSubsystem intake, Hopper hopper, boolean mirroredX) {
        return Commands.sequence(
            //Drive from trench to a point in neutral zone, collecting balls and bringing down intake
            Commands.deadline(
                DriveCommands.initialFollowPathCommand(drive, "a2TrenchToNZ", mirroredX),
                new StartupCmd(intake, hopper, shooter).beforeStarting(Commands.waitTime(Seconds.of(0.6)))
            ),
            //Drive robot to the shoot position and shoot balls into hub
            DriveCommands.followPathCommand("a2NZtoShoot1", mirroredX).deadlineFor(
                shooter.spinUpForDistanceHoodParked(() -> Meters.of(3.7)),
                hopper.preShoot()
            ),

            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(5.0)),

            //Drive back into the neutral zone, collecting more balls along the way
            DriveCommands.followPathCommand("a2Shoot1toHub", mirroredX)
                .deadlineFor(RobotCommands.intake(intake, hopper)),
            
            //Drive back to alliance zone and shoot the rest of the balls into the hub
            DriveCommands.followPathCommand("a2HubToShoot2", mirroredX).deadlineFor(
                shooter.spinUpForDistanceHoodParked(() -> Meters.of(3.7)),
                hopper.preShoot()
            ),

            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(10.0))
            
        ); 
    }

    public static Command a3DepotAuto(Drive drive, Shooter shooter, IntakeSubsystem intake, Hopper hopper) {
        return Commands.sequence(
            //Drive from trench to a point in neutral zone, collecting balls and bringing down intake
            Commands.deadline(
                DriveCommands.initialFollowPathCommand(drive, "a3BumpToDepot"),
                new StartupCmd(intake, hopper, shooter).beforeStarting(Commands.waitTime(Seconds.of(0.6)))
            ),

            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(5.0)),

            DriveCommands.followPathCommand("a3DepotToOutpost")
                .deadlineFor(RobotCommands.intake(intake, hopper)),

            DriveCommands.followPathCommand("a3OutpostToShoot"),

            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(8.0))
        ); 
    }
    
    public static Command a4OnlyPreload(Drive drive, Shooter shooter, IntakeSubsystem intake, Hopper hopper) {
        return Commands.sequence(
            DriveCommands.initialFollowPathCommand(drive, "a4OnlyPreload")
                .deadlineFor(new StartupCmd(intake, hopper, shooter).beforeStarting(Commands.waitTime(Seconds.of(0.3)))),

            RobotCommands.shoot(shooter, hopper, intake, drive).withTimeout(Seconds.of(4))
        );
    }

    public static Command aTest1DriveCollect(Drive drive, Shooter shooter, IntakeSubsystem intake, Hopper hopper) {
        return new StartupCmd(intake, hopper, shooter).alongWith(
            DriveCommands.initialFollowPathCommand(drive, "aTest1DriveCollect")
                .beforeStarting(Commands.waitTime(Seconds.one()))
        );
    }

    private static final Time intakeStartTime1 = Seconds.of(0.7);
    private static final Time intakeLength1 = Seconds.of(4.0);
    private static final Time shootingStartTime1 = Seconds.of(4.8);
    private static final Time shootingLength1 = Seconds.of(5.5);

    private static final Time intakeStartTime2 = Seconds.of(0.7);
    private static final Time intakeLength2 = Seconds.of(4.5);
    private static final Time shootingStartTime2 = Seconds.of(5.2);
    private static final Time shootingLength2 = Seconds.of(6);

    public static Command a5NZCollectAutoAlt(Drive drive, Shooter shooter, IntakeSubsystem intake, Hopper hopper, boolean mirroredX) {
        return Commands.sequence(
            //Drive from trench to a point in neutral zone, collecting balls and bringing down intake
            // Commands.parallel(
            //     DriveCommands.initialFollowPathCommand(drive, "a5CollectLoop1", mirroredX)
            //         .deadlineFor(new StartupCmd(intake, hopper, shooter).beforeStarting(Commands.waitTime(Seconds.of(0.6))))
                
            //     // RobotCommands.shootHubNoAim(shooter, hopper, intake, drive).beforeStarting(Commands.waitTime(shootingStartTime1))
            // ),

            Commands.parallel(
                DriveCommands.initialFollowPathCommand(drive, "a5CollectLoop1", mirroredX),
                Commands.sequence(
                    Commands.waitTime(intakeStartTime1)
                        .alongWith(Commands.runOnce(() -> Logger.recordOutput("AutoState", "Waiting"))),

                    new StartupCmd(intake, hopper, shooter).withTimeout(intakeLength1)
                        .alongWith(Commands.runOnce(() -> Logger.recordOutput("AutoState", "Intake"))),

                    shooter.spinUpForDistanceHoodParked(() -> Meters.of(2)).withTimeout(shootingStartTime1.minus(intakeLength1))
                        .alongWith(Commands.runOnce(() -> Logger.recordOutput("AutoState", "Spinup"))),

                    shooter.shootAtDistance(RobotState::hubDistance, hopper, intake).withTimeout(shootingLength1)
                        .alongWith(Commands.runOnce(() -> Logger.recordOutput("AutoState", "Shooting")))
                )
            ),

            Commands.parallel(
                DriveCommands.initialFollowPathCommand(drive, "a5CollectLoop2", mirroredX),
                Commands.sequence(
                    Commands.waitTime(intakeStartTime2)
                        .alongWith(Commands.runOnce(() -> Logger.recordOutput("AutoState", "Waiting"))),

                    RobotCommands.intake(intake, hopper).withTimeout(intakeLength2)
                        .alongWith(Commands.runOnce(() -> Logger.recordOutput("AutoState", "Intake"))),

                    shooter.spinUpForDistanceHoodParked(() -> Meters.of(2)).withTimeout(shootingStartTime2.minus(intakeLength2))
                        .alongWith(Commands.runOnce(() -> Logger.recordOutput("AutoState", "Spinup"))),
                    
                    shooter.shootAtDistance(RobotState::hubDistance, hopper, intake).withTimeout(shootingLength2)
                        .alongWith(Commands.runOnce(() -> Logger.recordOutput("AutoState", "Shooting")))
                )
            )

            // Commands.parallel(
            //     // RobotCommands.shootHubNoAim(shooter, hopper, intake, drive).withTimeout(shootingLength2),
                
            //     DriveCommands.followPathCommand("a5CollectLoop2", mirroredX)
            //         .deadlineFor(RobotCommands.intake(intake, hopper))
            //         .beforeStarting(Commands.waitTime(shootingLength2.minus(shootingOverlap2)))
            // ),

            // RobotCommands.shoot(shooter, hopper, intake, drive)
        );
    }
}
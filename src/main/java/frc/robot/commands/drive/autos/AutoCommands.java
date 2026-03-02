package frc.robot.commands.drive.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSubsystem;


public class AutoCommands {
    public static Command NZCollectAuto(Drive drive, Shooter shooter, IntakeSubsystem intake, Hopper hopper){
        return Commands.sequence(
            //Drive to the first shooting position and shoot preloaded balls   
            DriveCommands.initialFollowPathCommand(drive, "TrenchToNZ", false)
                .deadlineFor(intake.intakeSequence()),
            
            DriveCommands.followPathCommand("NZ to Shoot1", false),
            shooter.shootCmd(hopper).withTimeout(3.8),

            DriveCommands.followPathCommand("Shoot1 to Hub")
                .deadlineFor(intake.intakeSequence()),
            
            DriveCommands.followPathCommand("HubToShoot2", false),
            shooter.shootCmd(hopper).withTimeout(4)
            
        ); 
    }
}

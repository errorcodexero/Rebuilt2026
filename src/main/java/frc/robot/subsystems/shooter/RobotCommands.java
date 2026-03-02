package frc.robot.subsystems.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;

public class RobotCommands {
    public static Command shoot(Shooter shooter, Hopper hopper, Drive drive, CommandXboxController gamepad, boolean shootOnMove) {
        return Commands.parallel(
                DriveCommands.pointAtShootingTarget(drive, gamepad, shootOnMove),
                Commands.sequence(
                    Commands.waitTime(ShooterConstants.dbRotationDelay), 
                    shooter.shoot(() -> drive.getVirtualTarget().minus(drive.getPose().getTranslation()), hopper)
                )
            );
    }
}

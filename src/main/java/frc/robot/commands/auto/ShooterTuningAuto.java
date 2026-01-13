package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.shooter.ShooterTuningCommand;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Autonomous mode for tuning the shooter.
 * Runs the shooter tuning command to allow tuning via SmartDashboard.
 */
public class ShooterTuningAuto {
  
  /**
   * Creates an autonomous command that runs the shooter tuning command.
   * This allows tuning the shooter during autonomous mode for testing.
   * 
   * @param shooter The shooter subsystem
   * @return A command that runs shooter tuning
   */
  public static Command getCommand(Shooter shooter) {
    return new ShooterTuningCommand(shooter);
  }
}

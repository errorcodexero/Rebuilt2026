package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.shooter.FlywheelVoltageCommand;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Auto mode that runs the flywheel voltage command.
 * Allows manual voltage control of the shooter during autonomous for testing/characterization.
 */
public class FlywheelVoltageAuto {
  
  /**
   * Get the command for this auto mode.
   * @param shooter The shooter subsystem
   * @return The auto command
   */
  public static Command getCommand(Shooter shooter) {
    return new FlywheelVoltageCommand(shooter);
  }
}

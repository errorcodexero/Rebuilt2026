package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Factory class for creating SysId characterization commands for the shooter subsystem.
 * These commands can be used to characterize the flywheel motors
 * for feedforward and feedback control tuning.
 */
public class ShooterSysIdCommands {
  
  /**
   * Create a SysId routine for the flywheel motors.
   * @param shooter The shooter subsystem
   * @return A SysIdRoutine for the flywheel motors
   */
  public static SysIdRoutine createFlywheelSysIdRoutine(Shooter shooter) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Voltage voltage) -> shooter.setFlywheelVoltage(voltage),
            null, // No log consumer (AdvantageKit handles logging)
            shooter
        )
    );
  }
  
  /**
   * Get the quasistatic forward test for the flywheel motors.
   * @param shooter The shooter subsystem
   * @return Command to run quasistatic forward test
   */
  public static Command flywheelQuasistaticForward(Shooter shooter) {
    return createFlywheelSysIdRoutine(shooter).quasistatic(SysIdRoutine.Direction.kForward);
  }
  
  /**
   * Get the quasistatic reverse test for the flywheel motors.
   * @param shooter The shooter subsystem
   * @return Command to run quasistatic reverse test
   */
  public static Command flywheelQuasistaticReverse(Shooter shooter) {
    return createFlywheelSysIdRoutine(shooter).quasistatic(SysIdRoutine.Direction.kReverse);
  }
  
  /**
   * Get the dynamic forward test for the flywheel motors.
   * @param shooter The shooter subsystem
   * @return Command to run dynamic forward test
   */
  public static Command flywheelDynamicForward(Shooter shooter) {
    return createFlywheelSysIdRoutine(shooter).dynamic(SysIdRoutine.Direction.kForward);
  }
  
  /**
   * Get the dynamic reverse test for the flywheel motors.
   * @param shooter The shooter subsystem
   * @return Command to run dynamic reverse test
   */
  public static Command flywheelDynamicReverse(Shooter shooter) {
    return createFlywheelSysIdRoutine(shooter).dynamic(SysIdRoutine.Direction.kReverse);
  }
}

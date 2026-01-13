package frc.robot.commands.hopper;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.hopper.Hopper;

/**
 * Factory class for creating SysId characterization commands for the hopper subsystem.
 * These commands can be used to characterize the agitator motors and feeder motor
 * for feedforward and feedback control tuning.
 */
public class HopperSysIdCommands {
  
  /**
   * Create a SysId routine for the agitator motors.
   * @param hopper The hopper subsystem
   * @return A SysIdRoutine for the agitator motors
   */
  public static SysIdRoutine createAgitatorSysIdRoutine(Hopper hopper) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Voltage voltage) -> hopper.setAgitatorVoltage(voltage),
            null, // No log consumer (AdvantageKit handles logging)
            hopper
        )
    );
  }
  
  /**
   * Create a SysId routine for the feeder motor.
   * @param hopper The hopper subsystem
   * @return A SysIdRoutine for the feeder motor
   */
  public static SysIdRoutine createFeederSysIdRoutine(Hopper hopper) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Voltage voltage) -> hopper.setFeederVoltage(voltage),
            null, // No log consumer (AdvantageKit handles logging)
            hopper
        )
    );
  }
  
  /**
   * Get the quasistatic forward test for the agitator motors.
   * @param hopper The hopper subsystem
   * @return Command to run quasistatic forward test
   */
  public static Command agitatorQuasistaticForward(Hopper hopper) {
    return createAgitatorSysIdRoutine(hopper).quasistatic(SysIdRoutine.Direction.kForward);
  }
  
  /**
   * Get the quasistatic reverse test for the agitator motors.
   * @param hopper The hopper subsystem
   * @return Command to run quasistatic reverse test
   */
  public static Command agitatorQuasistaticReverse(Hopper hopper) {
    return createAgitatorSysIdRoutine(hopper).quasistatic(SysIdRoutine.Direction.kReverse);
  }
  
  /**
   * Get the dynamic forward test for the agitator motors.
   * @param hopper The hopper subsystem
   * @return Command to run dynamic forward test
   */
  public static Command agitatorDynamicForward(Hopper hopper) {
    return createAgitatorSysIdRoutine(hopper).dynamic(SysIdRoutine.Direction.kForward);
  }
  
  /**
   * Get the dynamic reverse test for the agitator motors.
   * @param hopper The hopper subsystem
   * @return Command to run dynamic reverse test
   */
  public static Command agitatorDynamicReverse(Hopper hopper) {
    return createAgitatorSysIdRoutine(hopper).dynamic(SysIdRoutine.Direction.kReverse);
  }
  
  /**
   * Get the quasistatic forward test for the feeder motor.
   * @param hopper The hopper subsystem
   * @return Command to run quasistatic forward test
   */
  public static Command feederQuasistaticForward(Hopper hopper) {
    return createFeederSysIdRoutine(hopper).quasistatic(SysIdRoutine.Direction.kForward);
  }
  
  /**
   * Get the quasistatic reverse test for the feeder motor.
   * @param hopper The hopper subsystem
   * @return Command to run quasistatic reverse test
   */
  public static Command feederQuasistaticReverse(Hopper hopper) {
    return createFeederSysIdRoutine(hopper).quasistatic(SysIdRoutine.Direction.kReverse);
  }
  
  /**
   * Get the dynamic forward test for the feeder motor.
   * @param hopper The hopper subsystem
   * @return Command to run dynamic forward test
   */
  public static Command feederDynamicForward(Hopper hopper) {
    return createFeederSysIdRoutine(hopper).dynamic(SysIdRoutine.Direction.kForward);
  }
  
  /**
   * Get the dynamic reverse test for the feeder motor.
   * @param hopper The hopper subsystem
   * @return Command to run dynamic reverse test
   */
  public static Command feederDynamicReverse(Hopper hopper) {
    return createFeederSysIdRoutine(hopper).dynamic(SysIdRoutine.Direction.kReverse);
  }
}

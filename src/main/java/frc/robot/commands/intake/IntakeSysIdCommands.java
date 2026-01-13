package frc.robot.commands.intake;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.intake.Intake;

/**
 * Factory class for creating SysId characterization commands for the intake subsystem.
 * These commands can be used to characterize the deploy motor and spinner motor
 * for feedforward and feedback control tuning.
 */
public class IntakeSysIdCommands {
  
  /**
   * Create a SysId routine for the deploy motor.
   * @param intake The intake subsystem
   * @return A SysIdRoutine for the deploy motor
   */
  public static SysIdRoutine createDeploySysIdRoutine(Intake intake) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Voltage voltage) -> intake.setDeployVoltage(voltage),
            null, // No log consumer (AdvantageKit handles logging)
            intake
        )
    );
  }
  
  /**
   * Create a SysId routine for the spinner motor.
   * @param intake The intake subsystem
   * @return A SysIdRoutine for the spinner motor
   */
  public static SysIdRoutine createSpinnerSysIdRoutine(Intake intake) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Voltage voltage) -> intake.setSpinnerVoltage(voltage),
            null, // No log consumer (AdvantageKit handles logging)
            intake
        )
    );
  }
  
  /**
   * Get the quasistatic forward test for the deploy motor.
   * @param intake The intake subsystem
   * @return Command to run quasistatic forward test
   */
  public static Command deployQuasistaticForward(Intake intake) {
    return createDeploySysIdRoutine(intake).quasistatic(SysIdRoutine.Direction.kForward);
  }
  
  /**
   * Get the quasistatic reverse test for the deploy motor.
   * @param intake The intake subsystem
   * @return Command to run quasistatic reverse test
   */
  public static Command deployQuasistaticReverse(Intake intake) {
    return createDeploySysIdRoutine(intake).quasistatic(SysIdRoutine.Direction.kReverse);
  }
  
  /**
   * Get the dynamic forward test for the deploy motor.
   * @param intake The intake subsystem
   * @return Command to run dynamic forward test
   */
  public static Command deployDynamicForward(Intake intake) {
    return createDeploySysIdRoutine(intake).dynamic(SysIdRoutine.Direction.kForward);
  }
  
  /**
   * Get the dynamic reverse test for the deploy motor.
   * @param intake The intake subsystem
   * @return Command to run dynamic reverse test
   */
  public static Command deployDynamicReverse(Intake intake) {
    return createDeploySysIdRoutine(intake).dynamic(SysIdRoutine.Direction.kReverse);
  }
  
  /**
   * Get the quasistatic forward test for the spinner motor.
   * @param intake The intake subsystem
   * @return Command to run quasistatic forward test
   */
  public static Command spinnerQuasistaticForward(Intake intake) {
    return createSpinnerSysIdRoutine(intake).quasistatic(SysIdRoutine.Direction.kForward);
  }
  
  /**
   * Get the quasistatic reverse test for the spinner motor.
   * @param intake The intake subsystem
   * @return Command to run quasistatic reverse test
   */
  public static Command spinnerQuasistaticReverse(Intake intake) {
    return createSpinnerSysIdRoutine(intake).quasistatic(SysIdRoutine.Direction.kReverse);
  }
  
  /**
   * Get the dynamic forward test for the spinner motor.
   * @param intake The intake subsystem
   * @return Command to run dynamic forward test
   */
  public static Command spinnerDynamicForward(Intake intake) {
    return createSpinnerSysIdRoutine(intake).dynamic(SysIdRoutine.Direction.kForward);
  }
  
  /**
   * Get the dynamic reverse test for the spinner motor.
   * @param intake The intake subsystem
   * @return Command to run dynamic reverse test
   */
  public static Command spinnerDynamicReverse(Intake intake) {
    return createSpinnerSysIdRoutine(intake).dynamic(SysIdRoutine.Direction.kReverse);
  }
}

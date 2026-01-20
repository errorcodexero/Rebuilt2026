package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

import static edu.wpi.first.units.Units.*;

/**
 * Command that applies a specified voltage to the intake spinner motor.
 * This command runs continuously until interrupted.
 */
public class IntakeVoltageCommand extends Command {
  private final Intake intake;
  private final double voltage;

  /**
   * Creates a new IntakeVoltageCommand.
   * 
   * @param intake The intake subsystem to use
   * @param voltage The voltage to apply to the spinner motor (in volts)
   */
  public IntakeVoltageCommand(Intake intake, double voltage) {
    this.intake = intake;
    this.voltage = voltage;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setSpinnerVoltage(Volts.of(voltage));
  }

  @Override
  public void execute() {
    // Continuously apply voltage
    intake.setSpinnerVoltage(Volts.of(voltage));
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the motor when command ends
    intake.stopSpinner();
  }

  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }
}

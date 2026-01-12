package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private Angle deployGoal = IntakeConstants.DEPLOY_INITIAL_ANGLE;
  private AngularVelocity spinnerGoal = RotationsPerSecond.of(0.0);

  /** Creates a new Intake subsystem. */
  public Intake(IntakeIO io) {
    this.io = io;
    
    // Reset deploy position to initial angle on construction
    io.resetDeployPosition(IntakeConstants.DEPLOY_INITIAL_ANGLE);
    
    // Set brake modes
    io.setDeployBrake(true);
    io.setSpinnerBrake(false);
  }

  @Override
  public void periodic() {
    // Update and log inputs
    io.updateInputs(inputs);
    Logger.recordOutput("Intake/DeployGoalDegrees", deployGoal.in(Degrees));
    Logger.recordOutput("Intake/SpinnerGoalRPS", spinnerGoal.in(RotationsPerSecond));
    Logger.recordOutput("Intake/DeployAtGoal", isDeployAtGoal());
    Logger.recordOutput("Intake/SpinnerAtGoal", isSpinnerAtGoal());
    Logger.processInputs("Intake", inputs);
  }

  /**
   * Set the deploy position of the intake.
   * @param angle The target angle for the intake
   */
  public void setDeployPosition(Angle angle) {
    deployGoal = angle;
    io.setDeployPosition(angle);
  }

  /**
   * Set the spinner velocity.
   * @param velocity The target velocity for the spinner (positive = intake)
   */
  public void setSpinnerVelocity(AngularVelocity velocity) {
    spinnerGoal = velocity;
    io.setSpinnerVelocity(velocity);
  }

  /**
   * Deploy the intake to the deployed position.
   */
  public void deploy() {
    setDeployPosition(IntakeConstants.DEPLOY_DEPLOYED_ANGLE);
  }

  /**
   * Retract the intake to the stowed position.
   */
  public void retract() {
    setDeployPosition(IntakeConstants.DEPLOY_RETRACTED_ANGLE);
  }

  /**
   * Stop the deploy motor.
   */
  public void stopDeploy() {
    io.stopDeploy();
  }

  /**
   * Stop the spinner motor.
   */
  public void stopSpinner() {
    spinnerGoal = RotationsPerSecond.of(0.0);
    io.stopSpinner();
  }

  /**
   * Run the intake at the default intake velocity.
   */
  public void intake() {
    setSpinnerVelocity(IntakeConstants.SPINNER_INTAKE_VELOCITY);
  }

  /**
   * Run the intake at the default eject velocity.
   */
  public void eject() {
    setSpinnerVelocity(IntakeConstants.SPINNER_EJECT_VELOCITY);
  }

  /**
   * Get the current deploy position.
   * @return Current deploy position
   */
  public Angle getDeployPosition() {
    return inputs.deployPosition;
  }

  /**
   * Get the current deploy velocity.
   * @return Current deploy velocity
   */
  public AngularVelocity getDeployVelocity() {
    return inputs.deployVelocity;
  }

  /**
   * Get the current spinner velocity.
   * @return Current spinner velocity
   */
  public AngularVelocity getSpinnerVelocity() {
    return inputs.spinnerVelocity;
  }

  /**
   * Check if the deploy motor is at the goal position.
   * @return True if at goal within tolerance
   */
  public boolean isDeployAtGoal() {
    return Math.abs(
        inputs.deployPosition.in(Degrees) - deployGoal.in(Degrees)
    ) < IntakeConstants.DEPLOY_TOLERANCE.in(Degrees);
  }

  /**
   * Check if the spinner is at the goal velocity.
   * @return True if at goal within tolerance
   */
  public boolean isSpinnerAtGoal() {
    return Math.abs(
        inputs.spinnerVelocity.in(RotationsPerSecond) - spinnerGoal.in(RotationsPerSecond)
    ) < IntakeConstants.SPINNER_TOLERANCE.in(RotationsPerSecond);
  }

  /**
   * Check if the intake is deployed.
   * @return True if deployed
   */
  public boolean isDeployed() {
    return inputs.deployPosition.in(Degrees) > 
        (IntakeConstants.DEPLOY_DEPLOYED_ANGLE.in(Degrees) - 
         IntakeConstants.DEPLOY_TOLERANCE.in(Degrees));
  }

  /**
   * Check if the intake is retracted.
   * @return True if retracted
   */
  public boolean isRetracted() {
    return inputs.deployPosition.in(Degrees) < 
        (IntakeConstants.DEPLOY_RETRACTED_ANGLE.in(Degrees) + 
         IntakeConstants.DEPLOY_TOLERANCE.in(Degrees));
  }

  /**
   * Get the deploy motor current.
   * @return Deploy motor current in amps
   */
  public double getDeployCurrent() {
    return inputs.deployCurrent.in(Amps);
  }

  /**
   * Get the spinner motor current.
   * @return Spinner motor current in amps
   */
  public double getSpinnerCurrent() {
    return inputs.spinnerCurrent.in(Amps);
  }

  /**
   * Reset the deploy position to a known angle.
   * Call this when the robot is at a known position (e.g., after homing).
   * @param currentAngle The current known angle
   */
  public void resetDeployPosition(Angle currentAngle) {
    io.resetDeployPosition(currentAngle);
  }
}

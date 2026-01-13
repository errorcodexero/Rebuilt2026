package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim deployMotorSim;
  private final DCMotorSim spinnerMotorSim;
  
  private final PIDController deployPID;
  private final PIDController spinnerPID;
  
  private Angle deployPositionSetpoint = IntakeConstants.DEPLOY_INITIAL_ANGLE;
  private AngularVelocity spinnerVelocitySetpoint = RotationsPerSecond.of(0.0);
  
  private double deployAppliedVolts = 0.0;
  private double spinnerAppliedVolts = 0.0;
  
  private boolean deployVoltageControl = false;
  private boolean spinnerVoltageControl = false;

  public IntakeIOSim() {
    // Create simulated motors using Kraken X60 (TalonFX) motor model
    DCMotor deployGearbox = DCMotor.getKrakenX60(1);
    deployMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            deployGearbox,
            IntakeConstants.DEPLOY_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
            IntakeConstants.DEPLOY_GEAR_RATIO
        ),
        deployGearbox
    );
    
    DCMotor spinnerGearbox = DCMotor.getKrakenX60(1);
    spinnerMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            spinnerGearbox,
            IntakeConstants.SPINNER_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
            IntakeConstants.SPINNER_GEAR_RATIO
        ),
        spinnerGearbox
    );
    
    // Create PID controllers for simulation
    deployPID = new PIDController(
        IntakeConstants.DEPLOY_KP,
        IntakeConstants.DEPLOY_KI,
        IntakeConstants.DEPLOY_KD
    );
    
    spinnerPID = new PIDController(
        IntakeConstants.SPINNER_KP,
        IntakeConstants.SPINNER_KI,
        IntakeConstants.SPINNER_KD
    );
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Update deploy motor simulation
    deployMotorSim.update(0.020); // 20ms robot loop
    
    // Calculate deploy motor voltage with PID + gravity feedforward (only if not in voltage control mode)
    if (!deployVoltageControl) {
      double deployPIDOutput = deployPID.calculate(
          deployMotorSim.getAngularPositionRotations(),
          deployPositionSetpoint.in(Rotations)
      );
      
      // Simple gravity compensation (assumes horizontal = 0 degrees)
      double gravityFF = IntakeConstants.DEPLOY_KG * 
          Math.cos(deployMotorSim.getAngularPositionRad());
      
      deployAppliedVolts = MathUtil.clamp(
          deployPIDOutput + gravityFF,
          -12.0,
          12.0
      );
    }
    // Otherwise use the voltage set directly via setDeployVoltage()
    
    deployMotorSim.setInputVoltage(deployAppliedVolts);
    
    // Update spinner motor simulation
    spinnerMotorSim.update(0.020);
    
    // Calculate spinner motor voltage with PID (only if not in voltage control mode)
    if (!spinnerVoltageControl) {
      double spinnerPIDOutput = spinnerPID.calculate(
          spinnerMotorSim.getAngularVelocityRPM() / 60.0, // Convert to RPS
          spinnerVelocitySetpoint.in(RotationsPerSecond)
      );
      
      double spinnerFF = spinnerVelocitySetpoint.in(RotationsPerSecond) * IntakeConstants.SPINNER_KV;
      
      spinnerAppliedVolts = MathUtil.clamp(
          spinnerPIDOutput + spinnerFF,
          -12.0,
          12.0
      );
    }
    // Otherwise use the voltage set directly via setSpinnerVoltage()
    
    spinnerMotorSim.setInputVoltage(spinnerAppliedVolts);
    
    // Update inputs
    inputs.deployPosition = Rotations.of(deployMotorSim.getAngularPositionRotations());
    inputs.deployVelocity = RotationsPerSecond.of(deployMotorSim.getAngularVelocityRPM() / 60.0);
    inputs.deployAppliedVoltage = Volts.of(deployAppliedVolts);
    inputs.deployCurrent = Amps.of(Math.abs(deployMotorSim.getCurrentDrawAmps()));
    inputs.deployTemperature = 25.0; // Simulated constant temperature
    
    inputs.spinnerVelocity = RotationsPerSecond.of(spinnerMotorSim.getAngularVelocityRPM() / 60.0);
    inputs.spinnerAppliedVoltage = Volts.of(spinnerAppliedVolts);
    inputs.spinnerCurrent = Amps.of(Math.abs(spinnerMotorSim.getCurrentDrawAmps()));
    inputs.spinnerTemperature = 25.0; // Simulated constant temperature
  }

  @Override
  public void setDeployPosition(Angle position) {
    // Clamp position to valid range
    deployPositionSetpoint = Rotations.of(MathUtil.clamp(
        position.in(Rotations),
        IntakeConstants.DEPLOY_MIN_ANGLE.in(Rotations),
        IntakeConstants.DEPLOY_MAX_ANGLE.in(Rotations)
    ));
    deployVoltageControl = false; // Switch back to position control
  }

  @Override
  public void setDeployVoltage(edu.wpi.first.units.measure.Voltage voltage) {
    // In voltage control mode, bypass PID and directly apply voltage
    deployAppliedVolts = MathUtil.clamp(voltage.in(Volts), -12.0, 12.0);
    deployVoltageControl = true;
  }

  @Override
  public void setSpinnerVelocity(AngularVelocity velocity) {
    spinnerVelocitySetpoint = velocity;
    spinnerVoltageControl = false; // Switch back to velocity control
  }

  @Override
  public void setSpinnerVoltage(edu.wpi.first.units.measure.Voltage voltage) {
    // In voltage control mode, bypass PID and directly apply voltage
    spinnerAppliedVolts = MathUtil.clamp(voltage.in(Volts), -12.0, 12.0);
    spinnerVoltageControl = true;
  }

  @Override
  public void stopDeploy() {
    deployPositionSetpoint = Rotations.of(deployMotorSim.getAngularPositionRotations());
    deployAppliedVolts = 0.0;
    deployVoltageControl = false;
  }

  @Override
  public void stopSpinner() {
    spinnerVelocitySetpoint = RotationsPerSecond.of(0.0);
    spinnerAppliedVolts = 0.0;
    spinnerVoltageControl = false;
  }

  @Override
  public void setDeployBrake(boolean brake) {
    // Brake mode not applicable in simulation
  }

  @Override
  public void setSpinnerBrake(boolean brake) {
    // Brake mode not applicable in simulation
  }

  @Override
  public void resetDeployPosition(Angle currentAngle) {
    deployMotorSim.setState(currentAngle.in(Rotations), 0.0);
    deployPositionSetpoint = currentAngle;
  }
}

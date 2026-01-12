package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.*;

public class IntakeIOKraken implements IntakeIO {
  private final TalonFX deployMotor;
  private final TalonFX spinnerMotor;

  // Control requests
  private final MotionMagicVoltage deployPositionRequest = new MotionMagicVoltage(0.0);
  private final VelocityVoltage spinnerVelocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  // Status signals - Deploy Motor
  private final StatusSignal<Angle> deployPosition;
  private final StatusSignal<AngularVelocity> deployVelocity;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> deployAppliedVolts;
  private final StatusSignal<edu.wpi.first.units.measure.Current> deployCurrent;
  private final StatusSignal<edu.wpi.first.units.measure.Temperature> deployTemperature;

  // Status signals - Spinner Motor
  private final StatusSignal<AngularVelocity> spinnerVelocity;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> spinnerAppliedVolts;
  private final StatusSignal<edu.wpi.first.units.measure.Current> spinnerCurrent;
  private final StatusSignal<edu.wpi.first.units.measure.Temperature> spinnerTemperature;

  public IntakeIOKraken() {
    // Initialize motors
    deployMotor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_CAN_ID);
    spinnerMotor = new TalonFX(IntakeConstants.SPINNER_MOTOR_CAN_ID);

    // Configure deploy motor
    var deployConfig = new TalonFXConfiguration();
    
    // Motor output configuration
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    deployConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    // Current limits
    deployConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.DEPLOY_CURRENT_LIMIT.in(Amps);
    deployConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    // Feedback configuration
    deployConfig.Feedback.SensorToMechanismRatio = IntakeConstants.DEPLOY_GEAR_RATIO;
    
    // PID configuration
    deployConfig.Slot0.kP = IntakeConstants.DEPLOY_KP;
    deployConfig.Slot0.kI = IntakeConstants.DEPLOY_KI;
    deployConfig.Slot0.kD = IntakeConstants.DEPLOY_KD;
    deployConfig.Slot0.kS = IntakeConstants.DEPLOY_KS;
    deployConfig.Slot0.kV = IntakeConstants.DEPLOY_KV;
    deployConfig.Slot0.kA = IntakeConstants.DEPLOY_KA;
    deployConfig.Slot0.kG = IntakeConstants.DEPLOY_KG;
    deployConfig.Slot0.GravityType = com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine;
    
    // Motion Magic configuration
    deployConfig.MotionMagic.MotionMagicCruiseVelocity = 
        IntakeConstants.DEPLOY_CRUISE_VELOCITY.in(RotationsPerSecond);
    deployConfig.MotionMagic.MotionMagicAcceleration = 
        IntakeConstants.DEPLOY_MAX_ACCELERATION.in(RotationsPerSecond.per(Second));
    deployConfig.MotionMagic.MotionMagicJerk = 
        IntakeConstants.DEPLOY_MAX_JERK;
    
    // Soft limits
    deployConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    deployConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 
        IntakeConstants.DEPLOY_MAX_ANGLE.in(Rotations);
    deployConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    deployConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 
        IntakeConstants.DEPLOY_MIN_ANGLE.in(Rotations);
    
    tryUntilOk(5, () -> deployMotor.getConfigurator().apply(deployConfig, 0.25));
    
    // Configure spinner motor
    var spinnerConfig = new TalonFXConfiguration();
    
    // Motor output configuration
    spinnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    spinnerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    // Current limits
    spinnerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.SPINNER_CURRENT_LIMIT.in(Amps);
    spinnerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    // Feedback configuration
    spinnerConfig.Feedback.SensorToMechanismRatio = IntakeConstants.SPINNER_GEAR_RATIO;
    
    // PID configuration for velocity control
    spinnerConfig.Slot0.kP = IntakeConstants.SPINNER_KP;
    spinnerConfig.Slot0.kI = IntakeConstants.SPINNER_KI;
    spinnerConfig.Slot0.kD = IntakeConstants.SPINNER_KD;
    spinnerConfig.Slot0.kS = IntakeConstants.SPINNER_KS;
    spinnerConfig.Slot0.kV = IntakeConstants.SPINNER_KV;
    spinnerConfig.Slot0.kA = IntakeConstants.SPINNER_KA;
    
    tryUntilOk(5, () -> spinnerMotor.getConfigurator().apply(spinnerConfig, 0.25));
    
    // Initialize status signals - Deploy Motor
    deployPosition = deployMotor.getPosition();
    deployVelocity = deployMotor.getVelocity();
    deployAppliedVolts = deployMotor.getMotorVoltage();
    deployCurrent = deployMotor.getStatorCurrent();
    deployTemperature = deployMotor.getDeviceTemp();
    
    // Initialize status signals - Spinner Motor
    spinnerVelocity = spinnerMotor.getVelocity();
    spinnerAppliedVolts = spinnerMotor.getMotorVoltage();
    spinnerCurrent = spinnerMotor.getStatorCurrent();
    spinnerTemperature = spinnerMotor.getDeviceTemp();
    
    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, // 50 Hz
        deployPosition,
        deployVelocity,
        deployAppliedVolts,
        deployCurrent,
        spinnerVelocity,
        spinnerAppliedVolts,
        spinnerCurrent
    );
    
    BaseStatusSignal.setUpdateFrequencyForAll(
        4.0, // 4 Hz for temperature
        deployTemperature,
        spinnerTemperature
    );
    
    // Optimize bus utilization
    deployMotor.optimizeBusUtilization();
    spinnerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Refresh all signals
    BaseStatusSignal.refreshAll(
        deployPosition,
        deployVelocity,
        deployAppliedVolts,
        deployCurrent,
        deployTemperature,
        spinnerVelocity,
        spinnerAppliedVolts,
        spinnerCurrent,
        spinnerTemperature
    );
    
    // Update deploy motor inputs
    inputs.deployPosition = deployPosition.getValue();
    inputs.deployVelocity = deployVelocity.getValue();
    inputs.deployAppliedVoltage = deployAppliedVolts.getValue();
    inputs.deployCurrent = deployCurrent.getValue();
    inputs.deployTemperature = deployTemperature.getValue().in(Celsius);
    
    // Update spinner motor inputs
    inputs.spinnerVelocity = spinnerVelocity.getValue();
    inputs.spinnerAppliedVoltage = spinnerAppliedVolts.getValue();
    inputs.spinnerCurrent = spinnerCurrent.getValue();
    inputs.spinnerTemperature = spinnerTemperature.getValue().in(Celsius);
  }

  @Override
  public void setDeployPosition(Angle position) {
    deployMotor.setControl(
        deployPositionRequest
            .withPosition(position)
            .withSlot(0)
    );
  }

  @Override
  public void setSpinnerVelocity(AngularVelocity velocity) {
    spinnerMotor.setControl(
        spinnerVelocityRequest
            .withVelocity(velocity)
            .withSlot(0)
    );
  }

  @Override
  public void stopDeploy() {
    deployMotor.setControl(voltageRequest.withOutput(0.0));
  }

  @Override
  public void stopSpinner() {
    spinnerMotor.setControl(voltageRequest.withOutput(0.0));
  }

  @Override
  public void setDeployBrake(boolean brake) {
    deployMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setSpinnerBrake(boolean brake) {
    spinnerMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void resetDeployPosition(Angle currentAngle) {
    tryUntilOk(5, () -> deployMotor.setPosition(currentAngle.in(Rotations), 0.25));
  }
}

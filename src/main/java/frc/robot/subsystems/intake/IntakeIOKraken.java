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
import frc.robot.generated.CompTunerConstants;

import static edu.wpi.first.units.Units.*;

public class IntakeIOKraken implements IntakeIO {
  private final TalonFX spinnerMotor;

  // Control requests
  private final MotionMagicVoltage deployPositionRequest = new MotionMagicVoltage(0.0);
  private final VelocityVoltage spinnerVelocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  // Status signals - Spinner Motor
  private final StatusSignal<AngularVelocity> spinnerVelocity;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> spinnerAppliedVolts;
  private final StatusSignal<edu.wpi.first.units.measure.Current> spinnerCurrent;
  private final StatusSignal<edu.wpi.first.units.measure.Temperature> spinnerTemperature;

  public IntakeIOKraken() {
    // Initialize motors
    spinnerMotor = new TalonFX(IntakeConstants.SPINNER_MOTOR_CAN_ID, CompTunerConstants.kCANBus);

    // Current limits
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
    
    // Initialize status signals - Spinner Motor
    spinnerVelocity = spinnerMotor.getVelocity();
    spinnerAppliedVolts = spinnerMotor.getMotorVoltage();
    spinnerCurrent = spinnerMotor.getStatorCurrent();
    spinnerTemperature = spinnerMotor.getDeviceTemp();
    
    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, // 50 Hz
        spinnerVelocity,
        spinnerAppliedVolts,
        spinnerCurrent
    );
    
    BaseStatusSignal.setUpdateFrequencyForAll(
        4.0, // 4 Hz for temperature
        spinnerTemperature
    );
    
    // Optimize bus utilization
    spinnerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Refresh all signals
    BaseStatusSignal.refreshAll(
        spinnerVelocity,
        spinnerAppliedVolts,
        spinnerCurrent,
        spinnerTemperature
    );
    
    // Update spinner motor inputs
    inputs.spinnerVelocity = spinnerVelocity.getValue();
    inputs.spinnerAppliedVoltage = spinnerAppliedVolts.getValue();
    inputs.spinnerCurrent = spinnerCurrent.getValue();
    inputs.spinnerTemperature = spinnerTemperature.getValue().in(Celsius);
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
  public void setSpinnerVoltage(edu.wpi.first.units.measure.Voltage voltage) {
    spinnerMotor.setControl(voltageRequest.withOutput(voltage.in(Volts)));
  }

  @Override
  public void stopSpinner() {
    spinnerMotor.setControl(voltageRequest.withOutput(0.0));
  }

  @Override
  public void setSpinnerBrake(boolean brake) {
    spinnerMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

}

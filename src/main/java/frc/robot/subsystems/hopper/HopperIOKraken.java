package frc.robot.subsystems.hopper;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.*;

public class HopperIOKraken implements HopperIO {
  private final TalonFX agitatorLeader;
  private final TalonFX agitatorFollower;
  private final TalonFX feederMotor;

  // Control requests
  private final VelocityVoltage agitatorVelocityRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage feederVelocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  // Status signals - Agitator Leader
  private final StatusSignal<AngularVelocity> agitatorLeaderVelocity;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> agitatorLeaderAppliedVolts;
  private final StatusSignal<edu.wpi.first.units.measure.Current> agitatorLeaderCurrent;
  private final StatusSignal<edu.wpi.first.units.measure.Temperature> agitatorLeaderTemperature;

  // Status signals - Agitator Follower
  private final StatusSignal<AngularVelocity> agitatorFollowerVelocity;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> agitatorFollowerAppliedVolts;
  private final StatusSignal<edu.wpi.first.units.measure.Current> agitatorFollowerCurrent;
  private final StatusSignal<edu.wpi.first.units.measure.Temperature> agitatorFollowerTemperature;

  // Status signals - Feeder Motor
  private final StatusSignal<AngularVelocity> feederVelocity;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> feederAppliedVolts;
  private final StatusSignal<edu.wpi.first.units.measure.Current> feederCurrent;
  private final StatusSignal<edu.wpi.first.units.measure.Temperature> feederTemperature;

  public HopperIOKraken() {
    // Initialize motors
    agitatorLeader = new TalonFX(HopperConstants.AGITATOR_LEADER_CAN_ID);
    agitatorFollower = new TalonFX(HopperConstants.AGITATOR_FOLLOWER_CAN_ID);
    feederMotor = new TalonFX(HopperConstants.FEEDER_MOTOR_CAN_ID);

    // Configure agitator leader motor
    var agitatorConfig = new TalonFXConfiguration();
    
    // Motor output configuration
    agitatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    agitatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    // Current limits
    agitatorConfig.CurrentLimits.StatorCurrentLimit = HopperConstants.AGITATOR_CURRENT_LIMIT.in(Amps);
    agitatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    // Feedback configuration
    agitatorConfig.Feedback.SensorToMechanismRatio = HopperConstants.AGITATOR_GEAR_RATIO;
    
    // PID configuration for velocity control
    agitatorConfig.Slot0.kP = HopperConstants.AGITATOR_KP;
    agitatorConfig.Slot0.kI = HopperConstants.AGITATOR_KI;
    agitatorConfig.Slot0.kD = HopperConstants.AGITATOR_KD;
    agitatorConfig.Slot0.kS = HopperConstants.AGITATOR_KS;
    agitatorConfig.Slot0.kV = HopperConstants.AGITATOR_KV;
    agitatorConfig.Slot0.kA = HopperConstants.AGITATOR_KA;
    
    tryUntilOk(5, () -> agitatorLeader.getConfigurator().apply(agitatorConfig, 0.25));
    
    // Configure agitator follower motor (copy leader config)
    var followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Match leader direction
    followerConfig.CurrentLimits.StatorCurrentLimit = HopperConstants.AGITATOR_CURRENT_LIMIT.in(Amps);
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    tryUntilOk(5, () -> agitatorFollower.getConfigurator().apply(followerConfig, 0.25));
    
    // Set follower to follow leader 
    agitatorFollower.setControl(new com.ctre.phoenix6.controls.StrictFollower(HopperConstants.AGITATOR_LEADER_CAN_ID));
    
    // Configure feeder motor
    var feederConfig = new TalonFXConfiguration();
    
    // Motor output configuration
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    // Current limits
    feederConfig.CurrentLimits.StatorCurrentLimit = HopperConstants.FEEDER_CURRENT_LIMIT.in(Amps);
    feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    // Feedback configuration
    feederConfig.Feedback.SensorToMechanismRatio = HopperConstants.FEEDER_GEAR_RATIO;
    
    // PID configuration for velocity control
    feederConfig.Slot0.kP = HopperConstants.FEEDER_KP;
    feederConfig.Slot0.kI = HopperConstants.FEEDER_KI;
    feederConfig.Slot0.kD = HopperConstants.FEEDER_KD;
    feederConfig.Slot0.kS = HopperConstants.FEEDER_KS;
    feederConfig.Slot0.kV = HopperConstants.FEEDER_KV;
    feederConfig.Slot0.kA = HopperConstants.FEEDER_KA;
    
    tryUntilOk(5, () -> feederMotor.getConfigurator().apply(feederConfig, 0.25));
    
    // Initialize status signals - Agitator Leader
    agitatorLeaderVelocity = agitatorLeader.getVelocity();
    agitatorLeaderAppliedVolts = agitatorLeader.getMotorVoltage();
    agitatorLeaderCurrent = agitatorLeader.getStatorCurrent();
    agitatorLeaderTemperature = agitatorLeader.getDeviceTemp();
    
    // Initialize status signals - Agitator Follower
    agitatorFollowerVelocity = agitatorFollower.getVelocity();
    agitatorFollowerAppliedVolts = agitatorFollower.getMotorVoltage();
    agitatorFollowerCurrent = agitatorFollower.getStatorCurrent();
    agitatorFollowerTemperature = agitatorFollower.getDeviceTemp();
    
    // Initialize status signals - Feeder Motor
    feederVelocity = feederMotor.getVelocity();
    feederAppliedVolts = feederMotor.getMotorVoltage();
    feederCurrent = feederMotor.getStatorCurrent();
    feederTemperature = feederMotor.getDeviceTemp();
    
    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, // 50 Hz
        agitatorLeaderVelocity,
        agitatorLeaderAppliedVolts,
        agitatorLeaderCurrent,
        agitatorFollowerVelocity,
        agitatorFollowerAppliedVolts,
        agitatorFollowerCurrent,
        feederVelocity,
        feederAppliedVolts,
        feederCurrent
    );
    
    BaseStatusSignal.setUpdateFrequencyForAll(
        4.0, // 4 Hz for temperature
        agitatorLeaderTemperature,
        agitatorFollowerTemperature,
        feederTemperature
    );
    
    // Optimize bus utilization
    agitatorLeader.optimizeBusUtilization();
    agitatorFollower.optimizeBusUtilization();
    feederMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    // Refresh all signals
    BaseStatusSignal.refreshAll(
        agitatorLeaderVelocity,
        agitatorLeaderAppliedVolts,
        agitatorLeaderCurrent,
        agitatorLeaderTemperature,
        agitatorFollowerVelocity,
        agitatorFollowerAppliedVolts,
        agitatorFollowerCurrent,
        agitatorFollowerTemperature,
        feederVelocity,
        feederAppliedVolts,
        feederCurrent,
        feederTemperature
    );
    
    // Update agitator inputs (use leader velocity as primary)
    inputs.agitatorVelocity = agitatorLeaderVelocity.getValue();
    inputs.agitatorLeaderAppliedVoltage = agitatorLeaderAppliedVolts.getValue();
    inputs.agitatorFollowerAppliedVoltage = agitatorFollowerAppliedVolts.getValue();
    inputs.agitatorLeaderCurrent = agitatorLeaderCurrent.getValue();
    inputs.agitatorFollowerCurrent = agitatorFollowerCurrent.getValue();
    inputs.agitatorLeaderTemperature = agitatorLeaderTemperature.getValue().in(Celsius);
    inputs.agitatorFollowerTemperature = agitatorFollowerTemperature.getValue().in(Celsius);
    
    // Update feeder motor inputs
    inputs.feederVelocity = feederVelocity.getValue();
    inputs.feederAppliedVoltage = feederAppliedVolts.getValue();
    inputs.feederCurrent = feederCurrent.getValue();
    inputs.feederTemperature = feederTemperature.getValue().in(Celsius);
  }

  @Override
  public void setAgitatorVelocity(AngularVelocity velocity) {
    agitatorLeader.setControl(
        agitatorVelocityRequest
            .withVelocity(velocity)
            .withSlot(0)
    );
    // Follower automatically follows
  }

  @Override
  public void setAgitatorVoltage(edu.wpi.first.units.measure.Voltage voltage) {
    agitatorLeader.setControl(voltageRequest.withOutput(voltage.in(Volts)));
    // Follower automatically follows
  }

  @Override
  public void setFeederVelocity(AngularVelocity velocity) {
    feederMotor.setControl(
        feederVelocityRequest
            .withVelocity(velocity)
            .withSlot(0)
    );
  }

  @Override
  public void setFeederVoltage(edu.wpi.first.units.measure.Voltage voltage) {
    feederMotor.setControl(voltageRequest.withOutput(voltage.in(Volts)));
  }

  @Override
  public void stopAgitators() {
    agitatorLeader.setControl(voltageRequest.withOutput(0.0));
    // Follower automatically follows
  }

  @Override
  public void stopFeeder() {
    feederMotor.setControl(voltageRequest.withOutput(0.0));
  }

  @Override
  public void setAgitatorBrake(boolean brake) {
    agitatorLeader.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    agitatorFollower.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setFeederBrake(boolean brake) {
    feederMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}

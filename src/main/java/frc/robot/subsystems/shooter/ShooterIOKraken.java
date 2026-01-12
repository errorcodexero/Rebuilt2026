package frc.robot.subsystems.shooter;

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

public class ShooterIOKraken implements ShooterIO {
  private final TalonFX flywheelLeader;
  private final TalonFX flywheelFollower;
  private final TalonFX hoodMotor;

  // Control requests
  private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0);
  private final MotionMagicVoltage hoodPositionRequest = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  // Status signals - Flywheel Leader
  private final StatusSignal<AngularVelocity> flywheelLeaderVelocity;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> flywheelLeaderAppliedVolts;
  private final StatusSignal<edu.wpi.first.units.measure.Current> flywheelLeaderCurrent;
  private final StatusSignal<edu.wpi.first.units.measure.Temperature> flywheelLeaderTemperature;

  // Status signals - Flywheel Follower
  private final StatusSignal<AngularVelocity> flywheelFollowerVelocity;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> flywheelFollowerAppliedVolts;
  private final StatusSignal<edu.wpi.first.units.measure.Current> flywheelFollowerCurrent;
  private final StatusSignal<edu.wpi.first.units.measure.Temperature> flywheelFollowerTemperature;

  // Status signals - Hood Motor
  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<AngularVelocity> hoodVelocity;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> hoodAppliedVolts;
  private final StatusSignal<edu.wpi.first.units.measure.Current> hoodCurrent;
  private final StatusSignal<edu.wpi.first.units.measure.Temperature> hoodTemperature;

  public ShooterIOKraken() {
    // Initialize motors
    flywheelLeader = new TalonFX(ShooterConstants.FLYWHEEL_LEADER_CAN_ID);
    flywheelFollower = new TalonFX(ShooterConstants.FLYWHEEL_FOLLOWER_CAN_ID);
    hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_CAN_ID);

    // Configure flywheel leader motor
    var flywheelConfig = new TalonFXConfiguration();
    
    // Motor output configuration
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    // Current limits
    flywheelConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.FLYWHEEL_CURRENT_LIMIT.in(Amps);
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    // Feedback configuration
    flywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_GEAR_RATIO;
    
    // PID configuration for velocity control
    flywheelConfig.Slot0.kP = ShooterConstants.FLYWHEEL_KP;
    flywheelConfig.Slot0.kI = ShooterConstants.FLYWHEEL_KI;
    flywheelConfig.Slot0.kD = ShooterConstants.FLYWHEEL_KD;
    flywheelConfig.Slot0.kS = ShooterConstants.FLYWHEEL_KS;
    flywheelConfig.Slot0.kV = ShooterConstants.FLYWHEEL_KV;
    flywheelConfig.Slot0.kA = ShooterConstants.FLYWHEEL_KA;
    
    tryUntilOk(5, () -> flywheelLeader.getConfigurator().apply(flywheelConfig, 0.25));
    
    // Configure flywheel follower motor (copy leader config)
    var followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Match leader direction
    followerConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.FLYWHEEL_CURRENT_LIMIT.in(Amps);
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    tryUntilOk(5, () -> flywheelFollower.getConfigurator().apply(followerConfig, 0.25));
    
    // Set follower to follow leader 
    // In Phoenix 6, use StrictFollower for exact following
    flywheelFollower.setControl(new com.ctre.phoenix6.controls.StrictFollower(ShooterConstants.FLYWHEEL_LEADER_CAN_ID));
    
    // Configure hood motor
    var hoodConfig = new TalonFXConfiguration();
    
    // Motor output configuration
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    // Current limits
    hoodConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.HOOD_CURRENT_LIMIT.in(Amps);
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    // Feedback configuration
    hoodConfig.Feedback.SensorToMechanismRatio = ShooterConstants.HOOD_GEAR_RATIO;
    
    // PID configuration
    hoodConfig.Slot0.kP = ShooterConstants.HOOD_KP;
    hoodConfig.Slot0.kI = ShooterConstants.HOOD_KI;
    hoodConfig.Slot0.kD = ShooterConstants.HOOD_KD;
    hoodConfig.Slot0.kS = ShooterConstants.HOOD_KS;
    hoodConfig.Slot0.kV = ShooterConstants.HOOD_KV;
    hoodConfig.Slot0.kA = ShooterConstants.HOOD_KA;
    hoodConfig.Slot0.kG = ShooterConstants.HOOD_KG;
    hoodConfig.Slot0.GravityType = com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine;
    
    // Motion Magic configuration
    hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 
        ShooterConstants.HOOD_CRUISE_VELOCITY.in(RotationsPerSecond);
    hoodConfig.MotionMagic.MotionMagicAcceleration = 
        ShooterConstants.HOOD_MAX_ACCELERATION.in(RotationsPerSecond.per(Second));
    hoodConfig.MotionMagic.MotionMagicJerk = 
        ShooterConstants.HOOD_MAX_JERK;
    
    // Soft limits
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 
        ShooterConstants.HOOD_MAX_ANGLE.in(Rotations);
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 
        ShooterConstants.HOOD_MIN_ANGLE.in(Rotations);
    
    tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));
    
    // Initialize status signals - Flywheel Leader
    flywheelLeaderVelocity = flywheelLeader.getVelocity();
    flywheelLeaderAppliedVolts = flywheelLeader.getMotorVoltage();
    flywheelLeaderCurrent = flywheelLeader.getStatorCurrent();
    flywheelLeaderTemperature = flywheelLeader.getDeviceTemp();
    
    // Initialize status signals - Flywheel Follower
    flywheelFollowerVelocity = flywheelFollower.getVelocity();
    flywheelFollowerAppliedVolts = flywheelFollower.getMotorVoltage();
    flywheelFollowerCurrent = flywheelFollower.getStatorCurrent();
    flywheelFollowerTemperature = flywheelFollower.getDeviceTemp();
    
    // Initialize status signals - Hood Motor
    hoodPosition = hoodMotor.getPosition();
    hoodVelocity = hoodMotor.getVelocity();
    hoodAppliedVolts = hoodMotor.getMotorVoltage();
    hoodCurrent = hoodMotor.getStatorCurrent();
    hoodTemperature = hoodMotor.getDeviceTemp();
    
    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, // 50 Hz
        flywheelLeaderVelocity,
        flywheelLeaderAppliedVolts,
        flywheelLeaderCurrent,
        flywheelFollowerVelocity,
        flywheelFollowerAppliedVolts,
        flywheelFollowerCurrent,
        hoodPosition,
        hoodVelocity,
        hoodAppliedVolts,
        hoodCurrent
    );
    
    BaseStatusSignal.setUpdateFrequencyForAll(
        4.0, // 4 Hz for temperature
        flywheelLeaderTemperature,
        flywheelFollowerTemperature,
        hoodTemperature
    );
    
    // Optimize bus utilization
    flywheelLeader.optimizeBusUtilization();
    flywheelFollower.optimizeBusUtilization();
    hoodMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Refresh all signals
    BaseStatusSignal.refreshAll(
        flywheelLeaderVelocity,
        flywheelLeaderAppliedVolts,
        flywheelLeaderCurrent,
        flywheelLeaderTemperature,
        flywheelFollowerVelocity,
        flywheelFollowerAppliedVolts,
        flywheelFollowerCurrent,
        flywheelFollowerTemperature,
        hoodPosition,
        hoodVelocity,
        hoodAppliedVolts,
        hoodCurrent,
        hoodTemperature
    );
    
    // Update flywheel inputs (use leader velocity as primary)
    inputs.flywheelVelocity = flywheelLeaderVelocity.getValue();
    inputs.flywheelLeaderAppliedVoltage = flywheelLeaderAppliedVolts.getValue();
    inputs.flywheelFollowerAppliedVoltage = flywheelFollowerAppliedVolts.getValue();
    inputs.flywheelLeaderCurrent = flywheelLeaderCurrent.getValue();
    inputs.flywheelFollowerCurrent = flywheelFollowerCurrent.getValue();
    inputs.flywheelLeaderTemperature = flywheelLeaderTemperature.getValue().in(Celsius);
    inputs.flywheelFollowerTemperature = flywheelFollowerTemperature.getValue().in(Celsius);
    
    // Update hood motor inputs
    inputs.hoodPosition = hoodPosition.getValue();
    inputs.hoodVelocity = hoodVelocity.getValue();
    inputs.hoodAppliedVoltage = hoodAppliedVolts.getValue();
    inputs.hoodCurrent = hoodCurrent.getValue();
    inputs.hoodTemperature = hoodTemperature.getValue().in(Celsius);
  }

  @Override
  public void setFlywheelVelocity(AngularVelocity velocity) {
    flywheelLeader.setControl(
        flywheelVelocityRequest
            .withVelocity(velocity)
            .withSlot(0)
    );
    // Follower automatically follows
  }

  @Override
  public void setHoodPosition(Angle position) {
    hoodMotor.setControl(
        hoodPositionRequest
            .withPosition(position)
            .withSlot(0)
    );
  }

  @Override
  public void stopFlywheels() {
    flywheelLeader.setControl(voltageRequest.withOutput(0.0));
    // Follower automatically follows
  }

  @Override
  public void stopHood() {
    hoodMotor.setControl(voltageRequest.withOutput(0.0));
  }

  @Override
  public void setFlywheelBrake(boolean brake) {
    flywheelLeader.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    flywheelFollower.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setHoodBrake(boolean brake) {
    hoodMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void resetHoodPosition(Angle currentAngle) {
    tryUntilOk(5, () -> hoodMotor.setPosition(currentAngle.in(Rotations), 0.25));
  }
}

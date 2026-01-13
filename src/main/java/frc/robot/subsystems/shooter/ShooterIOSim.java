package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

public class ShooterIOSim implements ShooterIO {
  private final DCMotorSim flywheelSim;
  private final DCMotorSim hoodSim;
  
  private final PIDController flywheelPID;
  private final PIDController hoodPID;
  
  private AngularVelocity flywheelVelocitySetpoint = RotationsPerSecond.of(0.0);
  private Angle hoodPositionSetpoint = ShooterConstants.HOOD_INITIAL_ANGLE;
  
  private double flywheelAppliedVolts = 0.0;
  private double hoodAppliedVolts = 0.0;
  
  private boolean hoodVoltageControl = false;

  public ShooterIOSim() {
    // Create simulated motors using Kraken X60 (TalonFX) motor model
    // Flywheel uses 2 motors mechanically coupled
    DCMotor flywheelGearbox = DCMotor.getKrakenX60(2); // 2 motors coupled
    flywheelSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            flywheelGearbox,
            ShooterConstants.FLYWHEEL_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
            ShooterConstants.FLYWHEEL_GEAR_RATIO
        ),
        flywheelGearbox
    );
    
    DCMotor hoodGearbox = DCMotor.getKrakenX60(1);
    hoodSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            hoodGearbox,
            ShooterConstants.HOOD_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
            ShooterConstants.HOOD_GEAR_RATIO
        ),
        hoodGearbox
    );
    
    // Create PID controllers for simulation
    flywheelPID = new PIDController(
        ShooterConstants.FLYWHEEL_KP,
        ShooterConstants.FLYWHEEL_KI,
        ShooterConstants.FLYWHEEL_KD
    );
    
    hoodPID = new PIDController(
        ShooterConstants.HOOD_KP,
        ShooterConstants.HOOD_KI,
        ShooterConstants.HOOD_KD
    );
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Update flywheel simulation
    flywheelSim.update(0.020); // 20ms robot loop
    
    // Calculate flywheel motor voltage with PID + feedforward (only if velocity setpoint is non-zero)
    if (flywheelVelocitySetpoint.in(RotationsPerSecond) != 0.0) {
      double flywheelPIDOutput = flywheelPID.calculate(
          flywheelSim.getAngularVelocityRPM() / 60.0, // Convert to RPS
          flywheelVelocitySetpoint.in(RotationsPerSecond)
      );
      
      double flywheelFF = flywheelVelocitySetpoint.in(RotationsPerSecond) * ShooterConstants.FLYWHEEL_KV;
      
      flywheelAppliedVolts = MathUtil.clamp(
          flywheelPIDOutput + flywheelFF,
          -12.0,
          12.0
      );
    }
    // Otherwise use the voltage set directly via setFlywheelVoltage()
    
    flywheelSim.setInputVoltage(flywheelAppliedVolts);
    
    // Update hood simulation
    hoodSim.update(0.020);
    
    // Calculate hood motor voltage with PID + gravity feedforward (only if not in voltage control mode)
    if (!hoodVoltageControl) {
      double hoodPIDOutput = hoodPID.calculate(
          hoodSim.getAngularPositionRotations(),
          hoodPositionSetpoint.in(Rotations)
      );
      
      // Simple gravity compensation (assumes horizontal = 0 degrees)
      double gravityFF = ShooterConstants.HOOD_KG * 
          Math.cos(hoodSim.getAngularPositionRad());
      
      hoodAppliedVolts = MathUtil.clamp(
          hoodPIDOutput + gravityFF,
          -12.0,
          12.0
      );
    }
    // Otherwise use the voltage set directly via setHoodVoltage()
    
    hoodSim.setInputVoltage(hoodAppliedVolts);
    
    // Update inputs - Flywheels
    inputs.flywheelVelocity = RotationsPerSecond.of(flywheelSim.getAngularVelocityRPM() / 60.0);
    inputs.flywheelLeaderAppliedVoltage = Volts.of(flywheelAppliedVolts);
    inputs.flywheelFollowerAppliedVoltage = Volts.of(flywheelAppliedVolts); // Same as leader in sim
    inputs.flywheelLeaderCurrent = Amps.of(Math.abs(flywheelSim.getCurrentDrawAmps()) / 2.0); // Split between two motors
    inputs.flywheelFollowerCurrent = Amps.of(Math.abs(flywheelSim.getCurrentDrawAmps()) / 2.0);
    inputs.flywheelLeaderTemperature = 25.0; // Simulated constant temperature
    inputs.flywheelFollowerTemperature = 25.0;
    
    // Update inputs - Hood
    inputs.hoodPosition = Rotations.of(hoodSim.getAngularPositionRotations());
    inputs.hoodVelocity = RotationsPerSecond.of(hoodSim.getAngularVelocityRPM() / 60.0);
    inputs.hoodAppliedVoltage = Volts.of(hoodAppliedVolts);
    inputs.hoodCurrent = Amps.of(Math.abs(hoodSim.getCurrentDrawAmps()));
    inputs.hoodTemperature = 25.0;
  }

  @Override
  public void setFlywheelVelocity(AngularVelocity velocity) {
    flywheelVelocitySetpoint = velocity;
  }

  @Override
  public void setFlywheelVoltage(edu.wpi.first.units.measure.Voltage voltage) {
    // In voltage control mode, bypass PID and directly apply voltage
    flywheelAppliedVolts = MathUtil.clamp(voltage.in(Volts), -12.0, 12.0);
    flywheelVelocitySetpoint = RotationsPerSecond.of(0.0); // Clear velocity setpoint
  }

  @Override
  public void setHoodPosition(Angle position) {
    // Clamp position to valid range
    hoodPositionSetpoint = Rotations.of(MathUtil.clamp(
        position.in(Rotations),
        ShooterConstants.HOOD_MIN_ANGLE.in(Rotations),
        ShooterConstants.HOOD_MAX_ANGLE.in(Rotations)
    ));
    hoodVoltageControl = false; // Switch back to position control
  }

  @Override
  public void setHoodVoltage(edu.wpi.first.units.measure.Voltage voltage) {
    // In voltage control mode, bypass PID and directly apply voltage
    hoodAppliedVolts = MathUtil.clamp(voltage.in(Volts), -12.0, 12.0);
    hoodVoltageControl = true;
  }

  @Override
  public void stopFlywheels() {
    flywheelVelocitySetpoint = RotationsPerSecond.of(0.0);
    flywheelAppliedVolts = 0.0;
  }

  @Override
  public void stopHood() {
    hoodPositionSetpoint = Rotations.of(hoodSim.getAngularPositionRotations());
    hoodAppliedVolts = 0.0;
    hoodVoltageControl = false;
  }

  @Override
  public void setFlywheelBrake(boolean brake) {
    // Brake mode not applicable in simulation
  }

  @Override
  public void setHoodBrake(boolean brake) {
    // Brake mode not applicable in simulation
  }

  @Override
  public void resetHoodPosition(Angle currentAngle) {
    hoodSim.setState(currentAngle.in(Rotations), 0.0);
    hoodPositionSetpoint = currentAngle;
  }
}

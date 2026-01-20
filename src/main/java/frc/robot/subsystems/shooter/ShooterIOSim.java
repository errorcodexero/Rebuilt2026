package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

public class ShooterIOSim implements ShooterIO {
  private final DCMotorSim flywheelSim;
  
  private final PIDController flywheelPID;
  
  private AngularVelocity flywheelVelocitySetpoint = RotationsPerSecond.of(0.0);
  
  private double flywheelAppliedVolts = 0.0;

  public ShooterIOSim() {
    // Create simulated motors using Kraken X60 (TalonFX) motor model
    // Flywheel uses 3 motors mechanically coupled
    DCMotor flywheelGearbox = DCMotor.getKrakenX60(ShooterConstants.FLYWHEEL_MOTOR_CAN_IDS.length);
    flywheelSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            flywheelGearbox,
            ShooterConstants.FLYWHEEL_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
            ShooterConstants.FLYWHEEL_GEAR_RATIO
        ),
        flywheelGearbox
    );
    
    // Create PID controllers for simulation
    flywheelPID = new PIDController(
        ShooterConstants.FLYWHEEL_KP,
        ShooterConstants.FLYWHEEL_KI,
        ShooterConstants.FLYWHEEL_KD
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
    
    // Update inputs - Flywheels
    inputs.flywheelVelocity = RotationsPerSecond.of(flywheelSim.getAngularVelocityRPM() / 60.0);
    inputs.flywheelAppliedVoltage = Volts.of(flywheelAppliedVolts); // Leader voltage only
    
    // Split total current among motors for simulation
    double totalCurrent = Math.abs(flywheelSim.getCurrentDrawAmps());
    inputs.flywheelTotalCurrent = Amps.of(totalCurrent);
    double currentPerMotor = totalCurrent / ShooterConstants.FLYWHEEL_MOTOR_CAN_IDS.length;
    
    inputs.flywheelMaxTemperature = 25.0; // Simulated constant temperature
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
  public void stopFlywheels() {
    flywheelVelocitySetpoint = RotationsPerSecond.of(0.0);
    flywheelAppliedVolts = 0.0;
  }

  @Override
  public void setFlywheelBrake(boolean brake) {
    // Brake mode not applicable in simulation
  }
}

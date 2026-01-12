package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

public class HopperIOSim implements HopperIO {
  private final DCMotorSim agitatorSim;
  private final DCMotorSim feederSim;
  
  private final PIDController agitatorPID;
  private final PIDController feederPID;
  
  private AngularVelocity agitatorVelocitySetpoint = RotationsPerSecond.of(0.0);
  private AngularVelocity feederVelocitySetpoint = RotationsPerSecond.of(0.0);
  
  private double agitatorAppliedVolts = 0.0;
  private double feederAppliedVolts = 0.0;

  public HopperIOSim() {
    // Create simulated motors using Kraken X60 (TalonFX) motor model
    // Agitator uses 2 motors mechanically coupled
    DCMotor agitatorGearbox = DCMotor.getKrakenX60(2); // 2 motors coupled
    agitatorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            agitatorGearbox,
            HopperConstants.AGITATOR_MOMENT_OF_INERTIA,
            HopperConstants.AGITATOR_GEAR_RATIO
        ),
        agitatorGearbox
    );
    
    DCMotor feederGearbox = DCMotor.getKrakenX60(1);
    feederSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            feederGearbox,
            HopperConstants.FEEDER_MOMENT_OF_INERTIA,
            HopperConstants.FEEDER_GEAR_RATIO
        ),
        feederGearbox
    );
    
    // Create PID controllers for simulation
    agitatorPID = new PIDController(
        HopperConstants.AGITATOR_KP,
        HopperConstants.AGITATOR_KI,
        HopperConstants.AGITATOR_KD
    );
    
    feederPID = new PIDController(
        HopperConstants.FEEDER_KP,
        HopperConstants.FEEDER_KI,
        HopperConstants.FEEDER_KD
    );
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    // Update agitator simulation
    agitatorSim.update(0.020); // 20ms robot loop
    
    // Calculate agitator motor voltage with PID + feedforward
    double agitatorPIDOutput = agitatorPID.calculate(
        agitatorSim.getAngularVelocityRPM() / 60.0, // Convert to RPS
        agitatorVelocitySetpoint.in(RotationsPerSecond)
    );
    
    double agitatorFF = agitatorVelocitySetpoint.in(RotationsPerSecond) * HopperConstants.AGITATOR_KV;
    
    agitatorAppliedVolts = MathUtil.clamp(
        agitatorPIDOutput + agitatorFF,
        -12.0,
        12.0
    );
    agitatorSim.setInputVoltage(agitatorAppliedVolts);
    
    // Update feeder simulation
    feederSim.update(0.020);
    
    // Calculate feeder motor voltage with PID + feedforward
    double feederPIDOutput = feederPID.calculate(
        feederSim.getAngularVelocityRPM() / 60.0, // Convert to RPS
        feederVelocitySetpoint.in(RotationsPerSecond)
    );
    
    double feederFF = feederVelocitySetpoint.in(RotationsPerSecond) * HopperConstants.FEEDER_KV;
    
    feederAppliedVolts = MathUtil.clamp(
        feederPIDOutput + feederFF,
        -12.0,
        12.0
    );
    feederSim.setInputVoltage(feederAppliedVolts);
    
    // Update inputs - Agitators
    inputs.agitatorVelocity = RotationsPerSecond.of(agitatorSim.getAngularVelocityRPM() / 60.0);
    inputs.agitatorLeaderAppliedVoltage = Volts.of(agitatorAppliedVolts);
    inputs.agitatorFollowerAppliedVoltage = Volts.of(agitatorAppliedVolts); // Same as leader in sim
    inputs.agitatorLeaderCurrent = Amps.of(Math.abs(agitatorSim.getCurrentDrawAmps()) / 2.0); // Split between two motors
    inputs.agitatorFollowerCurrent = Amps.of(Math.abs(agitatorSim.getCurrentDrawAmps()) / 2.0);
    inputs.agitatorLeaderTemperature = 25.0; // Simulated constant temperature
    inputs.agitatorFollowerTemperature = 25.0;
    
    // Update inputs - Feeder
    inputs.feederVelocity = RotationsPerSecond.of(feederSim.getAngularVelocityRPM() / 60.0);
    inputs.feederAppliedVoltage = Volts.of(feederAppliedVolts);
    inputs.feederCurrent = Amps.of(Math.abs(feederSim.getCurrentDrawAmps()));
    inputs.feederTemperature = 25.0;
  }

  @Override
  public void setAgitatorVelocity(AngularVelocity velocity) {
    agitatorVelocitySetpoint = velocity;
  }

  @Override
  public void setFeederVelocity(AngularVelocity velocity) {
    feederVelocitySetpoint = velocity;
  }

  @Override
  public void stopAgitators() {
    agitatorVelocitySetpoint = RotationsPerSecond.of(0.0);
    agitatorAppliedVolts = 0.0;
  }

  @Override
  public void stopFeeder() {
    feederVelocitySetpoint = RotationsPerSecond.of(0.0);
    feederAppliedVolts = 0.0;
  }

  @Override
  public void setAgitatorBrake(boolean brake) {
    // Brake mode not applicable in simulation
  }

  @Override
  public void setFeederBrake(boolean brake) {
    // Brake mode not applicable in simulation
  }
}

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
  private final DCMotorSim spinnerMotorSim;
  
  private final PIDController spinnerPID;
  
  private AngularVelocity spinnerVelocitySetpoint = RotationsPerSecond.of(0.0);
  
  private double deployAppliedVolts = 0.0;
  private double spinnerAppliedVolts = 0.0;
  
  private boolean deployVoltageControl = false;
  private boolean spinnerVoltageControl = false;

  public IntakeIOSim() {
    
    DCMotor spinnerGearbox = DCMotor.getKrakenX60(1);
    spinnerMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            spinnerGearbox,
            IntakeConstants.SPINNER_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
            IntakeConstants.SPINNER_GEAR_RATIO
        ),
        spinnerGearbox
    );
    
    spinnerPID = new PIDController(
        IntakeConstants.SPINNER_KP,
        IntakeConstants.SPINNER_KI,
        IntakeConstants.SPINNER_KD
    );
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
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
    
    inputs.spinnerVelocity = RotationsPerSecond.of(spinnerMotorSim.getAngularVelocityRPM() / 60.0);
    inputs.spinnerAppliedVoltage = Volts.of(spinnerAppliedVolts);
    inputs.spinnerCurrent = Amps.of(Math.abs(spinnerMotorSim.getCurrentDrawAmps()));
    inputs.spinnerTemperature = 25.0; // Simulated constant temperature
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
  public void stopSpinner() {
    spinnerVelocitySetpoint = RotationsPerSecond.of(0.0);
    spinnerAppliedVolts = 0.0;
    spinnerVoltageControl = false;
  }

  @Override
  public void setSpinnerBrake(boolean brake) {
    // Brake mode not applicable in simulation
  }
}

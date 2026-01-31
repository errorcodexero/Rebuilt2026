// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.thriftyclimb;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Robot;
import frc.robot.subsystems.climber.ClimberConstants;

/**
 * Physics sim implementation for the ThriftyClimb IO. Mirrors the approach used by
 * {@link frc.robot.subsystems.climber.ClimberIOSim} — uses a linear DCMotor system + DCMotorSim and
 * forwards voltages from the TalonFX sim state into the motor sim.
 */
public class ThriftyClimbIOSim extends ThriftyClimbIOHardware {
  private final DCMotorSim climbSim;

  public ThriftyClimbIOSim() {
    LinearSystem<N2, N1, N2> sys = LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX60Foc(1),
        ClimberConstants.Sim.MOI.in(KilogramSquareMeters),
        ClimberConstants.Sim.gearRatio);

    climbSim = new DCMotorSim(sys, DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void updateInputs(ThriftyClimbIOInputs inputs) {
    // Read sim state from the TalonFX and drive the motor sim
  TalonFXSimState state = getClimb().getSimState();
    climbSim.setInputVoltage(state.getMotorVoltage());
    climbSim.update(Robot.defaultPeriodSecs);

    // Copy back rotor position/velocity into the TalonFX sim so hardware layer reports correct values
    state.setRawRotorPosition(climbSim.getAngularPosition());
    state.setRotorVelocity(climbSim.getAngularVelocity());

    // Delegate to hardware implementation to populate the inputs from the TalonFX signals
    super.updateInputs(inputs);
  }

  @Override
  public void applyOutputs(ThriftyClimbOutputs outputs) {
    // Reuse hardware behavior: convert setpoint to motor control (motion magic voltage)
    super.applyOutputs(outputs);
  }
}

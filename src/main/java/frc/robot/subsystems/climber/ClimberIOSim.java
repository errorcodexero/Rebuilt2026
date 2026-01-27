package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class ClimberIOSim extends ClimberIOTalonFX {
    private final DCMotorSim oneSim;
    private final DCMotorSim twoSim;

    public ClimberIOSim() {
        LinearSystem<N2, N1, N2> sys = LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 
            ClimberConstants.Sim.MOI.in(KilogramSquareMeters), 
            ClimberConstants.Sim.gearRatio
        );

        oneSim = new DCMotorSim(sys, DCMotor.getKrakenX60Foc(1));

        LinearSystem<N2, N1, N2> sys2 = LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), 
            ClimberConstants.Sim.MOI.in(KilogramSquareMeters), 
            ClimberConstants.Sim.gearRatio
        );

        twoSim = new DCMotorSim(sys2, DCMotor.getKrakenX60Foc(1));
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        TalonFXSimState oneState = motorOne.getSimState();
        oneSim.setInputVoltage(oneState.getMotorVoltage());
        oneSim.update(Robot.defaultPeriodSecs);
        oneState.setRawRotorPosition(oneSim.getAngularPosition());
        oneState.setRotorVelocity(oneSim.getAngularVelocity());

        TalonFXSimState twoState = motorOne.getSimState();
        twoSim.setInputVoltage(twoState.getMotorVoltage());
        twoSim.update(Robot.defaultPeriodSecs);
        twoState.setRawRotorPosition(twoSim.getAngularPosition());
        twoState.setRotorVelocity(twoSim.getAngularVelocity());

        super.updateInputs(inputs);
    }

}

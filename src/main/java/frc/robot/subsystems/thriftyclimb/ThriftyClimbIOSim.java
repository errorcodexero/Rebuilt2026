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

public class ThriftyClimbIOSim extends ThriftyClimbIOTalonFX {
    private final DCMotorSim climbSim;
    
    public ThriftyClimbIOSim() {
        LinearSystem<N2, N1, N2> sys = LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1),
            ClimberConstants.Sim.MOI.in(KilogramSquareMeters),
            ClimberConstants.Sim.gearRatio
        );
            
        climbSim = new DCMotorSim(sys, DCMotor.getKrakenX60Foc(1));
    }
        
    @Override
    public void updateInputs(ThriftyClimbInputs inputs) {
        // Read sim state from the TalonFX and drive the motor sim
        TalonFXSimState state = climb_.getSimState();
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
    
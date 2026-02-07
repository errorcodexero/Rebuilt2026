package frc.robot.subsystems.hopper;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;


public class HopperIOSim extends HopperIOTalonFX{
    
    private final DCMotorSim feederSim;
    private final DCMotorSim scramblerSim;
    private double feederGearRatio;
    private double scramblerGearRatio;
    
    public HopperIOSim() {
        feederGearRatio = HopperConstants.feederGearRatio;
        scramblerGearRatio = HopperConstants.scramblerGearRatio;
        feederSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.01, feederGearRatio
            ), 
            DCMotor.getKrakenX60Foc(1)
        );
        scramblerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.01, scramblerGearRatio
            ), 
            DCMotor.getKrakenX60Foc(1)
        );
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        TalonFXSimState feederSimState = feederMotor.getSimState();
        feederSim.setInputVoltage(feederSimState.getMotorVoltage());
        feederSim.update(Robot.defaultPeriodSecs);

        feederSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        feederSimState.setRawRotorPosition(feederSim.getAngularPosition().times(feederGearRatio));
        feederSimState.setRotorVelocity(feederSim.getAngularVelocity().times(feederGearRatio));

        TalonFXSimState scramblerSimState = scramblerMotor.getSimState();
        scramblerSim.setInputVoltage(scramblerSimState.getMotorVoltage());
        scramblerSim.update(Robot.defaultPeriodSecs);

        scramblerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        scramblerSimState.setRawRotorPosition(scramblerSim.getAngularPosition().times(scramblerGearRatio));
        scramblerSimState.setRotorVelocity(scramblerSim.getAngularVelocity().times(scramblerGearRatio));

        super.updateInputs(inputs);
    }
    
}

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;
import frc.robot.generated.CompTunerConstants;

public class ClimberIOSim extends ClimberIOTalonFX {
    public final DCMotorSim deployMotorSim;
    public final DCMotorSim twistMotorSim;

    public ClimberIOSim() {
        super(CompTunerConstants.kCANBus, CompTunerConstants.kCANBus);

        deployMotorSim= new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), ClimberConstants.MOI.deployMOI.in(KilogramSquareMeters), ClimberConstants.GearRatios.deployGearRatio
            ),
            DCMotor.getKrakenX60Foc(1)
        );
        
        twistMotorSim= new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), ClimberConstants.MOI.twistMOI.in(KilogramSquareMeters), ClimberConstants.GearRatios.twistGearRatio
            ),
            DCMotor.getKrakenX60Foc(1)
        );
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        TalonFXSimState deployMotorSimState= deployMotor.getSimState();
        TalonFXSimState twistMotorSimState= twistMotor.getSimState();

        deployMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        deployMotorSim.setInputVoltage(RobotController.getBatteryVoltage());
        deployMotorSim.update(Robot.defaultPeriodSecs);

        deployMotorSimState.setRawRotorPosition(deployMotorSim.getAngularPosition().times(ClimberConstants.GearRatios.deployGearRatio));
        deployMotorSimState.setRotorVelocity(deployMotorSim.getAngularVelocity().times(ClimberConstants.GearRatios.deployGearRatio));

        twistMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        twistMotorSim.setInputVoltage(RobotController.getBatteryVoltage());
        twistMotorSim.update(Robot.defaultPeriodSecs);
        
        twistMotorSimState.setRawRotorPosition(twistMotorSim.getAngularPosition().times(ClimberConstants.GearRatios.twistGearRatio));
        twistMotorSimState.setRotorVelocity(twistMotorSim.getAngularVelocity().times(ClimberConstants.GearRatios.twistGearRatio));

        super.updateInputs(inputs);
    }
}

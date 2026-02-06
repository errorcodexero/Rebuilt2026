package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;
import frc.robot.generated.CompTunerConstants;

public class IntakeIOSim extends IntakeIOTalonFX {
    public final DCMotorSim pivotMotorSim;
    public final DCMotorSim rollerMotorSim;
    
    public IntakeIOSim() {
        super(CompTunerConstants.kCANBus, CompTunerConstants.kCANBus); // This needs to be changed

        pivotMotorSim= new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), IntakeConstants.PIVOT_MOMENTOFINERTIA.in(KilogramSquareMeters), IntakeConstants.motorToPivotGearRatio
            ),
            DCMotor.getKrakenX60Foc(1)
        );  

        rollerMotorSim= new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), IntakeConstants.ROLLER_MOMENTOFINERTIA.in(KilogramSquareMeters), IntakeConstants.rollerGearRatio
            ),
            DCMotor.getKrakenX60Foc(1)
        );
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs){
        //Get the simulation states of each motor
        TalonFXSimState pivotMotorSimState= pivotMotor.getSimState();
        TalonFXSimState rollerMotorSimState= rollerMotor.getSimState();

        pivotMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        pivotMotorSim.setInputVoltage(pivotMotorSimState.getMotorVoltage());
        pivotMotorSim.update(Robot.defaultPeriodSecs);

        pivotMotorSimState.setRawRotorPosition(pivotMotorSim.getAngularPosition().times(IntakeConstants.motorToPivotGearRatio));
        pivotMotorSimState.setRotorVelocity(pivotMotorSim.getAngularVelocity().times(IntakeConstants.motorToPivotGearRatio));
        
        rollerMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        rollerMotorSim.setInputVoltage(rollerMotorSimState.getMotorVoltage());
        rollerMotorSim.update(Robot.defaultPeriodSecs);

        rollerMotorSimState.setRawRotorPosition(rollerMotorSim.getAngularPosition().times(IntakeConstants.rollerGearRatio));
        rollerMotorSimState.setRotorVelocity(rollerMotorSim.getAngularVelocity().times(IntakeConstants.rollerGearRatio));

        super.updateInputs(inputs);
    }
}

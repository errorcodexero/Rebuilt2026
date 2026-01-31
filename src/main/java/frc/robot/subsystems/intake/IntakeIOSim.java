package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.generated.CompTunerConstants;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

public class IntakeIOSim extends IntakeIOTalonFX{
    private final DCMotorSim pivotMotorSim;
    private final DCMotorSim rollerMotorSim;
    
    public IntakeIOSim() {
        super(CompTunerConstants.kCANBus, CompTunerConstants.kCANBus);
        pivotMotorSim= new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), IntakeConstants.PIVOT_MOMENTOFINERTIA.in(KilogramSquareMeters), IntakeConstants.pivotGearRatio
            ),
        DCMotor.getKrakenX60Foc(1) //Not sure how many motors
        );  

        rollerMotorSim= new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), IntakeConstants.ROLLER_MOMENTOFINERTIA.in(KilogramSquareMeters), IntakeConstants.rollerGearRatio
            ),
        DCMotor.getKrakenX60Foc(1) //Not sure how many motors
        );
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs){
        //Get the simulation states of each motor
        TalonFXSimState pivotMotorSimState= pivotMotor.getSimState();
        TalonFXSimState rollerMotorSimState= rollerMotor.getSimState();

        pivotMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        pivotMotorSim.setInputVoltage(pivotMotorSimState.getMotorVoltage());
        pivotMotorSim.update(0.020); //20 millisecond robot sim loop

        pivotMotorSimState.setRawRotorPosition(pivotMotorSim.getAngularPosition().times(IntakeConstants.pivotGearRatio));
        pivotMotorSimState.setRotorVelocity(pivotMotorSim.getAngularVelocity().times(IntakeConstants.pivotGearRatio));

        rollerMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        rollerMotorSim.setInputVoltage(rollerMotorSimState.getMotorVoltage());
        rollerMotorSim.update(0.020); //20 millisecond robot sim loop

        rollerMotorSimState.setRawRotorPosition(rollerMotorSim.getAngularPosition().times(IntakeConstants.rollerGearRatio));
        rollerMotorSimState.setRotorVelocity(rollerMotorSim.getAngularVelocity().times(IntakeConstants.rollerGearRatio));

        super.updateInputs(inputs);
    }
}

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.generated.CompTunerConstants;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.KilogramSquareMeters;



public class IntakeIOSim extends IntakeIOHardware{
    private final DCMotorSim pivotMotorSim;
    private final DCMotorSim rollerMotorSim;
    public IntakeIOSim() {
        super(CompTunerConstants.kCANBus, CompTunerConstants.kCANBus);
        pivotMotorSim  = new DCMotorSim(
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
        TalonFXSimState pivotMotorSimState= pivotMotor.getSimState();
        rollerMotorSim= rollerMotorSim.getSimState();
    }
}

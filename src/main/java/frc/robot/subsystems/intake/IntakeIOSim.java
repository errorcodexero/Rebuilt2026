package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.KilogramSquareMeters;


:
public class IntakeIOSim extends IntakeIOHardware{
    private final DCMotorSim pivotMotorSim;
    private final DCMotorSim rollerMotorSim;

    public IntakeIOSim() throws Exception{
    private final DCMotorSim m_motorSimModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1), IntakeConstants.pivotGearRatio.in(KilogramSquareMeters), IntakeConstants.pivotGearRatio
   ),
        DCMotor.getKrakenX60Foc(1)
);
    }
}

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

public class ShooterIOSim extends ShooterIOTalonFX {

    private final FlywheelSim flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60Foc(3), 0.01, 1.0 / ShooterConstants.gearRatio
        ),
        DCMotor.getKrakenX60Foc(3)
    );

    public ShooterIOSim() {
        super(new CANBus());
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        TalonFXSimState[] motorStates = {
            shooter1Motor.getSimState(),
            shooter2Motor.getSimState(),
            shooter3Motor.getSimState()
        };

        flywheelSim.setInputVoltage(motorStates[0].getMotorVoltage());
        flywheelSim.update(Robot.defaultPeriodSecs);

        for (TalonFXSimState simState : motorStates) {
            simState.setSupplyVoltage(RobotController.getBatteryVoltage());
            simState.setRotorVelocity(flywheelSim.getAngularVelocity().div(ShooterConstants.gearRatio));
            simState.setRotorAcceleration(flywheelSim.getAngularAcceleration().div(ShooterConstants.gearRatio));
        }

        super.updateInputs(inputs);
    }
    
}

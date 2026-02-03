package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIO.ClimberOutputs;

@Deprecated
public class Climber extends SubsystemBase {

    private final ClimberIO io;
    private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();
    private final ClimberOutputs outputs = new ClimberOutputs();

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);

        // Do periodic logic

        io.applyOutputs(outputs);
    }

    public Command motorOneAngle(Angle angle) {
        return runOnce(() -> { outputs.oneSetpoint = angle; });
    }
}
package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private final ClimberIO io_;
    private final ClimberInputsAutoLogged inputs_ = new ClimberInputsAutoLogged();

    public Climber(ClimberIO io) {
        io_ = io;
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs(getName(), inputs_);

        // Do periodic logic
    }
}
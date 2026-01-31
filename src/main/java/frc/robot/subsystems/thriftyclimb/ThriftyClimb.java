package frc.robot.subsystems.thriftyclimb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants;

public class ThriftyClimb extends SubsystemBase {
    private final ThriftyClimbIO io_;
    private final ThriftyClimbIOInputsAutoLogged inputs_ = new ThriftyClimbIOInputsAutoLogged();
    private final ThriftyClimbIO.ThriftyClimbOutputs outputs_ = new ThriftyClimbIO.ThriftyClimbOutputs();
    
    private boolean climbing_ = false;

    public ThriftyClimb(ThriftyClimbIO io) {
        io_ = io;
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs(getName(), inputs_);

        // Do periodic logic

        io_.applyOutputs(outputs_);
    }

    public ThriftyClimbIO.ThriftyClimbIOInputs getInputs() {
        return inputs_;
    }

    public Command ToggleClimb() {
        return Commands.runOnce(() -> {
            if(climbing_){
                outputs_.setpoint = ClimberConstants.thriftyStowedHeight;
            } else {
                outputs_.setpoint = ClimberConstants.thriftyClimbHeight;
            }
            climbing_ = !climbing_;
        });
    }

    public Command SetClimbTarget(Distance target) {
        return Commands.runOnce(() -> {
            outputs_.setpoint = target;
        });
    }
}

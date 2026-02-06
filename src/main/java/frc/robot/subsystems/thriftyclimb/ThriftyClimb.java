package frc.robot.subsystems.thriftyclimb;

import static edu.wpi.first.units.Units.Centimeter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.thriftyclimb.ThriftyClimbIO.ThriftyClimbOutputs;

public class ThriftyClimb extends SubsystemBase {
    private final ThriftyClimbIO io_;
    private final ThriftyClimbInputsAutoLogged inputs_ = new ThriftyClimbInputsAutoLogged();
    private final ThriftyClimbOutputs outputs_ = new ThriftyClimbIO.ThriftyClimbOutputs();
    
    private boolean climbing_ = false;

    public ThriftyClimb(ThriftyClimbIO io) {
        io_ = io;
        outputs_.setpoint = ThriftyClimbConstants.thriftyStowedHeight;
    }

    @Override
    public void periodic() {
        io_.updateInputs(inputs_);
        Logger.processInputs(getName(), inputs_);

        // Do periodic logic
        Logger.recordOutput("Climber/Setpoint", outputs_.setpoint);
        
        io_.applyOutputs(outputs_);
    }

    public Command toggle() {
        return runOnce(() -> {
            if(climbing_){
                outputs_.setpoint = ThriftyClimbConstants.thriftyStowedHeight;
            } else {
                outputs_.setpoint = ThriftyClimbConstants.thriftyClimbHeight;
            }
            climbing_ = !climbing_;
        }).andThen(Commands.waitUntil(() -> inputs_.position.isNear(outputs_.setpoint, Centimeter.one())));
    }

    public Command setTarget(Distance target) {
        return runOnce(() -> {
            outputs_.setpoint = target;
        });
    }
}

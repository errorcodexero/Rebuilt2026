package frc.robot.climber;
import java.util.InputMismatchException;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class climber extends SubsystemBase {
    
    private final climberIO IO;
    private final climberIOinputs inputs = new climberIO.ClimberOutputs();

     public climber(climberIO io) {
        this.IO=io;
    }

    @Override 
    public void periodic() {
        IO.updateInputs(inputs);
        Logger.processInputs(getName(), null);

        IO.applyOutputs(outputs);
    }


    public Command motorOneAngle(Angle angle){
        return Commands.runOnce(()-> {
            outputs.oneSetPoint= angle;
        });
    }
}

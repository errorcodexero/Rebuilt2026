package frc.robot.climber;
import java.util.InputMismatchException;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.climber.climberIO.ClimberInputs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Climber extends SubsystemBase {
    
    private final climberIO io;
    private final ClimberInputs inputs = new ClimberInputs();

     public Climber(climberIO io) {
        this.IO=io;
    }

    @Override 
    public void periodic() {
        IO.updateInputs(inputs);
        Logger.processInputs(getName(), null);

        IO.applyOutputs(inputs);
        
    }

    public Command motorOneAngle(Angle angle){
        return Commands.runOnce(()-> {
            inputs.oneSetPoint= angle;
        });
    }
}

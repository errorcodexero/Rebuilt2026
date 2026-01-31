package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class HopperCommands {
    
    public static Command setFeederVoltage(Hopper hopper, Voltage voltage) {
        return Commands.run(() -> hopper.setFeederVoltage(voltage), hopper).withName("setFeederVoltage");
    }

    public static Command setScramblerVoltage(Hopper hopper, Voltage voltage) {
        return Commands.run(() -> hopper.setScramblerVoltage(voltage), hopper).withName("setScramblerVoltage");
    }
    //Adding more commands
}

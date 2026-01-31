package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class HopperCommands {
    //Start Motor Commands
    public static Command setFeederVoltage(Hopper hopper, Voltage voltage) {
        return Commands.run(() -> hopper.setFeederVoltage(voltage), hopper).withName("setFeederVoltage");
    }

    public static Command setScramblerVoltage(Hopper hopper, Voltage voltage) {
        return Commands.run(() -> hopper.setScramblerVoltage(voltage), hopper).withName("setScramblerVoltage");
    }
    
    //Stop Motor Commands
    public static Command stopFeeder(Hopper hopper) {
        return Commands.run(hopper::stopFeeder, hopper).withName("stopFeeder");
    }

    public static Command stopScrambler(Hopper hopper) {
        return Commands.run(hopper::stopScrambler, hopper).withName("stopScrambler");
    }

    public static Command stopAll(Hopper hopper) {
        return Commands.run(hopper::stopAll, hopper).withName("stopAll");
    }
}

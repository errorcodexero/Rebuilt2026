package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class HopperCommands {
    public static Command setFeederVoltageCommand(Hopper hopper, Voltage volts) {
        return Commands.run(() -> hopper.setFeederVoltage(volts), hopper)
        .withName("Set Feeder Voltage");
}
}

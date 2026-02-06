package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public class Hopper extends SubsystemBase {

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private AngularVelocity scramblerGoal = RotationsPerSecond.of(0.0);
  private AngularVelocity feederGoal = RotationsPerSecond.of(0.0);

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Hopper", inputs);

    Logger.recordOutput("Hopper/ScramblerGoalRPS", scramblerGoal.in(RotationsPerSecond));
    Logger.recordOutput("Hopper/FeederGoalRPS", feederGoal.in(RotationsPerSecond));
    Logger.recordOutput("Hopper/ScramblerAtGoal", isScramblerAtGoal());
    Logger.recordOutput("Hopper/FeederAtGoal", isFeederAtGoal());
  }

  //scrambler
  public void setScramblerVelocity(AngularVelocity velocity) {
    scramblerGoal = velocity;
    io.setScramblerVelocity(velocity);
  }

  public void setScramblerVoltage(Voltage voltage) {
    scramblerGoal = RotationsPerSecond.of(0.0);
    io.setScramblerVoltage(voltage);
  }

  public void idleScrambler() {
    double fraction = 0.15; // idle = 15% of configured max
    setScramblerVelocity(
        RotationsPerSecond.of(HopperConstants.scramblerMaxVelocity.in(RotationsPerSecond) * fraction)
    );
  }

  public void activeScrambler() {
    setScramblerVelocity(HopperConstants.scramblerMaxVelocity);
  }

  public void stopScrambler() {
    scramblerGoal = RotationsPerSecond.of(0.0);
    io.setScramblerVoltage(Volts.of(0.0));
  }

  //feeder
  public void setFeederVelocity(AngularVelocity velocity) {
    feederGoal = velocity;
    io.setFeederVelocity(velocity);
  }

  public void setFeederVoltage(Voltage voltage) {
    feederGoal = RotationsPerSecond.of(0.0);
    io.setFeederVoltage(voltage);
  }

  public void stopFeeder() {
    feederGoal = RotationsPerSecond.of(0.0);
    io.setFeederVoltage(Volts.of(0.0));
  }

  public void stopAll() {
    stopScrambler();
    stopFeeder();
  }

  public void feed() {
    setFeederVelocity(HopperConstants.feederMaxVelocity);
  }

  public void feedSlow() {
    setFeederVelocity(RotationsPerSecond.of(HopperConstants.feederMaxVelocity.in(RotationsPerSecond) * 0.5));
  }

  public void reverseFeed() {
    setFeederVelocity(RotationsPerSecond.of(-HopperConstants.feederMaxVelocity.in(RotationsPerSecond) * 0.5));
  }

  public void scrambleAndFeed() {
    activeScrambler();
    feed();
  }

  //readbacks + state checks
  public AngularVelocity getScramblerVelocity() {
    return inputs.scramblerVelocity;
  }

  public AngularVelocity getFeederVelocity() {
    return inputs.feederVelocity;
  }

  public double getScramblerCurrent() {
    return inputs.scramblerCurrent.in(Amps);
  }

  public double getFeederCurrent() {
    return inputs.feederCurrent.in(Amps);
  }

  public boolean isScramblerAtGoal() {
    return Math.abs(inputs.scramblerVelocity.in(RotationsPerSecond) - scramblerGoal.in(RotationsPerSecond)) < 1.0;
  }

  public boolean isFeederAtGoal() {
    return Math.abs(inputs.feederVelocity.in(RotationsPerSecond) - feederGoal.in(RotationsPerSecond)) < 1.0;
  }

    // Hopper Commands
  public Command setFeederVoltageCommand(Voltage voltage) {
    return Commands.run(() -> setFeederVoltage(voltage), this)
        .withName("Hopper.SetFeederVoltage");
  }

  public Command setScramblerVoltageCommand(Voltage voltage) {
    return Commands.run(() -> setScramblerVoltage(voltage), this)
        .withName("Hopper.SetScramblerVoltage");
  }

  public Command stopFeederCommand() {
    return Commands.run(this::stopFeeder, this)
        .withName("Hopper.StopFeeder");
  }

  public Command stopScramblerCommand() {
    return Commands.run(this::stopScrambler, this)
        .withName("Hopper.StopScrambler");
  }

  public Command stopAllCommand() {
  return Commands.run(this::stopAll, this)
      .withName("Hopper.StopAll");
  }


}
package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.MapleSimUtil;

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

        if (Constants.getMode() == Mode.SIM) {
            // Set shooter running if feeder and scrambler are both rolling.
            MapleSimUtil.setShooterRunning(
                inputs.feederVelocity.gt(RadiansPerSecond.zero()) &&
                inputs.scramblerVelocity.gt(RadiansPerSecond.zero())
            );
        }

        Logger.recordOutput("Hopper/ScramblerGoal", scramblerGoal);
        Logger.recordOutput("Hopper/FeederGoal", feederGoal);
        Logger.recordOutput("Hopper/ScramblerAtGoal", isScramblerAtGoal());
        Logger.recordOutput("Hopper/FeederAtGoal", isFeederAtGoal());
    }
    
    // Scrambler control
    private void setScramblerVelocity(AngularVelocity velocity) {
        scramblerGoal = velocity;
        io.setScramblerVelocity(velocity);
    }
    
    private void setScramblerVoltage(Voltage voltage) {
        scramblerGoal = RotationsPerSecond.of(0.0);
        io.setScramblerVoltage(voltage);
    }
    
    private void stopScrambler() {
        setScramblerVoltage(Volts.of(0.0));
    }
    
    // Feeder control
    private void setFeederVelocity(AngularVelocity velocity) {
        feederGoal = velocity;
        io.setFeederVelocity(velocity);
    }
    
    private void setFeederVoltage(Voltage voltage) {
        feederGoal = RotationsPerSecond.of(0.0);
        io.setFeederVoltage(voltage);
    }
    
    private void stopFeeder() {
        setFeederVoltage(Volts.of(0.0));
    }
    
    private void stopAll() {
        stopScrambler();
        stopFeeder();
    }
    
    // Commands

    /**
     * Runs the scrambler at its idling speed until the command ends.
     * @return
     */
    public Command idleScrambler() {
        return startEnd(
            () -> setFeederVelocity(HopperConstants.feederMaxVelocity.times(0.15)),
            this::stopFeeder
        );
    }

    /**
     * Runs the feeder and scrambler at specified speeds.
     * @param feeder
     * @param scrambler
     * @return
     */
    public Command feed(AngularVelocity feeder, AngularVelocity scrambler) {
        return startEnd(() -> {
            setFeederVelocity(feeder);
            setScramblerVelocity(scrambler);
        }, this::stopAll);
    }

    /**
     * Runs the scrambler at its active speed, and the feeder.
     * @return
     */
    public Command forwardFeed() {
        return feed(HopperConstants.scramblerActiveVelocity, HopperConstants.feedingVelocity);
    }
    
    /**
     * Runs the scramble and feeder in reverse.
     * @return
     */
    public Command reverseFeed() {
        return feed(HopperConstants.scramblerActiveVelocity.times(-1), HopperConstants.feedingVelocity.times(-1));
    }
    
    // Readbacks + state checks
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
        return inputs.scramblerVelocity.isNear(scramblerGoal, RotationsPerSecond.one());
    }
    
    public boolean isFeederAtGoal() {
        return inputs.feederVelocity.isNear(feederGoal, RotationsPerSecond.one());
    }
    
    public Command setFeederVoltageCommand(Voltage voltage) {
        return runOnce(() -> setFeederVoltage(voltage))
            .withName("Hopper.SetFeederVoltage");
    }
    
    public Command setScramblerVoltageCommand(Voltage voltage) {
        return runOnce(() -> setScramblerVoltage(voltage))
            .withName("Hopper.SetScramblerVoltage");
    }
    
    public Command stopFeederCommand() {
        return runOnce(this::stopFeeder)
            .withName("Hopper.StopFeeder");
    }
    
    public Command stopScramblerCommand() {
        return runOnce(this::stopScrambler)
            .withName("Hopper.StopScrambler");
    }
    
    public Command stopAllCommand() {
        return runOnce(this::stopAll)
            .withName("Hopper.StopAll");
    }
}

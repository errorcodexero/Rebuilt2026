package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import frc.robot.util.MapleSimUtil;
import frc.robot.util.PhoenixUtil;

public class GyroIOMaple implements GyroIO {
    private final GyroSimulation sim;

    public GyroIOMaple() {
        sim = MapleSimUtil.getGyroSimulation();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = sim.getGyroReading();
        inputs.yawVelocityRadPerSec = sim.getMeasuredAngularVelocity().in(RadiansPerSecond);
        inputs.odometryYawPositions = sim.getCachedGyroReadings();
        inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimestamps();
    }
}

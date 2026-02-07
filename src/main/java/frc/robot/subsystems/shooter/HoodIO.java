package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface HoodIO {
    @AutoLog
    public static class HoodInputs {
        public Angle position;
    }

    public default void updateInputs(HoodInputs inputs) {};

    public default void runPosition(Angle angle) {};
}
package frc.robot.subsystems.thriftyclimb;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public interface ThriftyClimbIO {
    @AutoLog
    public static class ThriftyClimbInputs {
        public Distance position;
        public LinearVelocity velocity;
        public Current current;
    }

    public static class ThriftyClimbOutputs {
        public Distance setpoint;
    }

    public default void updateInputs(ThriftyClimbInputs inputs) {};

    public default void applyOutputs(ThriftyClimbOutputs outputs) {};
}
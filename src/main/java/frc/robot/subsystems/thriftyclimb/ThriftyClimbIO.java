package frc.robot.subsystems.thriftyclimb;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public interface ThriftyClimbIO {
    @AutoLog
    public static class ThriftyClimbIOInputs {
        public Distance pos;
        public LinearVelocity vel;
        public Current current;
    }

    public static class ThriftyClimbOutputs {
        public Distance setpoint;
    }

    public void updateInputs(ThriftyClimbIOInputs inputs);

    public void applyOutputs(ThriftyClimbOutputs outputs);
}
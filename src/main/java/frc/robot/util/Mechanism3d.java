package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mechanism3d extends SubsystemBase {
    private static final Pose3d hoodZero = new Pose3d(-0.098, 0, 0.457, Rotation3d.kZero);
    private static final Pose3d intakeZero = new Pose3d(0.252, 0, 0.207, Rotation3d.kZero);

    private final String name;

    // Current Positions
    private MutAngle hood = Degrees.zero().mutableCopy();
    private MutAngle intake = Degrees.zero().mutableCopy();
    private MutAngle hopper = Degrees.zero().mutableCopy();

    // Previous Positions
    private MutAngle previousIntake = Degrees.zero().mutableCopy();

    // These are the points where the intake will catch onto the hopper shields.
    private MutAngle hopperUpperCatch = Degrees.zero().mutableCopy();
    private MutAngle hopperLowerCatch = Degrees.of(35).mutableCopy();

    private Mechanism3d(String name) { this.name = name; }

    public void setHood(Angle angle) {
        hood.mut_replace(angle);
    }

    public void setIntake(Angle angle) {
        intake.mut_replace(angle);
    }

    public void zero() {
        hood.mut_replace(Degrees.zero());
        intake.mut_replace(Degrees.zero());
        hopper.mut_replace(Degrees.zero());
        previousIntake.mut_replace(Degrees.zero());
    }

    private Pose3d pitchUp(Pose3d pose, Angle pitch) {
        return pose.transformBy(new Transform3d(Translation3d.kZero, new Rotation3d(Degrees.zero(), pitch, Degrees.zero())));
    }

    public void periodic() {
        Angle deltaIntake = intake.minus(previousIntake);

        // If outside the catch, add this intake delta to the hopper.
        if (intake.gt(hopperLowerCatch) || intake.lt(hopperUpperCatch)) { 
            hopper.mut_plus(deltaIntake);
            hopperLowerCatch.mut_plus(deltaIntake);
            hopperUpperCatch.mut_plus(deltaIntake);
            Logger.recordOutput("Mechanism3d/" + name + "/Catching", true);
        } else {
            Logger.recordOutput("Mechanism3d/" + name + "/Catching", false);
        }

        previousIntake.mut_replace(intake);

        Logger.recordOutput("Mechanism3d/" + name, new Pose3d[] {
            pitchUp(hoodZero, hood),
            pitchUp(intakeZero, intake),
            pitchUp(intakeZero, hopper)
        });
    }

    public static final Mechanism3d measured = new Mechanism3d("Measured");
    public static final Mechanism3d setpoints = new Mechanism3d("Setpoints");
}

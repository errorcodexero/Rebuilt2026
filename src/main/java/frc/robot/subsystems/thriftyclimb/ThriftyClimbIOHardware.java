package frc.robot.subsystems.thriftyclimb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.climber.ClimberConstants;

public class ThriftyClimbIOHardware implements ThriftyClimbIO {
    private final TalonFX climb_;

    // Protected accessor for simulator subclasses
    protected TalonFX getClimb() {
        return climb_;
    }

    public ThriftyClimbIOHardware(){
        climb_ = new TalonFX(ClimberConstants.thriftyClimbId);

        TalonFXConfiguration motorOneConfiguration = new TalonFXConfiguration();
        motorOneConfiguration.Feedback.FeedbackRemoteSensorID = 3;
        motorOneConfiguration.Feedback.SensorToMechanismRatio = 1;
        climb_.getConfigurator().apply(motorOneConfiguration, 0.25);
        climb_.setPosition(Degrees.of(ClimberConstants.thriftyStowedHeight.in(Inches) * ClimberConstants.thriftyGearRatio * Math.PI * ClimberConstants.thriftyClimbSpoolRad.in(Inches)));
    }

    public void updateInputs(ThriftyClimbIOInputs inputs) {
        inputs.pos = Inches.of(climb_.getPosition().getValue().in(Revolutions) / (ClimberConstants.thriftyGearRatio * Math.PI * ClimberConstants.thriftyClimbSpoolRad.in(Inches)));
        inputs.vel = InchesPerSecond.of(climb_.getVelocity().getValue().in(RevolutionsPerSecond) / (ClimberConstants.thriftyGearRatio * Math.PI * ClimberConstants.thriftyClimbSpoolRad.in(Inches)));;
        inputs.current = climb_.getStatorCurrent().getValue();
    }

    public void applyOutputs(ThriftyClimbOutputs outputs) {
        // Example output application
        climb_.setControl(new MotionMagicVoltage(Revolutions.of(outputs.setpoint.in(Inches) * ClimberConstants.thriftyGearRatio * Math.PI * ClimberConstants.thriftyClimbSpoolRad.in(Inches))));
    }
}

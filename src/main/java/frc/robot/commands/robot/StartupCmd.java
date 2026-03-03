package frc.robot.commands.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;

public class StartupCmd extends SequentialCommandGroup {
    private final static AngularVelocity startupScramblerVelocity = RotationsPerSecond.of(-10);
    private final static AngularVelocity startupFeederVelocity = RotationsPerSecond.of(-10);
    private final static AngularVelocity startupShooterVelocity = RotationsPerSecond.of(-45);

    private IntakeSubsystem intake_ ;
    private Hopper hopper_ ;
    private Shooter shooter_ ;

    public StartupCmd(IntakeSubsystem intake, Hopper hopper, Shooter shooter) {
        intake_ = intake ;
        hopper_ = hopper ;
        shooter_ = shooter ;

        addCommands(
            intake_.deployCmd(),
            Commands.runOnce(()-> System.out.println("Intake deployment started")),          
            Commands.waitUntil(intake_::isIntakeDeployed),
            Commands.runOnce(()-> System.out.println("Intake deployment finished")),
            moveShooterBalls(),                                     // Run the startup sequence to deploy the intake and move balls from shooter to hopper
            intake_.waitCommand()                                   // Move the intake to the waiting position to avoid collisions with the ground and the robot during auto
        );

        setName("StartupCmd") ;
    }

    private Command moveShooterBalls() {
        return new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.0),
                hopper_.feed(startupFeederVelocity, startupScramblerVelocity).withTimeout(3.0)
            ),
            new SequentialCommandGroup(
                new WaitCommand(1.0),
                shooter_.requestToVelocityCmd(startupShooterVelocity).withTimeout(2.0)
            )
        );
    }
}

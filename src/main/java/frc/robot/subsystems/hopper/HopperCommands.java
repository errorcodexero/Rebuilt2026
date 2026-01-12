package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Factory class for creating hopper commands.
 */
public class HopperCommands {
  
  /**
   * Command to set the agitator velocity.
   * @param hopper The hopper subsystem
   * @param velocity The target velocity
   * @return A command that runs the agitators at the specified velocity
   */
  public static Command setAgitatorVelocity(Hopper hopper, AngularVelocity velocity) {
    return Commands.run(() -> hopper.setAgitatorVelocity(velocity), hopper)
        .withName("SetAgitatorVelocity");
  }

  /**
   * Command to set the feeder velocity.
   * @param hopper The hopper subsystem
   * @param velocity The target velocity
   * @return A command that runs the feeder at the specified velocity
   */
  public static Command setFeederVelocity(Hopper hopper, AngularVelocity velocity) {
    return Commands.run(() -> hopper.setFeederVelocity(velocity), hopper)
        .withName("SetFeederVelocity");
  }

  /**
   * Command to run agitators at idle speed.
   * @param hopper The hopper subsystem
   * @return A command that idles the agitators
   */
  public static Command idleAgitators(Hopper hopper) {
    return Commands.run(() -> hopper.idleAgitators(), hopper)
        .withName("IdleAgitators");
  }

  /**
   * Command to run agitators at active speed.
   * @param hopper The hopper subsystem
   * @return A command that runs the agitators actively
   */
  public static Command activeAgitators(Hopper hopper) {
    return Commands.run(() -> hopper.activeAgitators(), hopper)
        .withName("ActiveAgitators");
  }

  /**
   * Command to feed balls to the shooter.
   * @param hopper The hopper subsystem
   * @return A command that feeds balls
   */
  public static Command feed(Hopper hopper) {
    return Commands.run(() -> hopper.feed(), hopper)
        .withName("Feed");
  }

  /**
   * Command to feed balls slowly.
   * @param hopper The hopper subsystem
   * @return A command that feeds balls slowly
   */
  public static Command feedSlow(Hopper hopper) {
    return Commands.run(() -> hopper.feedSlow(), hopper)
        .withName("FeedSlow");
  }

  /**
   * Command to reverse the feeder to unjam.
   * @param hopper The hopper subsystem
   * @return A command that reverses the feeder
   */
  public static Command reverseFeeder(Hopper hopper) {
    return Commands.run(() -> hopper.reverseFeeder(), hopper)
        .withName("ReverseFeeder");
  }

  /**
   * Command to agitate and feed simultaneously.
   * @param hopper The hopper subsystem
   * @return A command that agitates and feeds
   */
  public static Command agitateAndFeed(Hopper hopper) {
    return Commands.run(() -> hopper.agitateAndFeed(), hopper)
        .withName("AgitateAndFeed");
  }

  /**
   * Command to stop the agitators.
   * @param hopper The hopper subsystem
   * @return A command that stops the agitators
   */
  public static Command stopAgitators(Hopper hopper) {
    return Commands.runOnce(() -> hopper.stopAgitators(), hopper)
        .withName("StopAgitators");
  }

  /**
   * Command to stop the feeder.
   * @param hopper The hopper subsystem
   * @return A command that stops the feeder
   */
  public static Command stopFeeder(Hopper hopper) {
    return Commands.runOnce(() -> hopper.stopFeeder(), hopper)
        .withName("StopFeeder");
  }

  /**
   * Command to stop all hopper motors.
   * @param hopper The hopper subsystem
   * @return A command that stops all motors
   */
  public static Command stopAll(Hopper hopper) {
    return Commands.runOnce(() -> hopper.stopAll(), hopper)
        .withName("StopAll");
  }

  /**
   * Command to agitate and feed, then wait until feeder is at speed.
   * @param hopper The hopper subsystem
   * @return A command that prepares to feed
   */
  public static Command prepareToFeed(Hopper hopper) {
    return Commands.runOnce(() -> hopper.agitateAndFeed(), hopper)
        .andThen(Commands.waitUntil(hopper::isFeederAtGoal))
        .withName("PrepareToFeed");
  }

  /**
   * Command to feed for a specific duration.
   * @param hopper The hopper subsystem
   * @param seconds The duration in seconds
   * @return A command that feeds for the specified time
   */
  public static Command feedForDuration(Hopper hopper, double seconds) {
    return feed(hopper)
        .withTimeout(seconds)
        .andThen(stopFeeder(hopper))
        .withName("FeedForDuration");
  }

  /**
   * Command to agitate and feed for a specific duration.
   * @param hopper The hopper subsystem
   * @param seconds The duration in seconds
   * @return A command that agitates and feeds for the specified time
   */
  public static Command agitateAndFeedForDuration(Hopper hopper, double seconds) {
    return agitateAndFeed(hopper)
        .withTimeout(seconds)
        .andThen(stopAll(hopper))
        .withName("AgitateAndFeedForDuration");
  }

  /**
   * Command to reverse feeder for a specific duration to unjam.
   * @param hopper The hopper subsystem
   * @param seconds The duration in seconds
   * @return A command that reverses the feeder for the specified time
   */
  public static Command unjamFeeder(Hopper hopper, double seconds) {
    return reverseFeeder(hopper)
        .withTimeout(seconds)
        .andThen(stopFeeder(hopper))
        .withName("UnjamFeeder");
  }

  /**
   * Command that keeps the agitators running at a specific speed.
   * This is a perpetual command that can be set as the default command.
   * @param hopper The hopper subsystem
   * @param velocity The agitator velocity to maintain
   * @return A command that maintains agitator speed
   */
  public static Command maintainAgitation(Hopper hopper, AngularVelocity velocity) {
    return Commands.run(() -> hopper.setAgitatorVelocity(velocity), hopper)
        .withName("MaintainAgitation");
  }
}

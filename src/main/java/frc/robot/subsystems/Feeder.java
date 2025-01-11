package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {

  SparkFlex topFeeder;
  SparkFlex bottomFeeder;

  DigitalInput feedBreak;

  /**
   * The Feeder class represents the subsystem responsible for controlling the feeder mechanism of
   * the robot. It initializes and configures the motors and sensors used by the feeder.
   */
  public Feeder() {
    SparkFlexConfig topConfig = new SparkFlexConfig();
    SparkFlexConfig bottomConfig = new SparkFlexConfig();

    topFeeder = new SparkFlex(FeederConstants.topFeederID, MotorType.kBrushless);
    bottomFeeder = new SparkFlex(FeederConstants.bottomFeederID, MotorType.kBrushless);

    topConfig.inverted(FeederConstants.topInvert);
    bottomConfig.inverted(FeederConstants.bottomInvert);

    topConfig.idleMode(IdleMode.kCoast);
    bottomConfig.idleMode(IdleMode.kCoast);

    topConfig.smartCurrentLimit(40);
    bottomConfig.smartCurrentLimit(40);

    topFeeder.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomFeeder.configure(
        bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    feedBreak = new DigitalInput(FeederConstants.feedBrakeID);
  }

  /**
   * Sets the speed of the feeder motors.
   *
   * @param speed the speed at which the feeder motors should run
   */
  public void setSpeed(double speed) {
    topFeeder.set(speed);
    bottomFeeder.set(speed);
  }

  /** Stops the feeder motors. */
  public void stop() {
    topFeeder.set(0);
    bottomFeeder.set(0);
    topFeeder.setInverted(FeederConstants.topInvert);
    bottomFeeder.setInverted(FeederConstants.bottomInvert);
  }

  /**
   * This method is called periodically by the scheduler. It updates the SmartDashboard with the
   * status of the holding note.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Holding Note", holdingNote());
  }

  /**
   * Checks if the feeder is currently holding a note.
   *
   * @return true if the feeder is holding a note, false otherwise
   */
  public boolean holdingNote() {
    return feedBreak.get();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  SparkMax leftClimber;

  SparkMax rightClimber;

  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;

  /**
   * The Climber class represents the subsystem responsible for controlling the climber mechanism of
   * the robot. It initializes and configures the motors, encoders, and other settings for the
   * climber.
   */
  public Climber() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    leftClimber = new SparkMax(ClimberConstants.leftMotorID, MotorType.kBrushless);
    rightClimber = new SparkMax(ClimberConstants.rightMotorID, MotorType.kBrushless);

    leftConfig.inverted(ClimberConstants.leftInvert);
    rightConfig.inverted(ClimberConstants.rightInvert);

    leftConfig.idleMode(IdleMode.kBrake);
    rightConfig.idleMode(IdleMode.kBrake);

    leftEncoder = leftClimber.getEncoder();
    rightEncoder = rightClimber.getEncoder();

    leftClimber.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightClimber.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    resetPostion();
  }

  /** Resets the position of the climber's left and right encoders to zero. */
  public void resetPostion() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  /** Stops the climber motors. */
  public void stop() {
    leftClimber.set(0);
    rightClimber.set(0);
  }

  /**
   * Extends the climber mechanism until a certain encoder position is reached. If the average
   * encoder position is less than 280, the left and right climber motors are set to climbSpeed.
   * Otherwise, the climber motors are stopped.
   */
  public void extend() {
    if (getAverageEncoderPosition() < 280) {
      leftClimber.set(ClimberConstants.climbSpeed);
      rightClimber.set(ClimberConstants.climbSpeed);
    } else stop();
  }

  /**
   * Retracts the climber mechanism. If the average encoder position is greater than 10, the left
   * and right climber motors are set to negative climb speed. Otherwise, the climber motors are
   * stopped.
   */
  public void retract() {
    if (getAverageEncoderPosition() > 10) {
      leftClimber.set(-ClimberConstants.climbSpeed);
      rightClimber.set(-ClimberConstants.climbSpeed);
    } else stop();
  }

  /**
   * Overrides the downward movement of the climber subsystem. If the average encoder position is
   * less than 0, the position is reset. The left and right climber motors are set to the negative
   * climb speed.
   */
  public void overideDown() {
    if (getAverageEncoderPosition() < 0) {
      resetPostion();
    }
    leftClimber.set(-ClimberConstants.climbSpeed);
    rightClimber.set(-ClimberConstants.climbSpeed);
  }

  /**
   * Sets the speed of the climber. If the speed is greater than 0.1, the climber will extend. If
   * the speed is less than -0.1, the climber will retract. If the speed is between -0.1 and 0.1,
   * the climber will stop.
   *
   * @param speed the speed at which to set the climber
   */
  public void setSpeed(double speed) {
    if (speed > .1) {
      extend();
    } else if (speed < -0.1) {
      retract();
    } else stop();
  }

  /**
   * Returns the average position of the left and right encoders.
   *
   * @return the average encoder position
   */
  public double getAverageEncoderPosition() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
  }

  /**
   * This method is called periodically by the scheduler. It updates the SmartDashboard with the
   * average climb value.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Average Climb", getAverageEncoderPosition());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  SparkFlex upperIntakeMotor;

  SparkFlex lowerIntakeMotor;

  RelativeEncoder upperEncoder;
  RelativeEncoder lowerEncoder;

  DigitalInput inputBreak;

  /**
   * The Intake class represents the intake subsystem of the robot. It controls the intake motors
   * and encoders.
   */
  public Intake() {
    SparkFlexConfig upperConfig = new SparkFlexConfig();
    SparkFlexConfig lowerConfig = new SparkFlexConfig();

    upperIntakeMotor = new SparkFlex(IntakeConstants.upIntakeID, MotorType.kBrushless);
    lowerIntakeMotor = new SparkFlex(IntakeConstants.lowIntakeID, MotorType.kBrushless);

    upperConfig.voltageCompensation(Constants.voltageComp);
    lowerConfig.voltageCompensation(Constants.voltageComp);

    upperConfig.inverted(IntakeConstants.upInvert);
    lowerConfig.inverted(IntakeConstants.lowInvert);

    upperConfig.smartCurrentLimit(60);
    lowerConfig.smartCurrentLimit(60);

    upperIntakeMotor.configure(
        upperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    lowerIntakeMotor.configure(
        lowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    upperEncoder = upperIntakeMotor.getEncoder();
    lowerEncoder = lowerIntakeMotor.getEncoder();

    inputBreak = new DigitalInput(IntakeConstants.intakeBreakID);
  }

  /**
   * This method is called periodically by the scheduler. It updates the SmartDashboard with the
   * boolean value indicating whether the intake has a note.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Has Note", hasNote());
  }

  /**
   * Sets the speed of the intake motors.
   *
   * @param speed the speed to set the intake motors to
   */
  public void setSpeed(double speed) {
    upperIntakeMotor.set(speed);
    lowerIntakeMotor.set(speed);
  }

  /**
   * Returns the average RPM (Rotations Per Minute) of the intake motors.
   *
   * @return The average RPM of the intake motors.
   */
  public int getRPM() {
    int rpm =
        (int) (Math.abs(upperEncoder.getVelocity()) + Math.abs(lowerEncoder.getVelocity())) / 2;
    return rpm;
  }

  /** Stops the intake motors. */
  public void stop() {
    upperIntakeMotor.set(0);
    lowerIntakeMotor.set(0);
  }

  /**
   * Checks if the intake has detected a note.
   *
   * @return true if a note is detected, false otherwise
   */
  public boolean hasNote() {
    return inputBreak.get();
  }
}

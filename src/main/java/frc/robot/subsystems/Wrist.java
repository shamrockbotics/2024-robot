// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
  /** Creates a new Intake. */
  DutyCycleEncoder angleEncoder;

  SparkMax leftMotor;
  SparkMax rightMotor;

  PIDController angleController;
  PIDController angleControllerHigh;

  double target = WristConstants.STOW;

  /**
   * The Wrist() constructor initiates the wrist subsystem of the robot. This method is responsible
   * for the control of the wrist angle and the management of the motors and encoders associated
   * with the wrist.
   *
   * <p>Key Components: DutyCycleEncoder: It reads PWM signals and outputs the duty cycle as a
   * percentage. Here, it's being used as an encoder for the wrist angle.
   *
   * <p>SparkMax: These are types of motor controllers with built-in encoders. There are two of
   * these, leftMotor and rightMotor, controlling the two motors on the wrist respectively.
   *
   * <p>PIDController: These are used to control the wrist angles. Two PID controllers,
   * angleControllerHigh and angleController, are defined here.
   *
   * <p>Functionality includes: - Initializes angleEncoder using the PWM port defined in
   * WristConstants. - Configures two motors with the defined motor type - brushless. - Restores
   * motors to factory default settings. - Enables voltage compensation on the motors. - Initializes
   * two PID controllers with their respective coefficients. - Sets the tolerance for the PID
   * controllers. - Configures the motors' idle modes to brake. - Sets the inversion state of
   * motors, this decides the direction of rotations when a positive input is given.
   */
  public Wrist() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    leftMotor = new SparkMax(WristConstants.wristID, MotorType.kBrushless);
    rightMotor = new SparkMax(WristConstants.wrist2ID, MotorType.kBrushless);

    leftConfig.voltageCompensation(Constants.voltageComp);
    rightConfig.voltageCompensation(Constants.voltageComp);

    leftConfig.idleMode(IdleMode.kBrake);
    rightConfig.idleMode(IdleMode.kBrake);

    leftConfig.inverted(WristConstants.rightInvert);
    rightConfig.inverted(WristConstants.leftInvert);

    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    angleEncoder = new DutyCycleEncoder(WristConstants.wristPort);

    angleControllerHigh = new PIDController(0.0085, 0, 0);
    angleController =
        new PIDController(WristConstants.wristP, WristConstants.wristI, WristConstants.wristD);

    angleController.setTolerance(3);
  }

  /**
   * Sets the output for the leftMotor and rightMotor on the wrist subsystem. This method applies a
   * given output value to both motors.
   *
   * @param output the desired output power ratio for the wrist motors in range from -1.0 (full
   *     reverse) to 1.0 (full forwards)
   */
  public void setOutput(double output) {
    leftMotor.set(output);
    rightMotor.set(output);
  }

  /** Stops the wrist movement by setting the left and right motors to 0. */
  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  /**
   * Sets the speed of the wrist motors. If the speed is positive and the wrist angle is greater
   * than the stow angle, the wrist motors will stop. If the speed is negative and the wrist angle
   * is less than the pickup angle, the wrist motors will stop. Otherwise, the wrist motors will be
   * set to the specified speed.
   *
   * @param speed the speed to set the wrist motors to
   */
  public void setSpeed(double speed) {
    if (speed > 0 && getDegrees() > WristConstants.STOW) {
      stop();
    } else if (speed < 0 && getDegrees() < WristConstants.PICKUP) {
      stop();
    } else {
      leftMotor.set(speed);
      rightMotor.set(speed);
    }
  }

  /**
   * Returns the current angle of the wrist in degrees.
   *
   * @return The angle of the wrist in degrees.
   */
  public double getDegrees() {
    double degrees = 360 * (angleEncoder.get()) - WristConstants.offset;

    if (degrees > 360) {
      degrees -= 360;
    } else if (degrees < 0) {
      degrees += 360;
    }
    return degrees;
  }

  /**
   * Holds the wrist at the specified setpoint. If the setpoint is greater than the stow position,
   * it will be set to the stow position. The output is calculated based on the current wrist angle
   * and the setpoint. If the wrist angle is greater than 120 degrees, a high angle controller is
   * used. Otherwise, a regular angle controller is used. The output is limited to the maximum
   * rotate up speed and the maximum rotate speed. The output is then applied to both the left and
   * right wrist motors.
   *
   * @param setpoint The desired angle setpoint for the wrist.
   */
  public void holdWrist(double setpoint) {

    if (setpoint > WristConstants.STOW) {
      setpoint = WristConstants.STOW;
    }
    target = setpoint;
    double output;
    /*
     * if (getDegrees() - setpoint < 0){
     * output = angleUpController.calculate(getDegrees(), setpoint);
     * } else {
     */
    if (getDegrees() > 120) {
      output = angleControllerHigh.calculate(getDegrees(), setpoint);
    } else {
      output = angleController.calculate(getDegrees(), setpoint);
    }

    // }

    if (output > WristConstants.MaxRotateUpSpeed) {
      output = WristConstants.MaxRotateUpSpeed;
    } else if (output < WristConstants.MaxRotateSpeed) {
      output = WristConstants.MaxRotateSpeed;
    }

    leftMotor.set(output);
    rightMotor.set(output);
  }

  /**
   * Returns the target value for the wrist.
   *
   * @return the target value for the wrist
   */
  public double getTarget() {
    return target;
  }

  /**
   * Sets the target rotation for the wrist.
   *
   * @param rotation the target rotation in degrees
   */
  public void setTarget(double rotation) {
    this.target = rotation;
  }

  /**
   * Checks if the wrist is on target.
   *
   * @return true if the wrist is on target, false otherwise
   */
  public boolean onTarget() {
    return angleController.atSetpoint();
  }

  /**
   * This method is called periodically by the scheduler. It updates the value of the "Wrist
   * Encoder" on the SmartDashboard.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Encoder", getDegrees());
  }
}

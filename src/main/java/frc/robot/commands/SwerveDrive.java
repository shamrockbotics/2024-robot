package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveDrive extends Command {
  private Swerve swerve;
  private DoubleSupplier m_translationSup;
  private DoubleSupplier m_strafeSup;
  private DoubleSupplier m_rotationSup;
  private BooleanSupplier m_robotCentricSup;
  private SlewRateLimiter m_xAxisLimiter;
  private SlewRateLimiter m_yAxisLimiter;
  private BooleanSupplier m_isEvading;
  private BooleanSupplier m_isLocked;

  public SwerveDrive(
      Swerve swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier isEvading,
      BooleanSupplier isLocked) {

    this.swerve = swerve;
    addRequirements(swerve);

    m_translationSup = translationSup;
    m_strafeSup = strafeSup;
    m_rotationSup = rotationSup;
    m_robotCentricSup = robotCentricSup;
    m_isEvading = isEvading;
    m_isLocked = isLocked;

    m_xAxisLimiter = new SlewRateLimiter(OperatorConstants.MagnitudeSlewRate);
    m_yAxisLimiter = new SlewRateLimiter(OperatorConstants.MagnitudeSlewRate);
  }

  /**
   * Executes the swerve drive command. This method gets the values for the x-axis, y-axis, and
   * rotation axis from the input sources, applies a deadband to remove small joystick movements,
   * squares the values and applies the sign to preserve the direction, filters the values using the
   * axis limiters, and finally drives the swerve module based on the filtered values and other
   * input conditions.
   */
  @Override
  public void execute() {
    /* Get Values, Deadband */
    double xAxis =
        MathUtil.applyDeadband(m_translationSup.getAsDouble(), OperatorConstants.stickDeadband);
    double yAxis =
        MathUtil.applyDeadband(m_strafeSup.getAsDouble(), OperatorConstants.stickDeadband);
    double rAxis =
        MathUtil.applyDeadband(m_rotationSup.getAsDouble(), OperatorConstants.stickDeadband);

    double xAxisSquared = xAxis * xAxis * Math.signum(xAxis);
    double yAxisSquared = yAxis * yAxis * Math.signum(yAxis);
    // double rAxisSquared = rAxis * rAxis * Math.signum(rAxis);

    double xAxisFiltered = m_xAxisLimiter.calculate(xAxisSquared);
    double yAxisFiltered = m_yAxisLimiter.calculate(yAxisSquared);

    /* Drive */
    swerve.drive(
        new Translation2d(-xAxisFiltered, -yAxisFiltered)
            .times(SmartDashboard.getNumber("Robot Max Speed", 1)),
        rAxis * DriveConstants.MaxAngularSpeed,
        !m_robotCentricSup.getAsBoolean(),
        true,
        m_isEvading.getAsBoolean(),
        m_isLocked.getAsBoolean());
  }
}

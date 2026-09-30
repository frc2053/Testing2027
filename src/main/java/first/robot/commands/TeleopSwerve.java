// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import first.robot.Constants.SwerveConstants;
import first.robot.subsystems.Swerve;

import org.wpilib.command2.Command;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.LinearVelocity;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TeleopSwerve extends Command {
  /** Creates a new TeleopSwerve. */
  private final Swerve swerve;

  private SwerveRequest.FieldCentric fieldOriented =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withSteerRequestType(SteerRequestType.Position);

  private final DoubleSupplier forwardSupplier;
  private final DoubleSupplier strafeSupplier;
  private final DoubleSupplier rotationSupplier;

  private final Supplier<LinearVelocity> maxTranslationalSpeedSupplier;

  private SlewRateLimiter forwardRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTransationalAcceleration.in(MetersPerSecondPerSecond));
  private SlewRateLimiter strafeRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTransationalAcceleration.in(MetersPerSecondPerSecond));
  private SlewRateLimiter rotationRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxAngularAcceleration.in(RotationsPerSecondPerSecond));

  public TeleopSwerve(
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier,
      Supplier<LinearVelocity> maxTranslationalSpeed,
      Swerve swerve) {
    this.forwardSupplier = forwardSupplier;
    this.strafeSupplier = strafeSupplier;
    this.rotationSupplier = rotationSupplier;
    this.swerve = swerve;
    this.maxTranslationalSpeedSupplier = maxTranslationalSpeed;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double maxTranslationalSpeed = maxTranslationalSpeedSupplier.get().in(MetersPerSecond);
    double forwardSpeed = -forwardSupplier.getAsDouble() * maxTranslationalSpeed;
    double strafeSpeed = -strafeSupplier.getAsDouble() * maxTranslationalSpeed;
    double rotationSpeed =
        -rotationSupplier.getAsDouble() * SwerveConstants.maxRotationalSpeed.in(RotationsPerSecond);

    forwardSpeed = forwardRateLimiter.calculate(forwardSpeed);
    strafeSpeed = strafeRateLimiter.calculate(strafeSpeed);
    rotationSpeed = rotationRateLimiter.calculate(rotationSpeed);

    if (Math.hypot(forwardSpeed, strafeSpeed) <= Units.inchesToMeters(0.5)) {
      forwardSpeed = 0;
      strafeSpeed = 0;

      forwardRateLimiter.reset(0);
      strafeRateLimiter.reset(0);
    }

    if (Math.abs(rotationSpeed) <= Units.degreesToRadians(1.5)) {
      rotationSpeed = 0;

      rotationRateLimiter.reset(0);
    }

    swerve.setControl(
        fieldOriented
            .withVelocityX(forwardSpeed)
            .withVelocityY(strafeSpeed)
            .withRotationalRate(Units.rotationsToRadians(rotationSpeed)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    forwardRateLimiter.reset(0);
    strafeRateLimiter.reset(0);
    rotationRateLimiter.reset(0);
    swerve.setControl(fieldOriented.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
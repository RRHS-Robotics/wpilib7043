/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.drive;

import java.util.StringJoiner;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class OmniDrive extends RobotDriveBase {
  private static int instances;

  private final SpeedController m_frontMotor;
  private final SpeedController m_leftMotor;
  private final SpeedController m_rightMotor;
  private final SpeedController m_rearMotor;

  private double m_rightSideInvertMultiplier = -1.0;
  private double m_invertMultiplier = 1.0;
  private boolean m_reported;

  /**
   * An invocation of "new" on this class will create a new OmniDrive object.
   * This controls for SpeedControllers attached to OmniWheels.
   * These will have a diagonal shape on the frame.
   */
  public OmniDrive(SpeedController frontMotor, SpeedController leftMotor,
                      SpeedController rightMotor, SpeedController rearMotor) {
    verify(frontMotor, leftMotor, rightMotor, rearMotor);
    m_frontMotor = frontMotor;
    m_leftMotor = leftMotor;
    m_rightMotor = rightMotor;
    m_rearMotor = rearMotor;
    addChild(m_frontMotor);
    addChild(m_leftMotor);
    addChild(m_rightMotor);
    addChild(m_rearMotor);
    instances++;
    setName("OmniDrive", instances);
  }

  /**
   * Verifies that all motors are nonnull, throwing a NullPointerException if any of them are.
   * The exception's error message will specify all null motors, e.g. {@code
   * NullPointerException("frontMotor, rearMotor")}, to give as much information as
   * possible to the programmer.
   *
   * @throws NullPointerException if any of the given motors are null
   */
  @SuppressWarnings({"PMD.AvoidThrowingNullPointerException", "PMD.CyclomaticComplexity"})
  private void verify(SpeedController frontLeft, SpeedController rearLeft,
                      SpeedController frontRight, SpeedController rearMotor) {
    if (frontLeft != null && rearLeft != null && frontRight != null && rearMotor != null) {
      return;
    }
    StringJoiner joiner = new StringJoiner(", ");
    if (frontLeft == null) {
      joiner.add("frontMotor");
    }
    if (rearLeft == null) {
      joiner.add("leftMotor");
    }
    if (frontRight == null) {
      joiner.add("rightMotor");
    }
    if (rearMotor == null) {
      joiner.add("rearMotor");
    }
    throw new NullPointerException(joiner.toString());
  }

  /**
   * Drive method for Omni platform.
   *
   * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent
   * from its angle or rotation rate.
   *
   * @param ySpeed    The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                  positive.
   */
  @SuppressWarnings("ParameterName")
  public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
    driveCartesian(ySpeed, xSpeed, zRotation, 0.0);
  }

  /**
   * Drive method for Omni platform.
   *
   * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent
   * from its angle or rotation rate.
   *
   * @param ySpeed    The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                  positive.
   * @param gyroAngle The current angle reading from the gyro in degrees around the Z axis. Use
   *                  this to implement field-oriented controls.
   */
  @SuppressWarnings("ParameterName")
  public void driveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle) {
    if (!m_reported) {
      HAL.report(tResourceType.kResourceType_RobotDrive, 4,
                 tInstances.kRobotDrive2_OmniCartesian);
      m_reported = true;
    }

    ySpeed = limit(ySpeed);
    ySpeed = applyDeadband(ySpeed, m_deadband);

    xSpeed = limit(xSpeed);
    xSpeed = applyDeadband(xSpeed, m_deadband);

    // Compensate for gyro angle.
    Vector2d input = new Vector2d(ySpeed, xSpeed);
    input.rotate(-gyroAngle);

    double[] wheelSpeeds = new double[4];
    wheelSpeeds[MotorType.kFrontLeft.value] = input.x + zRotation; // Front
    wheelSpeeds[MotorType.kFrontRight.value] = input.y + zRotation; // Right
    wheelSpeeds[MotorType.kRearLeft.value] = input.y + zRotation; // Left
    wheelSpeeds[MotorType.kRearRight.value] = input.x + zRotation; // Rear

    normalize(wheelSpeeds);

    m_frontMotor.set(wheelSpeeds[MotorType.kFrontLeft.value] * m_maxOutput 
		* m_invertMultiplier); // Front
    m_rightMotor.set(wheelSpeeds[MotorType.kFrontRight.value] * m_maxOutput
		* m_rightSideInvertMultiplier * m_invertMultiplier); // Right
    m_leftMotor.set(wheelSpeeds[MotorType.kRearLeft.value] * m_maxOutput
		* m_invertMultiplier); // Left
    m_rearMotor.set(wheelSpeeds[MotorType.kRearRight.value] * m_maxOutput 
		* m_rightSideInvertMultiplier * m_invertMultiplier); // Rear

    feed();
  }

  /**
   * Gets if the power sent to the right side of the drivetrain is multipled by -1.
   *
   * @return true if the right side is inverted
   */
  public boolean isRightSideInverted() {
    return m_rightSideInvertMultiplier == -1.0;
  }

  /**
   * Sets if the power sent to the right side of the drivetrain should be multipled by -1.
   *
   * @param rightSideInverted true if right side power should be multipled by -1
   */
  public void setRightSideInverted(boolean rightSideInverted) {
    m_rightSideInvertMultiplier = rightSideInverted ? -1.0 : 1.0;
  }
  
  /**
   * Gets if the power sent to all sides of the drivetrain is multipled by -1.
   *
   * @return true if all sides are inverted
   */
  public boolean isInverted() {
    return m_invertMultiplier == -1.0;
  }

  /**
   * Sets if the power sent to all sides of the drivetrain should be multipled by -1.
   *
   * @param inverted true if all power should be multipled by -1
   */
  public void setInverted(boolean inverted) {
    m_invertMultiplier = inverted ? -1.0 : 1.0;
  }
  
  /**
   * Multiplies the power sent to all sides of the drivetrain by -1.
   */
  public void invertInverted() {
    m_invertMultiplier *= -1;
  }

  @Override
  public void stopMotor() {
    m_frontMotor.stopMotor();
    m_rightMotor.stopMotor();
    m_leftMotor.stopMotor();
    m_rearMotor.stopMotor();
    feed();
  }

  @Override
  public String getDescription() {
    return "OmniDrive";
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("OmniDrive");
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Front Motor Speed",
        m_frontMotor::get,
        m_frontMotor::set);
    builder.addDoubleProperty("Right Motor Speed",
        () -> m_rightMotor.get() * m_rightSideInvertMultiplier,
        value -> m_rightMotor.set(value * m_rightSideInvertMultiplier));
    builder.addDoubleProperty("Left Motor Speed",
        m_leftMotor::get,
        m_leftMotor::set);
    builder.addDoubleProperty("Rear Motor Speed",
        () -> m_rearMotor.get() * m_rightSideInvertMultiplier,
        value -> m_rearMotor.set(value * m_rightSideInvertMultiplier));
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  //motor instances
  private final SparkMax m_frontleft;
  private final SparkMax m_backleft;
  private final SparkMax m_frontright;
  private final SparkMax m_backright;
  private final SparkMax m_shooter;

  //controller instances
  private final XboxController ctrl_driver;
  private final XboxController ctrl_operator;

  //kinematics instance
  private final DifferentialDriveKinematics obj_kinematics;

  //class-level constants
  private static final double cnst_trackwidth = Units.inchesToMeters(22.5); //width between centers of drivetrain wheels
  private static final double cnst_ctrldeadband = 0.1; //deadband for controller axis data

  //class-level variables for drive
  private double var_xaxis; //axis generated from transformed controller data
  private double var_zaxis; //axis generated from transformed controller data
  private double var_throttle; //axis generated from transformed controller data, used to limit X axis speed
  private boolean var_operator_a; //button state for shooter
  private boolean var_operator_b; //button state for shooter
  private DifferentialDriveWheelSpeeds var_wheelspeeds; //wheel speeds from kinematics

  //class-level variables for outtake
  private double var_shootspeed; //shooter speed variable

  public Robot() {

    //declare motor params, adjust node IDs as needed
    m_frontleft = new SparkMax(2, MotorType.kBrushed);
    m_backleft = new SparkMax(3, MotorType.kBrushed);
    m_frontright = new SparkMax(4, MotorType.kBrushed);
    m_backright = new SparkMax(5, MotorType.kBrushed);
    m_shooter = new SparkMax(6, MotorType.kBrushed);

    //controller params, adjust ports as needed
    ctrl_driver = new XboxController(0);
    ctrl_operator = new XboxController(1);

    //kinematic params
    obj_kinematics = new DifferentialDriveKinematics(cnst_trackwidth);
  }

  @Override
  public void robotPeriodic() {

    //telemetry
    SmartDashboard.putNumber("x axis", var_xaxis);
    SmartDashboard.putNumber("z axis", var_zaxis);
    SmartDashboard.putNumber("throttle", var_throttle);
    SmartDashboard.putNumber("left speed", var_wheelspeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("right speed", var_wheelspeeds.rightMetersPerSecond);
    SmartDashboard.putBoolean("operator a button", var_operator_a);
    SmartDashboard.putBoolean("operator b button", var_operator_b);
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

    //axis transformations, consume and perform kinematics math using transformed axis data
    var_xaxis = -MathUtil.applyDeadband(ctrl_driver.getRightY(), cnst_ctrldeadband) * var_throttle; //use throttle to limit speed for finer control
    var_zaxis = -MathUtil.applyDeadband(ctrl_driver.getRightX(), cnst_ctrldeadband);
    var_throttle = 1 - MathUtil.applyDeadband(ctrl_driver.getRightTriggerAxis(), cnst_ctrldeadband); //subtract axis data from axis maximum to invert scale of axis
    var_wheelspeeds = obj_kinematics.toWheelSpeeds(new ChassisSpeeds(var_xaxis, 0, var_zaxis));

    //bools for shooter operation
    var_operator_a = ctrl_operator.getAButton();
    var_operator_b = ctrl_operator.getBButton();

    //shooter speed conditional, conditionals for each button must be nested, as mutiple separate else statements writing zero will not allow consistent movement
    if (var_operator_a || var_operator_b) {
      if (var_operator_a && var_operator_b == false) {var_shootspeed = 1;}
      if (var_operator_a == false && var_operator_b) {var_shootspeed = -1;}
    } else {
      var_shootspeed = 0;
    }

    //write speeds to motors
    m_frontleft.set(var_wheelspeeds.leftMetersPerSecond);
    m_backleft.set(var_wheelspeeds.leftMetersPerSecond);
    m_frontright.set(var_wheelspeeds.rightMetersPerSecond);
    m_backright.set(var_wheelspeeds.rightMetersPerSecond);
    m_shooter.set(var_shootspeed);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}

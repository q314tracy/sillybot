// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

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
  private final SparkMax m_outtake;

  //controller instances
  private final XboxController ctrl_driver;
  private final XboxController ctrl_operator;

  //kinematics instance
  private final DifferentialDriveKinematics obj_kinematics;

  //variables
  private double var_xaxis;
  private double var_zaxis;
  private ChassisSpeeds var_speeds;
  private DifferentialDriveWheelSpeeds var_wheelspeeds;

  public Robot() {

    //declare motor params, adjust node IDs as needed
    m_frontleft = new SparkMax(2, MotorType.kBrushed);
    m_backleft = new SparkMax(3, MotorType.kBrushed);
    m_frontright = new SparkMax(4, MotorType.kBrushed);
    m_backright = new SparkMax(5, MotorType.kBrushed);
    m_outtake = new SparkMax(6, MotorType.kBrushed);

    //controller params, adjust ports as needed
    ctrl_driver = new XboxController(0);
    ctrl_operator = new XboxController(1);

    //kinematic params, converts inches to meters
    obj_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22.5));

  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

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

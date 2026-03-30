// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ConstShutter;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShutterItk;
import frc.robot.subsystems.Turret;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private ShutterItk shutterItk = new ShutterItk(0.05, 0.25);
  public static final LimeLight limeLight = new LimeLight();
  // Turret turret = new Turret(0, 6, 0.25, -0.1, 0.15);

  PneumaticHub m_pH = new PneumaticHub(30);
  Solenoid m_solenoid = m_pH.makeSolenoid(2);
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    if(ConstShutter.sensor_shutterItk.get()==true && RobotContainer.engatilhado==false)
    {
      shutterItk.getNote();
    }

    SmartDashboard.putNumber("Encoder Shutter Inc L", ConstShutter.shutter_inc_left.getEncoder().getPosition());
    SmartDashboard.putNumber("Encoder Shutter Inc R",  ConstShutter.shutter_inc_right.getEncoder().getPosition()); 
    SmartDashboard.putNumber("Encoder Shutter Itk", ConstShutter.shutter_itk.getEncoder().getPosition());
    SmartDashboard.putBoolean("Sensor Shutik", ConstShutter.sensor_shutterItk.get());
    SmartDashboard.putBoolean("Engatilhado?", RobotContainer.engatilhado);
    SmartDashboard.putNumber("Velocity Shutter Sup?", ConstShutter.shutter_disp_sup.getEncoder().getVelocity());
    SmartDashboard.putNumber("Velocity Shutter Inf?", ConstShutter.shutter_disp_inf.getEncoder().getVelocity());
    SmartDashboard.putNumber("Encoder Disp Sup", ConstShutter.shutter_disp_sup.getEncoder().getPosition());
    SmartDashboard.putNumber("Encoder Disp Inf", ConstShutter.shutter_disp_inf.getEncoder().getPosition());

    
    // SmartDashboard.putNumber("Mapeamento", RobotContainer.map(limeLight.targeTY, -10, 0, 0, 6));
    

    // limeLight.periodic();
    // turret.moveShotterTarget(RobotContainer.map(Robot.limeLight.targeTY, -10, 0, 0, 6));
    // turret.periodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    m_solenoid.set(true);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
import java.io.IOException;

import com.ctre.phoenix6.hardware.Pigeon2;

import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private SwerveSubsystem mSwerveSubsystem;

  private Timer disabledTimer;

  public static boolean controlVar=false;
  private int contagem=0;

  private double pose2dGetX, pose2dGetY;

  private Pigeon2 pigeon = new Pigeon2(13);

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
    pigeon.setYaw(0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    double positionX=0, positionY=0;

    CommandScheduler.getInstance().run();

    LimelightHelpers.PoseEstimate limelight1Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");
    LimelightHelpers.PoseEstimate limelight2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");

    if((limelight1Pose.tagCount >= 2 && limelight2Pose.tagCount <= 1) || (limelight1Pose.tagCount == 1 && limelight2Pose.tagCount == 0)){
      pose2dGetX = limelight1Pose.pose.getTranslation().getX();
      pose2dGetY = limelight1Pose.pose.getTranslation().getY();
    }
    if((limelight2Pose.tagCount >= 2 && limelight1Pose.tagCount <= 1) || (limelight2Pose.tagCount == 1 && limelight1Pose.tagCount == 0)){
      pose2dGetX = limelight2Pose.pose.getTranslation().getX();
      pose2dGetY = limelight2Pose.pose.getTranslation().getY();
    }

    if(limelight2Pose.tagCount == 0 && limelight1Pose.tagCount == 0){
      m_robotContainer.zeroTag=true;
      // positionX = mSwerveSubsystem.getPose().getX();
      // positionY = mSwerveSubsystem.getPose().getY();
    }
    else{
      m_robotContainer.zeroTag=false;
      positionX = pose2dGetX;
      positionY = pose2dGetY;
    }

    RobotContainer.varX = pose2dGetX;
    RobotContainer.varY = pose2dGetY;

    double yaw = pigeon.getYaw().getValue() % 360;
    if (yaw < 0) {
        yaw += 360;
    }

    NetworkTableInstance.getDefault().getTable("pose").getEntry("botPose").setDoubleArray(new double[]{
      positionX,
      positionY,
      yaw*0.01745200698080279232111692844677 // Degrees
    });

    SmartDashboard.putNumber("limelight1Pose POSE X", limelight1Pose.pose.getTranslation().getX());
    SmartDashboard.putNumber("limelight1Pose POSE y", limelight1Pose.pose.getTranslation().getY());

    SmartDashboard.putNumber("limelight2Pose POSE X", limelight2Pose.pose.getTranslation().getX());
    SmartDashboard.putNumber("limelight2Pose POSE y", limelight2Pose.pose.getTranslation().getY());

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
    //m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);
    contagem=40;
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    if((joystickStimulated(m_robotContainer.driverXbox.getLeftX()) || joystickStimulated(m_robotContainer.driverXbox.getLeftY()) || joystickStimulated(m_robotContainer.driverXbox.getRightX()) || joystickStimulated(m_robotContainer.driverXbox.getRightY())) && contagem>=40){
      m_robotContainer.setDriveMode("angularVelocity");
      controlVar=false;
      contagem=0;
    }
    
    if((m_robotContainer.driverXbox.a().getAsBoolean() ||  m_robotContainer.driverXbox.b().getAsBoolean() || m_robotContainer.driverXbox.x().getAsBoolean() || m_robotContainer.driverXbox.y().getAsBoolean()) && controlVar==false){
      m_robotContainer.setDriveMode("absolute");
      controlVar=true;
    }
    if(controlVar==true){
      contagem++;
    }
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try
    {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }

  private boolean joystickStimulated(double joystick){
    // boolean value=false;
    // if(joystick<(-0.08) || joystick>(0.08))
    // {
    //   value=true;
    // }else{
    //   value=false;
    // }
    return joystick<(-0.08) || joystick>(0.08) ? true : false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shutter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PIDConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  public CommandXboxController driverXbox = new CommandXboxController(0);
  public CommandXboxController coDriverXbox = new CommandXboxController(1);

  private Pose2d nPose2d = new Pose2d(0, 0, new Rotation2d(0));

  private Intake intake = new Intake(0.25, 0.3);
  private Turret turret = new Turret(0,10.5,0.42,-0.08,0.25,0,360,0.1,-1,1);
  private Shutter shutter = new Shutter(1.5, 1, 1, 0.3, 0.6);

  ///private ConditionalCommand  dada = new ConditionalCommand(getAutonomousCommand(), getAutonomousCommand(), null);
  public static boolean engatilhado=false;
  public static int timerInterrupt=0;
  public static boolean driverCommand=true;

  private Command currentDriveCommand;
  
  SendableChooser<Command> autoChooser;
  public static boolean Value;
  public static boolean Cond;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Chosser", autoChooser);
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    NamedCommands.registerCommand("Intake",new InstantCommand(intake::getNote));
    NamedCommands.registerCommand("Turret0", new InstantCommand(() -> turret.getNoteTurret(0)));
    NamedCommands.registerCommand("Shutter0", new InstantCommand(() -> turret.moveShotter(0)));
  
    NamedCommands.registerCommand("Disparar",new InstantCommand(shutter::disparoNoteSpk));
    NamedCommands.registerCommand("DispararAmp",new InstantCommand(shutter::disparoNoteSpk));
    // NamedCommands.registerCommand("AlinhamentoShutter",new InstantCommand(() -> turret.targetMoveShotter(-5)));
    // NamedCommands.registerCommand("AlinhamentoTurret",new InstantCommand(() -> turret.targetMoveTurret()));
    
    NamedCommands.registerCommand("Posi0",new InstantCommand(() -> turret.moveShotter(4)));
    NamedCommands.registerCommand("Posi1",new InstantCommand(() -> turret.moveShotter(1.5)));
    NamedCommands.registerCommand("Posi2",new InstantCommand(() -> turret.moveShotter(2.1)));
    NamedCommands.registerCommand("Posi3",new InstantCommand(() -> turret.moveShotter(3)));
    NamedCommands.registerCommand("Posi4",new InstantCommand(() -> turret.moveShotter(1.75)));
    NamedCommands.registerCommand("Turret340g",new InstantCommand(() -> turret.moveTurret(340)));
    NamedCommands.registerCommand("Turret30g",new InstantCommand(() -> turret.moveTurret(30)));

    NamedCommands.registerCommand("FinishRota", new InstantCommand(() -> intake.isFinishTrue()));
    NamedCommands.registerCommand("FinishRotaClear", new InstantCommand(() -> intake.isFinishClear()));

    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    coDriverXbox.y().onTrue((Commands.runOnce(drivebase::lock)));

    coDriverXbox.a().onTrue(new InstantCommand(intake::getNote));
    coDriverXbox.a().onTrue(new InstantCommand(() -> turret.moveShotter(0)));
    coDriverXbox.a().onTrue(new InstantCommand(() -> turret.getNoteTurret(0)));

    driverXbox.povUp().onTrue(new InstantCommand(() -> turret.moveShotterDisp(3)));
    driverXbox.a().onTrue(new InstantCommand(intake::getNote));
    driverXbox.povDown().onTrue(new InstantCommand(() -> turret.moveShotterDisp(0)));
    driverXbox.b().onTrue(new InstantCommand(shutter::disparoNoteSpk));
    driverXbox.x().onTrue(new InstantCommand(shutter::disparoNoteAmp));
    driverXbox.povLeft().onTrue(new InstantCommand(() -> turret.moveTurret(180)));
    driverXbox.povRight().onTrue(new InstantCommand(() -> turret.moveTurret(0)));

    driverXbox.a().onTrue(new InstantCommand(() -> turret.moveShotter(0)));
    driverXbox.a().onTrue(new InstantCommand(() -> turret.getNoteTurret(0)));
    //coDriverXbox.a().onTrue(new SequentialCommandGroup(new InstantCommand(() -> turret.getNoteTurret(0)), new InstantCommand(() -> turret.getNoteTurret(0))));

    //coDriverXbox.a().onTrue(new ConditionalCommand(currentDriveCommand, currentDriveCommand, null));
    // coDriverXbox.y().onTrue(new InstantCommand(() -> turret.moveShotter(0)));
    // coDriverXbox.y().onTrue(new InstantCommand(() -> turret.getNoteTurret(0)));

    coDriverXbox.x().onTrue(new InstantCommand(shutter::disparoNoteSpk));
    coDriverXbox.b().onTrue(new InstantCommand(shutter::disparoNoteAmp));

    // coDriverXbox.b().onTrue(new InstantCommand(() -> turret.moveTurret(10)));

    // coDriverXbox.povRight().onTrue(new InstantCommand(() -> turret.moveShotter(10)));

    // coDriverXbox.povLeft().onTrue(new InstantCommand(() -> turret.moveShotter(3.5)));

    // // // // coDriverXbox.back().onTrue(new InstantCommand(() -> turret.moveShotter(0)));

    // // // // coDriverXbox.back().onTrue(new InstantCommand(shutter::setOff));
    
    // coDriverXbox.leftBumper().onTrue(new InstantCommand(() -> turret.targetMoveShotter(-10)));
    // coDriverXbox.leftBumper().onTrue(new InstantCommand(() -> turret.targetMoveTurret()));
    // // // // coDriverXbox.y().onTrue(new InstantCommand(() -> shutter.dispararSpkTarget()));

    
    coDriverXbox.povUp().onTrue(new InstantCommand(() -> turret.moveTurret(180)));
    coDriverXbox.povDown().onTrue(new InstantCommand(() -> turret.moveTurret(0)));
    coDriverXbox.povLeft().onTrue(new InstantCommand(() -> turret.moveShotterDisp(2)));
    coDriverXbox.povRight().onTrue(new InstantCommand(() -> turret.moveShotterDisp(3)));
    coDriverXbox.back().onTrue(new InstantCommand(() -> turret.moveShotterDisp(0)));

    coDriverXbox.back().onTrue(new InstantCommand(()-> drivebase.resetOdometry(nPose2d)));

    //coDriverXbox.back().onTrue(new InstantCommand(() -> {
    //  Command autonomousCommand = SetAuto("PATH2");
    //  if (autonomousCommand != null) {
    //      autonomousCommand.schedule(); // Dispara o comando autônomo ao pressionar "A"
    //  }
    //}));
    
    /* Start auto por condição
     * Autono de 2 etapas validado, porém, ele para de realizar as sequencias do rota2
     * 
     */
    //  Trigger sensorTrigger = new Trigger(() -> Value);

    //  sensorTrigger.onTrue(new ConditionalCommand(
    //      SetAuto("PATH1"), // Comando executado se sensor.isTriggered() for true
    //      SetAuto("PATH2"), // Comando executado se sensor.isTriggered() for false
    //      ()-> Cond // Condição
    //  ));

    /*
     * Start auto sequencial
     * 
     */
    //Trigger sensorTrigger = new Trigger(() -> Value);

    //sensorTrigger.onTrue(new SequentialCommandGroup(SetAuto("ROTA1"), SetAuto("PATH2"), SetAuto("PATH1")));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("PATH1");
    
    return autoChooser.getSelected();
  }

  public Command SetAuto(String pathSLC)
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand(pathSLC);
    
    // return autoChooser.getSelected();
  }

  public void setDriveMode(String mode)
  {
  //   drivebase.setDefaultCommand();

    if (currentDriveCommand != null) {
      currentDriveCommand.cancel();
    }

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND)*driverXbox.getRightTriggerAxis(),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND)*driverXbox.getRightTriggerAxis(),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX()*(-1),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getLeftBumperPressed,
                                                                   driverXbox.getHID()::getRightBumperPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND)*driverXbox.getRightTriggerAxis(),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND)*driverXbox.getRightTriggerAxis(),
        () -> driverXbox.getRightX() * 0.8 * (-1),
        () -> driverXbox.getRightY() * 0.8 * (-1));

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND)*driverXbox.getRightTriggerAxis()*(-1),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND)*driverXbox.getRightTriggerAxis()*(-1),
        () -> driverXbox.getRightX() * 0.8 * (-1));

    // Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> driverXbox.getRawAxis(2));

    switch (mode) {
      case "absolute":
        currentDriveCommand = closedAbsoluteDriveAdv;
        break;
      case "directAngle":
        currentDriveCommand = driveFieldOrientedDirectAngle;
        break;
      case "angularVelocity":
        currentDriveCommand = driveFieldOrientedAnglularVelocity;
        break;
      default:
        currentDriveCommand = closedAbsoluteDriveAdv;
        break;
    }

    drivebase.setDefaultCommand(currentDriveCommand);
  }
  
  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}

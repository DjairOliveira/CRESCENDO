// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shutter;
import frc.robot.subsystems.ShutterItk;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static boolean engatilhado=false;

  private Shutter shutter = new Shutter(0.8, 1, 1);
  // Turret turret = new Turret(0, 5, 0.42, -0.1, 0.2);

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_driverController.a().onTrue(new InstantCommand(shutter::disparoNote));
    // m_driverController.rightBumper().onTrue(new InstantCommand(() -> turret.moveShotter(2)));
    // m_driverController.leftBumper().onTrue(new InstantCommand(() -> turret.moveShotter(5)));
    // m_driverController.back().onTrue(new InstantCommand(() -> turret.moveShotter(0)));
    //m_driverController.start().onTrue(new InstantCommand(() -> turret.moveShotter(6)));
    //m_driverController.x().onTrue(new InstantCommand(() -> turret.moveShotterTarget(map(Robot.limeLight.targeTY, -10, 10, 0, 7))));
    

    // turret.periodic();
    // turret.moveShotterTarget(map(Robot.limeLight.targeTY, -10, 10, 0, 7));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
  public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
    double value=0;

    value = (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;

    if(value>outMax || value<outMin)
    {
      if(value>outMax) value = outMax;
      if(value<outMin) value = outMin;
    }
    
    return value;
  }
}

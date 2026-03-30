// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private NetworkTable LimeLTable;
  int PipelineSelect = 0;

  @Override
  public void robotInit() 
  {
    LimeLTable = NetworkTableInstance.getDefault().getTable("limelight");
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
  public void teleopPeriodic()
  {
    LimeLTable.getEntry("pipeline").setNumber(PipelineSelect);  // Define o valor da pipeline desejada
    LimeLTable.getEntry("ledMode").setNumber(1);          // Modo do Led definido para 1
    /* Opções de configura do LED
     * 0 = Modo padrão definido pelo pipeline
     * 1 = Desliga os LEDs
     * 2 = Acende os LEDs em modo de visão
     * 3 = Acende os LEDs em modo de processamento
     */

    double TargetX = LimeLTable.getEntry("tx").getDouble(0.0);    // Recebe os valores de TX da camera
    double TargetY = LimeLTable.getEntry("ty").getDouble(0.0);    // Recebe os valores de TY da camera
    double TargetArea = LimeLTable.getEntry("ta").getDouble(0.0); // Recebe os valores de TA da camera
    /*
    try
    {
      Thread.sleep(500);
      if(PipelineSelect==1) PipelineSelect = 0;
      if(PipelineSelect==0) PipelineSelect = 1;
    } catch (InterruptedException e)
    {
      e.printStackTrace();
    }
    */
    Timer.delay(1);

    if(PipelineSelect==1) /* Exemplo de toggle  entre pipelines*/
    {
      PipelineSelect = 0;
    }
    else 
    {
      PipelineSelect = 1;
    }

    System.out.println("Pipeline alterada para: "+PipelineSelect);
    System.out.println("TX: "+TargetX+"TY: "+TargetY+"TA: "+TargetArea);
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

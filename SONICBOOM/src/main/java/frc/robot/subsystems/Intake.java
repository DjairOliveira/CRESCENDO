package frc.robot.subsystems;

import frc.robot.Constants.ConstChassy;
import frc.robot.Constants.ConstController;
import frc.robot.Constants.ConstShutter;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

public class Intake {
    XboxController xb = new XboxController(1);
    private int inProgres;
    // Turret m_Turret = new Turret(0,11,0.42,-0.1,0.2,0,360,0.05,-0.15,0.15);
    private int timerInterrupt;

    double tAvanco;
    double tRecuo;

    public Intake(double timerAvanco, double timerRecuo){
        inProgres=0;
        timerInterrupt=0;

        inProgres=0;

        tAvanco=timerAvanco;
        tRecuo=timerRecuo;

        ConstChassy.itk_Back1.setIdleMode(IdleMode.kCoast);
        ConstChassy.itk_Back2.setIdleMode(IdleMode.kCoast);
        ConstChassy.itk_Front.setIdleMode(IdleMode.kCoast);
        ConstChassy.itk_Mid.setIdleMode(IdleMode.kCoast);

        ConstChassy.itk_Back1.follow(ConstChassy.itk_Back2, true);
        ConstChassy.itk_Front.setInverted(true);

        ConstShutter.shutter_disp_sup.setIdleMode(IdleMode.kCoast);
        ConstShutter.shutter_disp_inf.setIdleMode(IdleMode.kCoast);
        ConstShutter.shutter_itk.setIdleMode(IdleMode.kCoast);
        ConstShutter.shutter_disp_sup.setInverted(false);

    }
    public class GetNote extends Thread {
        @Override
        public void run(){
            while(!interrupted()){
                SmartDashboard.putBoolean("THREAD intake", true);
                if(inProgres==0){
                    ConstShutter.shutter_disp_sup.setIdleMode(IdleMode.kBrake);
                    ConstShutter.shutter_disp_inf.setIdleMode(IdleMode.kBrake);
                    ConstController.controllerCoDriver.setRumble(RumbleType.kBothRumble, 1);
                    ConstController.controllerDriver.setRumble(RumbleType.kBothRumble, 0.5);
                    if(ConstShutter.shutter_inc_left.getEncoder().getPosition()<=2.5 && (ConstChassy.turret_Hor.getEncoder().getPosition()<=1 || ConstChassy.turret_Hor.getEncoder().getPosition()>=359))
                    {
                        inProgres=1;
                    }
                }
                if(RobotContainer.engatilhado==false && inProgres==1){
                    ConstChassy.itk_Back2.set(0.4);
                    ConstChassy.itk_Front.set(0.3);
                    ConstChassy.itk_Mid.set(0.6);
                    ConstShutter.shutter_itk.set(-1);
                    if(ConstChassy.sensor_Itk.get()==true && RobotContainer.engatilhado==false){
                        inProgres=2;
                    }
                    
                    timerInterrupt++;
                }
                if(ConstChassy.sensor_Itk.get()==false && timerInterrupt>=80000 && inProgres==1)//40000
                {
                    ConstChassy.itk_Back2.set(0);
                    ConstChassy.itk_Front.set(0);
                    ConstChassy.itk_Mid.set(0);
                    ConstShutter.shutter_disp_inf.set(0);
                    ConstShutter.shutter_disp_sup.set(0);
                    ConstShutter.shutter_itk.set(0);
                    inProgres=0;
                    timerInterrupt=0;
                    ConstController.controllerCoDriver.setRumble(RumbleType.kBothRumble, 0);
                    ConstController.controllerDriver.setRumble(RumbleType.kBothRumble, 0);
                    SmartDashboard.putBoolean("THREAD intake", false);
                    interrupt();
                }
                if(inProgres==2)
                {
                    ConstChassy.itk_Back2.set(0);
                    ConstChassy.itk_Front.set(0);
                    ConstShutter.shutter_disp_sup.setIdleMode(IdleMode.kBrake);
                    ConstShutter.shutter_disp_inf.setIdleMode(IdleMode.kBrake);
                    if(ConstChassy.sensor_Itk.get()==false)
                    {
                      inProgres=3;
                    }
                }
                if(inProgres==3)
                {
                    Timer.delay(tAvanco);
                    ConstChassy.itk_Mid.set(0);
                    inProgres=4;
                }
                if(inProgres==4){
                    ConstShutter.shutter_itk.set(1);
                    ConstShutter.shutter_disp_inf.set(-0.2);
                    ConstShutter.shutter_disp_sup.set(-0.2);
                    Timer.delay(tRecuo);
                    inProgres=5;
                }
                if(inProgres==5){
                    ConstShutter.shutter_disp_inf.set(0);
                    ConstShutter.shutter_disp_sup.set(0);
                    ConstShutter.shutter_itk.set(0);
                    ConstShutter.shutter_disp_sup.setIdleMode(IdleMode.kCoast);
                    ConstShutter.shutter_disp_inf.setIdleMode(IdleMode.kCoast);
                    RobotContainer.engatilhado=true;
                    timerInterrupt=0;
                    inProgres=0;
                    SmartDashboard.putBoolean("THREAD GetNote", false);
                    ConstController.controllerCoDriver.setRumble(RumbleType.kBothRumble, 0);
                    ConstController.controllerDriver.setRumble(RumbleType.kBothRumble, 0);
                    interrupt();
                }
                if(RobotContainer.engatilhado==true){
                    ConstController.controllerCoDriver.setRumble(RumbleType.kBothRumble, 0);
                    ConstController.controllerDriver.setRumble(RumbleType.kBothRumble, 0);
                    timerInterrupt=0;
                    SmartDashboard.putBoolean("THREAD intake", false);
                    interrupt();
                }
            }
            try{
                Thread.sleep(100);
            } catch(InterruptedException e){
                interrupt();
                return;
            }
        }
    }
    public void getNote()
    {
        inProgres=0;
        timerInterrupt=0;
        GetNote pegar = new GetNote();
        pegar.start();
    }

    public boolean isEngatilhado(){
        return xb.getBackButton();   /// sensor da note
    }

    public void isFinishTrue(){
        RobotContainer.Value=true;
    }
    public void isFinishClear(){
        RobotContainer.Value=false;
    }
}

package frc.robot.subsystems;
import frc.robot.RobotContainer;
import frc.robot.Constants.ConstShutter;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;

public class Shutter {
    int inProgres;
    double vMotorDisp;
    double vMotorItk;
    double tMotor;

    public Shutter(double speedMotorDisparo, double speedMotorIntake, double timerEmbalo){
        inProgres=0;
        vMotorDisp=speedMotorDisparo;
        vMotorItk=speedMotorIntake*(-1);
        tMotor=timerEmbalo;

        ConstShutter.shutter_disp_sup.setIdleMode(IdleMode.kCoast);
        ConstShutter.shutter_disp_inf.setIdleMode(IdleMode.kCoast);
        ConstShutter.shutter_itk.setIdleMode(IdleMode.kCoast);
        ConstShutter.shutter_disp_sup.setInverted(false);
    }

    public class Disparar extends Thread {
        @Override
        public void run(){
            while(!interrupted()){
                if(inProgres==0)
                {
                    ConstShutter.shutter_disp_inf.set(vMotorDisp);
                    ConstShutter.shutter_disp_sup.set(vMotorDisp);
                    Timer.delay(tMotor);
                    inProgres=1;
                }
                if(inProgres==1)
                {
                    ConstShutter.shutter_itk.set(vMotorItk);
                    Timer.delay(1);
                    inProgres=2;
                }
                if(inProgres==2)
                {
                    ConstShutter.shutter_disp_inf.set(0);
                    ConstShutter.shutter_disp_sup.set(0);
                    ConstShutter.shutter_itk.set(0);
                    RobotContainer.engatilhado = false;
                    inProgres=0;
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

    public void disparoNote()
    {
        Disparar dispara = new Disparar();
        dispara.start();
    }
}

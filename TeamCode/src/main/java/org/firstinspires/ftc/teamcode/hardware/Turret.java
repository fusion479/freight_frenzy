package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret extends Mechanism{
    public static double armEnd = 1.0;
    public static double armStart = 0.0;
    ServoManager turret = new ServoManager("turret",armStart,armEnd);
    public void init(HardwareMap hwmap){
        turret.init(hwmap);
    }

    public void resetLimits(){
        turret.resetLimits(armStart,armEnd);
    }

    public void setPosRatio(double pos){
        turret.setPosRatio(pos);
    }


    public void left(){
        turret.setPosRatio(armEnd);
    }

    public void right(){
        turret.setPosRatio(armStart);
    }


    public void middle(){
        turret.setPosRatio(0.5);
    }


    public void setPosAngle(double ang){
        double angConversion = ang / 180.0;

    }

    public void analogToPos(double X, double Y){

    }


}

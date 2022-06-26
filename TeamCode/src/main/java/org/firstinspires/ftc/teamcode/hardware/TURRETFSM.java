package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class TURRETFSM extends Mechanism{
    ElapsedTime timer = new ElapsedTime();
    public SCORINGFSM score = new SCORINGFSM();
    public Turret turret = new Turret();
    int goal = 0;
    public static double turtime = 800;
    public static double predrop = 200;
    public enum states {
        ready,
        score,
        downing,
    }
    public states s;
    @Override
    public void init(HardwareMap hwMap) {
        s = states.downing;
        score.init(hwMap);
        turret.init(hwMap);
    }
    public void init(HardwareMap hwMap, boolean a) {
        init(hwMap);
        score.arm.auton = a;
    }
    public void loop() {
        switch(s) {
            case ready:
                score.setGoal(goal);
                if(timer.milliseconds() >= turtime) {
                    turret.defaultSide();
                }
                break;
            case score:
                score.score();
                timer.reset();
                setDirMiddle();
                s = states.downing;
            case downing:
                if(timer.milliseconds() >= predrop) {
                    turret.middle();
                    setDirMiddle();
                    if(timer.milliseconds() >= 2*predrop) {
                        score.down();
                        timer.reset();
                    }
                }
                break;
        }
        score.loop();
    }
    public void down() {
        s = states.downing;
    }
    public void toggleHigh() {
        if (s == states.ready && goal == 3) {
            down();
        }else {
            s = states.ready;
            goal = 3;
        }
    }
    public void toggleLow() {
        if (s == states.ready && goal == 1) {
            down();
        }else {
            s = states.ready;
            goal = 1;
        }
    }
    public void toggleMid() {
        if (s == states.ready && goal == 2) {
            down();
        }else {
            s = states.ready;
            goal = 2;
        }
    }
    public void toggleGoal(int target) {
        if( s == states.ready && goal == target) {
            down();
        }else {
            s = states.ready;
            goal = target;
            if(target == 0) {
                goal = 3;
            }
        }
    }
    public void toggleShare() {
        if (s == states.ready && goal == 4) {
            down();
        }else {
            s = states.ready;
            goal = 4;
        }
    }
    public void score() {
        if(s == states.ready) {
            s = states.score;
        }
    }
    public void ready() {
        s = states.ready;
    }
    public void setDirRight() {
        turret.setDefaultSide(Turret.Side.RIGHT);
    }
    public void setDirLeft() {
        turret.setDefaultSide(Turret.Side.LEFT);
    }
    public void setDirMiddle() {
        turret.setDefaultSide(Turret.Side.MIDDLE);
    }
    public void high() {s = states.ready; goal = 0;}
    public void low() {s = states.ready; goal = 3;}
    public void mid() {s = states.ready; goal = 1;}
    public void kOverToggle() {
        score.arm.kOverride = !score.arm.kOverride;
    }
}

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
    public static double turtime = 1000;
    public static double predrop = 100;
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
    public void loop() {
        switch(s) {
            case ready:
                score.setGoal(goal);
                if(timer.milliseconds() >= turtime) {
                    turret.left();
                }
                break;
            case score:
                score.score();
                timer.reset();
                s = states.downing;
            case downing:
                if(timer.milliseconds() >= predrop) {
                    score.down();
                    turret.middle();
                    timer.reset();
                }
                break;
        }
        score.loop();
    }
    public void down() {
        s = states.downing;
    }
    public void toggleHigh() {
        if (s == states.ready) {
            down();
        }else {
            s = states.ready;
            goal = 0;
        }
    }
    public void toggleLow() {
        if (s == states.ready) {
            down();
        }else {
            s = states.ready;
            goal = 3;
        }
    }
    public void toggleMid() {
        if (s == states.ready) {
            down();
        }else {
            s = states.ready;
            goal = 1;
        }
    }
    public void score() {
        if(s == states.ready) {
            s = states.score;
        }
    }
    public void high() {s = states.ready; goal = 0;}
    public void low() {s = states.ready; goal = 3;}
    public void mid() {s = states.ready; goal = 1;}
}

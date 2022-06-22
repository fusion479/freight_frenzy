package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.hardware.util.DelayCommand;

@Config
public class ScoringArm extends ServoMechanism{
    /****
     * WHAT TO DO HERE:
     * Figure out the DESIRED endpoints using ServoTest OpMode
     * Ensure that it's really what you want
     * Test it out, with low commitment
     * If it works, you're good to go
     ****/

    /////////////ARM SERVO LIMITS
    public static double LIMIT_L_START = 1;
    public static double LIMIT_L_END = 0;

    public static double LIMIT_R_START = 0.0;
    public static double LIMIT_R_END = 1;

    ////////////DEPO SERVO LIMITS
    public static double LIMIT_KICK_START = 0.0;
    public static double LIMIT_KICK_END = 1.0;


    private GearedServos pivotArm = new GearedServos(
            "armServoR", LIMIT_R_START, LIMIT_R_END,
            "armServoL", LIMIT_L_START, LIMIT_L_END
            );

    private ServoManager kicker = new ServoManager("deposit", LIMIT_KICK_START, LIMIT_KICK_END);

    /////ARM SERVO POSITIONS
    //Constants are fucked and thrown around
    public static double armStartPos = 0.83;//0.85;//0.07; //homed position
    public static double armEndPos = 0.125; //goes to... end?
    public static double armLowGoalPos = 0.7; //goes to... far end for low goal?
    public static double armMidPos = 0.5; //goes to... the middle that's not the middle?
    public static double armReadyPosTeleop = 0.45; //.35 old

    public static double armDuckPos = 0.8;
    /////DEPO SERVO POSITIONS
    public static double kickStartPos = 0.45;//0.45;//0.8; //init position of depo
    public static double kickTuckPos = 0.23;//0.23;//1.0; //tuck position for movement while going upwards
    public static double kickDumpPos_Hard = 0.75; //0.2; //position to go to for dump movement HARD
    public static double kickDumpPos_Soft = 0.5; //position to go to for dump movement SOFT
    public static double kickLowGoalPos = kickStartPos; //position to go to for lowGoal prep
    /////TURRET SERVO POSITIONS

    private boolean formerBoolArm;
    private boolean homed;


    @Override
    public void init(HardwareMap hwMap) {
        pivotArm.init(hwMap);
        kicker.init(hwMap);
        goToStart();
        depositReset();
        homed = true;
    }
    //GO TO POS RATIO
    public void goTo(double desiredPosition){
        pivotArm.goTo(desiredPosition);
        homed = false;
    }
    // MAX

    /**
     * moves to low goal scoring position with arm and depo
     */
    public void goToLowGoal(){

        pivotArm.goTo(armLowGoalPos);
        //kicker.setPosRatio(depoLowGoalPos);
        homed = false;
        
    }


    /**
     * repositioning for picking up cap
     */
    @Deprecated
    public void pickUpCap(){
        pivotArm.goTo(armLowGoalPos);
        kicker.setPosRatio(0.9);
    }

    /**
     * repositioning of arm and depo for placing cap
     */
    @Deprecated
    public void reposCap() {
        pivotArm.goTo(armMidPos);
        kicker.setPosRatio(1);
    }

    /**
     * tucks for lowgoal scoring
     */
    public void lowGoalTuck(){
        kicker.setPosRatio(kickLowGoalPos);
    }

    /**
     * moves arm to end pos
     */
    public void goToEnd(){
        pivotArm.goTo(armEndPos);
        homed = false;
    }

    /**
     * moves arm to start pos
     */
    public void goToStart(){
        pivotArm.goTo(armStartPos);
        homed = true;
    }


    /**
     * sets depo to tuck pos
     */
    public void tuckPos(){
        kicker.setPosRatio(kickTuckPos);
    }

    /**
     * dumps freight
     */
    public void dumpHard() {
        kicker.setPosRatio(kickDumpPos_Hard);
        Runnable run = new Runnable() {
            @Override
            public void run() {
                kicker.setPosRatio(kickStartPos);
            }

        };
        //delay.delay(run, 350);

    }

    public void dumpPos(){
        kicker.setPosRatio(kickDumpPos_Hard);
    }
    public void dumpSoft() {
        kicker.setPosRatio(kickDumpPos_Soft);
        Runnable run = new Runnable() {
            @Override
            public void run() {
                kicker.setPosRatio(kickStartPos);
            }

        };
        //delay.delay(run, 350);

    }

    public void dumpDuck(){
        kicker.setPosRatio(kickDumpPos_Soft);
        pivotArm.goTo(armDuckPos);
        Runnable run = new Runnable() {
            @Override
            public void run() {
                kicker.setPosRatio(kickStartPos);
            }

        };
    }

    /**
     * resets depo position
     */
    public void depositReset() {
        kicker.setPosRatio(kickStartPos);
    }

    public void readyPos(){

            pivotArm.goTo(armReadyPosTeleop);
    }





    @Deprecated
    public void run(boolean bool){
        if(bool){
            formerBoolArm = true;
        }

        if(formerBoolArm){
            if(!bool){
                if(homed) goToEnd();
                else goToStart();
                formerBoolArm = false;
            }
        }
    }


}

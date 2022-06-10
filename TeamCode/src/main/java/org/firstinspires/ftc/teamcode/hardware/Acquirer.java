package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;
@Config
//Code for acquirer mech
public class  Acquirer extends Mechanism {
    private DcMotor acquirerM;

    public static double flipUpStart = 0.8;
    public static double flipUpEnd = 0.4;
    private ServoManager pullUpServo = new ServoManager("flipUp",flipUpStart,flipUpEnd);

    //tune numbers as desired
    public static double outake = 0.8;
    public static double intake = 1.0;
    //Goal in init is to just initialize the motors

    private enum State{
        up,
        down
    }

    private State state;
    public void init(HardwareMap hwMap) {
        pullUpServo.init(hwMap);
        acquirerM = hwMap.dcMotor.get("intake");
        acquirerM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        acquirerM.setDirection(DcMotorSimple.Direction.REVERSE);
        flipUp();
    }

    //Simple intake and outake code, directing the motors to power and rotate in a certain direction.
    public void intake(double power){
        acquirerM.setPower(power);
    }

    public void outake(double power){
        acquirerM.setPower(-power);
    }

    public void run(boolean left, boolean right){
        if(left) outake(outake);
        else if (right) intake(intake);
        else intake(0);
    }

    public void flipUp(){
        pullUpServo.startPos();
        state = State.up;
    }

    public void flipDown(){
        pullUpServo.endPos();
        state = State.down;
    }

    public void toggle(){
        if (state == State.down){
            flipUp();
        }
        else if (state == State.up){
            flipDown();
        }
    }

    public void resetFlipLimits(){
        pullUpServo.resetLimits(flipUpStart,flipUpEnd);

    }

}
package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SixWheel extends LinearOpMode {
    public DcMotor leftFront = hardwareMap.get(DcMotor.class,
            "leftFront");
    public DcMotor leftRear = hardwareMap.get(DcMotor.class,
            "leftRear");
    public DcMotor rightFront = hardwareMap.get(DcMotor.class,
            "rightFront");
    public DcMotor rightRear = hardwareMap.get(DcMotor.class,
            "rightRear");
    public double returnBiggerMag(double a,double b){
        if (Math.abs(a)>Math.abs(b)){
            return a;
        } else {
            return b;
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive()){
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double left = x;
            double right = y;
            double squareCoord =returnBiggerMag(x,y);
            if (squareCoord==x){
                if (y>0){
                    right-= squareCoord;
                } else {
                    right= -squareCoord;
                    left =left+y;
                }
            } else {
                if (x<0){
                    left+= squareCoord;
                } else {
                    left = squareCoord;
                    right= right- x;
                }
            }
            processInputs(left,right);
        }
    }

    public void processInputs(double left,double right){
        leftFront.setPower(left);
        leftRear.setPower(left);
        rightFront.setPower(right);
        rightRear.setPower(right);
    }
}

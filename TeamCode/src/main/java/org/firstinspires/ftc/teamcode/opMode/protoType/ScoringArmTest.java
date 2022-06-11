package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.ScoringArm;
import org.firstinspires.ftc.teamcode.hardware.Turret;

@Config
@TeleOp (group = "prototype")
public class ScoringArmTest extends LinearOpMode {
    private ScoringArm scoringArm = new ScoringArm();
    private Turret turret = new Turret();
    private Acquirer acquirer = new Acquirer();

    public static double goToDouble = 0.5;

    @Override
    public void runOpMode() throws InterruptedException{
        scoringArm.init(hardwareMap);
        turret.init(hardwareMap);
        acquirer.init(hardwareMap);
        boolean formerA = false;
        boolean formerX = false;

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
//why doesnt my code work
        while(opModeIsActive()){
            /**
             * Left Right Triggers activate intake
             * a button send arm to start
             * b button readies arm
             * x position goes to scoring position
             * left bumper sends flicker to tuck position
             * right bumper sends flicker to ready position
             * y button sends flicker to dump position
             * left dpad moves turret left
             * top dpad moves turret middle
             * right dpad moves turret right
             */
            if(gamepad1.left_trigger > 0) acquirer.outake(gamepad1.left_trigger);
            else acquirer.intake(gamepad1.right_trigger);

            if(gamepad1.a){
                scoringArm.goToStart();
            }

            else if(gamepad1.b){
                scoringArm.readyPos();

            }

            else if(gamepad1.x){
                scoringArm.goToEnd();
            }

            if(gamepad1.left_bumper){
                scoringArm.tuckPos();
            }
            else if(gamepad1.right_bumper){
                scoringArm.depositReset();
            }
            else if(gamepad1.y){
                scoringArm.dumpHard();
            }

            if(gamepad1.dpad_left){
                turret.left();
            }
            else if(gamepad1.dpad_up){
                turret.middle();
            }
            else if(gamepad1.dpad_right){
                turret.right();
            }

            telemetry.addData("Left_trigger",gamepad1.left_trigger);
            telemetry.addData("Right_trigger",gamepad1.right_trigger);
            telemetry.update();


        }
    }

}
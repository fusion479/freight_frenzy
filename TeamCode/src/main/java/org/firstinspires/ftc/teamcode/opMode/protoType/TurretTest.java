package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.ScoringArm;
import org.firstinspires.ftc.teamcode.hardware.Turret;

@TeleOp (group = "prototype")
@Config
public class TurretTest extends LinearOpMode {
    private Turret turret = new Turret();
    //0.1
    //
    public static double position = 0.0;
    @Override
    public void runOpMode() throws InterruptedException{
        turret.init(hardwareMap);
        boolean formerA = false;
        boolean formerB = false;
        boolean formerX = false;
        boolean formerY = false;

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
//why doesnt my code work
        while(opModeIsActive()){
            if (gamepad1.a){
                formerA = true;
            }

            if(formerA){
                if (!gamepad1.a){
                    //DO STUFF
                    formerA = false;
                    turret.middle();
                }
            }

            if (gamepad1.b){
                formerB = true;
            }

            if(formerB){
                if (!gamepad1.b){
                    //DO STUFF
                    formerB = false;
                    turret.right();
                }
            }

            if (gamepad1.x){
                formerX = true;
            }

            if(formerX){
                if (!gamepad1.x){
                    //DO STUFF
                    formerX = false;
                    turret.left();
                }
            }

            if (gamepad1.y){
                formerY = true;
            }

            if(formerY){
                if (!gamepad1.y){
                    //DO STUFF
                    formerY = false;
                    turret.resetLimits();
                }
            }

            telemetry.update();


        }
    }

}
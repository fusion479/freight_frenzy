package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.CapVision;
import org.firstinspires.ftc.teamcode.hardware.Capper;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.FreightSensor;
import org.firstinspires.ftc.teamcode.hardware.LiftScoringV2;
import org.firstinspires.ftc.teamcode.hardware.RetractableOdoSys;
import org.firstinspires.ftc.teamcode.hardware.TURRETFSM;
import org.firstinspires.ftc.teamcode.hardware.util.DelayCommand;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous (group = "BlueAuton")
public class BlueCycles extends LinearOpMode {
    private CapVision cv = new CapVision();
    private Carousel carousel = new Carousel();
    private DelayCommand delay = new DelayCommand();
    private FreightSensor sensor = new FreightSensor();
    private TURRETFSM scoringMech = new TURRETFSM();
    private RetractableOdoSys odoSys = new RetractableOdoSys();
    private Acquirer intake = new Acquirer();
    private Capper cap = new Capper();
    public static double startx = 15.0;
    public static double starty = 70.0;
    public static double startAng = Math.toRadians(90);
    public static double slidelay = 1;

    public static double scoreHubPosx = 0;
    public static double scoreHubPosy = 52;

    public static double scoreHubPosAngB = 65;
    public static double scoreHubPosAngR = -40;

    public static double repositionX = 15.0;
    public static double reposistionY = 71.5;

    public static double preSplineY = 53.5;
    public static double bEnterX = 30;
    public static double bExitX = 30;
    public static double bEnterY = 70;
    public static double warehouseX = 51;
    public static double bExitY = 72;
    public static double inc = 0.5;
    public static Pose2d startPos = new Pose2d(startx, starty, startAng);

    public static double localeReadjustX = 0.0;
    public static double localeReadjustY = -0.25;
    public static Pose2d firstRepos = new Pose2d(scoreHubPosx + 8 , scoreHubPosy + 8, Math.toRadians(22.5));
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static String goal = "highgoal";
    public static double wInc = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        //initialize mechanisms
//        carousel.init(hardwareMap);

        sensor.init(hardwareMap);
        odoSys.init(hardwareMap, true);
        scoringMech.init(hardwareMap);
        cv.init(hardwareMap);
//        odoSys.init(hardwareMap, true);
        intake.init(hardwareMap);
        Vector2d scoreHubPosB = new Vector2d(scoreHubPosx, scoreHubPosy);
        Vector2d bEnter = new Vector2d(bEnterX, bEnterY);


        //drive train + async updates of mechanisms
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setSlides(scoringMech);

        //important coordinates here
        //set startPose

        //trajectory


        telemetry.addData("Status", "Waiting in init");
        telemetry.update();


        telemetry.update();


        drive.setPoseEstimate(startPos);
        TrajectorySequence depoPath = drive.trajectorySequenceBuilder(startPos)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    scoringMech.toggleHigh();
                })
                .lineToLinearHeading(new Pose2d(scoreHubPosx,scoreHubPosy-2, Math.toRadians(scoreHubPosAngB)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringMech.score();
                    intake.intake(1);
                })
                //.waitSeconds(.1)
                //.lineTo(preSpline)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(bExitX, bEnterY, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(warehouseX, bEnterY))
                //.waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(slidelay, () -> {
                    scoringMech.toggleHigh();

                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.outake(1.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intake.intake(0);
                })
                .setReversed(true)
                .lineTo(new Vector2d(bExitX, bExitY))
                .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringMech.score();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    intake.intake(1);
                })
                //.waitSeconds(.1)
                //.lineTo(preSpline)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(bEnterX, bEnterY, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(warehouseX, bEnterY))
                //.waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(slidelay, () -> {
                    scoringMech.toggleHigh();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.outake(1.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intake.intake(0);
                })
                .setReversed(true)
                .lineTo(new Vector2d(bExitX, bExitY))
                .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    readjustLocale(drive);
                    scoringMech.score();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    intake.intake(1);
                })
                //.lineTo(preSpline)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(bEnterX, bEnterY+2*inc, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(warehouseX+inc, bEnterY+2*inc))
                //.waitSeconds(0.1)

                .UNSTABLE_addTemporalMarkerOffset(slidelay, () -> {
                    scoringMech.toggleHigh();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.outake(1.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intake.intake(0);
                })
                .setReversed(true)
                .lineTo(new Vector2d(bExitX, bExitY+2*inc))
                .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringMech.score();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    intake.intake(1);
                })
                //.lineTo(preSpline)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(bEnterX, bEnterY+3*inc, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(warehouseX+2*wInc, bEnterY+3*inc))
                //.waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(slidelay, () -> {
                    scoringMech.toggleHigh();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.outake(1.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intake.intake(0);
                })
                .setReversed(true)
                .lineTo(new Vector2d(bExitX, bExitY+3*inc))
                .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringMech.score();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    intake.intake(1);
                })
                //.lineTo(preSpline)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(bEnterX, bEnterY+3*inc, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(warehouseX+3*wInc, bEnterY+3*inc))
                //.waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(slidelay, () -> {
                    scoringMech.toggleHigh();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.outake(1.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intake.intake(0);
                })
                .setReversed(true)
                .lineTo(new Vector2d(bExitX, bExitY+3*inc))
                .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringMech.score();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    intake.intake(1);
                })
                //.lineTo(preSpline)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(bEnterX, bEnterY+3*inc, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(warehouseX+4*wInc, bEnterY+3*inc))
//                //.waitSeconds(0.1)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //scoringMech.toggle("highgoal");
//                    // drive.acquirerRuns = false;
//                })
//                .setReversed(true)
//                .lineTo(new Vector2d(bExitX, bEnterY))
//                .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    // scoringMech.releaseHard();
//                    // drive.acquirerRuns = true;
//                })
//                //.lineTo(preSpline)
//                .setReversed(false)
//                .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
//                .lineToLinearHeading(new Pose2d(warehouseX-1, bEnterY))
//                //.waitSeconds(0.1)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    //scoringMech.toggle("highgoal");
//                    // drive.acquirerRuns = false;
//                })
//                .setReversed(true)
//                .lineTo(new Vector2d(bExitX, bEnterY))
//                .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    // scoringMech.releaseHard();
//                    // drive.acquirerRuns = true;
//                })
//                .setReversed(false)
//                //.lineTo(preSpline)
//                .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
//                .lineToLinearHeading(new Pose2d(warehouseX-10, bEnterY))
                .build();

        waitForStart();
        if (cv.whichRegion() == 1) {
            goal = "lowgoal";
        }
        if (cv.whichRegion() == 2) {
            goal = "midgoal";
        }
        if (cv.whichRegion() == 3) {
            goal = "highgoal";
        }
        telemetry.addData("goal: ", goal);
        telemetry.addData("region", cv.whichRegion());
        //scoringMech.toggleHigh();
        cap.init(hardwareMap);

        drive.followTrajectorySequence(depoPath);
    }

    public void readjustLocale(SampleMecanumDrive drive){
        Pose2d driveCurrent = drive.getPoseEstimate();
        Pose2d poseReadjustment = new Pose2d(
                driveCurrent.getX() + localeReadjustX, driveCurrent.getY() + localeReadjustY, driveCurrent.getHeading()
        );
        drive.setPoseEstimate(poseReadjustment);
    }

    public static Vector2d pose2Vector(Pose2d givenPose){
        return new Vector2d(givenPose.getX(),givenPose.getY());
    }
}

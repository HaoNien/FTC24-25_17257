package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.visionprocessor.SampleVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class AUTOTest1 extends robotBase{
    //Pose2d startPose = new Pose2d(-8, 60, Math.toRadians(90));
    SampleVisionProcessor samplevisionprocessor;
    VisionPortal visionPortal;
    @Override
    protected void robotInit() {

         samplevisionprocessor=new SampleVisionProcessor();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(samplevisionprocessor)
                .setCameraResolution(new Size(320, 240))
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        visionPortal.stopLiveView();


        Pose2d startPose = new Pose2d(-10, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        //armTarget= 45;
        slideTarget = 40;
        lift=-90;
        turn = 0;
        Claw.setPosition(claw_Close);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                .addTemporalMarker(() -> {
                    lift=50;
                    armTarget= 37;
                    slideTarget = 75;
                })

                .forward(27)
                .addTemporalMarker(() -> {        Claw.setPosition(claw_Open);
                })
                .back(20)
                .addTemporalMarker(() -> {
                    armTarget=5;
                    lift=-90;
                    turn=45;
                })
                .turn(Math.toRadians(-45))
                .splineTo(new Vector2d(-29,41),Math.toRadians(220))
                .addTemporalMarker(() -> {
                    armTarget=0;
                    lift=-90;
                    turn=45;
                })

                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {Claw.setPosition(claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    armTarget=5;
                })
                .waitSeconds(0.5)
                .turn(Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    Claw.setPosition(claw_Open);


                })
                .waitSeconds(0.5)

                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(-39,41))
                .addTemporalMarker(() -> {armTarget=0;})
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Claw.setPosition(claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    armTarget=5;
                })
                .waitSeconds(0.5)
                .turn(Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    Claw.setPosition(claw_Open);



                })
                .waitSeconds(0.5)
                .turn(Math.toRadians(90))


                .strafeTo(new Vector2d(-49,41))
                .addTemporalMarker(() -> {armTarget=0;})
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Claw.setPosition(claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {

                    armTarget=5;
                    slideTarget= 40;
                })
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-49,51))
                .turn(Math.toRadians(-90))
                .addTemporalMarker(() -> {

                    Claw.setPosition(claw_Open);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                armTarget=20;
                lift = -10;
                turn=0;
                })
                .waitSeconds(0.5)
                .turn(Math.toRadians(-45))
                .forward(5)
                .addTemporalMarker(() -> {
                    Claw.setPosition(claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    armTarget = 90;
                    slideTarget = 52;
                    lift = 90;
                })
                .strafeTo(new Vector2d(-10, 35))

                .addTemporalMarker(() -> {

                })




                //.strafeLeft(20)
                //.strafeTo(new Vector2d(-38, 12))
                //.strafeTo(new Vector2d(-45, 50))
                //.strafeTo(new Vector2d(-45, 12))


                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
        telemetry.addData("Arm Po0",samplevisionprocessor.getTargetAngle());
        telemetry.update();



    }

    @Override
    protected void robotInitLoop() {
        armPosNow = armL.getCurrentPosition() / arm2deg; // 讀取手臂當前角度
        armTurn2angle(45);                       // 將手臂維持在目標角度
        telemetry.addData("Angle",samplevisionprocessor.getTargetAngle());


    }

    @Override
    protected void robotStart(){
        limelight.updateRobotOrientation(Math.toDegrees(drive.getRawExternalHeading()));
        LLResult result = limelight.getLatestResult();

        if(gamepad2.a){
            visionPortal.resumeStreaming();

        }
        if(gamepad2.b){
            visionPortal.resumeStreaming();
        }
        if (result.isValid()) {
            telemetry.addData("tx", result.getBotpose_MT2());
            telemetry.addData("tx", convertToPose2d(result.getBotpose_MT2().toString()));
            telemetry.addData("deg", Math.toDegrees(drive.getRawExternalHeading()));

            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC());
        }
        telemetry.addData("Angle",samplevisionprocessor.getTargetAngle());

        telemetry.update();
        turn=(samplevisionprocessor.getTargetAngle()-90);
        armTarget=10;
        //drive.update();
       armTurn2angle(armTarget);
//        slideToPosition(slideTarget);
        wristToPosition(lift, turn);



    }
}

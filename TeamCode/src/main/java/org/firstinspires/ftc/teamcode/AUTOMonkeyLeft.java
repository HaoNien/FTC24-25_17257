package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AUTOMonkeyLeft extends robotBase{
    //Pose2d startPose = new Pose2d(-8, 60, Math.toRadians(90));

    @Override
    protected void robotInit() {

        Pose2d startPose = new Pose2d(33.5, 61.5, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        armTarget= armStartAngle;
        slideTarget = 40;
        lift=-90;
        turn = 0;
        Claw.setPosition(claw_Close);


        Claw.setPosition(claw_Close);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    armTarget = 93;
                    lift = 20;
                    slideTarget = smax;
                    turn = 0;
                })
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(58,53,Math.toRadians(225)),Math.toRadians(225))
                .back(5)
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Open);
                    lift = 40;
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->{
                    lift = -90;
                    armTarget = 50;
                    slideTarget = 40;
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->{
                    lift = -90;
                    armTarget = 10;
                    slideTarget = 59;
                })
                .waitSeconds(0.5)

                //catch2
                .splineToLinearHeading(new Pose2d(50,45,Math.toRadians(270)),Math.toRadians(270))
                .addTemporalMarker(() ->{
                    armTarget = 0;
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Close);
                })
                .waitSeconds(.3)
                .addTemporalMarker(() -> {
                    armTarget = 93;
                    lift = 20;
                    turn = 0;
                    slideTarget = 40;
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    slideTarget = smax;
                })
                .waitSeconds(0.5)
                //put2
                .splineToLinearHeading(new Pose2d(58,53,Math.toRadians(225)),Math.toRadians(225))
                .back(5)
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Open);
                    lift = 40;
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->{
                    lift = -90;
                    armTarget = 50;
                    slideTarget = 40;
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->{
                    lift = -90;
                    armTarget = 10;
                    slideTarget = 59;
                })
                .waitSeconds(0.5)

                //catch3
                .splineToLinearHeading(new Pose2d(61,45,Math.toRadians(270)),Math.toRadians(270))
                .addTemporalMarker(() ->{
                    armTarget = 0;
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Close);

                })
                .waitSeconds(.3)
                .addTemporalMarker(() -> {
                    armTarget = 93;
                    lift = 20;
                    slideTarget = 40;
                    turn = 0;
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    slideTarget = smax;
                })
                .waitSeconds(0.5)
                //put3
                .splineToLinearHeading(new Pose2d(58,53,Math.toRadians(225)),Math.toRadians(225))
                .back(5)
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Open);
                    lift = 40;
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->{
                    lift = -90;
                    armTarget = 45;
                    slideTarget = 40;
                })
                .waitSeconds(1)
                .addTemporalMarker(() ->{
                    armTarget = 10;
                    slideTarget = 70;
                    turn = -65;
                })
                .waitSeconds(0.5)


                //catch4
                .splineToLinearHeading(new Pose2d(58,45,Math.toRadians(310)),Math.toRadians(310))
                .addTemporalMarker(() ->{
                    armTarget = 0;
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Close);

                })
                .waitSeconds(.3)
                .addTemporalMarker(() -> {
                    armTarget = 93;
                    lift = 20;
                    slideTarget = 40;
                    turn = 0;
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    turn = 0;
                    slideTarget = smax;
                })
                .waitSeconds(0.5)
                //put4
                .splineToLinearHeading(new Pose2d(58,53,Math.toRadians(225)),Math.toRadians(225))
                .back(5)
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Open);
                    lift = 40;
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->{
                    armTarget = 50;
                    slideTarget = 70;
                    lift = -90;

                })
                //park
                .splineToLinearHeading(new Pose2d(50,10,Math.toRadians(180)),Math.toRadians(230))
                .addTemporalMarker(() ->{
                    lift = 90;
                })
                .splineToLinearHeading(new Pose2d(26,10,Math.toRadians(180)),Math.toRadians(180))
                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
        telemetry.addData("","");
        telemetry.update();



    }

    @Override
    protected void robotInitLoop() {
        armTurn2angle(armStartAngle);                       // 將手臂維持在目標角度

        slideToPosition(slideTarget);
        wristToPosition(lift, turn);

    }

    @Override
    protected void robotStart(){

        drive.update();
        armTurn2angle(armTarget);
        slideToPosition(slideTarget);
        wristToPosition(lift, turn);



    }
}

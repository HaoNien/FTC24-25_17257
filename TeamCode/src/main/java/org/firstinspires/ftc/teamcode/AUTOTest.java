package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.security.PublicKey;

@Autonomous
public class AUTOTest extends robotBase{
    //Pose2d startPose = new Pose2d(-8, 60, Math.toRadians(90));

    @Override
    protected void robotInit() {
        Pose2d startPose = new Pose2d(36, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        //armTarget= 45;
        slideTarget = 40;
        lift=-90;
        turn = 0;
        Claw.setPosition(claw_Close);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    armTarget = 93;
                    lift = 20;

                    slideTarget = smax;
                    turn = 0;
                })
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(58,56,Math.toRadians(225)),Math.toRadians(270))
                .waitSeconds(1)
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Open);
                    lift = 40;
                })
                .waitSeconds(1)
                .addTemporalMarker(() ->{
                    armTarget = 45;
                    slideTarget = smax0;
                    lift = -90;

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->{
                    armTarget = 5;
                    slideTarget = smax0;
                })
                .splineToLinearHeading(new Pose2d(56,47,Math.toRadians(250)),Math.toRadians(270))
                .addTemporalMarker(() ->{
                    armTarget = 0;
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Close);

                })
                .waitSeconds(.3)
                .addTemporalMarker(() -> {
                    armTarget = 93;
                    lift = 20;

                    slideTarget = smax;
                    turn = 0;
                })
                .splineToLinearHeading(new Pose2d(58,58,Math.toRadians(225)),Math.toRadians(270))
                .waitSeconds(1)
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Open);
                    lift = 40;
                })

                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
        telemetry.addData("Arm Po000000000000000000000000000000000000000000000","");
        telemetry.update();



    }

    @Override
    protected void robotInitLoop() {
        armPosNow = armL.getCurrentPosition() / arm2deg; // 讀取手臂當前角度
        armTurn2angle(45);                       // 將手臂維持在目標角度

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

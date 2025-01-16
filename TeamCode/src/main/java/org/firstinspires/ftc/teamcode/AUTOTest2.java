package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AUTOTest2 extends robotBase{
    //Pose2d startPose = new Pose2d(-8, 60, Math.toRadians(90));

    @Override
    protected void robotInit() {
        Pose2d startPose = new Pose2d(-10, 60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        armTarget= 45;
        slideTarget = 40;
        lift=-90;
        turn = 0;
        Claw.setPosition(claw_Close);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                .setReversed(true)
                .back(27)
                .forward(5)
                //.turn(Math.toRadians(-45))

                .splineToConstantHeading(new Vector2d(-34,27),Math.toRadians(270))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(-39,10),Math.toRadians(180))
                //.strafeTo(new Vector2d(-42,10))
                .splineToConstantHeading(new Vector2d(-43,55),Math.toRadians(90))
                .waitSeconds(0.1)


                .splineToConstantHeading(new Vector2d(-48,13),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-55,50),Math.toRadians(90))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(-60,13),Math.toRadians(180))


                .splineToConstantHeading(new Vector2d(-60,50),Math.toRadians(90))




                //.strafeLeft(20)
                //.strafeTo(new Vector2d(-38, 12))
                //.strafeTo(new Vector2d(-45, 50))
                //.strafeTo(new Vector2d(-45, 12))


                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
        telemetry.addData("Arm Po000000000000000000000000000000000000000000000","");
        telemetry.update();



    }

    @Override
    protected void robotInitLoop() {
        armPosNow = armL.getCurrentPosition() / arm2deg; // 讀取手臂當前角度
        armTurn2angle(45);                       // 將手臂維持在目標角度


    }

    @Override
    protected void robotStart(){

        drive.update();
        armTurn2angle(armTarget);
        slideToPosition(slideTarget);
        wristToPosition(lift, turn);



    }
}

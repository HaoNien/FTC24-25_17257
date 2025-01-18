package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.lang.reflect.Array;
import java.util.Arrays;

@Autonomous
public class AUTOTestMonkey extends robotBase{
    //Pose2d startPose = new Pose2d(-8, 60, Math.toRadians(90));

    @Override
    protected void robotInit() {
        Pose2d startPose = new Pose2d(-8, 60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        armTarget= 45;
        slideTarget = 40;
        lift=-90;
        turn = 0;
        Claw.setPosition(claw_Close);
        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(

                new TranslationalVelocityConstraint(10),
                new AngularVelocityConstraint(1)
        ));
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() ->{
                    armTarget = 90;
                    lift = 60;
                })
                .setReversed(true)
                .back(28)
                .addTemporalMarker(() ->{
                    armTarget = 90;
                    lift = 60;
                })
                .addTemporalMarker(() ->{
                    slideTarget = 70;
                })
                .waitSeconds(1)
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Open);
                })
                .addTemporalMarker(() ->{
                    armTarget= 20;
                    slideTarget = 40;
                    lift=-10;
                    turn = 0;
                })
                .forward(5)
                .setReversed(false)

                .splineTo(new Vector2d(-34,27),Math.toRadians(270))
                .waitSeconds(0.1)
                //.setReversed(true)

                .splineTo(new Vector2d(-43,13),Math.toRadians(180))
                //.strafeTo(new Vector2d(-42,10))


                .splineTo(new Vector2d(-43,50),Math.toRadians(90))

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-48,13),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-55,50),Math.toRadians(90))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(-60,13),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-62,45),Math.toRadians(90))

                .setVelConstraint(velConstraint)
                //

                .forward(5)
                .waitSeconds(0.1)

                .resetConstraints()
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Close);
                })
                .waitSeconds(0.7)
                .addTemporalMarker(() ->{
                    armTarget = 90;
                    lift = 60;
                })
                .splineTo(new Vector2d(-10,30),Math.toRadians(270))
                .addTemporalMarker(() ->{
                    slideTarget = 70;//拉上去
                })
                .waitSeconds(1)
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Open);
                })
                .addTemporalMarker(() ->{
                    armTarget= 20;
                    slideTarget = 40;
                    lift=-10;
                    turn = 0;
                })
                .forward(0.1)

                .splineTo(new Vector2d(-40,40),Math.toRadians(90))//夾第二科

                //_________________________________________________________
                .setVelConstraint(velConstraint)

                .forward(5)
                .resetConstraints()
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Close);
                })
                .waitSeconds(0.7)
                .addTemporalMarker(() ->{
                    armTarget = 90;
                    lift = 60;
                })
                .splineTo(new Vector2d(-12,30),Math.toRadians(270))
                .addTemporalMarker(() ->{
                    slideTarget = 70;//拉上去
                })
                .waitSeconds(1)
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Open);
                })
                .addTemporalMarker(() ->{
                    armTarget= 20;
                    slideTarget = 40;
                    lift=-10;
                    turn = 0;
                })
                .forward(0.1)

                .splineTo(new Vector2d(-40,40),Math.toRadians(90))//夾第三科
                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
        telemetry.addData("Arm Po000000000000000000000000000000000000000000000","");
        telemetry.update();



    }

    @Override
    protected void robotInitLoop() {
        armPosNow = armL.getCurrentPosition() / arm2deg; // 讀取手臂當前角度
        //armTurn2angle(45);                       // 將手臂維持在目標角度
        armTurn2angle(armTarget);
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


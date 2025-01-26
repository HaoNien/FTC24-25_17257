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
public class AUTOMonkeyNew extends robotBase{
    //Pose2d startPose = new Pose2d(-8, 60, Math.toRadians(90));

    @Override
    protected void robotInit() {

        armPowerMax=0.6;


        Pose2d startPose = new Pose2d(-8, 60, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        armTarget= armStartAngle;
        slideTarget = 40;
        lift=-90;
        turn = 0;
        Claw.setPosition(claw_Close);
         TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(15),
                new AngularVelocityConstraint(1)
        ));

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
//                .lineTo(new Vector2d(0,28))
//                .lineTo(new Vector2d(-5,55))
                .addTemporalMarker(() ->{
                    lift = 80;
                })
                .splineToLinearHeading(new Pose2d(0,28,Math.toRadians(90)),Math.toRadians(270))
                //put
                .addTemporalMarker(() ->{
                    armPowerMax=0.8;
                    turn = 0;
                    armTarget = 81;
                    slideTarget = 43;
                    lift = 80;
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->{
                    slideTarget = 85;
                })
                .waitSeconds(0.7)
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Open);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() ->{
                    armPowerMax=0.6;
                    armTarget= 20;
                    slideTarget = 40;
                    lift=-20;
                    turn = 0;
                })
                .waitSeconds(0.2)
                ////////


                .splineToConstantHeading(new Vector2d(-10,40),Math.toRadians(180))
                .lineTo(new Vector2d(-18,40))
                .splineToConstantHeading(new Vector2d(-32,30),Math.toRadians(270))
                .lineTo(new Vector2d(-32,20))


                .splineToLinearHeading(new Pose2d(-43,13,Math.toRadians(90)),Math.toRadians(180))//推第一顆

                .lineTo(new Vector2d(-43,47))
                .lineTo(new Vector2d(-43,20))
                .splineToLinearHeading(new Pose2d(-53,13,Math.toRadians(90)),Math.toRadians(180))//推第一顆
                .lineTo(new Vector2d(-53,40))
//                .waitSeconds(0.3)
                //-----------------------------two
                .setVelConstraint(velConstraint)
                .lineTo(new Vector2d(-53,52))
                .resetVelConstraint()
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Close);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() ->{
                    slideTarget = 40;
                    armTarget= 45;
                })

                .splineToConstantHeading(new Vector2d(-40,40),Math.toRadians(0))
                .lineTo(new Vector2d(-10,40))
                .splineToConstantHeading(new Vector2d(2,27),Math.toRadians(270))
                .setReversed(false)

                //put
                .addTemporalMarker(() ->{
                    armPowerMax=0.8;
                    turn = 0;
                    armTarget = 81;
                    slideTarget = 46;
                    lift = 80;
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->{
                    slideTarget = 85;
                })
                .waitSeconds(0.7)
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Open);
                })
                .waitSeconds(0.2)

                .addTemporalMarker(() ->{
                    armPowerMax=0.6;
                    armTarget= 20;
                    slideTarget = 40;
                    lift=-20;
                    turn = 0;
                })
                .waitSeconds(0.5)
                ////////

                //------------------------three


                .splineToConstantHeading(new Vector2d(-15,38),Math.toRadians(180))


                .lineTo(new Vector2d(-30,38))
                .splineToConstantHeading((new Vector2d(-45,35)),Math.toRadians(90))
//                .waitSeconds(0.3)
                .setVelConstraint(velConstraint)
                .lineTo(new Vector2d(-45,52))
                .resetVelConstraint()

                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Close);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() ->{
                    slideTarget = 40;
                    armTarget= 45;
                })
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-20,40),Math.toRadians(0))
                .lineTo(new Vector2d(-10,40))
                .splineToConstantHeading(new Vector2d(-3,27),Math.toRadians(270))
                .setReversed(false)

                //put
                .addTemporalMarker(() ->{
                    armPowerMax=0.8;
                    turn = 0;
                    armTarget = 81;
                    slideTarget = 46;
                    lift = 80;
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->{
                    slideTarget = 85;
                })
                .waitSeconds(0.7)
                .addTemporalMarker(() ->{
                    Claw.setPosition(claw_Open);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() ->{
                    armPowerMax=0.6;
                    armTarget= 20;
                    slideTarget = 70;
                    lift=90;
                    turn = 0;
                })
                .waitSeconds(0.5)
                ////////

                //park
                .splineTo(new Vector2d(-50,60),Math.toRadians(180))
                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
        telemetry.addData("Arm Po000000000000000000000000000000000000000000000","");
        telemetry.update();



    }

    @Override
    protected void robotInitLoop() {
        armPosNow = armL.getCurrentPosition() / arm2deg; // 讀取手臂當前角度
        //armTurn2angle(45);                       // 將手臂維持在目標角度
        armTurn2angle(armStartAngle);
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



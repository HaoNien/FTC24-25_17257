package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(

                new TranslationalVelocityConstraint(10),
                new AngularVelocityConstraint(1)
        ));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 30, 5,5, 13)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(55, 55, Math.toRadians(225)))
                        .splineToLinearHeading(new Pose2d(50,9,Math.toRadians(180)),Math.toRadians(260))
                        .splineToLinearHeading(new Pose2d(26,9,Math.toRadians(180)),Math.toRadians(180))
//                        .setReversed(true)
//                        .addTemporalMarker(() ->{
//
//                        })
//                        .lineTo(new Vector2d(-5,55))
//                        .splineToLinearHeading(new Pose2d(0,28,Math.toRadians(90)),Math.toRadians(270))
//                        .addTemporalMarker(() ->{
//
//                        })
//                        .waitSeconds(0.1)
//                        .addTemporalMarker(() ->{
//
//                        })
//                        .waitSeconds(0.05)
//                        .addTemporalMarker(() ->{
//
//                        })
//                        .waitSeconds(0.2)
//
//
//                        //.lineTo(new Vector2d(-32,40))
////                .splineTo(new Vector2d(-20,45),Math.toRadians(180))
////
////                .splineToLinearHeading(new Pose2d(-45,8,Math.toRadians(90)),Math.toRadians(240))//推第一顆
////
////                .splineToLinearHeading(new Pose2d(-43,40,Math.toRadians(90)),Math.toRadians(90))
////
////
////                .lineTo(new Vector2d(-55,10))
////                .splineToConstantHeading(new Vector2d(-55,45),Math.toRadians(90))
////
////                .lineTo(new Vector2d(-58,15))
////
////
////                .splineToConstantHeading(new Vector2d(-63,30),Math.toRadians(90))
//                        //.waitSeconds(0.7)
//
//
//                        .splineToConstantHeading(new Vector2d(-10,40),Math.toRadians(180))
//                        .lineTo(new Vector2d(-20,40))
//                        .splineToConstantHeading(new Vector2d(-35,30),Math.toRadians(270))
//
//
//                        .lineTo(new Vector2d(-35,20))
//
//                        .splineToLinearHeading(new Pose2d(-40,13,Math.toRadians(90)),Math.toRadians(180))//推第一顆
//
//                        .lineTo(new Vector2d(-40,45))
//                        .lineTo(new Vector2d(-40,20))
//                        .splineToLinearHeading(new Pose2d(-50,13,Math.toRadians(90)),Math.toRadians(180))//推第一顆
//                        .lineTo(new Vector2d(-50,45))
////
////                .splineToConstantHeading(new Vector2d(-50,10),Math.toRadians(160))
////                .splineToConstantHeading(new Vector2d(-60,40),Math.toRadians(90))
////                .waitSeconds(0.3)
//
//                        //-----------------------------two
//
//                        .lineTo(new Vector2d(-50,50))
//
//                        .addTemporalMarker(() ->{
//
//                        })
//                        .waitSeconds(0.2)
//                        .addTemporalMarker(() ->{
//                          //  Claw.setPosition(claw_Close);
//                        })
//                        .waitSeconds(0.3)
////                .back(6)
////                .waitSeconds(0.5)
//                        .addTemporalMarker(() ->{
//                            //armTarget= 30;
//                        })
//                        .waitSeconds(0.2)
//
//                        .waitSeconds(0.2)
//                        .splineToConstantHeading(new Vector2d(-40,40),Math.toRadians(0))
//                        .lineTo(new Vector2d(-10,40))
//                        .splineToConstantHeading(new Vector2d(-2,28),Math.toRadians(270))
//                        .addTemporalMarker(() ->{
//                            //
//                        })
//                        //.UNSTABLE_addTemporalMarkerOffset(0.2,()->{
//                            //armTarget= 95;
//
//
//                        //.waitSeconds(0.2)
//                        .lineTo(new Vector2d(-3,28))
//                        .addTemporalMarker(() ->{
//              //
//                        })
//                        .setReversed(false)
//                        .waitSeconds(0.2)
//                        .addTemporalMarker(() ->{
//                            //lift = 90;
//                        })
//                        .waitSeconds(0.2)
//                        .addTemporalMarker(() ->{
//                      //      Claw.setPosition(claw_Open);
//                        })
//                        .addTemporalMarker(() ->{
//
//                        })
//
//
//                        //------------------------three
//
//                        .splineToConstantHeading(new Vector2d(-15,40),Math.toRadians(180))
//
//
//                        .lineTo(new Vector2d(-30,40))
//                        .splineToConstantHeading((new Vector2d(-35,45)),Math.toRadians(90))
//
//                        //.setVelConstraint(velConstraint)
//                        .lineTo(new Vector2d(-35,50))
//                        .setReversed(false)
//                        //.resetVelConstraint()
//                        .addTemporalMarker(() ->{
//
//                        })
//                        .waitSeconds(0.3)
//                        .addTemporalMarker(() ->{
//                        //    Claw.setPosition(claw_Close);
//                        })
//                        .waitSeconds(0.5)
//                        .addTemporalMarker(() ->{
//                            //armTarget= 15;
//                        })
//                        .waitSeconds(0.3)
//                        .setReversed(true)
//                        .splineToConstantHeading(new Vector2d(-20,40),Math.toRadians(0))
//                        .lineTo(new Vector2d(-10,40))
//                        .splineToConstantHeading(new Vector2d(-3,28),Math.toRadians(270))
//                        .addTemporalMarker(() ->{
//
//                        })
//
//                        .addTemporalMarker(() ->{
//
//                        })
//                        .waitSeconds(0.3)
//
//                        .setReversed(false)
//                        .addTemporalMarker(() ->{
//
//                        })
//                        .waitSeconds(0.5)
//                        .addTemporalMarker(() ->{
//
//                        })
//
//
//                        //park
//                        .splineTo(new Vector2d(-50,60),Math.toRadians(180))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
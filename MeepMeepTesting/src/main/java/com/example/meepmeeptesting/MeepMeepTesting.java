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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-8, 60, Math.toRadians(90)))
                        .setReversed(true)
                        .back(27)
                        .forward(5)
                        //.turn(Math.toRadians(-45))
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
                        .splineToConstantHeading(new Vector2d(-62,40),Math.toRadians(90))
                        //.waitSeconds(0.5)
                        .setVelConstraint(velConstraint)
                        //

                        .forward(8)
                        .resetConstraints()
                        .waitSeconds(0.1)
                        .setReversed(true)
                        .splineTo(new Vector2d(-4,30),Math.toRadians(270))
                        .waitSeconds(0.1)
                        .forward(0.1)
                        //.setReversed(true)
                        .splineTo(new Vector2d(-40,45),Math.toRadians(90))//夾第二科
                        .forward(5)


                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AUTOServo extends robotBase{
    //Pose2d startPose = new Pose2d(-8, 60, Math.toRadians(90));

    @Override
    protected void robotInit() {
        motorResetDone=false;
        motorReset();

        //armTarget= 45;
        slideTarget = 40;



        telemetry.addData("","");
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

        //drive.update();
        armTurn2angle(armTarget);
        slideToPosition(slideTarget);
        wristToPosition(lift, turn);



    }
}

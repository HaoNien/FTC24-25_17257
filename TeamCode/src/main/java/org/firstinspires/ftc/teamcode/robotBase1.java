package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Disabled
public class robotBase1 extends LinearOpMode {
    protected ServoImplEx test;
    public static double ttp=0.5;
    @Override
    public void runOpMode() throws InterruptedException{
        test= hardwareMap.get(ServoImplEx.class,"tt");

        waitForStart();

        while (opModeIsActive()){
        test.setPosition(ttp);}
    }
}

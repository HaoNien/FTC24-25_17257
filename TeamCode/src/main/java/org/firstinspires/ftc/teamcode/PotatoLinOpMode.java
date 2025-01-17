
package org.firstinspires.ftc.teamcode;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = " PotatoLinOpMode", group = "Linear OpMode")
public class PotatoLinOpMode extends robotBase {
    private final ElapsedTime runtime = new ElapsedTime();


    public static double slide_Speed = 10;
    public static double arm_Speed = 5;
    private long lastTime = System.currentTimeMillis();
    private int loopCount = 0;
    int loopCountHZ = 0;
    double clawPos = 0.5;

    // 定義兩個獨立的狀態機
    private enum RobotStateB {
        STATE_1, STATE_2, STATE_3, STATE_4, STATE_5, STATE_6, STATE_7
    }

    private enum RobotStateY {
        STATE_1, STATE_2, STATE_3, STATE_4, STATE_5, STATE_6, STATE_7,STATE_8
    }

    private enum RobotStateX {
        STATE_1, STATE_2, STATE_3, STATE_4, STATE_5, STATE_6, STATE_7, STATE_8
    }

    private enum RobotStateA {
        STATE_1, STATE_2
    }

    boolean togglePressed = false; // 防抖動變數

    private RobotStateB currentStateB = RobotStateB.STATE_7;
    private RobotStateY currentStateY = RobotStateY.STATE_7;

    private RobotStateX currentStateX = RobotStateX.STATE_7;

    private RobotStateA currentStateA = RobotStateA.STATE_1;

    private boolean wasButtonPressedB = false, stateExecutedB = true;
    private boolean wasButtonPressedY = false, stateExecutedY = true;
    private boolean wasButtonPressedX = false, stateExecutedX = true;

    private boolean wasButtonPressedA = false, stateExecutedA = true;


    public void robotInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        wristToPosition(-90, 0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


    }

    @Override
    protected void robotInitLoop() {
        armTurn2angle(45);                       // 將手臂維持在目標角度

        telemetry.addData("Arm Position", armPosNow);
        telemetry.addData("Target Angle", armTarget);
        telemetry.update();
    }

    @Override
    public void robotStart() {

        long currentTime = System.currentTimeMillis();
        loopCount++; // 每次迴圈執行時增加計數器(看更新頻率而已



        // 管理按鈕的狀態機
        manageStateMachineB();
        manageStateMachineY();
        manageStateMachineX();
        manageStateMachineA();

        if (gamepad2.left_bumper) Claw.setPosition(claw_Open);//夾子打開
        if (gamepad2.right_bumper) Claw.setPosition(claw_Close);

        //吊掛模式
        if (gamepad2.left_stick_button && !togglePressed) {
            isHangingMode = !isHangingMode; // 切換模式
            togglePressed = true; // 防抖動
        } else if (!gamepad2.left_stick_button) {
            togglePressed = false; // 重置防抖動
        }

        //手臂伸縮
        double gp2_r_Y = gamepad2.right_stick_y;

        if (gp2_r_Y > 0.1 || gp2_r_Y < -0.1)
            slideTarget = slidePosNow + (-gp2_r_Y * slide_Speed);
        slideTarget = clamp(slideTarget,smin,smax);

        slideToPosition(slideTarget);


        //手臂上下
        double gp2_l_Y = -gamepad2.left_stick_y;

        if (gp2_l_Y < -0.3 || gp2_l_Y > 0.3) armTarget = armPosNow - (gp2_l_Y * arm_Speed);
        armTarget = clamp(armTarget, armBottomLimit, armUpLimit);

        armTurn2angle(armTarget);


        //夾子抬起
        if (gamepad2.dpad_up||gamepad1.dpad_up) lift += 5;
        else if (gamepad2.dpad_down||gamepad1.dpad_down) lift -= 5;
        lift =clamp(lift, lift_Mini, lift_Max);
        //夾子旋轉
        if (gamepad2.dpad_left||gamepad1.dpad_left) turn += 10;
        else if (gamepad2.dpad_right||gamepad1.dpad_right) turn -= 10;
        turn = clamp(turn, turn_Mini, turn_Max);

        wristToPosition(lift, turn);

        //底盤遙控
        double gp1ly = -gamepad1.left_stick_y;
        double gp1lx = -gamepad1.left_stick_x;
        double gp1rx = -gamepad1.right_stick_x;
        double gp1ltr = gamepad1.left_trigger;
        double gp1rtr = gamepad1.right_trigger;

        double axial, lateral, yaw;
        if (Math.abs(gp1ly) < 0.7 && Math.abs(gp1ly) > 0.1) axial = gp1ly / 2;
        //else if (Math.abs(gp1ly) < 0.1) axial=0;
        else axial = gp1ly;
        if (Math.abs(gp1lx) < 0.7 && Math.abs(gp1lx) > 0.1) lateral = gp1lx / 2;
        //else if (Math.abs(gp1lx) < 0.1) lateral=0;
        else lateral = gp1lx;
        if (Math.abs(gp1rx) < 0.7 && Math.abs(gp1rx) > 0.1)
            yaw = (gp1rx) + (gp1ltr / 3) - (gp1rtr / 3);
        //else if (Math.abs(gp1lx) < 0.1) yaw =0;
        else yaw = gp1rx*1.33 + (gp1ltr / 3) - (gp1rtr / 3);
        drive.setDrivePower(new Pose2d(axial, lateral, yaw));


        telemetry.addData("Loop Frequency", "%d Hz", loopCountHZ);
        // 每隔一秒計算一次頻率
        if (currentTime - lastTime >= 1000) {
            loopCountHZ = loopCount;


            loopCount = 0; // 重置計數器
            lastTime = currentTime; // 重置時間戳
        }

        //螢幕顯示數值
        telemetry.addData("slideCM", slidePosNow);
        telemetry.addData("slideTAR", slideTarget);
        telemetry.addData("slidePOW", slidePower);
        telemetry.addData("slideAmp", slide.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("armNOW", armPosNow);
        telemetry.addData("armTAR", armTarget);
        telemetry.addData("armPOWER", armOutput);
        telemetry.addData("armF", armF);
        telemetry.addData("armLAmp", armL.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("armRAmp", armR.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("lift", lift);
        telemetry.addData("turn", turn);


        telemetry.update();
    }

    // **B 按鈕狀態機邏輯**
    private void manageStateMachineB() {
        if (gamepad2.b && !wasButtonPressedB) {
            switchStateB();
            wasButtonPressedB = true;
            stateExecutedB = false;
        } else if (!gamepad2.b) {
            wasButtonPressedB = false;
        }

        if (!stateExecutedB) {
            executeStateLogicB();
            stateExecutedB = true;
        }
    }

    private void switchStateB() {
        currentStateY = RobotStateY.STATE_7; // 重置另一個模式的狀態
        currentStateX = RobotStateX.STATE_8; // 重置另一個模式的狀態

        switch (currentStateB) {
            case STATE_1:
                currentStateB = RobotStateB.STATE_2;
                break;
            case STATE_2:
                currentStateB = RobotStateB.STATE_3;
                break;
            case STATE_3:
                currentStateB = RobotStateB.STATE_4;
                break;
            case STATE_4:
                currentStateB = RobotStateB.STATE_5;
                break;
            case STATE_5:
                currentStateB = RobotStateB.STATE_6;
                break;
            case STATE_6:
                currentStateB = RobotStateB.STATE_1;
                break;
            case STATE_7:
                currentStateB = RobotStateB.STATE_1;
                break;
        }
    }

    private void executeStateLogicB() {
        //鉤子模式
        switch (currentStateB) {
            case STATE_1:
                armTarget = 20;
                lift = -20;
                turn = 0;
                slideTarget = 40;
                Claw.setPosition(claw_Open);

                break;
            case STATE_2:
                Claw.setPosition(claw_Close);

                break;
            case STATE_3:
                armTarget = 45;

                break;
            case STATE_4:
                armTarget = 90;
                slideTarget = 52;
                lift = 90;
                break;
            case STATE_5:
                slideTarget = 68;
                break;
            case STATE_6:
                Claw.setPosition(claw_Open);
                break;
            case STATE_7:
                Claw.setPosition(claw_Open);

                break;

        }
    }

    // **Y 按鈕狀態機邏輯**
    private void manageStateMachineY() {
        // 檢查 gamepad1 和 gamepad2 的 y 按鈕
        boolean yPressed = gamepad1.y || gamepad2.y;

        if (yPressed && !wasButtonPressedY) {
            switchStateY();
            wasButtonPressedY = true;
            stateExecutedY = false;
        } else if (!yPressed) {
            wasButtonPressedY = false;
        }

        if (!stateExecutedY) {
            executeStateLogicY();
            stateExecutedY = true;
        }
    }

    private void switchStateY() {
        currentStateB = RobotStateB.STATE_7; // 重置另一個模式的狀態
        currentStateX = RobotStateX.STATE_8; // 重置另一個模式的狀態

        switch (currentStateY) {
            case STATE_1:
                currentStateY = RobotStateY.STATE_3;
                break;
            case STATE_2:
                currentStateY = RobotStateY.STATE_3;
                break;
            case STATE_3:
                currentStateY = RobotStateY.STATE_4;
                break;
            case STATE_4:
                currentStateY = RobotStateY.STATE_5;
                break;
            case STATE_5:
                currentStateY = RobotStateY.STATE_6;
                break;
            case STATE_6:
                currentStateY = RobotStateY.STATE_7;
                break;
            case STATE_7:
                currentStateY = RobotStateY.STATE_8;
                break;
            case STATE_8:
                currentStateY = RobotStateY.STATE_1;
                break;

        }
    }

    private void executeStateLogicY() {

        //高塔模式
        switch (currentStateY) {
            case STATE_1:
                armTarget = 6;
                slideTarget = 70;
                lift = -90;
                turn = 0;
                Claw.setPosition(claw_Open);

                break;
            case STATE_3:
                armTarget = 0;
                break;
            case STATE_4:
                Claw.setPosition(claw_Close);
                break;
            case STATE_5:
                armTarget = 15;
                slideTarget = 40;
                lift = 0;
                turn = 0;

                break;
            case STATE_6:
                armTarget = 90;
                lift = 50;
                turn = 0;
                slideTarget = smax;
                break;
            case STATE_7:
                Claw.setPosition(claw_Open);
                break;
            case STATE_8:
                armTarget = 25;
                slideTarget = 40;
                lift = 0;
                turn = 0;

                break;
        }
    }

    // **X 按鈕狀態機邏輯**
    private void manageStateMachineX() {
        if (gamepad2.x && !wasButtonPressedX) {
            switchStateX();
            wasButtonPressedX = true;
            stateExecutedX = false;
        } else if (!gamepad2.x) {
            wasButtonPressedX = false;
        }

        if (!stateExecutedX) {
            executeStateLogicX();
            stateExecutedX = true;
        }
    }

    private void switchStateX() {
        currentStateY = RobotStateY.STATE_1; // 重置另一個模式的狀態
        currentStateB = RobotStateB.STATE_1; // 重置另一個模式的狀態

        switch (currentStateX) {
            case STATE_1:
                currentStateX = RobotStateX.STATE_2;
                break;
            case STATE_2:
                currentStateX = RobotStateX.STATE_3;
                break;
            case STATE_3:
                currentStateX = RobotStateX.STATE_4;
                break;
            case STATE_4:
                currentStateX = RobotStateX.STATE_5;
                break;
            case STATE_5:
                currentStateX = RobotStateX.STATE_6;
                break;
            case STATE_6:
                currentStateX = RobotStateX.STATE_7;
                break;
            case STATE_7:
                currentStateX = RobotStateX.STATE_8;
                break;
            case STATE_8:
                currentStateX = RobotStateX.STATE_1;
                break;
        }
    }

    private void executeStateLogicX() {


        switch (currentStateX) {
            case STATE_1:
                slideTarget = 70;
                armTarget = 6;
                lift = -90;
                turn = 0;
                Claw.setPosition(claw_Open);

                break;
            case STATE_2:
                Claw.setPosition(claw_Close);

                break;


            case STATE_3:
                armTarget = 10;
                lift = -10;
                turn = 0;
                slideTarget = 40;
                break;

            case STATE_4:
                armTarget = 20;

                Claw.setPosition(claw_Open);
                break;

            case STATE_5:
                Claw.setPosition(claw_Close);

                break;

            case STATE_6:
                slideTarget = 65;
                armTarget = 49;
                lift=-10;

                break;

            case STATE_7:
                armTarget = 35;

                break;

            case STATE_8:
                Claw.setPosition(claw_Open);


                break;
        }
    }// **A 按鈕狀態機邏輯**

    private void manageStateMachineA() {
        // 檢查 gamepad1 和 gamepad2 的 a 按鈕
        boolean aPressed = gamepad1.a || gamepad2.a;

        if (aPressed && !wasButtonPressedA) {
            switchStateA();
            wasButtonPressedA = true;
            stateExecutedA = false;
        } else if (!aPressed) {
            wasButtonPressedA = false;
        }

        if (!stateExecutedA) {
            executeStateLogicA();
            stateExecutedA = true;
        }
    }

    private void switchStateA() {


        switch (currentStateA) {
            case STATE_1:
                currentStateA = RobotStateA.STATE_2;
                break;
            case STATE_2:
                currentStateA = RobotStateA.STATE_1;
                break;

        }
    }

    private void executeStateLogicA() {
        currentStateY = RobotStateY.STATE_2; // 重置另一個模式的狀態

        // 地面收集模式
        switch (currentStateA) {
            case STATE_1:
                armTarget = 6;
                slideTarget = 70;
                lift = -90;

                Claw.setPosition(claw_Open);
                lift = -90;

                break;

            case STATE_2:
                armTarget = 6;
                slideTarget = 70;
                lift = -90;

                Claw.setPosition(claw_Open);
                lift = -90;

                break;


        }
    }


}





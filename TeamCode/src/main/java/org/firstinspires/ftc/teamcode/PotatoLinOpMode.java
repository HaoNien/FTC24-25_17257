
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
@TeleOp(name = "PotatoLinOpMode", group = "Linear OpMode")
public class PotatoLinOpMode extends robotBase {
    private final ElapsedTime runtime = new ElapsedTime();

    // 滑軌與手臂速度參數
    public static double slide_Speed = 10; // 滑軌速度
    public static double arm_Speed = 5; // 手臂速度

    // 時間相關變數
    private long lastTime = System.currentTimeMillis();
    private int loopCount = 0;
    int loopCountHZ = 0;

    double clawPos = 0.5; // 爪子初始位置

    // 狀態機定義
    private enum RobotStateB { STATE_1, STATE_2, STATE_3, STATE_4, STATE_5, STATE_6, STATE_7 }
    private enum RobotStateC { STATE_1, STATE_2 }
    private enum RobotStateY { STATE_1, STATE_2, STATE_3, STATE_4, STATE_5, STATE_6, STATE_7, STATE_8 }
    private enum RobotStateX { STATE_1, STATE_2, STATE_3, STATE_4, STATE_5, STATE_6, STATE_7, STATE_8 }
    private enum RobotStateA { STATE_1, STATE_2 }
    private enum RobotStateHang { STATE_1, STATE_2, STATE_3, STATE_4, STATE_5 }

    // 防抖動變數
    boolean togglePressed = false;
    boolean cirPressd = false;

    // 狀態機初始狀態
    private RobotStateB currentStateB = RobotStateB.STATE_6;
    private RobotStateC currentStateC = RobotStateC.STATE_1;
    private RobotStateY currentStateY = RobotStateY.STATE_7;
    private RobotStateX currentStateX = RobotStateX.STATE_7;
    private RobotStateA currentStateA = RobotStateA.STATE_1;
    private RobotStateHang currentStateHang = RobotStateHang.STATE_5;

    // 按鈕防抖動與執行狀態
    private boolean wasButtonPressedB = false, stateExecutedB = true;
    private boolean wasButtonPressedC = false, stateExecutedC = true;
    private boolean wasButtonPressedY = false, stateExecutedY = true;
    private boolean wasButtonPressedX = false, stateExecutedX = true;
    private boolean wasButtonPressedA = false, stateExecutedA = true;
    private boolean wasButtonPressed1X = false, stateExecuted1X = true;

    @Override
    public void robotInit() {
        armPowerMax = 1; // 設定手臂最大功率

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        wristToPosition(-90, 0); // 初始化手腕位置

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    protected void robotInitLoop() {
        armTurn2angle(60); // 手臂初始化角度
        wristToPosition(90, 0); // 手腕初始化位置
        slideToPosition(40); // 滑軌初始化位置

        // 顯示初始化狀態
        telemetry.addData("Arm Position", armPosNow);
        telemetry.addData("Target Angle", armTarget);
        telemetry.update();
    }

    @Override
    public void robotStart() {
        // 更新手臂與滑軌當前位置
        armPosNow = armEnc.getCurrentPosition() / armEnc2deg + armOffset;
        slidePosNow = (slide.getCurrentPosition() / slide2lenth) * 2 + smin;

        long currentTime = System.currentTimeMillis();
        loopCount++; // 計數迴圈次數

        // 管理按鈕的狀態機
        manageStateMachineB();
        manageStateMachineY();
        manageStateMachineX();
        manageStateMachineA();
        manageStateMachineHang();

        // 控制爪子
        if (gamepad2.left_bumper) Claw.setPosition(claw_Open); // 打開爪子
        if (gamepad2.right_bumper) Claw.setPosition(claw_Close); // 關閉爪子

        // 吊掛模式切換
        if (gamepad2.left_stick_button && !togglePressed) {
            isHangingMode = !isHangingMode;
            togglePressed = true; // 防止抖動
        } else if (!gamepad2.left_stick_button) {
            togglePressed = false;
        }

        // 手臂伸縮控制
        double gp2_r_Y = gamepad2.right_stick_y;
        if (gp2_r_Y > 0.1 || gp2_r_Y < -0.1)
            slideTarget = slidePosNow + (-gp2_r_Y * slide_Speed);
        slideTarget = clamp(slideTarget, smin, smax);

        if (isHangingMode) {
            slide.setPower(gp2_r_Y);
            slideOffset = -slidePosNow + 40;
            slideTarget = 40;
            if (!gamepad1.isRumbling()) gamepad1.runRumbleEffect(effect);
        } else {
            gamepad1.stopRumble();
            slideToPosition(slideTarget);
        }

        // 手臂上下控制
        double gp2_l_Y = -gamepad2.left_stick_y;
        if (gp2_l_Y < -0.3 || gp2_l_Y > 0.3) armTarget = armPosNow - (gp2_l_Y * arm_Speed);
        armTarget = clamp(armTarget, armBottomLimit, armUpLimit);

        if (isHangingMode) {
            armL.setPower(gp2_l_Y);
            armR.setPower(gp2_l_Y);
        } else {
            armTurn2angle(armTarget);
        }

        if (gamepad2.right_stick_button) {
            armOffset = -armPosNow;
            armTarget = 0;
        }

        // 爪子抬起與旋轉控制
        if (gamepad2.dpad_up || gamepad1.dpad_up) lift += 5;
        else if (gamepad2.dpad_down || gamepad1.dpad_down) lift -= 5;
        lift = clamp(lift, lift_Mini, lift_Max);

        if (gamepad2.dpad_left || gamepad1.dpad_left) turn += 10;
        else if (gamepad2.dpad_right || gamepad1.dpad_right) turn -= 10;
        turn = clamp(turn, turn_Mini, turn_Max);

        wristToPosition(lift, turn);

        // 底盤遙控
        double gp1ly = -gamepad1.left_stick_y;
        double gp1lx = -gamepad1.left_stick_x;
        double gp1rx = -gamepad1.right_stick_x;
        double gp1ltr = gamepad1.left_trigger;
        double gp1rtr = gamepad1.right_trigger;

        double axial = gp1ly;
        double lateral = gp1lx;
        double yaw = gp1rx + (gp1ltr / 3) - (gp1rtr / 3);
        drive.setWeightedDrivePower(new Pose2d(axial, lateral, yaw));

        // 顯示迴圈頻率
        telemetry.addData("Loop Frequency", "%d Hz", loopCountHZ);
        if (currentTime - lastTime >= 1000) {
            loopCountHZ = loopCount;
            loopCount = 0;
            lastTime = currentTime;
        }

        // 顯示數據
        telemetry.addData("slideCM", slidePosNow);
        telemetry.addData("slideTAR", slideTarget);
        telemetry.addData("slidePOW", slidePower);
        telemetry.addData("slideOffset", slideOffset);
        telemetry.addData("slideAmp", slide.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("armNOW", armPosNow);
        telemetry.addData("armTAR", armTarget);
        telemetry.addData("armPOWER", armOutput);
        telemetry.addData("armOffset", armOffset);

        telemetry.addData("armF", armF);
        telemetry.addData("armLAmp", armL.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("armRAmp", armR.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("lift", lift);
        telemetry.addData("turn", turn);
        telemetry.update();
    }

    // 狀態機管理函數略（保持完整邏輯）
    // **B 按鈕狀態機邏輯**背勾夾
    private void manageStateMachineC() {
        if (gamepad1.circle && !wasButtonPressedC) {
            switchStateC();
            wasButtonPressedC = true;
            stateExecutedC = false;
        } else if (!gamepad1.circle) {
            wasButtonPressedC = false;
        }

        if (!stateExecutedC) {
            executeStateLogicC();
            stateExecutedC = true;
        }
    }

    private void switchStateC() {
        switch (currentStateC) {
            case STATE_1:
                currentStateC = RobotStateC.STATE_2;
                break;
            case STATE_2:
                currentStateC = RobotStateC.STATE_1;
                break;
        }
    }

    private void executeStateLogicC() {
        //鉤子模式
        switch (currentStateC) {
            case STATE_1:
                Claw.setPosition(claw_Open);
                break;
            case STATE_2:
                Claw.setPosition(claw_Close);
                break;
        }
    }

    // **B 按鈕狀態機邏輯**背勾夾
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
        boolean yPressed = gamepad1. right_bumper|| gamepad2.y;

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

        boolean isGamepad2YPressed = gamepad2.y;

        switch (currentStateY) {
            case STATE_1:
                currentStateY = isGamepad2YPressed ? RobotStateY.STATE_4 : RobotStateY.STATE_3;
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
                armTarget = 8;
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
                armTarget = 20;
                slideTarget = 40;
                lift = 0;
                turn = 0;

                break;
        }
    }

    // **X 按鈕狀態機邏輯**
    private void manageStateMachineX() {
        boolean xPress=gamepad1.left_bumper||gamepad2.x;
        if (xPress && !wasButtonPressedX) {
            switchStateX();
            wasButtonPressedX = true;
            stateExecutedX = false;
        } else if (!xPress) {
            wasButtonPressedX = false;
        }

        if (!stateExecutedX) {
            executeStateLogicX();
            stateExecutedX = true;
        }
    }

    private void switchStateX() {
        currentStateY = RobotStateY.STATE_1; // 重置另一個模式的狀態
        currentStateB = RobotStateB.STATE_6; // 重置另一個模式的狀態

        boolean isGamepad2XPressed = gamepad2.x;

        switch (currentStateX) {
            case STATE_1:

                currentStateX = isGamepad2XPressed ? RobotStateX.STATE_3 : RobotStateX.STATE_2;

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

                currentStateX =RobotStateX.STATE_7;
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
                armTarget = 8;
                lift = -90;
                turn = 0;
                Claw.setPosition(claw_Open);

                break;

            case STATE_2:

                armTarget = 0;
                break;
            case STATE_3:
                Claw.setPosition(claw_Close);

                break;


            case STATE_4:
                armTarget = 10;
                lift = -10;
                turn = 0;
                slideTarget = 40;
                break;

            case STATE_5:
                armTarget = 20;

                Claw.setPosition(claw_Open);
                break;

            case STATE_6:
                Claw.setPosition(claw_Close);

                break;

            case STATE_7:
                slideTarget = 65;
                armTarget = 40;
                lift=50;

                break;

//            case STATE_7:
//                armTarget = 35;
//
//                break;

            case STATE_8:
                Claw.setPosition(claw_Open);


                break;
        }
    }
    // **A 按鈕狀態機邏輯**

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
        currentStateX = RobotStateX.STATE_1;
        // 地面收集模式
        switch (currentStateA) {
            case STATE_1:
                armTarget = 8;
                slideTarget = 70;
                lift = -90;

                Claw.setPosition(claw_Open);
                lift = -90;

                break;

            case STATE_2:
                armTarget = 8;
                slideTarget = 70;
                lift = -90;

                Claw.setPosition(claw_Open);
                lift = -90;

                break;


        }
    }
    private void manageStateMachineHang() {
        // 檢查 gamepad1 和 gamepad2 的 a 按鈕
        //boolean aPressed = gamepad1.a || gamepad2.a;

        if (gamepad1.x && !wasButtonPressed1X) {
            switchStateHang();
            wasButtonPressed1X = true;
            stateExecuted1X = false;
        } else if (!gamepad1.x) {
            wasButtonPressed1X = false;
        }

        if (!stateExecuted1X) {
            executeStateLogic1X();
            stateExecuted1X = true;
        }
    }

    private void switchStateHang() {


        switch (currentStateHang) {
            case STATE_1:
                currentStateHang = RobotStateHang.STATE_2;
                break;
            case STATE_2:
                currentStateHang = RobotStateHang.STATE_3;
                break;

            case STATE_3:
                currentStateHang = RobotStateHang.STATE_4;
                break;

            case STATE_4:
                currentStateHang = RobotStateHang.STATE_5;
                break;
            case STATE_5:
                currentStateHang = RobotStateHang.STATE_1;
                break;


        }
    }

    private void executeStateLogic1X() {

        // 地面收集模式
        switch (currentStateHang) {
            case STATE_1:
                armTarget = 95;
                slideTarget = 70;
                lift = -90;

                Claw.setPosition(claw_Open);


                break;

            case STATE_2:
                slide_f_coeff=slideF_hang;
                slideTarget=40;



                break;
            case STATE_3:
                armPowerMin=-1;
                armD=0;
                arm_f_coeff=arm_f_hang;
                armTarget=10;
                armP=armP_hang;



                break;
            case STATE_4:
                armPowerMin=-0.4;
                armD=0.004;
                armP=0.1;
                arm_f_coeff=arm_ff;
                slide_f_coeff=0;


                break;
            case STATE_5:

                break;


        }
    }


}





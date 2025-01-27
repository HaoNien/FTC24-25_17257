package org.firstinspires.ftc.teamcode;

// 導入所需的類別和靜態函數
import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import androidx.core.content.pm.PermissionInfoCompat;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import java.util.List;

@Config
public abstract class robotBase extends LinearOpMode {
    // _____________底盤程式在drive_/SampleMecanumDrive___________
    protected SampleMecanumDrive drive;

    //----------------定義直流馬達-----------------
    protected DcMotorEx slide; // 升降滑軌馬達
    protected DcMotorEx armR; // 右側手臂馬達
    protected DcMotorEx armL; // 左側手臂馬達

    //----------------定義伺服馬達-----------------
    protected Servo FrontL, FrontR; // 手腕控制伺服馬達 (左/右)
    protected Servo Claw; // 爪子控制伺服馬達

    //-------------外掛Encoder--------------
    protected Encoder armEnc; // 手臂的外掛編碼器

    //-------------Limelight 相機--------------
    protected Limelight3A limelight; // Limelight 相機，用於目標追蹤

    //-------------PID 控制器---------------
    private PIDController ArmPID = new PIDController(0, 0, 0); // 手臂 PID 控制器
    private PIDController SlidePID = new PIDController(0, 0, 0); // 升降滑軌 PID 控制器

    /*------------------手臂參數-----------------------*/

    //-----------------手臂目標角度------------------
    public static double armTarget = 49;

    //-----------------手臂 PID 參數------------------
    public static double armP = 0.1; // 比例增益
    public static double armP_hang = 2; // 吊掛時比例增益
    public static double armI = 0.1; // 積分增益
    public static double armD = 0.004; // 微分增益
    public static double armF = 0; // 前饋控制
    public static double arm_f_coeff = 0.0053; // 前饋係數

    public static double arm_ff = 0.0053; // 前饋係數

    public static double arm_f_hang=-0.3;


    //-----------------手臂現在位置------------------
    public static double armPosNow; // 手臂當前角度

    //-----------------手臂限制------------------
    public static double armUpLimit = 95; // 上限角度
    public static double armBottomLimit = 0; // 下限角度

    //-----------------手臂電力限制------------------
    public static double armPowerMax = 0.6; // 最大電力
    public static double armPowerMin = -0.4; // 最小電力

    //-----------------手臂編碼器轉角度換算------------------
    public static double arm2deg = 6.27; // 角度換算常數
    public static double armEnc2deg = 8192 / 360; // 編碼器值換算角度
    public static double armStartAngle = 49; // 起始角度
    public static double armOffset = 0; // 偏移量

    public static double armOutput;

    /*------------------滑軌 PIDF 參數-----------------------*/
    public static double slideP = 0.5; // 滑軌比例增益
    public static double slideI = 0; // 滑軌積分增益
    public static double slideD = 0; // 滑軌微分增益

    public static double slideF_hang = -0.3; // 吊掛滑軌前饋
    public static double slideFF = 0; // 滑軌前饋
    public static double slide_f_coeff = 0; // 滑軌前饋係數

    public static double slide_motorEnc = 103.8; // 滑軌馬達編碼器常數
    public static double slide_Ratio = 1 / 1.4; // 傳動比
    public static double slide2lenth = slide_motorEnc * slide_Ratio / 0.8; // 編碼器值轉長度

    public double slidePower;

    //-----------------滑軌限制------------------
    public static double smax = 98; // 最大高度
    public static double smax0 = 76; // 最大初始高度
    public static double smin = 40; // 最小高度
    public static double slideOffset = 0; // 滑軌偏移

    //-----------------滑軌當前位置------------------
    public static double slidePosNow = 0; // 滑軌當前高度
    public double slideTarget = 0; // 滑軌目標高度

    /*-----------------手腕----------------------*/
    public static double gear_ratio = 2.888888; // 手腕齒輪比
    public static double currentAngleRight; // 右側當前角度
    public static double tarAngleLeft = 0; // 左側目標角度
    public static double tarAngleRight = 0; // 右側目標角度

    //-----------------爪子開關角度------------------
    public static double claw_Open = 0.7; // 爪子開啟角度
    public static double claw_Close = 0.39; // 爪子關閉角度

    //-----------------伺服馬達角度限制------------------
    public static double lift_Offset = -10, turn_Offset = -20; // 手腕偏移量
    public static double lift_Max = 90, lift_Mini = -90; // 升降最大/最小角度
    public static double turn_Max = 90, turn_Mini = -90; // 旋轉最大/最小角度

    //-----------------伺服馬達角度常數------------------
    public static double maxServoAngleLeft = 270.078; // 左伺服馬達最大角度
    public static double maxServoAngleRight = 270.078; // 右伺服馬達最大角度

    //-----------------伺服馬達目標位置------------------
    public static double tarPositionLeft, tarPositionRight; // 左/右伺服馬達目標位置

    public boolean isHangingMode = false; // 吊掛模式（預設為普通模式）

    Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 1000) // 右馬達全速震動 1000 毫秒
            .addStep(0.0, 0.0, 1000) // 暫停 1000 毫秒
            .build();

    double lift = 0, turn = 0; // 手腕升降與旋轉控制變量
    public static boolean motorResetDone = false; // 馬達重置狀態

    @Override
    public void runOpMode() {
        // 初始化硬體元件
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); // 設置批量緩存模式
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap); // 初始化驅動系統

        // 初始化直流馬達
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        armL = hardwareMap.get(DcMotorEx.class, "armL");
        armR = hardwareMap.get(DcMotorEx.class, "armR");

        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        armL.setDirection(DcMotorSimple.Direction.REVERSE);
        armR.setDirection(DcMotorSimple.Direction.FORWARD);

        // 初始化伺服馬達
        FrontL = hardwareMap.get(Servo.class, "frontl");
        FrontR = hardwareMap.get(Servo.class, "frontr");
        Claw = hardwareMap.get(Servo.class, "claw");

        // 初始化手臂編碼器
        armEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "armR"));
        armEnc.setDirection(Encoder.Direction.REVERSE);

        // 如果馬達尚未重置，執行重置
        if (!motorResetDone) {
            motorReset();
        }

        // 初始化馬達參數
        slide.setPower(0);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armR.setPower(0);
        armL.setPower(0);
        armL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 初始化 Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); // 切換到目標識別管道
        limelight.start();

        robotInit(); // 執行自定義初始化邏輯

        telemetry.addData("init", "done");
        telemetry.update();

        // 等待比賽開始
        while (!isStarted() && !isStopRequested()) {
            robotInitLoop(); // 自定義初始化迴圈
            telemetry.addData("Status", "Waiting for start...");
            telemetry.update();
        }

        // 比賽進行中
        while (opModeIsActive()) {
            robotStart(); // 自定義執行邏輯
        }

        limelight.stop(); // 停止 Limelight
    }

    // 抽象方法，供子類實現具體邏輯
    protected abstract void robotInit();
    protected abstract void robotInitLoop();
    protected abstract void robotStart();

    // 手臂轉動到指定角度
    public void armTurn2angle(double target) {
        armPosNow = armEnc.getCurrentPosition() / armEnc2deg + armOffset; // 計算當前角度
        target = clamp(target, armBottomLimit, armUpLimit); // 限制目標角度範圍
        armF = Math.cos(Math.toRadians(armPosNow)) * slidePosNow * arm_f_coeff; // 計算前饋
        ArmPID.setPID(armP, armI, armD); // 設置 PID
        ArmPID.setTolerance(10); // 設置誤差容忍度
        armOutput = ArmPID.calculate(armPosNow, target) + armF; // 計算輸出
        armOutput = clamp(armOutput, armPowerMin, armPowerMax); // 限制輸出範圍
        armL.setPower(armOutput);
        armR.setPower(armOutput);
    }

    // 滑軌移動到指定位置
    public void slideToPosition(double slidePos) {
        slidePosNow = (slide.getCurrentPosition() / slide2lenth) * 2 + smin; // 計算當前位置
        slidePos = Math.max(Math.min(slidePos, smax), smin); // 限制目標範圍
        SlidePID.setPID(slideP, slideI, slideD); // 設置 PID
        slidePower = SlidePID.calculate(slidePosNow, slidePos) + slide_f_coeff; // 計算輸出
        slide.setPower(slidePower);
    }

    // 手腕移動到指定位置
    public void wristToPosition(double liftAng, double turnAng) {
        turnAng /= gear_ratio; // 考慮齒輪比
        turnAng = clamp(turnAng, turn_Mini, turn_Max);
        liftAng = clamp(liftAng, lift_Mini, lift_Max);
        turnAng -= turn_Offset;
        liftAng -= lift_Offset;
        tarAngleLeft = liftAng + (maxServoAngleLeft / 2);
        tarAngleRight = liftAng - turnAng + (maxServoAngleRight / 2);
        tarPositionLeft = tarAngleLeft / maxServoAngleLeft;
        tarPositionRight = tarAngleRight / maxServoAngleRight;
        FrontL.setPosition(1 - tarPositionLeft);
        FrontR.setPosition(tarPositionRight);
    }

    // 將 Limelight 輸出轉換為 Pose2d
    public Pose2d convertToPose2d(String telemetryOutput) {
        String positionData = telemetryOutput.split("position=\\(")[1].split("\\)")[0];
        String orientationData = telemetryOutput.split("yaw=")[1].split(",")[0];
        String[] positionParts = positionData.split(" ");
        double xMeters = Double.parseDouble(positionParts[0]);
        double yMeters = Double.parseDouble(positionParts[1]);
        double yawDegrees = Double.parseDouble(orientationData);
        double xInches = xMeters * 39.3701;
        double yInches = yMeters * 39.3701;
        double yawRadians = Math.toRadians(yawDegrees);
        return new Pose2d(xInches, yInches, yawRadians);
    }

    // 計算角度誤差
    private double calculateAngleError(double target, double current) {
        double error = target - current;
        if (error > 180) error -= 360; // 誤差大於 180 時，取反方向
        if (error < -180) error += 360; // 誤差小於 -180 時，取反方向
        return error;
    }

    // 重置馬達編碼器
    public void motorReset() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorResetDone = true;
    }
}

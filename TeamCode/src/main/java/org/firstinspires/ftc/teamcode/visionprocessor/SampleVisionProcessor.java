package org.firstinspires.ftc.teamcode.visionprocessor;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class SampleVisionProcessor implements VisionProcessor {

    private Mat hsvMat = new Mat();
    private Mat mask = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    private RotatedRect targetRect = null;
    private double targetAngle = 0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // 初始化時可以執行的設定，這裡可以留空。
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // 1. 將影像從 RGB 轉換為 HSV 色彩空間
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        // 2. 篩選特定顏色 (以紅色為例)
        Scalar lowerRed = new Scalar(0, 100, 100);
        Scalar upperRed = new Scalar(10, 255, 255);
        Core.inRange(hsvMat, lowerRed, upperRed, mask);

        // 3. 偵測輪廓
        contours.clear();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        targetRect = null;
        targetAngle = 0;

        // 4. 找到最大輪廓並計算角度
        double maxArea = 0;
        for (MatOfPoint contour : contours) {
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            double area = rect.size.width * rect.size.height;
            if (area > maxArea) {
                maxArea = area;
                targetRect = rect;
                targetAngle = rect.angle;

                // 確保角度範圍在 0-180 度
                if (rect.size.width < rect.size.height) {
                    targetAngle += 90;
                }
            }
        }

        return targetAngle; // 返回計算出的目標角度
    }

    @Override
    public void onDrawFrame(android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (targetRect != null) {
            // 繪製矩形與角度
            android.graphics.Paint paint = new android.graphics.Paint();
            paint.setColor(android.graphics.Color.GREEN);
            paint.setStyle(android.graphics.Paint.Style.STROKE);
            paint.setStrokeWidth(scaleCanvasDensity * 4);

            Point[] points = new Point[4];
            targetRect.points(points);

            for (int i = 0; i < 4; i++) {
                Point start = points[i];
                Point end = points[(i + 1) % 4];
                canvas.drawLine((float) start.x * scaleBmpPxToCanvasPx, (float) start.y * scaleBmpPxToCanvasPx,
                        (float) end.x * scaleBmpPxToCanvasPx, (float) end.y * scaleBmpPxToCanvasPx, paint);
            }

            // 顯示角度
            android.graphics.Paint textPaint = new android.graphics.Paint();
            textPaint.setColor(android.graphics.Color.WHITE);
            textPaint.setTextSize(scaleCanvasDensity * 20);
            canvas.drawText("Angle: " + targetAngle,
                    (float) targetRect.center.x * scaleBmpPxToCanvasPx,
                    (float) targetRect.center.y * scaleBmpPxToCanvasPx,
                    textPaint);
        }
    }

    public double getTargetAngle() {
        return targetAngle;
    }
}

package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class BarCodeDetection extends OpenCvPipeline {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

    Mat mat = new Mat();
    Mat mat1 = new Mat();
    Mat mat2 = new Mat();
    Color pixel;
    public Scalar lowHSV;
    public Scalar highHSV;
    public Scalar lowHSV1;
    public Scalar highHSV1;

    public enum BarcodePosition {
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND
    }

    public enum Color {
        RED,
        BLUE
    }

    private BarcodePosition barcodePosition = BarcodePosition.NOT_FOUND;

     static final Rect LEFT_ROW = new Rect(
            new Point( 80, 180 ),
            new Point( 125, 230 )
    );
    static final Rect MIDDLE_ROW = new Rect(
            new Point( 310, 175 ),
            new Point( 340, 210 )
    );
     static final Rect RIGHT_ROW = new Rect(
            new Point( 520, 170 ),
            new Point( 560, 230 )
    );

    static double PERCENT_COLOR_THRESHOLD = 0.2;

    public BarCodeDetection(Telemetry t, Color color) {
        telemetry = t;
        pixel = color;
    }

    @Override
    public Mat processFrame( Mat input ) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        if( pixel == Color.RED ) {
            lowHSV = new Scalar(0, 80, 50);
            highHSV = new Scalar(50, 255, 255);

            lowHSV1 = new Scalar(357, 95, 42);
            highHSV1 = new Scalar(179, 255, 255);

            Core.inRange(mat, lowHSV1, highHSV1, mat1);
        } else {
            lowHSV = new Scalar(60, 85, 40);
            highHSV = new Scalar(120, 255, 255);

            lowHSV1 = new Scalar(209, 93, 68);
            highHSV1 = new Scalar(235, 91, 90);

            Core.inRange(mat, lowHSV1, highHSV1, mat1);
        }

        Core.inRange(mat, lowHSV, highHSV, mat);
        Core.add(mat, mat1, mat);

        Mat left = mat.submat(LEFT_ROW);
        Mat middle = mat.submat(MIDDLE_ROW);
        Mat right = mat.submat(RIGHT_ROW);

        double leftValue = Core.sumElems( left ).val[0] / LEFT_ROW.area( ) / 255;
        double middleValue = Core.sumElems( middle ).val[0] / MIDDLE_ROW.area( ) / 255;
        double rightValue = Core.sumElems( right ).val[0] / RIGHT_ROW.area( ) / 255;

        left.release();
        middle.release();
        right.release();

        boolean leftBool = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean middleBool = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean rightBool = rightValue > PERCENT_COLOR_THRESHOLD;

        if( rightBool ) {
            barcodePosition = BarcodePosition.RIGHT;
            telemetry.addData( "Location",  " right" );
        } else if( leftBool ) {
            barcodePosition = BarcodePosition.LEFT;
            telemetry.addData( "Location",  " left" );
        } else if( middleBool ) {
            barcodePosition = BarcodePosition.MIDDLE;
            telemetry.addData( "Location",  " middle" );
        } else {
            barcodePosition = BarcodePosition.NOT_FOUND;
            telemetry.addData( "Location",  " not found" );
        }
        Imgproc.cvtColor( mat, mat, Imgproc.COLOR_GRAY2RGB );

        Scalar elementColor = new Scalar( 255, 0, 0 );
        Scalar notElement = new Scalar( 0, 255, 0 );

        Imgproc.rectangle( mat, LEFT_ROW, barcodePosition == BarcodePosition.LEFT ? notElement : elementColor );
        Imgproc.rectangle( mat, RIGHT_ROW, barcodePosition == BarcodePosition.RIGHT ? notElement : elementColor );
        Imgproc.rectangle( mat, MIDDLE_ROW, barcodePosition == BarcodePosition.MIDDLE ? notElement : elementColor );
        return mat;
    }

    public BarcodePosition getBarcodePosition( ) {
        return barcodePosition;
    }
}

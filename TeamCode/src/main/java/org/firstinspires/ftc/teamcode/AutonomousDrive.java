package org.firstinspires.ftc.teamcode;

//import com.google.blocks.ftcrobotcontroller.runtime.DcMotorAccess;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Blue Duck And Parking", group = "BLUE", preselectTeleOp = "Controller")
//@Disabled

public class AutonomousDrive extends LinearOpMode {
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;

    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    public ElapsedTime runTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        driveByEncoder(0.5, 0, 100, 0);
    }

    public void driveByEncoder(double power, double angleDegrees, int CM, int angleAheadDegrees) {
        // 9.6 CM Koter
        // 9.6 * PI = Hekef
        // Hekef = 30.16
        // encoder for one round = 537.6
        // (30.16 / 537.6) * encoder = CM
        // 0.0561 * encoder = CM
        // encoder = CM / 0.0561
        double encoder = CM / 0.0561;

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runTime.reset();

        double y = -(power * Math.sin(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double x = -(power * Math.cos(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double rx = -(angleAheadDegrees - getCurrentAngleFromIMU()) * 0.01;
//         Denominator is the largest motor power (absolute value) or 1
//         This ensures all the powers maintain the same ratio, but only when
//         at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        double frontRightEncoder = encoder;
        double backRightEncoder = encoder;

        boolean motorFrontRightEncoderIsNotFinished = Math.abs(motorFrontRight.getCurrentPosition()) < frontRightEncoder;
//        boolean motorBackRightEncoderIsNotFinished = Math.abs(motorBackRight.getCurrentPosition()) < backRightEncoder;


//        while (motorFrontRightEncoderIsNotFinished || motorBackRightEncoderIsNotFinished) {
        while (motorFrontRightEncoderIsNotFinished) {

                drive(power, angleDegrees,angleAheadDegrees);
//            rx = -(angleAheadDegrees - getCurrentAngleFromIMU()) * 0.01;
//            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
////
//            if (motorFrontRight.getCurrentPosition() == 0 && motorBackRight.getCurrentPosition() == 0 && runTime.seconds() > 1) {
//                stopRobot();
//            }
//            frontLeftPower = (y + x + rx) / denominator;
//            backLeftPower = (y - x + rx) / denominator;
//            frontRightPower = (y - x - rx) / denominator;
//            backRightPower = (y + x - rx) / denominator;
//
//            motorFrontLeft.setPower(frontLeftPower);
//            motorBackLeft.setPower(backLeftPower);
//            motorFrontRight.setPower(frontRightPower);
//            motorBackRight.setPower(backRightPower);
            motorFrontRightEncoderIsNotFinished = Math.abs(motorFrontRight.getCurrentPosition()) < frontRightEncoder;
        }
        stopRobot();
    }

    private void stopRobot() {
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    public double getCurrentAngleFromIMU() {


        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) {

            deltaAngle += 360;

        } else if (deltaAngle > 180) {

            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;

        lastAngles = angles;

        return -globalAngle;
    }
    public void drive(double power, double angle, double rotation) {

        double y = -(power * Math.sin(angle));
        double x = -(power * Math.cos(angle));
        double rx = -rotation;


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
    }
}
//package org.firstinspires.ftc.teamcode;
//
//public class Controller {
//}
//


package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "SingleController", group = "BTJ")
//@Disabled
public class Controller extends LinearOpMode {

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;


    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;

    DcMotor motorArm;
    DcMotor motorPumping;

    CRServo duckRight, duckLeft;
    Servo drop, tokenServo, pushUpServoRight, pushUpServoLeft;

    double power;
//    double duckPower = 0;
    double driverAngle;
    double rotation;
    double targetAngle;
    double dropPosition = 0;
    double pumpPower = 0;
    double tokenArmPosition = 0;
    double armPower = 0;
    int armPosition = 0;
    int bottomLevel = 0;
    int middleLevel = 1000;
    int topLevel = 2000;
    double pushUpServoRightPosition = 1;
    double pushUpServoLeftPosition = 0;


    boolean bPressed = false;
    boolean xPressed = false;
    boolean a1Pressed = false;
    boolean right_stick_buttonPressed = false;
    boolean left_stick_buttonPressed = false;
    // boolean right_bumperPressed = false;


    private final ElapsedTime timerServoDrop = new ElapsedTime();
    private final ElapsedTime timerServoDucks = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.mode = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled = false;
//
//        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//        // and named "imu".
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//        imu.initialize(parameters);

        // Declare our motors
        // Make sure your ID's match your configuration


        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");


        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);


        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorPumping.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        while (opModeIsActive()) {

//            if(!motorArm.isBusy()){
//                motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                armPower = 0;
//            }
//            motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Start drive part /*******************************************************************/
//            motorArm.setPower(armPower);
//            motorArm.setTargetPosition(armPosition);


            drive(power, targetAngle, rotation);


//            motorPumping.setPower(pumpPower);

//            tokenServo.setPosition(tokenArmPosition);


            power = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));

            driverAngle = Math.atan(-gamepad1.left_stick_y / gamepad1.left_stick_x);

            rotation = gamepad1.right_stick_x;


            if (Double.isNaN(driverAngle)) {

                driverAngle = 0;
            }


            if (gamepad1.left_stick_x < 0) {

                driverAngle += Math.PI;
            }


            targetAngle = driverAngle; //- (-Math.toRadians(getAngle()));

            //End drive part /*********************************************************************/

            //Start action part /******************************************************************/
//            telemetry.addData("encoder", motorArm.getCurrentPosition());
//            telemetry.addData("encoder", motorFrontRight.getCurrentPosition());
//            telemetry.addData("armPosition", armPosition);
//            telemetry.addData("tokenPosition", tokenArmPosition);
//            telemetry.addData("runtime", timerServoDrop);
//            telemetry.addData("runtime", timerServoDucks);
            telemetry.update();

//            if(gamepad1.left_bumper){
//                timerServoDrop.reset();
//            }
            //Duck part
//            if (gamepad1.x && !xPressed) {
//                xPressed = true;
//                duckPower = Math.min(1, 1 - duckPower);
//            }
//            if (!gamepad1.x) {
//                xPressed = false;
//            }
//
//            if (gamepad1.b && !bPressed) {
//                bPressed = true;
//                duckPower = Math.max(-1, -1 - duckPower);
//            }
//            if (!gamepad1.b) {
//                bPressed = false;
//            }
//
//
//            // Pump
//            if (gamepad1.a && !a1Pressed) {
//                a1Pressed = true;
//                pumpPower = 1 - pumpPower;
//            }
//            if (!gamepad1.a) {
//                a1Pressed = false;
//            }
//            if (gamepad1.right_stick_button && !right_stick_buttonPressed) {
//                pumpPower = -1;
//            }
//            if (!gamepad1.right_stick_button && !gamepad1.a) {
//                pumpPower = 0;
//                right_stick_buttonPressed = false;
//            }


            //Push Up
//            if(gamepad1.left_stick_button && !left_stick_buttonPressed){
//                left_stick_buttonPressed = true;
//                if(pushUpServoLeftPosition == 0.65 || pushUpServoLeftPosition == 0){
//                    pushUpServoLeftPosition = 0.65 - pushUpServoLeftPosition;
//                    pushUpServoRightPosition =  1.5 - pushUpServoRightPosition;
//                }
//                else {
//                    pushUpServoLeftPosition = 0.65;
//                    pushUpServoRightPosition = 0.5;
//                }
//            }
//            if(!gamepad1.left_stick_button) {
//                left_stick_buttonPressed = false;
//            }




            //Elevator


            //End action part /********************************************************************/
        }
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

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;


        lastAngles = angles;

        return -globalAngle;
    }
}
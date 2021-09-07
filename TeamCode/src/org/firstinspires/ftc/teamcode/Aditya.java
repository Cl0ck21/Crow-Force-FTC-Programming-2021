package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Aditya's teleop", group = "Aditya")
public class Aditya extends LinearOpMode {

    // State used for updating telemetry
    Orientation angles;

    BNO055IMU imu;

    double xPower;
    double yPower;

    double hyp = 0;

    double a = 0;

    double angCentric = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor topLeft = hardwareMap.dcMotor.get("front_left_motor"); //declaring motors and telling it where the motors are on the in the cortex
        DcMotor topRight = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bottomLeft = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor bottomRight = hardwareMap.dcMotor.get("back_right_motor");

        // Our sensors, motors, and other devices go here, along with other long term state
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        Servo servo = hardwareMap.servo.get("back_servo"); //declaring servo

        topLeft.setDirection(DcMotor.Direction.REVERSE); //reversing motors
        bottomLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart(); //method to wait until we press start to run the following code
        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            if (gamepad1.left_stick_x < 0.1 && gamepad1.left_stick_x > -0.1 && gamepad1.left_stick_y < 0.1 && gamepad1.left_stick_y > -0.1 && gamepad1.right_stick_x < 0.1 && gamepad1.right_stick_x > -0.1) {
                topLeft.setPower(0);
                topRight.setPower(0);
                bottomLeft.setPower(0);
                bottomRight.setPower(0);
            } else if (gamepad1.right_stick_x < 0.1 && gamepad1.right_stick_x > -0.1) {
                xPower = (gamepad1.left_stick_x * Math.cos(-(Math.PI/4 + angles.firstAngle)) + gamepad1.left_stick_y * Math.sin(-(Math.PI/4 + angles.firstAngle)));
                yPower = (gamepad1.left_stick_x * Math.sin(-(Math.PI/4 + angles.firstAngle)) - gamepad1.left_stick_y * Math.cos(-(Math.PI/4 + angles.firstAngle)));

                //xPower = -(gamepad1.left_stick_x * Math.cos(-Math.PI / 4) - gamepad1.left_stick_y * Math.sin(-Math.PI / 4));
                //yPower = -(gamepad1.left_stick_x * Math.sin(-Math.PI / 4) + gamepad1.left_stick_y * Math.cos(-Math.PI / 4));

                topLeft.setPower(xPower);
                topRight.setPower(yPower);
                bottomLeft.setPower(yPower);
                bottomRight.setPower(xPower);

                telemetry.addData("xcoords ", gamepad1.left_stick_x);
                telemetry.addData("ycoords ", gamepad1.left_stick_y);
                telemetry.addData("hyp: ", hyp);
                telemetry.addData("angle", a);
                telemetry.addData("angles", angles.firstAngle);
                telemetry.addData("angCentric", angCentric);
                telemetry.addData("topLeft Power", xPower + gamepad1.right_stick_x);
                telemetry.addData("topRight Power", yPower - (-gamepad1.right_stick_x));
                telemetry.addData("botLeft Power", yPower + gamepad1.right_stick_x);
                telemetry.addData("botRight Power", xPower - (-gamepad1.right_stick_x));
                telemetry.addData("y", yPower);
                telemetry.addData("x", xPower);
                telemetry.update();
            } else {
                topLeft.setPower(gamepad1.right_stick_x);
                topRight.setPower(-gamepad1.right_stick_x);
                bottomLeft.setPower(gamepad1.right_stick_x);
                bottomRight.setPower(-gamepad1.right_stick_x);
            }
        }
    }

    public void turnTo(int desiredAngle){
        DcMotor topLeft = hardwareMap.dcMotor.get("front_left_motor"); //declaring motors and telling it where the motors are on the in the cortex
        DcMotor topRight = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bottomLeft = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor bottomRight = hardwareMap.dcMotor.get("back_right_motor");

        // Our sensors, motors, and other devices go here, along with other long term state
        /*BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu.initialize(parameters);*/

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startTime = System.nanoTime();
        double target = desiredAngle;

        double angle = 0;

        int totalTime = 0;

        double error = 90, P, I, D, kp = 1, ki = 0, kd = 0.1, integral = 0, derivative, correction, t, lastTime = 0, dt = 0.1, lastError = 90;

        while(Math.abs(error) > 3){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = angles.firstAngle;

            t = (double)System.nanoTime()/10;
            if (lastTime != 0){
                dt = t - lastTime;
            }

            error = target - angle;
            integral = ki * ((error - lastError) * dt);
            derivative = kd * ((error - lastError) / dt);

            P = kp * error;
            I = ki * integral;
            D = kd * derivative;

            correction = P + I + D;

            topLeft.setPower(-correction);
            topRight.setPower(correction);
            bottomLeft.setPower(-correction);
            bottomRight.setPower(correction);

            //System.out.println(P + " " + I + " " + D);
            System.out.println(error);
            System.out.println(angles);

            telemetry.addData("Dt", dt);
            telemetry.addData("Error", error);
            telemetry.addData("correction:", correction);
            telemetry.addData("top Left Power", topLeft.getPower());
            telemetry.addData("top Right Power", topRight.getPower());
            telemetry.addData("bottom Left Power", bottomLeft.getPower());
            telemetry.addData("bottom Right Power", bottomRight.getPower());
            telemetry.addData("Angle", angle);
            telemetry.update();

            lastError = error;
            lastTime = t;
            totalTime += t;
        }
        telemetry.addData("Angle", angle);
        telemetry.addData("totalTime: ", totalTime * 10E-9);
        telemetry.update();

        System.out.println("angle: " + angle);
        System.out.println("totalTime: " + (totalTime * 10E-9));

        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);
    }
}

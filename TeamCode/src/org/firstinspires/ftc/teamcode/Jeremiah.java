package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "jeremiah's teleop", group = "jeremiah")
public class Jeremiah extends LinearOpMode {

    Orientation angles;

    double error = 90, P, I, D, kp = 1, ki = 0, kd = 0.1, integral = 0, derivative, correction, t, lastTime = 0, dt = 0.1, lastError = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor backLeft = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor frontRight = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor backRight = hardwareMap.dcMotor.get("back_right_motor");
        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");

        Servo servo = hardwareMap.servo.get("back_servo");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        double xPower;
        double yPower;

        imu.initialize(parameters);

        ElapsedTime et = new ElapsedTime();

        et.milliseconds();

        waitForStart();

        int flag = 0;
        int servoPos = 0;
        double motorMultiplier = 1;

        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            telemetry.addData("1", "");

            if(gamepad1.b) {

                motorMultiplier = 1.5;

            }else if(gamepad1.x) {

                motorMultiplier = 0.5;

            }else if(gamepad1.y) {

                motorMultiplier = 1;

            }

            if (gamepad1.left_stick_x < 0.1 && gamepad1.left_stick_x > -0.1 && gamepad1.left_stick_y < 0.1 && gamepad1.left_stick_y > -0.1 && gamepad1.right_stick_x < 0.1 && gamepad1.right_stick_x > -0.1) {
                telemetry.addData("2", "");
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);

            } else if (gamepad1.right_stick_x < 0.1 && gamepad1.right_stick_x > -0.1) {
                telemetry.addData("3", "");
                xPower = (gamepad1.left_stick_x * Math.cos(-(Math.PI/4 + angles.firstAngle)) + gamepad1.left_stick_y * Math.sin(-(Math.PI/4 + angles.firstAngle)));
                yPower = (gamepad1.left_stick_x * Math.sin(-(Math.PI/4 + angles.firstAngle)) - gamepad1.left_stick_y * Math.cos(-(Math.PI/4 + angles.firstAngle)));

                frontRight.setPower(yPower);
                frontLeft.setPower(xPower);
                backRight.setPower(xPower);
                backLeft.setPower(yPower);

            }else{
                telemetry.addData("4", "");
                frontRight.setPower(-gamepad1.right_stick_x * motorMultiplier);
                frontLeft.setPower(gamepad1.right_stick_x * motorMultiplier);
                backRight.setPower(-gamepad1.right_stick_x * motorMultiplier);
                backLeft.setPower(gamepad1.right_stick_x * motorMultiplier);
            }

            if(gamepad1.a && flag == 0 && servoPos == 0) {

                servo.setPosition(180);
                flag = 1;
                servoPos = 1;

            }else if(gamepad1.a && flag == 0 && servoPos == 1) {

                servo.setPosition(0);
                flag = 1;
                servoPos = 0;

            }else if(!gamepad1.a && flag == 1) {
                flag = 0;
            }



            /*
            topLeft.setPower(0);
            topRight.setPower(0);
            bottomLeft.setPower(0);
            bottomRight.setmepad1.left_stick_x < 0.1 && gak_x > -0.1 && gamepad1.left_sti    pad1.left_stick_y > -0.1 && gamepad1.right_stick_x < 0.1 && gamepad1.right_stick_x > -0.1) {
            }
            
            int encoder = 0; 
            
            while(encoder < 5000) {
                frontLeft.setPower(1);
                frontRight.setPower(1);
                backLeft.setPower(1);
                backRight.setPower(1); 
                encoder = frontLeft.getCurrentPosition();
            }

            turnTo(90);

            while(encoder < 10000) {
                frontLeft.setPower(1);
                frontRight.setPower(1);
                backLeft.setPower(1);
                backRight.setPower(1);
                encoder = frontLeft.getCurrentPosition();
            }

            turnTo(180);

            while(encoder < 15000) {
                frontLeft.setPower(1);
                frontRight.setPower(1);
                backLeft.setPower(1);
                backRight.setPower(1);
                encoder = frontLeft.getCurrentPosition();
            }

            turnTo(-90);

            while(encoder < 20000) {
                frontLeft.setPower(1);
                frontRight.setPower(1);
                backLeft.setPower(1);
                backRight.setPower(1);
                encoder = frontLeft.getCurrentPosition();
            }

            turnTo(0);

            while(encoder < 20000) {
                frontLeft.setPower(1);
                frontRight.setPower(1);
                backLeft.setPower(1);
                backRight.setPower(1);
                encoder = frontLeft.getCurrentPosition();
            }

             while(colorSensor.red() < 170) {

                frontLeft.setPower(1);
                frontRight.setPower(1);
                backLeft.setPower(1);
                backRight.setPower(1);
            }
            et.reset();
            while(et.milliseconds() < 1000) {
                frontLeft.setPower(1);
                frontRight.setPower(-1);
                backLeft.setPower(1);
                backRight.setPower(-1);
                telemetry.addData("encoders", frontLeft.getCurrentPosition());
                telemetry.addData("gyroscope", imu.getAngularOrientation());
                telemetry.addData("red", colorSensor.red());
                telemetry.addData("blue", colorSensor.blue());
                telemetry.update();
            }
            //forwards
            frontLeft.setPower(1);
            frontRight.setPower(1);
            backLeft.setPower(1);
            backRight.setPower(1);
            //backwards
            frontLeft.setPower(-1);
            frontRight.setPower(-1);
            backLeft.setPower(-1);
            backRight.setPower(-1);
            //strafe
            frontLeft.setPower(1);
            frontRight.setPower(-1);
            backLeft.setPower(-1);
            backRight.setPower(1);
            //spin
            frontLeft.setPower(1);
            frontRight.setPower(-1);
            backLeft.setPower(1);
            backRight.setPower(-1);

            //servo
            servo.setPosition(-40);
          */

            telemetry.update();
        }
        }
    //}


    public void turnTo(int desiredAngle) {
        DcMotor topLeft = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor topRight = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bottomLeft = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor bottomRight = hardwareMap.dcMotor.get("back_right_motor");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double startTime = System.nanoTime();
        double target = desiredAngle; 
        
        int totalTime = 0;

        while (error > 3) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            t = (double)System.nanoTime()/10;
            if(lastTime != 0) {
                dt = t -lastTime;
            }

            error = target - angles.firstAngle;
            integral = ki * ((error - lastError) * dt);
            derivative = kd * ((error - lastError) / dt);

            P = kp * error;
            I = ki * error;
            D = kd * error;

            correction = P + I + D;

            topLeft.setPower(-correction);
            topRight.setPower(correction);
            bottomLeft.setPower(-correction);
            bottomRight.setPower(correction);

            System.out.println(error);
            System.out.println(angles);

            telemetry.addData("Dt", dt);
            telemetry.addData("Error", error);
            telemetry.addData("correction:", correction);
            telemetry.addData("top Left Power", topLeft.getPower());
            telemetry.addData("top Right Power", topRight.getPower());
            telemetry.addData("bottom Left Power", bottomLeft.getPower());
            telemetry.addData("bottom Right Power", bottomRight.getPower());
            telemetry.addData("Angle", angles.firstAngle);
            telemetry.update();

            lastError = error;
            lastTime = t;
            totalTime += t;

            telemetry.addData("orientation: ", angles);
            telemetry.addData("totalTime: ", totalTime * 10E-9);
            telemetry.update();



        }
        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);
    }
} //}

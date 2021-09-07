package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Mary Code", group = "Mary")
public class Mary extends LinearOpMode{

    Orientation angles;

    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        //Mecanum Drivetrain Controls
        DcMotor frontLeft = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor frontRight = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor backLeft = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor backRight = hardwareMap.dcMotor.get("back_right_motor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        
        //Encoder set to front left wheel
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo thisIsAServo = hardwareMap.servo.get("back_servo");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        waitForStart();
        while (opModeIsActive()){
            int encoderCount = 0;
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while(encoderCount<5001){
                RobotFullPower();
                encoderCount = frontLeft.getCurrentPosition();
                telemetry.addData("1", "");
                telemetry.addData("Front Left Encoder",frontLeft.getCurrentPosition());
                telemetry.update();
            }
            turnTo(90, imu);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            encoderCount = 0;

            while(encoderCount<5001){
                RobotFullPower();
                encoderCount = frontLeft.getCurrentPosition();
                telemetry.addData("2", "");
                telemetry.addData("Front Left Encoder",frontLeft.getCurrentPosition());
                telemetry.update();
            }
            turnTo(180, imu);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            encoderCount = 0;

            while(encoderCount<5001){
                RobotFullPower();
                encoderCount = frontLeft.getCurrentPosition();
                telemetry.addData("3", "");
                telemetry.addData("Front Left Encoder",frontLeft.getCurrentPosition());
                telemetry.update();
            }
            turnTo(-90, imu);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            encoderCount = 0;

            while(encoderCount<5001){
                RobotFullPower();
                encoderCount = frontLeft.getCurrentPosition();
                telemetry.addData("4", "");
                telemetry.addData("Front Left Encoder",frontLeft.getCurrentPosition());
                telemetry.update();
            }
            turnTo(0, imu);

            break;
        }
    }
    public void turnTo(int desiredAngle, BNO055IMU imu){
        telemetry.addData("turning", "");
        telemetry.update();

        //Motors
        DcMotor topLeft = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor topRight = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bottomLeft = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor bottomRight = hardwareMap.dcMotor.get("back_right_motor");

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startTime = System.nanoTime();
        double target = desiredAngle;

        int totalTime = 0;

        double error = 90, P, I, D, kp = 1, ki = 0, kd = 0.1, integral = 0, derivative, correction, t, lastTime = 0, dt = 0.1, lastError = 90;

        while(Math.abs(error) > 4){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            t = (double)System.nanoTime()/10;
            if (lastTime != 0){
                dt = t - lastTime;
            }
            error = target - angles.firstAngle;

            if (error < 0) {
                error += 360;
            }

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
    public void RobotFullPower() {
        //Motors
        DcMotor topLeft = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor topRight = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bottomLeft = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor bottomRight = hardwareMap.dcMotor.get("back_right_motor");
        

        //Hope this works
        topLeft.setPower(1);
        topRight.setPower(1);
        bottomLeft.setPower(1);
        bottomRight.setPower(1);
        
    }
 }




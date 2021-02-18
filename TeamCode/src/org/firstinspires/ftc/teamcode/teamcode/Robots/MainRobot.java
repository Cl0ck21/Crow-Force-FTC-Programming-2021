package org.firstinspires.ftc.teamcode.teamcode.Robots;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.Subsystems.ForMainRobot.*;
import system.robot.Robot;
import system.robot.localizer.HolonomicDriveEncoderIMULocalizer;
import system.robot.roadrunner_util.RoadrunnerConfig;
import system.robot.subsystems.drivetrain.MecanumDrive;
import util.control.Button;
import util.control.PIDController;
import util.functional_interfaces.BiFunction;
import util.math.units.HALAngleUnit;

import static java.lang.Math.PI;

public class MainRobot extends Robot {

    public MecanumDrive mDrive;
    //public FoundationGrabberSubsystem grabber;
    public EncoderSubsystem distance;
    public AutonomousSelectorSubsystemUsingConfig selector;
    //public IntakeSubSystemServo blockIntakeServo;
    public IntakeSubSystemMotors blockIntakeMotors;
    //public MarkerServoSubsystem markerServo;
    //public opencvSkystoneDetector_v2 openCV;
    //public IntakeSubSystemServoNew2Servos blockIntakeServo;
    public LinearSlidesMotorsSubsystem linearMotors;
    public LinearSlidesServosSubsystem linearServos;
    public CustomOdometryGlobalCoordinatePosition odometry;
    public GoalTargetingSubSystem aimbot;

    public MainRobot(OpMode opMode) {
        super(opMode);

        // Drivetrain to control
        mDrive = new MecanumDrive(
                this,
                new RoadrunnerConfig(2, 1, 15, 1120, 133.9),
                "topLeft",
                "topRight",
                "bottomLeft",
                "bottomRight", false);
        //We also need to set a localizer for mDrive
        // Idk which side to reverse
        mDrive.setReverseType(MecanumDrive.ReverseType.LEFT);

        //Localizer (need imu config)
        mDrive.setLocalizer(new HolonomicDriveEncoderIMULocalizer(
                this,
                mDrive,
                "",
                "topLeft",
                "topRight",
                "bottomLeft",
                "bottomRight"
        ));

        //

        mDrive.setTurnPID(new PIDCoefficients(1, 1, 1));
        mDrive.setHeadingPID(new PIDCoefficients(1, 1, 1));
        //PID tolerance needed?  Heading of 0.1 degrees tolerance
        mDrive.setHeadingPIDTolerance(0.1, HALAngleUnit.DEGREES);

        //probably dont need this
        PIDController dsa = new PIDController(1.6, 0, 0, new BiFunction<Double,Double,Double>() {
            @Override
            public Double apply(Double target, Double current) {
                BiFunction<Double, Double, Double> mod = new BiFunction<Double, Double, Double>() {
                    @Override
                    public Double apply(Double x, Double m) {
                        return (x % m + m) % m;
                    }
                };

                double m = 2 * PI;

                //cw - ccw +
                double cw = mod.apply(mod.apply(current, m) - mod.apply(target, m), m);
                double ccw = -mod.apply(mod.apply(target, m) - mod.apply(current, m), m);

                return Math.abs(ccw) < Math.abs(cw) ? ccw : cw;
            }
        });
        /*PIDController stability = new PIDController(0.055, 0, 0, new BiFunction<Double,Double,Double>() {
            @Override
            public Double apply(Double target, Double current) {
                BiFunction<Double, Double, Double> mod = new BiFunction<Double, Double, Double>() {
                    @Override
                    public Double apply(Double x, Double m) {
                        return (x % m + m) % m;
                    }
                };

                double m = 2 * PI;

                //cw - ccw +
                double cw = mod.apply(mod.apply(current, m) - mod.apply(target, m), m);
                double ccw = -mod.apply(mod.apply(target, m) - mod.apply(current, m), m);

                return Math.abs(ccw) < Math.abs(cw) ? ccw : cw;
            }
        });*/
        dsa.deadband = PI / 90;
        //stability.deadband = 0;


        startGui(new Button(1, Button.BooleanInputs.y));  // etiene sucks
        //grabber = new FoundationGrabberSubsystem(this, "armL", "armR");
        /* need to set up new mechanum drive
        mDrive = new CustomMechinumDrive(this, new CustomMechinumDrive.Params("topLeft", "topRight", "bottomLeft", "bottomRight")
                .setDriveStick(new Button(1, Button.VectorInputs.left_stick))
                .setTurnStick(new Button(1, Button.DoubleInputs.right_stick_x))
                .setConstantSpeedModifier(1)
                .setSpeedModeMultiplier(.5)
                .setSpeedModeButton(new Button(1, Button.BooleanInputs.a))
                .setTurnPID(dsa)
                //.setStabilityPID(stability)
                .setImuNumber(2)); */         //blockIntakeMotor = new IntakeSubSystemMotors(this,"blockIntakeLeft", "blockIntakeRight");
        selector = new AutonomousSelectorSubsystemUsingConfig(this);
        //blockIntakeServo = new IntakeSubSystemServo(this, "blockIntakeServo");
        blockIntakeMotors = new IntakeSubSystemMotors(this, "leftIntake", "rightIntake");
        linearMotors = new LinearSlidesMotorsSubsystem(this, "forwardEncoder", "strafeEncoder");
        linearServos = new LinearSlidesServosSubsystem(this, "blockIntakeServoGrabber", "blockIntakeServoVertical");
        //blockIntakeServo = new IntakeSubSystemServoNew2Servos(this, "servoVertical", "servoGrab");
        distance = new EncoderSubsystem(this, "forwardEncoder", "mDrive");
        //markerServo = new MarkerServoSubsystem(this, "markerOutput");
        //todo update odometry subsystem with correct encoder names and counts per inch
        odometry = new CustomOdometryGlobalCoordinatePosition(this, "fill1", "fill2", "fill3", 600, 50);
        aimbot = new GoalTargetingSubSystem(this, "fill");
        mDrive.getMotors()[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mDrive.getMotors()[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mDrive.getMotors()[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mDrive.getMotors()[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*putSubSystem("MechanumDrive", mDrive);
        putSubSystem("FoundationGrabber", grabber);
        putSubSystem("AutonomousSelector", selector);
        putSubSystem("MotorIntake", blockIntakeMotors);
        putSubSystem("MarkerServo", markerServo);
        putSubSystem("OpenCV", openCV);
        putSubSystem("EncoderSubsystem", distance);
        putSubSystem("LinearSlidesMotorSubsystem", linearMotors);
        putSubSystem("LinearSlidesServosSubsystem", linearServos);
        putSubSystem("IntakeSubSystemServo", blockIntakeServo);
        putSubSystem("OdometryPositionUpdate", odometry);
        putSubSystem("RobotAimingWithOdometry", aimbot);*/
        //putSubSystem("IntakeSubSystemServoNew2Servos", blockIntakeServo);
    }
}
/*
        2 = top right
        3 = bottom left
        0= top left
        1 = bottom right

        */
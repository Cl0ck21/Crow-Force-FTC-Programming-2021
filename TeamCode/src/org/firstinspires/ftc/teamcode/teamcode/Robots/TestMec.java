package org.firstinspires.ftc.teamcode.teamcode.Robots;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.OneTimeUse.TestMecOp;
import system.robot.Robot;

import system.robot.localizer.HolonomicDriveEncoderIMULocalizer;
import system.robot.roadrunner_util.RoadrunnerConfig;
import system.robot.subsystems.drivetrain.MecanumDrive;
import util.control.Button;
import util.control.PIDController;
import util.functional_interfaces.BiFunction;


import static java.lang.Math.PI;

public class TestMec extends Robot {

    public MecanumDrive mDrive;
    public TestMecOp testMec;


    public TestMec(OpMode opMode) {
        super(opMode);
        mDrive = new MecanumDrive(
                this,
                new RoadrunnerConfig(2, 1, 15, 1120, 133.9),
                "topLeft",
                "topRight",
                "bottomLeft",
                "bottomRight", false);
        mDrive.setLocalizer(new HolonomicDriveEncoderIMULocalizer(
                this,
                mDrive,
                "imu",
                "topLeft",
                "topRight",
                "bottomLeft",
                "bottomRight"
        ));
        mDrive.setDriveStick(new Button(1, Button.VectorInputs.left_stick));
        mDrive.setTurnStick(new Button(1, Button.DoubleInputs.right_stick_x));
        mDrive.setAllMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mDrive.setReverseType(MecanumDrive.ReverseType.LEFT);
        PIDController dsa = new PIDController(1.3, 0, 0, new BiFunction<Double,Double,Double>() {
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
        dsa.deadband = PI / 90;

        startGui(new Button(1, Button.BooleanInputs.y));

/*        mDrive = new MechanumDrive(this, new MechanumDrive.Params("topLeft", "topRight", "bottomLeft", "bottomRight")
                .setDriveStick(new Button(1, Button.VectorInputs.left_stick))
                .setTurnStick(new Button(1, Button.DoubleInputs.right_stick_x))
                .setConstantSpeedModifier(1)
                .setSpeedModeMultiplier(.5)
                .setSpeedModeButton(new Button(1, Button.BooleanInputs.a))
                .setTurnPID(dsa)
                .setImuNumber(2));        */

        //blockIntakeMotor = new IntakeSubSystemMotors(this,"blockIntakeLeft", "blockIntakeRight");
        // putSubSystem("MechanumDrive", mDrive);

        // estMec = new TestMecOp(this);
        // putSubSystem("TestMec", testMec);
    }
}
package org.firstinspires.ftc.teamcode.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import system.robot.Robot;
import system.robot.localizer.HolonomicDriveEncoderIMULocalizer;
import system.robot.roadrunner_util.RoadrunnerConfig;
import system.robot.subsystems.drivetrain.MecanumDrive;
import util.control.Button;


public class MotorTest extends Robot {
    public MecanumDrive mDrive;
    /**
     * Constructor for robot.
     *
     * @pacom.SCHSRobotics.HAL9001.system.robot.RobotpMode - The opmode the robot is currently running.
     */
    public MotorTest(OpMode opMode) {
        super(opMode);
        mDrive = new MecanumDrive(
                this,
                new RoadrunnerConfig(2, 1, 15, 1120, 133.9),
                "topLeft",
                "topRight",
                "bottomLeft",
                "bottomRight", false);
        //We also need to set a localizer for mDrive
        // Idk which side to reverse
        mDrive.setDriveStick(new Button(1, Button.VectorInputs.left_stick));
        mDrive.setTurnStick(new Button(1, Button.DoubleInputs.right_stick_x));
        mDrive.setAllMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mDrive.setReverseType(MecanumDrive.ReverseType.LEFT);

        //Localizer (need imu config)
        mDrive.setLocalizer(new HolonomicDriveEncoderIMULocalizer(
                this,
                mDrive,
                "imu",
                "topLeft",
                "topRight",
                "bottomLeft",
                "bottomRight"
        ));
    }
}

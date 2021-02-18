package org.firstinspires.ftc.teamcode.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.ForMainRobot.EncoderSubsystem;
import system.robot.Robot;


public class EncoderTestRobot extends Robot {


    public EncoderSubsystem EncoderTestRobot;


    public EncoderTestRobot(OpMode opMode) {
        super(opMode);

        //EncoderTestRobot = new EncoderSubsystem(this, "bottomRight");
        //skystoneDetector = new opencvSkystoneDetector();
    }
}
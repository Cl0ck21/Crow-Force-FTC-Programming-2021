package org.firstinspires.ftc.teamcode.teamcode.OpModes.OneTimeOp;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.teamcode.Robots.EncoderTestRobot;
import system.config.StandAlone;
import system.robot.BaseTeleop;
import system.robot.Robot;


@StandAlone
@TeleOp(name="EncoderTest")
public class EncoderTest extends BaseTeleop {
    private EncoderTestRobot robot;
    @Override
    protected Robot buildRobot() {
        robot = new EncoderTestRobot(this);
        return robot;
    }

}
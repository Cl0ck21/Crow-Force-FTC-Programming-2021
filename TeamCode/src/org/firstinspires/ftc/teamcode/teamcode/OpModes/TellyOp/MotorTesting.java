package org.firstinspires.ftc.teamcode.teamcode.OpModes.TellyOp;


import org.firstinspires.ftc.teamcode.teamcode.Robots.MotorTest;
import system.robot.BaseTeleop;
import system.robot.Robot;

public class MotorTesting extends BaseTeleop {
    @Override
    protected Robot buildRobot() {
        return new MotorTest(this);
    }
}

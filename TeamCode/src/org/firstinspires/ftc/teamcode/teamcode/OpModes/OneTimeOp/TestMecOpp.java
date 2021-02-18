package org.firstinspires.ftc.teamcode.teamcode.OpModes.OneTimeOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.teamcode.Robots.TestMec;
import system.config.StandAlone;
import system.robot.BaseTeleop;
import system.robot.Robot;

@StandAlone
@TeleOp(name = "TestMecOpp")
public class TestMecOpp extends BaseTeleop {
    TestMec robot;
    @Override
    protected Robot buildRobot() {
        robot = new TestMec(this);
        return robot;
    }

}

package org.firstinspires.ftc.teamcode.teamcode.OpModes.TellyOp;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import system.robot.BaseTeleop;
import system.robot.Robot;

//import org.firstinspires.ftc.teamcode.Robots.opencvRobot;

@TeleOp (name = "OpencvCamera")
public class opencvTelly extends BaseTeleop {
    @Override
    protected Robot buildRobot() {
        return null;
    }
    /*opencvRobot robot;
    @Override
    protected Robot buildRobot() {
        robot = new opencvRobot(this);
        return robot;
    }
    public void update()throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("status: ", robot.camera.check());
            telemetry.update();
        }
    }*/
}

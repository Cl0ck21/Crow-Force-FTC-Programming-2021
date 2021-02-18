package org.firstinspires.ftc.teamcode.teamcode.OpModes.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamcode.Robots.MainRobot;
import system.config.StandAlone;
import system.robot.BaseAutonomous;
import system.robot.Robot;
import util.math.EncoderToDistanceProcessor;
import util.math.geometry.Vector2D;
import util.math.units.HALAngleUnit;
import util.math.units.HALDistanceUnit;


import java.text.DecimalFormat;
import java.io.*;
import java.util.*;

import static java.lang.Math.PI;
import static java.lang.Math.pow;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;


@StandAlone
@Autonomous(name = "Autonomouss")
public class Autonomouss extends BaseAutonomous {
    @system.robot.MainRobot
    private MainRobot robot;
    private EncoderToDistanceProcessor processor = new EncoderToDistanceProcessor(2.5, HALDistanceUnit.INCHES);
    private EncoderToDistanceProcessor cmprocessor = new EncoderToDistanceProcessor(6, HALDistanceUnit.CENTIMETERS);
    // list of common encoder
    //  distances:1tile
    int oneTile = processor.getEncoderAmount(18, HALDistanceUnit.INCHES);
    //2tiles
    int twoTile = processor.getEncoderAmount(35, HALDistanceUnit.INCHES);
    //2.5

    // tiles(right before of intake)

    int blockPos = processor.getEncoderAmount(7, HALDistanceUnit.INCHES);

    private void turnEncoders(double power, int encoder) {
        int encoderStart = robot.distance.fEncoders();
        robot.mDrive.turn(power);
        while (Math.abs(robot.distance.fEncoders() - encoderStart) < encoder) {
            double temp = Math.abs(robot.distance.fEncoders() - encoderStart);
        }
        robot.mDrive.stopAllMotors();
    }

    private void strafeEncoders(double power, int encoder) {
        int encoderStart = robot.distance.sEncoders();
        robot.mDrive.movePower(new Vector2D(power, 0));
        while (Math.abs(robot.distance.sEncoders() - encoderStart) < encoder) {
            double temp = Math.abs(robot.distance.sEncoders() - encoderStart);
            telemetry.addData("Strafe Encoders:", robot.distance.sEncoders());
            telemetry.update();
        }
        robot.mDrive.stopAllMotors();
    }

    private void stopTime() {
        long startTime = System.currentTimeMillis();
        robot.mDrive.stopAllMotors();

        while (System.currentTimeMillis() - startTime < 1000) {
        }
    }

    private void driveEncoders(double power, int encoder) {
        int encoderStart = robot.distance.fEncoders();
        robot.mDrive.movePower(new Vector2D(0, -power));
        telemetry.addData("Before: ", -power);
        telemetry.update();
        while (Math.abs(robot.distance.fEncoders() - encoderStart) < encoder) {
            double temp = Math.abs(robot.distance.fEncoders() - encoderStart);
            telemetry.addData("Forward Encoders:", robot.distance.fEncoders());
            telemetry.addData("Before: ", -power);
            telemetry.update();
        }
        robot.mDrive.stopAllMotors();
    }

    public void nin(int distDif) throws InterruptedException {
        //  45 degrees is 500 ms
        //turnEncoders(0.3, 400 + processor.getEncoderAmount(distDif, HALDistanceUnit.INCHES));
        robot.mDrive.turnPID(PI/2);
    }

    public static double mod(double number, double number2) {
        byte negative = 1;

        if (number < 0) {
            negative = -1;
        }
        number = Math.abs(number);

        while (number > number2) {
            number -= number2;
        }
        return number * negative;
    }

    public void nin() {
        //45 degrees is 500 ms
        turnTo(-Math.PI / 2);
    }

    public void negnin() {
        //turnEncoders(-0.3, 400 + processor.getEncoderAmount(distDif, HALDistanceUnit.INCHES));
        turnTo(Math.PI / 2);
    }

    public String radToDeg(double rads) {
        DecimalFormat df = new DecimalFormat("###.###");
        return df.format(rads * 180 / Math.PI);
    }

    public void turnTo(double angle, boolean yesDegree) {
        if (yesDegree) robot.mDrive.turnPID(angle, HALAngleUnit.DEGREES);
    }

    public void turnTo(double rad) {
        robot.mDrive.turnPID(rad,0.002);
    }

    public void turnToFromAngle(double angle) {
        double ending = (mod(angle + robot.mDrive.getPoseEstimate().getHeading() - Math.PI, 2 * Math.PI)) + Math.PI;
        //telemetry.addLine("Angle: " + radToDeg(angle) + " Ending: " + radToDeg(ending) + " " + radToDeg(angle - ending));
        //telemetry.addLine(radToDeg(startAngle));
        telemetry.update();
        turnTo(ending);

    }


    Vector straight = new Vector(0, 1);
    Vector backwards = new Vector(0, -1);
    Vector left = new Vector(-1, 0);
    Vector right = new Vector(1, 0);
    int specBlock1 = cmprocessor.getEncoderAmount(130, HALDistanceUnit.CENTIMETERS);
    int specBlock2 = cmprocessor.getEncoderAmount(200, HALDistanceUnit.CENTIMETERS);
    int specBlock3 = cmprocessor.getEncoderAmount(210, HALDistanceUnit.CENTIMETERS);
    boolean layout1 = true;
    boolean layout2 = false;
    boolean layout3 = false;
    boolean once = true;
    int i = 0;
    double pow1 = -1;
    double pow2 = 0.5;
    DcMotor topLeft;
    DcMotor topRight;
    DcMotor botRight;
    DcMotor botLeft;
    long startTime = System.currentTimeMillis();
    double startAngle;
    int side = 1;
    String asd;
    Long ree;




    @Override
    public void main() {
        if (once) {
            startAngle = robot.mDrive.getPoseEstimate().getHeading(); //is there still a mDrive method for this? or is it HAL odometry now? should we use connor's odometry?
            once = false;
        }
        //telemetry.setAutoClear(true);
        String position;
        switch (robot.selector.autonomous) {
            // Playback Autonomous from playback telly
            /*case ("Playback Autonomous"):
                // Read from File:
                BufferedReader br = new BufferedReader(new FileReader("playbackTellyOutputs.txt"));
                // Make a Bill Zhang object
                StringTokenizer Bill_Zhang = new StringTokenizer(br.readLine());
                double d1 = Double.parseDouble(Bill_Zhang.nextToken());
                Bill_Zhang = new StringTokenizer(br.readLine());
                double d2 = Double.parseDouble(Bill_Zhang.nextToken());*/




            case ("Strafe Test"):
                //robot.mDrive.driveEncoders(new Vector(-1,0), 4000, true);
                strafeEncoders(pow1, cmprocessor.getEncoderAmount(10, HALDistanceUnit.CENTIMETERS));
                strafeEncoders(-pow1, cmprocessor.getEncoderAmount(10, HALDistanceUnit.CENTIMETERS));
                strafeEncoders(pow1, cmprocessor.getEncoderAmount(10, HALDistanceUnit.CENTIMETERS));
                strafeEncoders(-pow1, cmprocessor.getEncoderAmount(10, HALDistanceUnit.CENTIMETERS));
                strafeEncoders(pow1, cmprocessor.getEncoderAmount(10, HALDistanceUnit.CENTIMETERS));
                strafeEncoders(-pow1, cmprocessor.getEncoderAmount(10, HALDistanceUnit.CENTIMETERS));
                strafeEncoders(pow1, cmprocessor.getEncoderAmount(10, HALDistanceUnit.CENTIMETERS));
                strafeEncoders(-pow1, cmprocessor.getEncoderAmount(10, HALDistanceUnit.CENTIMETERS));
                strafeEncoders(pow1, cmprocessor.getEncoderAmount(10, HALDistanceUnit.CENTIMETERS));
                strafeEncoders(-pow1, cmprocessor.getEncoderAmount(10, HALDistanceUnit.CENTIMETERS));
                break;
            case ("Turn90"):
                //telemetry.addData("", robot.mDrive.getPoseEstimate().getHeading(AngleUnit.DEGREES));
                //telemetry.update();
                turnTo(Math.PI / 2);
                turnTo(0);
                turnTo(Math.PI);
                turnTo(5 * Math.PI / 4);
                break;
            /*case ("OpenCV"):
                position = robot.openCV.check();
                break;*/

            case ("Park"): //remember to change the code for this to the new game
                if (robot.selector.color.equals("Red")) {
                    driveEncoders(pow1, oneTile + 850);
                } else if (robot.selector.color.equals("Blue")) {
                    driveEncoders(pow1, oneTile + 850);
                }

            case ("intakeTest"):
                //robot.blockIntakeServo.intakeDown();

            case ("MaxPoints"):
                //asd = robot.openCV.check();
                //telemetry.addLine(asd);
                telemetry.update();
                ree = System.currentTimeMillis();
                while (System.currentTimeMillis() - ree < 2000) {
                }
                switch (asd) {
                    case "A":
                        //starting position: turned facing the donuts
                        turnTo(side * 0);
                        //1: we want to first rotate towards the center of the field, and launch a donut at each of the goals in the middle
                        //1: turn so the robot is facing the marker thing that is closest to our teams' goal
                        driveEncoders(1, 20);
                        turnTo(side * -25);
                        //2: launch donut 1 at the marker
                        //insert launching method here
                        //3: rotate a lil more to face the middle marker
                        turnTo(side * -35);
                        //4: launch donut 2 at the marker
                        //insert launching method here
                        //5: rotate a lil more to face the farthest marker, or the marker most far from our teams' goal
                        turnTo(side * -45);
                        //6: launch donut 3 at the marker
                        //insert launching method here
                        //2: then we want to drop the wobble goal
                        turnTo(side * 0);
                        //1: then drive forward so we are lined up with area A, while being in front of our goal
                        driveEncoders(1, 500);
                        //2: rotate towards area A
                        turnTo(side * 90);
                        //3: drive forward so the wobble goal is in the area
                        driveEncoders(1, 200);
                        //4: and drop the wobble goal
                        //insert plopper function
                        //3: then we gotta get back to the line and park
                        //1: reverse so we're lined up with the goal
                        //2: strafe to the line and park
                        driveEncoders(-1, 250);
                        strafeEncoders(side * 1, 250);


                    case "B":
                        //starting position: turned facing the donuts
                        turnTo(side * 0);
                        //1: we want to first rotate towards the center of the field, and launch a donut at each of the goals in the middle
                        //1: turn so the robot is facing the marker thing that is closest to our teams' goal
                        driveEncoders(1, 20);
                        turnTo(side * -25);
                        //2: launch donut 1 at the marker
                        //insert launching method here
                        //3: rotate a lil more to face the middle marker
                        turnTo(side * -35);
                        //4: launch donut 2 at the marker
                        //insert launching method here
                        //5: rotate a lil more to face the farthest marker, or the marker most far from our teams' goal
                        turnTo(side * -45);
                        //6: launch donut 3 at the marker
                        //insert launching method here
                        //2: then we want to pick up any donuts on the field and send them into the top goal
                        //1: we want to rotate so we're flush facing our team's goal
                        turnTo(side * 0);
                        //driveEncoders(-0.1, 20);
                        //2: i think the next step should be strafing a lil bit so the donut(s) that we scan at the start are lined up with the intake
                        strafeEncoders(side * 0.5, 50);
                        //3: then drive forward and intake 1 donut
                        driveEncoders(1, 250);
                        //insert intake method here
                        //4: launch da donut into the top goal
                        //insert launching method here
                        //3: then we want to drop the wobble goal
                        //1: then drive forward to area B
                        driveEncoders(1, 500);
                        //2: and drop the wobble goal
                        //insert plopper method here
                        //4: then reverse onto the line thing and park
                        //1: reverse onto the line thing
                        //2: park lol idk
                        driveEncoders(-1, 250);

                    case "C":
                        //starting position: turned facing the donuts
                        turnTo(side * 0);
                        //1: we want to first rotate towards the center of the field, and launch a donut at each of the goals in the middle
                        //1: turn so the robot is facing the marker thing that is closest to our teams' goal
                        driveEncoders(1, 20);
                        turnTo(side * -25);
                        //2: launch donut 1 at the marker
                        //insert launching method here
                        //3: rotate a lil more to face the middle marker
                        turnTo(side * -35);
                        //4: launch donut 2 at the marker
                        //insert launching method here
                        //5: rotate a lil more to face the farthest marker, or the marker most far from our teams' goal
                        turnTo(side * -45);
                        //6: launch donut 3 at the marker
                        //insert launching method here
                        //2: then we want to pick up any donuts on the field and send them into the top goal
                        //1: we want to rotate so we're flush facing our team's goal
                        turnTo(side * 0);
                        //driveEncoders(-0.1, 20);
                        //2: i think the next step should be strafing a lil bit so the donut(s) that we scan at the start are lined up with the intake
                        strafeEncoders(0.5, side * 50);
                        //3: then drive forward and intake 3 donuts
                        driveEncoders(1, 250);
                        //insert intake method here
                        //insert intake method here
                        //insert intake method here
                        //4: launch da donuts into the top goal
                        //insert launching method here
                        //5: intake the last donut
                        //insert intake method here
                        //6: launch da donut into the top goal
                        //insert launching method here
                        //3: then we want to drop the wobble goal
                        //1: drive forward until we're in front of our team's goal
                        driveEncoders(1, 500);
                        //2: then you rotate towards area C
                        turnTo(side * 90);
                        //3: then drive forward into the area
                        driveEncoders(1, 200);
                        //4: drop the wobble goal
                        //insert plopper method here
                        //4: then we have to return to the line thing and park
                        //1: reverse so that we're in front of our goal
                        driveEncoders(-1, 200);
                        //2: strafe to the line and park
                        strafeEncoders(1, side * 500);
                }




                /*if(asd.equals("A")) //if there are 0 donuts
                    if (robot.selector.color.equals("Blue")) {
                        side = -1;
                        //max points blue for A
                        telemetry.update();
                        break;
                    } else {
                        //max points red for A
                        telemetry.update();
                        break;
                    }
                else if(asd.equals("B")) { //if there are <2 donuts
                    if (robot.selector.color.equals("Blue")) {
                        side = -1;
                        //max points blue for B
                        break;
                    } else {
                        //max points red for B
                        telemetry.update();
                        break;
                    }
                }
                else if(asd.equals("C")){ //if there are >2 donuts
                    if (robot.selector.color.equals("Blue")) {
                        side = -1;
                        //max points blue for C
                        telemetry.update();
                        break;
                    } else {
                        //max points red for C
                        telemetry.update();
                        break;
                    }
                }
                else {
                    telemetry.addData("shits displaced ", i);
                    telemetry.update();
                    while (true){

                    }
                } */
            case ("MidOption1"):
                //asd = robot.openCV.check();
                //telemetry.addLine(asd);
                telemetry.update();
                ree = System.currentTimeMillis();
                while (System.currentTimeMillis() - ree < 2000) {
                }
                switch (asd) {
                    case "A":
                        //starting position: turned facing the donuts
                        turnTo(side * 0);
                        //1: we want to first rotate towards the center of the field, and launch a donut at each of the goals in the middle
                        //1: turn so the robot is facing the marker thing that is closest to our teams' goal
                        driveEncoders(1, 20);
                        turnTo(side * -25);
                        //2: launch donut 1 at the marker
                        //insert launching method here
                        //3: rotate a lil more to face the middle marker
                        turnTo(side * -35);
                        //4: launch donut 2 at the marker
                        //insert launching method here
                        //5: rotate a lil more to face the farthest marker, or the marker most far from our teams' goal
                        turnTo(side * -45);
                        //6: launch donut 3 at the marker
                        //insert launching method here
                        //1: we want to rotate so we're flush facing our team's goal
                        turnTo(side * 0);
                        strafeEncoders(side * 1, 200);
                        driveEncoders(1, 500);
                        // insert plopper method here
                        driveEncoders(-1, 100);

                    case "B":
                        //starting position: turned facing the donuts
                        turnTo(side * 0);
                        //1: we want to first rotate towards the center of the field, and launch a donut at each of the goals in the middle
                        //1: turn so the robot is facing the marker thing that is closest to our teams' goal
                        driveEncoders(1, 20);
                        turnTo(side * -25);
                        //2: launch donut 1 at the marker
                        //insert launching method here
                        //3: rotate a lil more to face the middle marker
                        turnTo(side * -35);
                        //4: launch donut 2 at the marker
                        //insert launching method here
                        //5: rotate a lil more to face the farthest marker, or the marker most far from our teams' goal
                        turnTo(side * -45);
                        //6: launch donut 3 at the marker
                        //insert launching method here
                        //1: we want to rotate so we're flush facing our team's goal
                        strafeEncoders(side * 1, 200);
                        driveEncoders(1, 750);
                        turnTo(-side * 90);
                        // insert plopper method here
                        strafeEncoders(-1, -250);

                    case "C":
                        //starting position: turned facing the donuts
                        turnTo(side * 0);
                        //1: we want to first rotate towards the center of the field, and launch a donut at each of the goals in the middle
                        //1: turn so the robot is facing the marker thing that is closest to our teams' goal
                        driveEncoders(1, 20);
                        turnTo(side * -25);
                        //2: launch donut 1 at the marker
                        //insert launching method here
                        //3: rotate a lil more to face the middle marker
                        turnTo(side * -35);
                        //4: launch donut 2 at the marker
                        //insert launching method here
                        //5: rotate a lil more to face the farthest marker, or the marker most far from our teams' goal
                        turnTo(side * -45);
                        //6: launch donut 3 at the marker
                        //insert launching method here
                        //1: we want to rotate so we're flush facing our team's goal
                        strafeEncoders(side * 1, 200);
                        driveEncoders(1, 1000);
                        // insert plopper method here
                        driveEncoders(-1, -500);
                }
            case ("MidOption2"):
                //asd = robot.openCV.check();
                //telemetry.addLine(asd);
                telemetry.update();
                ree = System.currentTimeMillis();
                while (System.currentTimeMillis() - ree < 2000) {
                    switch (asd) {
                        case "A":
                            strafeEncoders(side*1, 200);
                            driveEncoders(1, 200);
                            //4: and drop the wobble goal
                            //insert plopper function
                            //3: then we gotta get back to the line and park
                            //1: reverse so we're lined up with the goal
                            //2: strafe to the line and park
                            driveEncoders(-1, 250);



                        case "B":
                            strafeEncoders(side * -1, 100);
                            //3: then drive forward and intake 1 donut
                            driveEncoders(1, 250);
                            //insert intake method here
                            //4: launch da donut into the top goal
                            //insert launching method here
                            //3: then we want to drop the wobble goal
                            //1: then drive forward to area B
                            driveEncoders(1, 500);
                            //2: and drop the wobble goal
                            //insert plopper method here
                            //4: then reverse onto the line thing and park
                            //1: reverse onto the line thing
                            //2: park lol idk
                            driveEncoders(-1, 250);

                        case "C":

                            //2: i think the next step should be strafing a lil bit so the donut(s) that we scan at the start are lined up with the intake
                            strafeEncoders(-0.5,  100);
                            //3: then drive forward and intake 3 donuts
                            driveEncoders(1, 250);
                            //insert intake method here
                            //insert intake method here
                            //insert intake method here
                            //4: launch da donuts into the top goal
                            //insert launching method here
                            //5: intake the last donut
                            //insert intake method here
                            //6: launch da donut into the top goal
                            //insert launching method here
                            //3: then we want to drop the wobble goal
                            //1: drive forward until we're in front of our team's goal
                            driveEncoders(1, 500);
                            //2: then you rotate towards area C
                            turnTo(side * 90);
                            //3: then drive forward into the area
                            driveEncoders(1, 200);
                            //4: drop the wobble goal
                            //insert plopper method here
                            //4: then we have to return to the line thing and park
                            //1: reverse so that we're in front of our goal
                            driveEncoders(-1, 200);
                            //2: strafe to the line and park
                            strafeEncoders(1, side * 500);
                    }
                }

                    case ("MidOption3"):
                        //asd = robot.openCV.check();
                        //telemetry.addLine(asd);
                        telemetry.update();
                        ree = System.currentTimeMillis();
                        while (System.currentTimeMillis() - ree < 2000) {
                        }
                        switch (asd) {
                            case "A":
                                //starting position: turned facing the donuts
                                turnTo(side*0);
                                //1: we want to first rotate towards the center of the field, and launch a donut at each of the goals in the middle
                                //1: turn so the robot is facing the marker thing that is closest to our teams' goal
                                driveEncoders(1, 20);
                                turnTo(side*-35);
                                //2: launch donut 1 at the marker
                                //insert launching method here
                                //3: rotate a lil more to face the middle marker
                                turnTo(side*-45);
                                //4: launch donut 2 at the marker
                                //insert launching method here
                                //5: rotate a lil more to face the farthest marker, or the marker most far from our teams' goal
                                turnTo(side*-55);
                                //6: launch donut 3 at the marker
                                //insert launching method here
                                //2: then we want to drop the wobble goal
                                turnTo(side*0);
                                strafeEncoders(side*1,200);
                                //1: then drive forward so we are lined up with area A, while being in front of our goal
                                driveEncoders(1, 500);
                                //2: rotate towards area A
                                turnTo(side*-90);
                                //3: drive forward so the wobble goal is in the area
                                driveEncoders(1, 200);
                                //4: and drop the wobble goal
                                //insert plopper function
                                //3: then we gotta get back to the line and park
                                //1: reverse so we're lined up with the goal
                                //2: strafe to the line and park
                                driveEncoders(-1, 250);
                                strafeEncoders(side*-1,250);



                            case "B":
                                //starting position: turned facing the donuts
                                turnTo(side*0);
                                //1: we want to first rotate towards the center of the field, and launch a donut at each of the goals in the middle
                                //1: turn so the robot is facing the marker thing that is closest to our teams' goal
                                driveEncoders(1, 20);
                                turnTo(side*-35);
                                //2: launch donut 1 at the marker
                                //insert launching method here
                                //3: rotate a lil more to face the middle marker
                                turnTo(side*-45);
                                //4: launch donut 2 at the marker
                                //insert launching method here
                                //5: rotate a lil more to face the farthest marker, or the marker most far from our teams' goal
                                turnTo(side*-55);
                                //6: launch donut 3 at the marker
                                //insert launching method here
                                //2: then we want to pick up any donuts on the field and send them into the top goal
                                //1: we want to rotate so we're flush facing our team's goal
                                turnTo(side*0);
                                strafeEncoders(side*1,200);
                                //2: i think the next step should be strafing a lil bit so the donut(s) that we scan at the start are lined up with the intake
                                //3: then we want to drop the wobble goal
                                //1: then drive forward to area B
                                driveEncoders(1, 750);
                                strafeEncoders(side*1,250);
                                //2: and drop the wobble goal
                                //insert plopper method here
                                //4: then reverse onto the line thing and park
                                //1: reverse onto the line thing
                                //2: park lol idk
                                driveEncoders(-1,500);

                            case "C":
                                //starting position: turned facing the donuts
                                turnTo(side*0);
                                //1: we want to first rotate towards the center of the field, and launch a donut at each of the goals in the middle
                                //1: turn so the robot is facing the marker thing that is closest to our teams' goal
                                driveEncoders(1, 20);
                                turnTo(side*-35);
                                //2: launch donut 1 at the marker
                                //insert launching method here
                                //3: rotate a lil more to face the middle marker
                                turnTo(side*-45);
                                //4: launch donut 2 at the marker
                                //insert launching method here
                                //5: rotate a lil more to face the farthest marker, or the marker most far from our teams' goal
                                turnTo(side*-55);
                                //6: launch donut 3 at the marker
                                //insert launching method here
                                //2: then we want to pick up any donuts on the field and send them into the top goal
                                //1: we want to rotate so we're flush facing our team's goal
                                turnTo(side*0);
                                //driveEncoders(-0.1, 20);
                                //3: then we want to drop the wobble goal
                                //1: drive forward until we're in front of our team's goal
                                driveEncoders(1,750);
                                //4: drop the wobble goal
                                //insert plopper method here
                                //4: then we have to return to the line thing and park
                                //1: reverse so that we're in front of our goal
                                driveEncoders(-1,200);
                        }

                    case ("MinPoints"):
                        while (System.currentTimeMillis() - ree < 2000) { }
                        turnTo(side * 30);
                        //asd = robot.openCV.check();
                        //telemetry.addLine(asd);
                        telemetry.update();
                        ree = System.currentTimeMillis();
                        turnTo(-side * 30);
                        switch (asd) {
                            case "A":
                                strafeEncoders(side * 1, 200);
                                driveEncoders(1, 500);
                                // insert plopper method here
                                driveEncoders(-1, 100);

                            case "B":
                                strafeEncoders(side * 1, 200);
                                driveEncoders(1, 750);
                                turnTo(-side * 90);
                                // insert plopper method here
                                strafeEncoders(-1, -250);

                            case "C":
                                strafeEncoders(side * 1, 200);
                                driveEncoders(1, 1000);
                                // insert plopper method here
                                driveEncoders(-1, -500);
                        }
                /*if(asd.equals("A")) //if there are 0 donuts
                    if (robot.selector.color.equals("Blue")) {
                        side = -1;
                        //min points blue for A
                        telemetry.update();
                        break;
                    } else {
                        //min points red for A
                        telemetry.update();
                        break;
                    }
                    else if(asd.equals("B")) { //if there are <2 donuts
                        if (robot.selector.color.equals("Blue")) {
                            side = -1;
                            //max points blue for B
                            break;
                        } else {
                            //max points red for B
                            telemetry.update();
                            break;
                        }
                    }
                    else if(asd.equals("C")){ //if there are >2 donuts
                        if (robot.selector.color.equals("Blue")) {
                            side = -1;
                            //max points blue for C
                            telemetry.update();
                            break;
                        } else {
                            //max points red for C
                            telemetry.update();
                            break;
                        }
                    }
                    else {
                        telemetry.addData("shits displaced ", i);
                        telemetry.update();
                        while (true){

                        }*/
                }
        }
    }


package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/*
 * Carries the robot's pose from Autonomous into TeleOp. Static fields survive between OpModes
 * as long as the robot app isn't restarted. See https://learnroadrunner.com/advanced.html
 */
public class PoseStorage {
    public static Pose2d currentPose = new Pose2d();
}

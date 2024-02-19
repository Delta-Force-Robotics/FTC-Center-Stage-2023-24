package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleTankDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) throws InterruptedException {
        MeepMeep meepMeep = new MeepMeep(800);
        Trajectory traj2;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(73, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -61.5, Math.toRadians(90)))

                                .lineToLinearHeading(new Pose2d(-46, -24.5, Math.toRadians(180)))

                                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-56.5,-11.5,Math.toRadians(180)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-55,-12, Math.toRadians(180)),Math.toRadians(0))

                .lineTo(new Vector2d(-48,-11.5))
                .lineTo(new Vector2d(-44,-11.5))

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,-11.5,Math.toRadians(180)),Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(52,-36,Math.toRadians(180)),Math.toRadians(0))

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-50,-11.5, Math.toRadians(180)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-52,-11.5, Math.toRadians(180)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-50,-11.5, Math.toRadians(180)),Math.toRadians(180))

                .lineTo(new Vector2d(-44,-11.5))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,-11.5,Math.toRadians(180)),Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50.5,-38, Math.toRadians(180)),Math.toRadians(0))
                                .setTangent(130)
                                .splineToLinearHeading(new Pose2d(50.5,-20, Math.toRadians(180)),Math.toRadians(0))


                                /*.lineToLinearHeading(new Pose2d(-53,24.5, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-53,9.6, Math.toRadians(180)))

                                .lineToLinearHeading(new Pose2d(32,9.6, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(48.8,35.5, Math.toRadians(180)))

                                .lineToLinearHeading(new Pose2d(32,9.6, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57,11.5, Math.toRadians(180)))

                                .lineToLinearHeading(new Pose2d(-50, 9.6, Math.toRadians(180)))

                                .lineToLinearHeading(new Pose2d(32, 9.6, Math.toRadians(180)))

                                .lineTo(new Vector2d(48,28.5))*/




                //.lineToLinearHeading(new Pose2d(32, 8, Math.toRadians(175)))

                //.lineToLinearHeading(new Pose2d(48,20, Math.toRadians(175)))
/*
                                .lineToLinearHeading(new Pose2d(-54,-24,Math.toRadians(135)))

                                .lineTo(new Vector2d(-57,-23))
                                .lineToLinearHeading(new Pose2d(-55,-12, Math.toRadians(180)))

                                .lineTo(new Vector2d(32,-12))
                                .lineTo(new Vector2d(49,-30))

                                .lineTo(new Vector2d(32,-15))
                                .lineToSplineHeading(new Pose2d(-59,-15.5, Math.toRadians(190)))

                                .lineToLinearHeading(new Pose2d(-50, -15.5, Math.toRadians(190)))

                                .lineToLinearHeading(new Pose2d(32, -15.5, Math.toRadians(175)))

                                .lineTo(new Vector2d(49.6,-40))

*/
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}
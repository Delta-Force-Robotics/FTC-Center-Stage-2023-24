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
                        drive.trajectorySequenceBuilder(new Pose2d(-38,-58,Math.toRadians(180)))
                                .splineToLinearHeading(new Pose2d(-50,-34, Math.toRadians(180)), Math.toRadians(90))

                                /*.setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-47,-40,Math.toRadians(270)),Math.toRadians(90))
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(-36,-50,Math.toRadians(270)),Math.toRadians(0))
                                .setTangent(Math.toRadians(190))
                                .lineToLinearHeading(new Pose2d(-36,-11,Math.toRadians(270)))

                                .turn(Math.toRadians(-90))

                                .setTangent(Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(24,-11,Math.toRadians(184)))
                                .splineToLinearHeading(new Pose2d(54.3,-25.6, Math.toRadians(184)),Math.toRadians(0))

                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(24,-12, Math.toRadians(184)),Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-58,-13, Math.toRadians(184)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-57, -17, Math.toRadians(195)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-57.5,-13, Math.toRadians(184)), Math.toRadians(180))

                                .lineToLinearHeading(new Pose2d(-53,-13.5, Math.toRadians(182)))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(24,-13.5, Math.toRadians(182)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(54,-36, Math.toRadians(182)),Math.toRadians(0))

                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(24,-12, Math.toRadians(184)),Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-58,-13, Math.toRadians(184)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-57, -17, Math.toRadians(195)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-57.5,-13, Math.toRadians(184)), Math.toRadians(180))

                                .lineToLinearHeading(new Pose2d(-53,-13.5, Math.toRadians(182)))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(24,-13.5, Math.toRadians(182)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(54,-36, Math.toRadians(182)),Math.toRadians(0))

                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(56.5,-11,Math.toRadians(184)),Math.toRadians(0))*/

                                //2+4 Blue

                                /*.setTangent(Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(-31.5,34, Math.toRadians(180)),Math.toRadians(0))

                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-40.7,11,Math.toRadians(182)),Math.toRadians(270))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(24,11, Math.toRadians(182)),Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(51.7,42,Math.toRadians(182)),Math.toRadians(0))

                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(30, 8.3,Math.toRadians(182)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-56,8.3,Math.toRadians(182)), Math.toRadians(180))

                                .splineToLinearHeading(new Pose2d(-56, 12, Math.toRadians(165)), Math.toRadians(180))

                                .lineToLinearHeading(new Pose2d(-51,8, Math.toRadians(182)))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(24,7, Math.toRadians(182)),Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(51.7,31.5,Math.toRadians(182)),Math.toRadians(0))

                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(24, 4.5,Math.toRadians(182)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-56.4,6,Math.toRadians(182)), Math.toRadians(180))

                                .lineToLinearHeading(new Pose2d(-51,5, Math.toRadians(180)))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(24,5, Math.toRadians(180)),Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(52.5,36,Math.toRadians(180)),Math.toRadians(0))*/

                                /*
                                // CSH Blue vvv

                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(-31.5,34, Math.toRadians(180)),Math.toRadians(0))

                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-36,59,Math.toRadians(184)),Math.toRadians(90))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(25,59,Math.toRadians(184)),Math.toRadians(0)) // line????
                                .splineToLinearHeading(new Pose2d(49.5,41.5,Math.toRadians(184)),Math.toRadians(0))

                                // ----- traj same vvv

                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(25,59,Math.toRadians(184)),Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-36,59,Math.toRadians(184)),Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-59.5,38,Math.toRadians(190)))

                                .lineToLinearHeading(new Pose2d(-36,59,Math.toRadians(184)))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(24,59,Math.toRadians(184)),Math.toRadians(0)) // line????
                                .splineToLinearHeading(new Pose2d(49.5,28.5,Math.toRadians(184)),Math.toRadians(0))

                                // Case A ^ | Case B v

                                .setTangent(Math.toRadians(180)) // same
                                .splineToLinearHeading(new Pose2d(25,59,Math.toRadians(184)),Math.toRadians(180)) // same
                                .splineToLinearHeading(new Pose2d(-36,59,Math.toRadians(184)),Math.toRadians(180)) // same
                                .lineToLinearHeading(new Pose2d(-59.5,38,Math.toRadians(190))) // same

                                .lineToLinearHeading(new Pose2d(-36,59,Math.toRadians(184))) // same
                                .setTangent(Math.toRadians(0)) // same
                                .splineToLinearHeading(new Pose2d(24,59,Math.toRadians(184)),Math.toRadians(0)) // line???? same
                                .splineToLinearHeading(new Pose2d(49.5,28.5,Math.toRadians(184)),Math.toRadians(0)) // same

                                // Case B ^ | Case C v

                                .setTangent(Math.toRadians(180)) // same
                                .splineToLinearHeading(new Pose2d(25,59,Math.toRadians(184)),Math.toRadians(180)) //  same
                                .splineToLinearHeading(new Pose2d(-36,59,Math.toRadians(184)),Math.toRadians(180)) // same
                                .lineToLinearHeading(new Pose2d(-59.5,38,Math.toRadians(190))) // same

                                .lineToLinearHeading(new Pose2d(-36,59,Math.toRadians(184))) // same
                                .setTangent(Math.toRadians(0)) // same
                                .splineToLinearHeading(new Pose2d(24,59,Math.toRadians(184)),Math.toRadians(0)) // line???? same
                                .splineToLinearHeading(new Pose2d(49.5,28.5,Math.toRadians(184)),Math.toRadians(0)) // same*/

                                /* CSH Red vvv

                                .lineToLinearHeading(new Pose2d(-40,-59,Math.toRadians(184)))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(25,-59,Math.toRadians(184)),Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(49.5,-28.5,Math.toRadians(184)),Math.toRadians(0))
                                // -------------------------
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(25,-59,Math.toRadians(184)),Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-36,-59,Math.toRadians(184)),Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-59.5,-38,Math.toRadians(170))) //

                                .lineToLinearHeading(new Pose2d(-36,-59,Math.toRadians(184)))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(24,-59,Math.toRadians(184)),Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(49.5,-28.5,Math.toRadians(184)),Math.toRadians(0))

                                // Case B v | Case A ^
                                .setTangent(Math.toRadians(180)) // same
                                .splineToLinearHeading(new Pose2d(25,-59,Math.toRadians(184)),Math.toRadians(180)) //  same
                                .splineToLinearHeading(new Pose2d(-36,-59,Math.toRadians(184)),Math.toRadians(180)) // same
                                .lineToLinearHeading(new Pose2d(-59.5,-38,Math.toRadians(170))) // same

                                .lineToLinearHeading(new Pose2d(-36,-59,Math.toRadians(184))) // same
                                .setTangent(Math.toRadians(0)) // same
                                .splineToLinearHeading(new Pose2d(24,-59,Math.toRadians(184)),Math.toRadians(0)) // same
                                .splineToLinearHeading(new Pose2d(49.5,-28.5,Math.toRadians(184)),Math.toRadians(0)) // same

                                // Case C v | Case B ^

                                .setTangent(Math.toRadians(180)) // same
                                .splineToLinearHeading(new Pose2d(25,-59,Math.toRadians(184)),Math.toRadians(180)) //  same
                                .splineToLinearHeading(new Pose2d(-36,-59,Math.toRadians(184)),Math.toRadians(180)) // same
                                .lineToLinearHeading(new Pose2d(-59.5,-38,Math.toRadians(170))) // same

                                .lineToLinearHeading(new Pose2d(-36,-59,Math.toRadians(184))) // same
                                .setTangent(Math.toRadians(0)) // same
                                .splineToLinearHeading(new Pose2d(24,-59,Math.toRadians(184)),Math.toRadians(0)) // same
                                .splineToLinearHeading(new Pose2d(49.5,-28.5,Math.toRadians(184)),Math.toRadians(0)) // same

                                */

                               /* .lineToLinearHeading(new Pose2d(24,-5, Math.toRadians(177)))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(52,-30, Math.toRadians(177)),Math.toRadians(0))
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(55,-11,Math.toRadians(184)),Math.toRadians(0)) */
                                /*.splineToLinearHeading(new Pose2d(-54,-9, Math.toRadians(184)),Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-58,-12, Math.toRadians(184)),Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-56,-12, Math.toRadians(184)),Math.toRadians(180))

                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(24,-12,Math.toRadians(184)),Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(53,-27, Math.toRadians(184)),Math.toRadians(0))

                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(24,-14, Math.toRadians(184)),Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-59,-16, Math.toRadians(184)),Math.toRadians(180))

                                .lineTo(new Vector2d(-44,-16))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(24,-15, Math.toRadians(184)),Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(52,-35, Math.toRadians(184)),Math.toRadians(0))*/


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
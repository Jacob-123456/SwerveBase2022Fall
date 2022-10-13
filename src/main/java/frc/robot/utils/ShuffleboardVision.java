// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Map;

import javax.swing.text.html.HTMLDocument.HTMLReader.TagAction;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Cameras;

/** Add your docs here. */
public class ShuffleboardVision {

        public ShuffleboardVision() {

        }

        public static void init(Cameras cam) {

                PhotonCamera picam = cam.picam;

                ShuffleboardLayout camLayout = Shuffleboard.getTab("Cameras")
                                .getLayout("CameraLayout", BuiltInLayouts.kList)
                                .withPosition(0, 0)
                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                camLayout.addNumber("ActivePipeline", () -> picam.getPipelineIndex());

                camLayout.addNumber("LatencySecs", () -> cam.latencySeconds);

                camLayout.addBoolean("DriverMode", () -> picam.getDriverMode());

                camLayout.addBoolean("HasTargets", () -> cam.hasTargets);

                camLayout.addNumber("TargetsAvailable", () -> cam.targetsAvailable);

                ShuffleboardLayout t0L;
                ShuffleboardLayout t1L;
                ShuffleboardLayout t2L;
                ShuffleboardLayout t0L3d;
                ShuffleboardLayout t1L3d;
                ShuffleboardLayout t2L3d;

                t0L = Shuffleboard.getTab("Cameras")
                                .getLayout("BestTarget", BuiltInLayouts.kList).withPosition(2, 0)

                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                writeValues(t0L, cam, 0);

                t1L = Shuffleboard.getTab("Cameras")
                                .getLayout("Second Target", BuiltInLayouts.kList).withPosition(4, 0)

                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                writeValues(t1L, cam, 1);
                //t1L.addString("TAGXY", () -> AprilTagData.getTranslation3d(cam.tagID[0]).toString());

                t2L = Shuffleboard.getTab("Cameras")
                                .getLayout("ThirdTarget", BuiltInLayouts.kList).withPosition(6, 0)

                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

                writeValues(t2L, cam, 2);

                t0L3d = Shuffleboard.getTab("Cameras")
                                .getLayout("BestTarget3D", BuiltInLayouts.kList).withPosition(2, 3)

                                .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

                write3DValues(t0L3d, cam, 0);

                t1L3d = Shuffleboard.getTab("Cameras")
                                .getLayout("Second Target3D", BuiltInLayouts.kList).withPosition(4, 3)

                                .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));

                write3DValues(t1L3d, cam, 1);

                t2L3d = Shuffleboard.getTab("Cameras")
                                .getLayout("ThirdTarget3D", BuiltInLayouts.kList).withPosition(6, 3)

                                .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));

                write3DValues(t2L3d, cam, 2);

                // t1L.addString("3D-ToCam Rd3", () -> cam.rotation[Cameras.idx]);

        }

        public static void writeValues(ShuffleboardLayout tnL, Cameras cam, int n) {

                tnL.addNumber("TargetNumber", () -> n);

                tnL.addNumber("AprilTagID", () -> cam.tagID[n]);

                tnL.addNumber("TargetYaw", () -> cam.yaw[n]);

                tnL.addNumber("TargetPitch", () -> cam.pitch[n]);

                tnL.addNumber("TargetSkew", () -> cam.skew[n]);

                tnL.addNumber("TargetArea", () -> cam.area[n]);

                tnL.addNumber("PoseAmbiguity", () -> cam.poseAmbiguity[n]);

        }

        public static void write3DValues(ShuffleboardLayout tnL, Cameras cam, int n) {

                tnL.addNumber("3D-ToCam X", () -> cam.X[n]);

                tnL.addNumber("3D-ToCam Y", () -> cam.Y[n]);

                tnL.addNumber("3D-ToCam Z", () -> cam.Z[n]);

        }
}
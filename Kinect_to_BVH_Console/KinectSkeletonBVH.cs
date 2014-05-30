﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Microsoft.Kinect;
using System.Windows.Media.Media3D;
//using Microsoft.Xna.Framework;

namespace Kinect_to_BVH_Console
{
    class KinectSkeletonBVH
    {

        public static void AddKinectSkeleton(BVHSkeleton Skeleton)
        {
            //Die Person steht falsch herum im Koordinatensystem der Kinect! Es wird erst beim Abspeichern korrigiert, weshalb die Verarbeitung noch mit umgekehrten Koordinaten erfolgt
            BVHBone hipCenter = new BVHBone(null, JointType.HipCenter.ToString(), 6, TransAxis.None, true);
            BVHBone hipCenter2 = new BVHBone(hipCenter, "HipCenter2", 3, TransAxis.Y, false);
            BVHBone spine = new BVHBone(hipCenter2, JointType.Spine.ToString(), 3, TransAxis.Y, true);
            BVHBone shoulderCenter = new BVHBone(spine, JointType.ShoulderCenter.ToString(), 3, TransAxis.Y, true);

            BVHBone collarLeft = new BVHBone(shoulderCenter, "CollarLeft", 3, TransAxis.X, false);
            BVHBone shoulderLeft = new BVHBone(collarLeft, JointType.ShoulderLeft.ToString(), 3, TransAxis.X, true);
            BVHBone elbowLeft = new BVHBone(shoulderLeft, JointType.ElbowLeft.ToString(), 3, TransAxis.X, true);
            BVHBone wristLeft = new BVHBone(elbowLeft, JointType.WristLeft.ToString(), 3, TransAxis.X, true);
            BVHBone handLeft = new BVHBone(wristLeft, JointType.HandLeft.ToString(), 0, TransAxis.X, true);

            BVHBone neck = new BVHBone(shoulderCenter, "Neck", 3, TransAxis.Y, false);
            BVHBone head = new BVHBone(neck, JointType.Head.ToString(), 3, TransAxis.Y, true);
            BVHBone headtop = new BVHBone(head, "Headtop", 0, TransAxis.None, false);

            BVHBone collarRight = new BVHBone(shoulderCenter, "CollarRight", 3, TransAxis.nX, false);
            BVHBone shoulderRight = new BVHBone(collarRight, JointType.ShoulderRight.ToString(), 3, TransAxis.nX, true);
            BVHBone elbowRight = new BVHBone(shoulderRight, JointType.ElbowRight.ToString(), 3, TransAxis.nX, true);
            BVHBone wristRight = new BVHBone(elbowRight, JointType.WristRight.ToString(), 3, TransAxis.nX, true);
            BVHBone handRight = new BVHBone(wristRight, JointType.HandRight.ToString(), 0, TransAxis.nX, true);

            BVHBone hipLeft = new BVHBone(hipCenter, JointType.HipLeft.ToString(), 3, TransAxis.X, true);
            BVHBone kneeLeft = new BVHBone(hipLeft, JointType.KneeLeft.ToString(), 3, TransAxis.nY, true);
            BVHBone ankleLeft = new BVHBone(kneeLeft, JointType.AnkleLeft.ToString(), 3, TransAxis.nY, true);
            BVHBone footLeft = new BVHBone(ankleLeft, JointType.FootLeft.ToString(), 0, TransAxis.Z, true);

            BVHBone hipRight = new BVHBone(hipCenter, JointType.HipRight.ToString(), 3, TransAxis.nX, true);
            BVHBone kneeRight = new BVHBone(hipRight, JointType.KneeRight.ToString(), 3, TransAxis.nY, true);
            BVHBone ankleRight = new BVHBone(kneeRight, JointType.AnkleRight.ToString(), 3, TransAxis.nY, true);
            BVHBone footRight = new BVHBone(ankleRight, JointType.FootRight.ToString(), 0, TransAxis.Z, true);

            Skeleton.AddBone(hipCenter);
            Skeleton.AddBone(hipCenter2);
            Skeleton.AddBone(spine);
            Skeleton.AddBone(shoulderCenter);
            Skeleton.AddBone(collarLeft);
            Skeleton.AddBone(shoulderLeft);
            Skeleton.AddBone(elbowLeft);
            Skeleton.AddBone(wristLeft);
            Skeleton.AddBone(handLeft);
            Skeleton.AddBone(neck);
            Skeleton.AddBone(head);
            Skeleton.AddBone(headtop);
            Skeleton.AddBone(collarRight);
            Skeleton.AddBone(shoulderRight);
            Skeleton.AddBone(elbowRight);
            Skeleton.AddBone(wristRight);
            Skeleton.AddBone(handRight);
            Skeleton.AddBone(hipLeft);
            Skeleton.AddBone(kneeLeft);
            Skeleton.AddBone(ankleLeft);
            Skeleton.AddBone(footLeft);
            Skeleton.AddBone(hipRight);
            Skeleton.AddBone(kneeRight);
            Skeleton.AddBone(ankleRight);
            Skeleton.AddBone(footRight);

            Skeleton.FinalizeBVHSkeleton();
        }

        public static JointType String2JointType(string boneName)
        {
            JointType value = (JointType)Enum.Parse(typeof(JointType), boneName);
            return value;
        }

        public static JointType getJointTypeFromBVHBone(BVHBone bone)
        {
            JointType kinectJoint = new JointType();

            switch (bone.Name)
            {
                case "HipCenter":
                    kinectJoint = JointType.HipCenter;
                    break;
                case "HipCenter2":
                    kinectJoint = JointType.HipCenter;
                    break;
                case "Spine":
                    kinectJoint = JointType.Spine;
                    break;
                case "ShoulderCenter":
                    kinectJoint = JointType.ShoulderCenter;
                    break;

                case "Neck":
                    kinectJoint = JointType.Head;
                    break;
                case "Head":
                    kinectJoint = JointType.Head;
                    break;

                case "CollarRight":
                    kinectJoint = JointType.ShoulderRight;
                    break;
                case "ShoulderRight":
                    kinectJoint = JointType.ElbowRight;
                    break;
                case "ElbowRight":
                    kinectJoint = JointType.WristRight;
                    break;
                case "WristRight":
                    kinectJoint = JointType.HandRight;
                    break;

                case "CollarLeft":
                    kinectJoint = JointType.ShoulderLeft;
                    break;
                case "ShoulderLeft":
                    kinectJoint = JointType.ElbowLeft;
                    break;
                case "ElbowLeft":
                    kinectJoint = JointType.WristLeft;
                    break;
                case "WristLeft":
                    kinectJoint = JointType.HandLeft;
                    break;

                case "HipLeft":
                    kinectJoint = JointType.KneeLeft;
                    break;
                case "KneeLeft":
                    kinectJoint = JointType.AnkleLeft;
                    break;
                case "AnkleLeft":
                    kinectJoint = JointType.FootLeft;
                    break;

                case "HipRight":
                    kinectJoint = JointType.KneeRight;
                    break;
                case "KneeRight":
                    kinectJoint = JointType.AnkleRight;
                    break;
                case "AnkleRight":
                    kinectJoint = JointType.FootRight;
                    break;
            }

            return kinectJoint;
        }

        public static double[] getEulerFromBone(BVHBone bone, Skeleton skel)
        {
            double[] degVec = new double[3] { 0, 0, 0 };
            double[] correctionDegVec = new double[3] { 0, 0, 0 };
            JointType kinectJoint = new JointType();
            JointType ParentKinectJoint = new JointType();
            bool noData = false;

            kinectJoint = getJointTypeFromBVHBone(bone);



            switch (bone.Name)
            {
                case "HipCenter2":
                    //correctionDegVec[0] = 150;
                    correctionDegVec[0] = -30;
                    //correctionDegVec[0] = -30;
                    break;
                case "ShoulderLeft":
                    correctionDegVec[0] = 30;
                    break;
                case "ShoulderRight":
                    correctionDegVec[0] = 30;
                    break;
                case "HipRight":
                    correctionDegVec[0] = -10;
                    break;
                case "HipLeft":
                    correctionDegVec[0] = -10;
                    break;
                case "KneeLeft":
                    correctionDegVec[0] = 10;
                    break;
                case "KneeRight":
                    correctionDegVec[0] = 10;
                    break;
                case "ShoulderCenter":
                    //correctionDegVec[0] = -20;
                    break;
                case "Neck":
                    correctionDegVec[0] = -20;
                    break;
                case "CollarRight":
                    noData = true;
                    break;
                case "CollarLeft":
                    noData = true;
                    break;
                case "Spine":
                    //Gibt die Rotation der Wirbelsäule zwischen Spine Joint und Shoulder Center an. 
                    degVec[0] = 30;
                    degVec[1] = 0;
                    degVec[2] = 0;
                    noData = true;
                    break;
                case "Head": //Informationen sind in "Neck"
                    noData = true;
                    break;
                case "AnkleRight":
                    noData = true;
                    break;
                case "AnkleLeft":
                    noData = true;
                    break;
                default:
                    break;
            }

            if (bone.Root == false)
            {

                //Das BVH Skelett hat mehr Knochen wie das Kinect Skelett, diese Joints haben dauerthaft die Rotationen 0 0 0 
                if (noData == false)
                {
                    Quaternion tempQuat;

                    if (!(bone.Name == "HipRight" || bone.Name == "HipLeft" || bone.Name == "ShoulderLeft" || bone.Name == "ShoulderRight" || bone.Name == "HipCenter2"))
                    {
                        tempQuat = MathHelper.vector42Quat(skel.BoneOrientations[kinectJoint].HierarchicalRotation.Quaternion);
                        degVec = MathHelper.quat2Deg(tempQuat);

                        if (bone.Name == "HipCenter2")
                        {
                            degVec[1] = 0;
                            degVec[2] = -degVec[2];
                        }

                        //Beine
                        if (bone.Axis == TransAxis.nY)
                        {
                            degVec[0] = -degVec[0];
                            degVec[1] = -degVec[1];
                            degVec[2] = degVec[2];

                        }

                        //Rechter Arm
                        if (bone.Axis == TransAxis.nX && bone.Name != "ShoulderRight")
                        {
                            double[] tempDecVec = new double[3] { degVec[0], degVec[1], degVec[2] };
                            degVec[0] = -tempDecVec[2];
                            degVec[1] = -tempDecVec[1];
                            degVec[2] = -tempDecVec[0];

                        }
                        /*
                        //Rechte Schulter
                        if (bone.Name == "ShoulderRight")
                        {
                            double[] tempDecVec = new double[3] { degVec[0], degVec[1], degVec[2] };
                            degVec[0] = -tempDecVec[0];
                            degVec[1] = tempDecVec[1];
                            degVec[2] = tempDecVec[2];

                        }

                        //Linke Schulter
                        if (bone.Name == "ShoulderLeft")
                        {
                            double[] tempDecVec = new double[3] { degVec[0], degVec[1], degVec[2] };
                            degVec[0] = -tempDecVec[0];
                            degVec[1] = tempDecVec[1];
                            degVec[2] = tempDecVec[2];

                        }
                        */

                        //Linker Arm
                        if (bone.Axis == TransAxis.X && bone.Name != "ShoulderLeft")
                        {
                            double[] tempDecVec = new double[3] { degVec[0], degVec[1], degVec[2] };
                            degVec[0] = tempDecVec[2];
                            degVec[1] = tempDecVec[1];
                            degVec[2] = tempDecVec[0];

                        }


                    }
                    else
                    {
                        //Rotation per "Hand" ausrechnen mithilfe von Vektoren. Ist nötig, da dass BVH Skelett an den Hüft- und Schulterknochen nicht mit dem Kinect Skelett übereinstimmen
                        Vector3D vec = new Vector3D();
                        Vector3D axis = new Vector3D();


                        switch (bone.Name)
                        {
                            case "HipRight":
                                axis = new Vector3D(0, -1, 0);
                                ParentKinectJoint = JointType.HipRight;
                                break;
                            case "HipLeft":
                                axis = new Vector3D(0, -1, 0);
                                ParentKinectJoint = JointType.HipLeft;
                                break;
                            case "HipCenter2":
                                axis = new Vector3D(0, 1, 0);
                                ParentKinectJoint = JointType.Spine;
                                kinectJoint = JointType.ShoulderCenter;
                                break;
                            case "ShoulderRight":
                                axis = new Vector3D(1, 0, 0);
                                ParentKinectJoint = JointType.ShoulderRight;
                                break;
                            case "ShoulderLeft":
                                axis = new Vector3D(-1, 0, 0);
                                ParentKinectJoint = JointType.ShoulderLeft;
                                break;
                        }

                        double skal = (skel.Joints[kinectJoint].Position.Z / skel.Joints[ParentKinectJoint].Position.Z);
                        skal = 1;

                        vec.X = skel.Joints[kinectJoint].Position.X * skal - skel.Joints[ParentKinectJoint].Position.X * 1 / skal;
                        vec.Y = skel.Joints[kinectJoint].Position.Y * skal - skel.Joints[ParentKinectJoint].Position.Y * 1 / skal;
                        vec.Z = skel.Joints[kinectJoint].Position.Z - skel.Joints[ParentKinectJoint].Position.Z;

                        vec.Normalize();


                        if (bone.Name == "ShoulderLeft" || bone.Name == "ShoulderRight")
                        {
                            double[] rotationOffset = new double[3];
                            rotationOffset = MathHelper.quat2Deg(skel.BoneOrientations[JointType.ShoulderCenter].AbsoluteRotation.Quaternion);
                            Matrix3D rotMat = MathHelper.GetRotationMatrix(-(rotationOffset[0] * Math.PI / 180) - 180, 0, 0);
                            Vector3D vec2 = Vector3D.Multiply(vec, rotMat);

                            tempQuat = MathHelper.getQuaternion(axis, vec2);
                            degVec = MathHelper.quat2Deg(tempQuat);

                            degVec[0] = degVec[0];
                            degVec[1] = degVec[1]; //+ rotationOffset[1];
                            degVec[2] = degVec[2];

                            degVec[2] = -degVec[2];
                        }

                        if (bone.Name == "HipCenter2")
                        {
                            double[] rotationOffset = new double[3] { 0, 0, 0 };
                            rotationOffset = MathHelper.quat2Deg(skel.BoneOrientations[JointType.HipCenter].AbsoluteRotation.Quaternion);
                            Vector3D vec2 = Vector3D.Multiply(vec, MathHelper.GetRotationMatrixY(-rotationOffset[1] * Math.PI / 180));
                            tempQuat = MathHelper.getQuaternion(axis, vec2);

                            degVec = MathHelper.quat2Deg(tempQuat);
                            degVec[1] = 0;

                            degVec[0] = -degVec[0];
                            degVec[2] = -degVec[2];

                        }


                        if (bone.Name == "HipRight" || bone.Name == "HipLeft")
                        {
                            double[] rotationOffset = new double[3] { 0, 0, 0 };
                            rotationOffset = MathHelper.quat2Deg(skel.BoneOrientations[JointType.HipCenter].AbsoluteRotation.Quaternion);
                            Vector3D vec2 = Vector3D.Multiply(vec, MathHelper.GetRotationMatrixY(-rotationOffset[1] * Math.PI / 180));

                            tempQuat = MathHelper.getQuaternion(axis, vec2);
                            degVec = MathHelper.quat2Deg(tempQuat);


                            degVec[0] = -degVec[0];
                            degVec[1] = -degVec[1]; //Nur Yaw WInkel Wichtig
                            degVec[2] = -degVec[2];
                        }
                    }

                }

            }
            else
            {

                //Kinect Kamera Koordinatensystem ist genau spiegelverkehrt zum User Koordinatensystem
                Vector4 tempQuat = skel.BoneOrientations[kinectJoint].AbsoluteRotation.Quaternion;
                degVec = MathHelper.quat2Deg(tempQuat);


                //Hüfte bleibt immer parallel zum Boden ausgerichtet. Nur Oberkörper kann sich "verbiegen"
                degVec[0] = 0;
                degVec[1] = -degVec[1]; //Drehung um die eigene Achse "YAW Winkel"
                degVec[2] = 0;

            }
            //Korrektur
            degVec = MathHelper.addArray(degVec, correctionDegVec);


            //Falls WInkel über 180 Grad ist
            for (int k = 0; k < 3; k++)
            {
                if (degVec[k] > 180)
                {
                    degVec[k] -= 360;
                }
                else if (degVec[k] < -180)
                {
                    degVec[k] += 360;
                }
            }

            bone.setRotOffset(degVec[0], degVec[1], degVec[2]); //wird eigentlich nicht benötigt
            return degVec;
        }

        public static double[] getBoneVectorOutofJointPosition(BVHBone bvhBone, Skeleton skel)
        {
            double[] boneVector = new double[3] { 0, 0, 0 };
            double[] boneVectorParent = new double[3] { 0, 0, 0 };
            string boneName = bvhBone.Name;

            JointType Joint;
            if (bvhBone.Root == true)
            {
                boneVector = new double[3] { 0, 0, 0 };
            }
            else
            {
                if (bvhBone.IsKinectJoint == true)
                {
                    Joint = KinectSkeletonBVH.String2JointType(boneName);

                    boneVector[0] = skel.Joints[Joint].Position.X;
                    boneVector[1] = skel.Joints[Joint].Position.Y;
                    boneVector[2] = skel.Joints[Joint].Position.Z;

                    try
                    {
                        Joint = KinectSkeletonBVH.String2JointType(bvhBone.Parent.Name);
                    }
                    catch
                    {
                        Joint = KinectSkeletonBVH.String2JointType(bvhBone.Parent.Parent.Name);
                    }

                    boneVector[0] -= skel.Joints[Joint].Position.X;
                    boneVector[1] -= skel.Joints[Joint].Position.Y;
                    boneVector[2] -= skel.Joints[Joint].Position.Z;
                }
            }

            return boneVector;
        }


    }
}

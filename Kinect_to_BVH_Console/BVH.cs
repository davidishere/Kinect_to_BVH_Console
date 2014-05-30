using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Kinect_to_BVH_Console
{
    public enum BVHChannel
    {
        Xposition,
        Yposition,
        Zposition,
        Xrotation,
        Yrotation,
        Zrotation
    }

    public enum TransAxis
    {
        None,
        X,
        Y,
        Z,
        nX,
        nY,
        nZ
    }

    public class BVHSkeleton
    {
        List<BVHBone> bones;
        int maxDepth = 0;
        int nrBones;
        int channels;

        public List<BVHBone> Bones
        {
            get { return bones; }
        }

        public int Channels
        {
            get { return channels; }
        }

        public BVHSkeleton()
        {
            bones = new List<BVHBone>();
        }

        public void AddBone(BVHBone Bone)
        {
            if (!Bones.Contains(Bone))
            {
                bones.Add(Bone);
            }
        }

        public void FinalizeBVHSkeleton()
        {
            for (int k = 0; k < Bones.Count(); k++)
            {
                // set max Depth
                if (Bones[k].Depth > maxDepth)
                    maxDepth = Bones[k].Depth;

                //set Bone Index for Motion Values Array
                int motionCount = 0;
                for (int n = 0; n < k; n++)
                {
                    motionCount += Bones[n].ChannelCount;
                }
                Bones[k].MotionSpace = motionCount;

                //set Count of Channels for Skeleton
                channels += Bones[k].ChannelCount;

                //set Children
                List<BVHBone> childBoneList = Bones.FindAll(i => i.Parent == Bones[k]);
                if (childBoneList.Count == 0)
                {
                    Bones[k].End = true;
                }
                else
                {
                    Bones[k].Children = childBoneList;

                }
            }
        }

        public void copyParameters(BVHSkeleton input)
        {
            channels = input.Channels;
            maxDepth = input.maxDepth;
            nrBones = input.nrBones;
        }

        public int getMaxDepth()
        {
            return maxDepth;
        }
    }

    public class BVHBone
    {
        BVHBone parent;
        List<BVHBone> children;
        string name;
        int depth;
        static int index = 1;
        BVHChannel[] channels;
        public double[] rotOffset = new double[] { 0, 0, 0 };
        public double[] translOffset = new double[] { 0, 0, 0 };
        bool end;
        bool root;
        int motionSpace; // gibt erstes Element in der Motionspalte an
        TransAxis axis;
        bool isKinectJoint;


        public List<BVHBone> Children
        {
            get { return children; }
            set { children = value; }
        }

        public bool IsKinectJoint
        {
            get { return isKinectJoint; }
            set { isKinectJoint = value; }
        }

        public bool Root
        {
            get { return root; }
            set { root = value; }
        }

        public bool End
        {
            get { return end; }
            set { end = value; }
        }

        public TransAxis Axis 
        {
            get { return axis; }
            set { axis = value; }
        }

        public int MotionSpace
        {
            get { return motionSpace; }
            set { motionSpace = value; }
        }

        public int Depth
        {
            get { return depth; }
        }
        public int ChannelCount
        {
            get { return channels.Length; }
        }
        public string Name
        {
            get { return name; }
        }
        public BVHBone Parent
        {
            get { return parent; }
        }
        public BVHChannel[] Channels
        {
            get { return channels; }
        }

        public BVHBone(BVHBone Parent, string Name, int nrChannels, TransAxis Axis, bool IsKinectJoint)
        {
            parent = Parent;
            index += index;
            name = Name;
            isKinectJoint = IsKinectJoint;
            axis = Axis;
            if (parent != null)
                depth = parent.Depth + 1;
            else
            {
                depth = 0;
                root = true;
            }
            channels = new BVHChannel[nrChannels];
            int ind = 5;
            for (int k = nrChannels - 1; k >= 0; k--)
            {
                channels[k] = (BVHChannel)ind;
                ind--;
            }
        }

        public void setTransOffset(double xOff, double yOff, double zOff)
        {
            translOffset = new double[] { xOff, yOff, zOff };
        }

        public void setRotOffset(double xOff, double yOff, double zOff)
        {
            rotOffset = new double[] { xOff, yOff, zOff };
        }
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace Kinect_to_BVH_Console
{
    class functions
    {
        private KinectSensor sensor;
        private writeBVH BVHFile;

        public functions() {
            ;
        }
        

        public void Kinect_start(string smooth)
        {
            if (sensor == null)
            {
                // Look through all sensors and start the first connected one.
                // This requires that a Kinect is connected at the time of app startup.
                // To make your app robust against plug/unplug, 
                // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
                foreach (var potentialSensor in KinectSensor.KinectSensors)
                {
                    if (potentialSensor.Status == KinectStatus.Connected)
                    {
                        this.sensor = potentialSensor;
                        break;
                    }
                }

                if (null != this.sensor)
                {
                    TransformSmoothParameters smoothingParam = new TransformSmoothParameters();

                    if (smooth == "Default")
                    {
                        // Some smoothing with little latency (defaults).
                        // Only filters out small jitters.
                        // Good for gesture recognition in games.
                        smoothingParam = new TransformSmoothParameters();
                        {
                            smoothingParam.Smoothing = 0.5f;
                            smoothingParam.Correction = 0.5f;
                            smoothingParam.Prediction = 0.5f;
                            smoothingParam.JitterRadius = 0.05f;
                            smoothingParam.MaxDeviationRadius = 0.04f;
                        };
                    }
                    else if (smooth == "Moderate")
                    {

                        // Smoothed with some latency.
                        // Filters out medium jitters.
                        // Good for a menu system that needs to be smooth but
                        // doesn't need the reduced latency as much as gesture recognition does.
                        smoothingParam = new TransformSmoothParameters();
                        {
                            smoothingParam.Smoothing = 0.5f;
                            smoothingParam.Correction = 0.1f;
                            smoothingParam.Prediction = 0.5f;
                            smoothingParam.JitterRadius = 0.1f;
                            smoothingParam.MaxDeviationRadius = 0.1f;
                        };
                    }
                    else if (smooth == "Intense")
                    {
                        // Very smooth, but with a lot of latency.
                        // Filters out large jitters.
                        // Good for situations where smooth data is absolutely required
                        // and latency is not an issue.
                        smoothingParam = new TransformSmoothParameters();
                        {
                            smoothingParam.Smoothing = 0.7f;
                            smoothingParam.Correction = 0.3f;
                            smoothingParam.Prediction = 1.0f;
                            smoothingParam.JitterRadius = 1.0f;
                            smoothingParam.MaxDeviationRadius = 1.0f;
                        };
                    }

                    // Turn on the stream to receive frames
                    this.sensor.SkeletonStream.Enable(smoothingParam);

                    // event handler to be called whenever there is new frame data
                    this.sensor.AllFramesReady += sensor_allFramesReady;


                    // Start the sensor!
                    try
                    {
                        this.sensor.Start();
                        Console.WriteLine("started stream");
                    }
                    catch (Exception)
                    {
                        this.sensor = null;
                        Console.WriteLine("Stream could not be started");
                    }
                }
                else
                {
                    Console.WriteLine("No Kinect found");
                }
            }

        }

        void sensor_allFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            int initFrames = 100;  // ??
            using (SkeletonFrame skelFrame = e.OpenSkeletonFrame())
            {
                if (skelFrame != null)
                {
                    Skeleton[] skeletons = new Skeleton[skelFrame.SkeletonArrayLength];
                    skelFrame.CopySkeletonDataTo(skeletons);
                    if (skeletons.Length != 0)
                    {
                        foreach (Skeleton skel in skeletons)
                        {
                            if (skel.TrackingState == SkeletonTrackingState.Tracked)
                            {
                                if (BVHFile != null)
                                {
                                    if (BVHFile.isRecording == true && BVHFile.isInitializing == true)
                                    {
                                        BVHFile.Entry(skel);

                                        if (BVHFile.intializingCounter > initFrames)
                                        {
                                            BVHFile.startWritingEntry();
                                        }

                                    }
                                    if (BVHFile.isRecording == true && BVHFile.isInitializing == false)
                                    {
                                        BVHFile.Motion(skel);
                                        Console.WriteLine("Record");
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        public void Kinect_stop()
        {
            if (BVHFile != null)
            {
                Console.WriteLine("First stop recording!");
            }
            else
            {
                if (sensor != null)
                {
                    if (sensor.IsRunning)
                    {
                        sensor.Stop();
                    }
                }
                Console.WriteLine("Closed stream");
            }
        }

        public void Start_record()
        {
            if (BVHFile == null && sensor != null)
            {
                Console.WriteLine("File initialize.");
                DateTime thisDay = DateTime.UtcNow;
                string txtFileName = thisDay.ToString("dd.MM.yyyy_HH.mm");
                BVHFile = new writeBVH(txtFileName);
            }
        }

        public void Stop_record()
        {
            if (BVHFile != null)
            {
                BVHFile.closeBVHFile();
                Console.WriteLine("recording saved");
                BVHFile = null;
            }
        }
    }
}

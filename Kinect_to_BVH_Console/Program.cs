using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace Kinect_to_BVH_Console
{
    class Program
    {
        static void Main(string[] args)
        {
            bool flag = true;
            functions fun = new functions();
            while (flag) { 
                string temp = Console.ReadLine();            
                switch (temp) {
                    case "s": 
                        { 
                            Console.WriteLine("start");
                            fun.Kinect_start("default");
                            break;
                        }
                    case "q":
                        {
                            Console.WriteLine("quit");
                            fun.Kinect_stop();
                            flag = false;
                            break;
                        }
                    case "r":
                        {
                            Console.WriteLine("start record");
                            fun.Start_record();
                            if (fun.BVHFile == null && fun.sensor != null)
                                fun.Kinect_start("default");
                            break;
                        }
                    case "t":
                        {
                            Console.WriteLine("stop record");
                            fun.Stop_record();
                            break;
                        }
                    default: Console.WriteLine("error"); break;
                }
            }
        }
    }
}

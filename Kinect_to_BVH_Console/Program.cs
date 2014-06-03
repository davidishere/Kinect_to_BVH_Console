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
            Console.WriteLine("--------------------------------------");
            Console.WriteLine("Please input:");
            Console.WriteLine("     s : Start the Kinect");
            Console.WriteLine("     r : Start record");
            Console.WriteLine("     t : Stop record");
            Console.WriteLine("     q : Stop the Kinect and quit");
            Console.WriteLine("--------------------------------------");
            while (flag) { 
                string temp = Console.ReadLine();            
                switch (temp) {
                    case "s": 
                        { 
                            Console.WriteLine("start");
                            fun.Kinect_start("Default"); // Default Moderate Intense
                            break;
                        }
                    case "q":
                        {
                            Console.WriteLine("quit");
                            if (fun.Kinect_stop() == 0)
                                flag = false;
                            break;
                        }
                    case "r":
                        {
                            Console.WriteLine("start record");
                            fun.Start_record();
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

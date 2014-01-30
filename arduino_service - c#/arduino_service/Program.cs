using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.IO.Ports;
using RobotRaconteur;
using arduino2_interface;

//thsi is the server to the arduino board. 
namespace ArduinoServer2
{
    class Program
    {
        public static arduino2 ard;
        static void Main(string[] args)
        {

            //Console.ReadLine();
            RobotRaconteurNativeLoader.Load();
            //Initialize the create robot object
            ard = new arduino2();
            ard.Start();


            // set the node name and id
            RobotRaconteurNode.s.NodeName = "ArduinoServer2";
            RobotRaconteurNode.s.NodeID = new NodeID("{f7a045d4-6bef-a942-8057-b81caa821f40}");

            // Initialize the TCP channel and start listening for connections on port 5444
            TcpTransport c = new TcpTransport();
            c.StartServer(5444);

            //Enable auto-discovery announcements
            c.EnableNodeAnnounce(IPNodeDiscoveryFlags.LINK_LOCAL | IPNodeDiscoveryFlags.NODE_LOCAL | IPNodeDiscoveryFlags.SITE_LOCAL);

            //Register the TCP channel
            RobotRaconteurNode.s.RegisterTransport(c);

            //Register the PhantomOmniRemote_interface type so that the node can understand the service definition
            RobotRaconteurNode.s.RegisterServiceType(new arduino2_interfaceFactory());

            //Register the create object as a service so that it can be connected to
            RobotRaconteurNode.s.RegisterService("arduino2", "arduino2_interface", ard);

            //Stay open until shut down
            Console.WriteLine("arduino server started.  Connect with URL tcp://localhost:5444/ArduinoServer2/arduino Press enter to exit");
            //Console.WriteLine("use reset to reset encoders to 0.");
            //Console.WriteLine("use sendcommand to send read attached encoder command.");
            //Console.WriteLine("use pos1 and pos2 to read encoders.ex: 
            Console.ReadLine();

            //Shutdown
            ard.Shutdown();

            //Shutdown the node.  This must be called or the program won't exit
            RobotRaconteurNode.s.Shutdown();
        }
    }

    public class arduino2 : arduino2_interface.arduino2
    {
        //creat a serialport to connect to arduino
        SerialPort port;


        object port_lock = new object();
        object recv_port_lock = new object();

        public void Start()
        {
            lock (port_lock)
            {
                port = new SerialPort("COM62");
                port.BaudRate = 9600;
                port.DataBits = 8;
                port.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);
                port.Open();
                if (port.IsOpen)
                {
                    Console.WriteLine("Serial port connected.");
                }
            }

        }


        //Choose a axis (-1 or 1) -1 is x axis 1 is y axis
        //Accepts an integer ranging from -100 to 100 which corresponds to the speed of the motor
        public void setPotentiometer(int axis, int speed)
        {

            lock (port_lock)
            {
                if (port.IsOpen)
                {
                    try
                    {
                        byte commandByte; //Byte to send to the arduino
                        if (axis == -1){
                            commandByte = Convert.ToByte((speed * -1 + 100) * 130 / 190 + 136);
                            if (commandByte > Convert.ToByte(255))
                                commandByte = Convert.ToByte(255);
                            else if (commandByte < Convert.ToByte(158))
                                commandByte = Convert.ToByte(158);
                        }

                        else if (axis == 1){
                            commandByte = Convert.ToByte((speed + 100) * 130 / 200 - 19);
                            if (commandByte > Convert.ToByte(115))
                                commandByte = Convert.ToByte(115);
                            else if(commandByte < 0)
                                commandByte = Convert.ToByte(0);
                        }

                        else
                            return;

                        byte[] command1 = new byte[] { commandByte };
                        Console.WriteLine("axis {0} {1}", axis, commandByte);
                        port.Write(command1, 0, command1.Length);

                    }
                    catch { }

                }

            }
        }
        


        public void Shutdown()
        {
            lock (port_lock)
            {
                port.DiscardInBuffer();
                port.DiscardOutBuffer();

                port.Close();
            }

        }
    }
}
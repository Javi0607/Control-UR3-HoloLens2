using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using System.Net.Sockets;
using static Connect_TCP_IP;
using System.Diagnostics;
using System.Threading;
using System;
using TMPro;
using System.Globalization;


public static class UR_Stream_Data
{
    // IP Port Number and IP Address
    public static string ip_address;
    //  Real-time (Read Only)
    public const ushort port_number = 30003;
    // Comunication Speed (ms)
    public static int time_step;
    // Joint Space:
    //  Orientation {J1 .. J6} (rad)
    public static double[] J_Orientation = new double[6];
    // Cartesian Space:
    //  Position {X, Y, Z} (mm)
    public static double[] C_Position = new double[3];
    //  Orientation {Euler Angles} (rad):
    public static double[] C_Orientation = new double[3];
}

public static class UR_Control_Data
{
    // IP Port Number and IP Address
    public static string ip_address;
    //  Real-time (Read/Write)
    public const ushort port_number = 30002;
    // Comunication Speed (ms)
    public static int time_step;
    // Home Parameters UR3/UR3e:
    //  Joint Space:
    //      Orientation {J1 .. J6} (rad)
    public static double[] J_Orientation = new double[6];
    //  Cartesian Space:
    //      Position {X, Y, Z} (mm)
    public static double[] C_Position = new double[3];
    //      Orientation {Euler Angles} (rad):
    public static double[] C_Orientation = new double[3];
    // Move Parameters: Velocity, Acceleration
    public static string velocity = "1.0";
    public static string acceleration = "1.0";
}


public class Connect_TCP_IP : MonoBehaviour
{

    public UR_Stream ur_stream_robot;
    public UR_Control ur_ctrl_robot;
    public bool isButtonOn = false;

    public GameObject Punto;
    public TMP_Text DebugText;
    public Vector3 PosicionPunto;
    public Vector3 axis;

    public static double[] ConvertVector3ToDoubleArray(Vector3 vector)
    {
        double[] result = new double[3];
        result[0] = vector.x;
        result[1] = vector.y;
        result[2] = vector.z;
        return result;
    }

    public string ArrayToString(double[] array)
    {
        string result = "";
        for (int i = 0; i < array.Length; i++)
        {
            result += array[i].ToString();
            if (i < array.Length - 1)
            {
                result += ", ";
            }
        }
        return result;
    }

    public void OnOff()
    {
        CultureInfo culture = new CultureInfo("en-US"); // "en-US" representa la cultura de Estados Unidos
        CultureInfo.CurrentCulture = culture;

  
        isButtonOn = !isButtonOn;

        if (isButtonOn) {
            // Initialization {TCP/IP Universal Robots}
            //  Read Data:
            UR_Stream_Data.ip_address = "138.100.76.100";
            //  Communication speed: CB-Series 125 Hz (8 ms), E-Series 500 Hz (2 ms)
            UR_Stream_Data.time_step = 2;
            //  Write Data:
            UR_Control_Data.ip_address = "138.100.76.100";
            //  Communication speed: CB-Series 125 Hz (8 ms), E-Series 500 Hz (2 ms)
            UR_Control_Data.time_step = 2;


            // Start Stream {Universal Robots TCP/IP}
            //UR_Stream ur_stream_robot = new UR_Stream();
            ur_stream_robot = new UR_Stream();
            ur_stream_robot.Start();
            UnityEngine.Debug.Log("Robot conectado");
            DebugText.text = "Robot conectado";
        }
        else { 
  
            //UnityEngine.Debug.Log(ArrayToString(UR_Control_Data.C_Position) + ArrayToString(UR_Control_Data.C_Orientation));
            UnityEngine.Debug.Log("J1: {0} | J2: {1} | J3: {2} | J4: {3} | J5: {4} | J6: {5}");
            UnityEngine.Debug.Log(ArrayToString(UR_Stream_Data.J_Orientation));

            ur_stream_robot.Stop();
            ur_ctrl_robot.Stop();
            ur_stream_robot.Destroy();
            ur_ctrl_robot.Destroy();

            // Application quit
            UnityEngine.Debug.Log("Robot desconectado");
            DebugText.text = "Robot desconectado";
        }
    }
    public void Send()
    {
        CultureInfo culture = new CultureInfo("en-US"); // "en-US" representa la cultura de Estados Unidos
        CultureInfo.CurrentCulture = culture;

        Quaternion RotacionPunto;

        PosicionPunto = Punto.transform.localPosition;

        RotacionPunto = Punto.transform.localRotation;
        axis =RotacionPunto.eulerAngles;

        UR_Control_Data.C_Position = ConvertVector3ToDoubleArray(PosicionPunto);
        UR_Control_Data.C_Orientation = ConvertVector3ToDoubleArray(axis);

        UR_Control_Data.C_Position[0] = -1 * UR_Control_Data.C_Position[0];


        UR_Control_Data.C_Orientation[0] = -1*UR_Control_Data.C_Orientation[0] * Math.PI * 2 / 360;
        UR_Control_Data.C_Orientation[1] = -1*UR_Control_Data.C_Orientation[1] * Math.PI * 2 / 360;
        UR_Control_Data.C_Orientation[2] = -1*UR_Control_Data.C_Orientation[2] * Math.PI * 2 / 360;

 


        DebugText.text = PosicionPunto.ToString() + axis.ToString();
  
        UnityEngine.Debug.Log("movej(p[" + UR_Control_Data.C_Position[0].ToString("F") + "," + UR_Control_Data.C_Position[1].ToString("F") + "," + UR_Control_Data.C_Position[2].ToString("F") + "," +
                    UR_Control_Data.C_Orientation[0].ToString("F") + "," + UR_Control_Data.C_Orientation[1].ToString("F") + "," + UR_Control_Data.C_Orientation[2].ToString("F") + "],a=0.5,v=0.5)" + "\n");
        ur_ctrl_robot = new UR_Control();
        ur_ctrl_robot.Start();
        
    }
   
 

 
}
public class UR_Stream
{
    // Initialization of Class variables
    //  Thread
    private Thread robot_thread = null;
    private bool exit_thread = false;
    //  TCP/IP Communication
    private TcpClient tcp_client = new TcpClient();
    private NetworkStream network_stream = null;
    //  Packet Buffer (Read)
    private byte[] packet = new byte[1116];

    // Offset:
    //  Size of first packet in bytes (Integer)
    private const byte first_packet_size = 4;
    //  Size of other packets in bytes (Double)
    private const byte offset = 8;

    // Total message length in bytes
    // Note: total_msg_length = 1409548288
    private const UInt32 total_msg_length = 3288596480;

    public void UR_Stream_Thread()
    {
        try
        {
            if (tcp_client.Connected == false)
            {
                // Connect to controller -> if the controller is disconnected
                tcp_client.Connect(UR_Stream_Data.ip_address, UR_Stream_Data.port_number);
            }

            // Initialization TCP/IP Communication (Stream)
            network_stream = tcp_client.GetStream();

            // Initialization timer
            var t = new Stopwatch();

            while (exit_thread == false)
            {
                // Get the data from the robot
                if (network_stream.Read(packet, 0, packet.Length) != 0)
                {
                    if (BitConverter.ToUInt32(packet, first_packet_size - 4) == total_msg_length)
                    {
                        // t_{0}: Timer start.
                        t.Start();

                        // Reverses the order of elements in a one-dimensional array or part of an array.
                        Array.Reverse(packet);

                        // Note:
                        //  For more information on values 32... 37, etc., see the UR Client Interface document.
                        // Read Joint Values in radians
                        UR_Stream_Data.J_Orientation[0] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (32 * offset));
                        UR_Stream_Data.J_Orientation[1] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (33 * offset));
                        UR_Stream_Data.J_Orientation[2] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (34 * offset));
                        UR_Stream_Data.J_Orientation[3] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (35 * offset));
                        UR_Stream_Data.J_Orientation[4] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (36 * offset));
                        UR_Stream_Data.J_Orientation[5] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (37 * offset));
                        // Read Cartesian (Positon) Values in metres
                        //Console.WriteLine(UR_Stream_Data.J_Orientation[0]);
                        UR_Stream_Data.C_Position[0] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (56 * offset));
                        UR_Stream_Data.C_Position[1] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (57 * offset));
                        UR_Stream_Data.C_Position[2] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (58 * offset));
                        // Read Cartesian (Orientation) Values in metres 
                        UR_Stream_Data.C_Orientation[0] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (59 * offset));
                        UR_Stream_Data.C_Orientation[1] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (60 * offset));
                        UR_Stream_Data.C_Orientation[2] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (61 * offset));

                        // t_{1}: Timer stop.
                        t.Stop();

                        // Recalculate the time: t = t_{1} - t_{0} -> Elapsed Time in milliseconds
                        if (t.ElapsedMilliseconds < UR_Stream_Data.time_step)
                        {
                            Thread.Sleep(UR_Stream_Data.time_step - (int)t.ElapsedMilliseconds);
                        }

                        // Reset (Restart) timer.
                        t.Restart();
                    }
                }
            }
        }
        catch (SocketException e)
        {
            UnityEngine.Debug.Log("SocketException: {0}");
            UnityEngine.Debug.Log(e);
        }
    }

    public void Start()
    {
        exit_thread = false;
        // Start a thread to control Universal Robots (UR)
        robot_thread = new Thread(new ThreadStart(UR_Stream_Thread));
        robot_thread.IsBackground = true;
        robot_thread.Start();
    }
    public void Stop()
    {
        exit_thread = true;
        // Stop a thread
        Thread.Sleep(100);
    }
    public void Destroy()
    {
        // Start a thread and disconnect tcp/ip communication
        Stop();
        if (tcp_client.Connected == true)
        {
            network_stream.Dispose();
            tcp_client.Close();
        }
        Thread.Sleep(100);
    }
}


public class UR_Control
{
    // Initialization of Class variables
    //  Thread
    private Thread robot_thread = null;
    private bool exit_thread = false;
    //  TCP/IP Communication
    private TcpClient tcp_client = new TcpClient();
    private NetworkStream network_stream = null;
    //  Packet Buffer (Write)
    private byte[] packet_cmd;
    //  Encoding
    private UTF8Encoding utf8 = new UTF8Encoding();

    public void UR_Control_Thread()
    {
    
        try
        {
            if (tcp_client.Connected == false)
            {
                // Connect to controller -> if the controller is disconnected
                tcp_client.Connect(UR_Control_Data.ip_address, UR_Control_Data.port_number);
            }

            // Initialization TCP/IP Communication (Stream)
            network_stream = tcp_client.GetStream();

            while (exit_thread == false)
            {
                packet_cmd = utf8.GetBytes("movel(p[" + UR_Control_Data.C_Position[0].ToString() + "," + UR_Control_Data.C_Position[1].ToString() + "," + UR_Control_Data.C_Position[2].ToString() + "," + 
                    UR_Control_Data.C_Orientation[0].ToString() + "," + UR_Control_Data.C_Orientation[1].ToString() + "," + UR_Control_Data.C_Orientation[2].ToString() + "],a=0.01,v=0.01)" + "\n");
                network_stream.Write(packet_cmd, 0, packet_cmd.Length);
                    Thread.Sleep(40000);
            }
        }
        catch (SocketException e)
        {
            UnityEngine.Debug.Log("SocketException: {0}");
            UnityEngine.Debug.Log(e);
        }
    }
    public void Start()
    {
        exit_thread = false;
        // Start a thread to control Universal Robots (UR)
        robot_thread = new Thread(new ThreadStart(UR_Control_Thread));
        robot_thread.IsBackground = true;
        robot_thread.Start();
    }
    public void Stop()
    {
        exit_thread = true;
        // Stop a thread
        Thread.Sleep(100);
    }
    public void Destroy()
    {
        // Start a thread and disconnect tcp/ip communication
        Stop();
        if (tcp_client.Connected == true)
        {
            network_stream.Dispose();
            tcp_client.Close();
        }
        Thread.Sleep(100);
    }
}

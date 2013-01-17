using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO.Ports;
using System.IO.Pipes;
using System.IO;
using System.Diagnostics;
using System.Collections;
using System.Threading;
using Microsoft.Win32;


namespace Wireshark_Sniffer_Interface
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        //members
        static SerialPort Com;
        static NamedPipeServerStream wspipe;
        static BinaryWriter ws;

        const uint BUFSIZE = 10240;
        static byte[] b = new byte[BUFSIZE];
        static FSM state;
        static long start_time_in_ticks;
        static byte frame_len = 0;
        static Queue q = new Queue();
        static bool wspipe_exist = false;
        static bool pipe_created = false;
        static byte previous_channel = 0;
        static bool restart_wireshark = false;
        static bool Pause_Button_Pressed = false;
        Process p;
        public Process CmdProcess;
        FileInfo Wireshark_Sniff_Packet;

        bool thread_alive;
        static Thread Writing_Pipe_Thread = new Thread(new ThreadStart(Pipe_Thread));

        enum FSM
        {
            START_CAPTURE,
            PACKET_CAPTURE,
            CONTROL_ENDPT
        }

        const int PACKET_FCS = 2;
        const int PACKET_LEN = 1;
        const bool DEBUG_PRINT = false;

        public MainWindow()
        {


            InitializeComponent();
            this.Closed += new EventHandler(MainWindow_Closed);

            do
            {
                var tt = new SerialPort();
                var t = SerialPort.GetPortNames();
                ComPort.ItemsSource = SerialPort.GetPortNames();


                InitilaiseUI();
            } while (Open.IsEnabled == true);
            channelNumber.ItemsSource = GetChannelValues();
            channelNumber.SelectedIndex = 0;

            startButton.IsEnabled = false;
            stopButton.IsEnabled = false;
            channelNumber.IsEnabled = false;
            setChannel.IsEnabled = false;
            PauseButton.IsEnabled = false;
            ResumeButton.IsEnabled = false;

            Writing_Pipe_Thread.Start();
            thread_alive = Writing_Pipe_Thread.IsAlive;
            this.CmdProcess = p;
          }

        void MainWindow_Closed(object sender, EventArgs e)
        {
                    Writing_Pipe_Thread.Abort();
                    p.Kill();
                    p.Close();            
                    close();

        }

        protected static void Pipe_Thread()
        {
            String frame_byte_ctrl;
            String frame_byte;
            while (true)
            {
            if(Pause_Button_Pressed != true)
            {

                while ((q.Count > 0))
                {

                    if (state == FSM.CONTROL_ENDPT)
                    {
                        var value1 = q.Dequeue();
                        if (value1 != null)
                        {
                            frame_byte_ctrl = value1.ToString();
                            frame_len = Byte.Parse(frame_byte_ctrl);
                            if (frame_len == 0x99)
                            {
                                state = FSM.START_CAPTURE;
                                lock (q)
                                {
                                    q.Clear();
                                }
                            }
                        }

                    }
                    else
                    {
                        if (wspipe != null)
                        {

                            var value = q.Dequeue();
                            if (value != null)
                            {
                                frame_byte = value.ToString();
                                frame_len = Byte.Parse(frame_byte);
                                write_frame(frame_len);
                            }
                        }
                    }
                }
            }
            }
        }


        private void InitilaiseUI()
        {
            if (ComPort.SelectedValue != null)
            {
                Open.IsEnabled = true;
            }
            else
            {
                Open.IsEnabled = false;
            }
        }

        /*private function to read com ports and give to UI*/
        private IEnumerable<string> GetChannelValues()
        {
            var channelvlues = new List<string>();
            for (int i = 11; i <= 26; i++)
            {
                channelvlues.Add(i.ToString());
            }
            return channelvlues;
        }

        private void button1_Click(object sender, RoutedEventArgs e)
        {
            var selectedChannel = channelNumber.SelectedItem.ToString();
            channelNumber.IsEnabled = false;
            startButton.IsEnabled = false;

        }
        private void button2_Click(object sender, RoutedEventArgs e)
        {
            channelNumber.IsEnabled = true;
            startButton.IsEnabled = true;
            OnStopButtonClicked();
        }

        private void OnStopButtonClicked()
        {
            MessageBox.Show(channelNumber.SelectedItem.ToString());
            //Should be removed.. test it.

        }

        private void OpenComPort(object sender, RoutedEventArgs e)
        {
            var comport = ComPort.SelectedItem.ToString();

            // Open serial port
            try
            {
                Com = new SerialPort(comport, 115200, Parity.None, 8, StopBits.One);
                Com.Open();
                Open.IsEnabled = false;
                ComPort.IsEnabled = false;
                setChannel.IsEnabled = true;
                channelNumber.IsEnabled = true;
                setChannel.IsEnabled = true;
                ConectionStatus.Content = "Sniffer Dongle Opened - Select Channel & Set";
            }
            catch (Exception serial)
            {
                MessageBox.Show("Error opening serial port. Msg = " + serial.Message);
            }


        }

        private void setChannel_Click(object sender, RoutedEventArgs e)
        {
            var selectedChannel = channelNumber.SelectedItem.ToString();
            byte Channel = byte.Parse(selectedChannel); // Try to parse the string as an integer

            if (wspipe_exist)
            {
                if (previous_channel != Channel)
                {
                    restart_wireshark = true;
                }
                else
                {
                    restart_wireshark = true;// false;
                }
            }
            previous_channel = Channel;

            byte[] BytesToSend = { Channel };
            Com.Write(BytesToSend, 0, 1);
            setChannel.IsEnabled = false;
            startButton.IsEnabled = true;
            channelNumber.IsEnabled = false;
            stopButton.IsEnabled = false;
            ComPort.IsEnabled = false;
            ConectionStatus.Content = "Click Start to capture in Wireshark";

        }

        private void File_Save()
        {
            StringBuilder outstr = new StringBuilder();
            string strFileLocation = "";
            string strDirectory = "C:/";
            string savedControls = outstr.ToString();
            string Wireshark_Local_FileName = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "Wireshark.cap");
            Wireshark_Sniff_Packet = new FileInfo(Wireshark_Local_FileName);
            long size = Wireshark_Sniff_Packet.Length;
            string  extension = Wireshark_Sniff_Packet.Extension;

            SaveFileDialog saveFileDialog1 = new SaveFileDialog();
            saveFileDialog1.RestoreDirectory = true;
            saveFileDialog1.Title = "Do you want save Wireshark capture Packet";
            saveFileDialog1.DefaultExt = "cap"; 
            saveFileDialog1.InitialDirectory = Convert.ToString(Environment.SpecialFolder.MyDocuments);
            saveFileDialog1.FileName = "Wireshark";
            saveFileDialog1.Filter = "Wireshark-libpcap (*.cap)|*.*";
            saveFileDialog1.FilterIndex = 1;  
            if (size >1024)
            {
                Nullable<bool> result = saveFileDialog1.ShowDialog();
                if(result == true)
                {
                    string filename = saveFileDialog1.FileName;
                    File.WriteAllText(filename, savedControls);
                    File.Copy(Wireshark_Sniff_Packet.FullName,filename,true); 
                } 
            }
            
        }

        private void restart()
        {               
                 
                    try
                    {
                        wspipe = new NamedPipeServerStream("wireshark", PipeDirection.Out);
                        wspipe_exist = true;
                    }
                    catch (Exception pipe)
                    {
                        MessageBox.Show("Error opening pipe. Msg = " + pipe.Message);
                        Com.Close();
                    }
                    try
                    {
                        p = new Process();
                        p.StartInfo.FileName = @"wireshark.exe";
                        string Wireshark_Local_FileName = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "Wireshark.cap");
                        using (System.IO.FileStream fs = System.IO.File.Create(Wireshark_Local_FileName))
                        {

                        }
                        p.StartInfo.Arguments = "-i \\\\.\\pipe\\wireshark -w \"" + Wireshark_Local_FileName + "\" -k";
                        p.StartInfo.CreateNoWindow = false;
                        p.Start();
                    }
                    catch (System.Exception ex)
                    {
                        MessageBox.Show(ex.Message);
                        MessageBox.Show("Wireshark Not Installed..Set Environment Path of Wireshark");
                        close();
                    }
                    try
                    {
                        wspipe.WaitForConnection();
                        ConectionStatus.Content = "Connected to Wireshark !!!";
                        ws = new BinaryWriter(wspipe);
                        Com.DataReceived += new SerialDataReceivedEventHandler(serialPort_DataReceived);
                        set_state_Config_header();
                    }
                    catch (Exception Process)
                    {
                        MessageBox.Show("Wireshark Restart Issue = " + Process.Message);
                        close();
                    }
                restart_wireshark = false;            
        }



        private void StartCaptureButton(object sender, RoutedEventArgs e)
        {
            if (!wspipe_exist)
            {
                try
                {
                    p = new Process();
                }
                catch (Exception Process)
                {
                    MessageBox.Show("Wireshark Not Installed..Set Environment Path of Wireshark");
                    Com.Close();
                }
                try
                {
                    wspipe = new NamedPipeServerStream("wireshark", PipeDirection.Out);
                    wspipe_exist = true;
                }
                catch (Exception pipe)
                {
                    MessageBox.Show("Error opening pipe. Msg = " + pipe.Message);
                    Com.Close();
                    //Environment.Exit(0);
                }

                try
                {

                    p.StartInfo.FileName = @"wireshark.exe";
                    string Wireshark_Local_FileName = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "Wireshark.cap");
                    using (System.IO.FileStream fs = System.IO.File.Create(Wireshark_Local_FileName))
                    {

                    }
                    p.StartInfo.Arguments = "-i \\\\.\\pipe\\wireshark -w \""+Wireshark_Local_FileName+"\" -k";
                    p.StartInfo.CreateNoWindow = false;
                    p.Start();
                }
                catch (Exception Process)
                {
                    MessageBox.Show("Wireshark Not Installed = " + Process.Message);
                }
                wspipe.WaitForConnection();
                ConectionStatus.Content = "Connected to Wireshark !!!";
                ws = new BinaryWriter(wspipe);
                Com.DataReceived += new SerialDataReceivedEventHandler(serialPort_DataReceived);
                set_state_Config_header();
            }

            if (restart_wireshark == true)
            {
                try
                {
                    lock (q)
                    {
                        q.Clear();
                        wspipe.Disconnect();
                        ws.Close();
                        wspipe.Close();
                        File_Save();
                        //Thread.Sleep(50);
                        p.Kill();
                        while (!p.HasExited) ;
                        Thread.Sleep(500);
                    }
                }
                catch (Exception Process)
                {
                    MessageBox.Show("Wireshark Saving & Restart Issue = " + Process.Message);
                    close();
                }
                restart();
            }  

            //send the start command
            byte[] BytesToSend = { 1 };
            Com.Write(BytesToSend, 0, 1);
            //enable the start button
            startButton.IsEnabled = false;
            stopButton.IsEnabled = true;
            setChannel.IsEnabled = false;
            ComPort.IsEnabled = false;
            Open.IsEnabled = false;
            channelNumber.IsEnabled = false;
            PauseButton.IsEnabled = true;
            ResumeButton.IsEnabled = true;
        }

        private void StopCaptureButton(object sender, RoutedEventArgs e)
        {
            state = FSM.CONTROL_ENDPT;
            ConectionStatus.Content = "Select Channel & Set";
            byte[] BytesToSend = { 2 };
            Com.Write(BytesToSend, 0, 1);
            //while (state == FSM.CONTROL_ENDPT) ;
            while (state == FSM.START_CAPTURE)
            {
                Com.Write(BytesToSend, 0, 1);
            }
            stopButton.IsEnabled = false;
            setChannel.IsEnabled = true;
            channelNumber.IsEnabled = true;
            setChannel.IsEnabled = true;
            startButton.IsEnabled = false;
            stopButton.IsEnabled = false;
            PauseButton.IsEnabled = false;
            ResumeButton.IsEnabled = false;
            state = FSM.START_CAPTURE;

        }

        private void PauseCaptureButton(object sender, RoutedEventArgs e)
        {
            //lock (q);
            Pause_Button_Pressed = true;
            ConectionStatus.Content = "Press Resume to Continue";
            stopButton.IsEnabled = false;
            setChannel.IsEnabled = false;
            channelNumber.IsEnabled = false;
            setChannel.IsEnabled = false;
            startButton.IsEnabled = false;
            PauseButton.IsEnabled = false;
            ResumeButton.IsEnabled = true;
        }
        private void ResumeCaptureButton(object sender, RoutedEventArgs e)
        {
            Pause_Button_Pressed = false;
            ConectionStatus.Content = "Press Stop or Resume";
            stopButton.IsEnabled = true;
            setChannel.IsEnabled = false;
            channelNumber.IsEnabled = false;
            setChannel.IsEnabled = false;
            startButton.IsEnabled = false;
            PauseButton.IsEnabled = true;
            ResumeButton.IsEnabled = false;
        }

        private void set_state_Config_header()
        {
            //Com.DataReceived += new SerialDataReceivedEventHandler(serialPort_DataReceived);
            state = FSM.START_CAPTURE;

            // keep track of time started. this will be used for timestamps
            start_time_in_ticks = DateTime.Now.Ticks;

            // generate header to identify the packet capture
            write_global_hdr();

            // run forever
        }

        private static void clearPipeBuffer()
        {
            MessageBox.Show("clearing pipe buffer");
            b = new byte[BUFSIZE];
        }


        private static void serialPort_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            // uint frame_len, bytes_in_buf;
            //uint bytes_in_buf;

            try
            {

                // loop until serial port buffer is empty
                while (Com.BytesToRead != 0)
                {
                    //q.Enqueue((byte)Com.ReadByte());
                    switch (state)
                    {
                        case FSM.START_CAPTURE:
                            byte value = (byte)Com.ReadByte();
                            if (value != null)
                            {
                                if (q.Count != 0xffffffff)
                                {
                                    q.Enqueue((byte)value);
                                }
                                else
                                {
                                    MessageBox.Show("Memory Full - Restart Wireshark and Sniffer Handler");

                                }
                            }

                            break;
                        case FSM.CONTROL_ENDPT:
                            byte value_ctr = (byte)Com.ReadByte();
                            if (value_ctr != null)
                            {
                                q.Enqueue((byte)value_ctr);
                            }

                            break;

                        default:
                            break;
                    }

                }
            }
            catch (System.Exception ex)
            {
                if (DEBUG_PRINT)
                {
                    //Console.WriteLine(ex);
                }
                ex = ex;
                //MessageBox.Show("Sniffer Dongle Port Closed - Kindly Restart");
            }

        }


        // this is the global header that starts any packet capture file. this will tell wireshark what 
        // kind of protocol it is (indicated by the DLT) as well as other information like endianness, etc.
        private static void write_global_hdr()
        {
            uint magic_num = 0xa1b2c3d4;    // used for endianness
            short version_major = 2;        // version
            short version_minor = 4;        // version
            int thiszone = 0;               // zone (unused)
            uint sigfigs = 0;               // significant figures (unused)
            uint snaplen = 65535;           // snapshot length (max value)
            uint network = 195;             // Data Link Type (DLT): indicates link layer protocol

            try
            {
                // write to wireshark pipe
                ws.Write(magic_num);
                ws.Write(version_major);
                ws.Write(version_minor);
                ws.Write(thiszone);
                ws.Write(sigfigs);
                ws.Write(snaplen);
                ws.Write(network);
            }
            catch (System.Exception ex)
            {
                MessageBox.Show("Inside write_global_hdr Pipe has been closed.");
                if (DEBUG_PRINT)
                {
                    Console.WriteLine(ex);
                }
                ex = ex;
                close();
            }
        }

        // this writes a frame header into wireshark in libpcap format. the format is simple and just
        // requires a timestamp and length
        static void write_frm_hdr(long sec, long usec, uint incl_len, uint orig_len)
        {
            //incl_len = 10;
            //orig_len = 12;
            try
            {
                if (ws != null)
                {
                    // write to wireshark
                    ws.Write((uint)sec);
                    ws.Write((uint)usec);
                    ws.Write(incl_len);
                    ws.Write((orig_len));
                }
            }
            catch (System.Exception ex)
            {
                if (DEBUG_PRINT)
                {
                    //Console.WriteLine(ex);
                    //Console.WriteLine("Something Happened Here - Pipe has been closed.");
                }
                ex = ex;
                close();
            }
        }

        // this writes a frame into wireshark. it calculates the timestamp and length and uses that 
        // for the frame header. it then writes captured bytes into wireshark
        static void write_frame(uint frame_len)
        {

             if ((frame_len >=5) && (frame_len < 127))

             {
            uint incl_len, orig_len;
            long sec, usec;

            String write_frame_byte_ctrl;

            // generating timestamp. its kind of cheesy but there isn't a unix timestamp mechanism in win. 
            // just counting ticks from when program was started. each tick is 100 nsec. 
            long diff_in_ticks = DateTime.Now.Ticks - start_time_in_ticks;  // get difference in ticks
            sec = diff_in_ticks / TimeSpan.TicksPerSecond;                  // get seconds
            diff_in_ticks -= (sec * TimeSpan.TicksPerSecond);               // subtract off seconds from total
            usec = diff_in_ticks / 10;                                      // get usec

            // calculate frame length. we won't be feeding frame checksum (FCS) into wireshark.
            incl_len = 0;
            orig_len = 0;
            incl_len = (uint)frame_len - PACKET_FCS;
            orig_len = frame_len;

            // write frame header first
            write_frm_hdr(sec, usec, incl_len, orig_len);
            int i = 0;
            while (i < (incl_len))
            {

                try
                {
                    if ((!(q.Count.Equals(0))) && (wspipe != null))
                    {
                        var write_frame_temp = q.Dequeue();
                        if (write_frame_temp != null)
                        {
                            write_frame_byte_ctrl = write_frame_temp.ToString();
                            byte data_sent = Byte.Parse(write_frame_byte_ctrl);
                            if (ws != null)
                            {
                                ws.Write(data_sent);
                                i++;
                            }
                            else
                            {
                                close();
                           }
                        }

                    }

                }
                catch (Exception e)
                {
                    if (DEBUG_PRINT)
                    {
                        //Console.WriteLine(e);
                        //Console.WriteLine(" Inside write_frame -> write_frame Pipe has been closed.");
                    }
                    close();
                }

            }
             }

        }


        public static void close()
        {
            Com.Close();
            //Writing_Pipe_Thread.Suspend();
            ws.Close();
            wspipe.Close();
           // p.Close();
            Environment.Exit(0);
        }

        private void Window_KeyDown(object sender, KeyEventArgs e)
        {

        }

        private void channelNumber_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {

        }

        private void ComPort_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (ComPort.SelectedValue != null)
            {
                Open.IsEnabled = true;
            }
            else
            {
                Open.IsEnabled = false;
            }
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {

        }
    }
}
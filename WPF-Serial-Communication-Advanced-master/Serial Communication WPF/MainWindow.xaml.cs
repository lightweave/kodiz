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
using System.IO;
using System.IO.Ports;
using System.Threading;
using System.Windows.Threading;
using System.Diagnostics;

namespace Serial_Communication_WPF
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    /// 
           // Позиция буфера, перед которой заканчиваются полученные из порта данные
    public partial class MainWindow : Window
    {
        #region variables
        //Richtextbox
        FlowDocument mcFlowDoc = new FlowDocument();
        Paragraph para = new Paragraph();

        FlowDocument mcFlowDoc1 = new FlowDocument();
        Paragraph para1 = new Paragraph();

        int old_i;
        //Serial 
        SerialPort serial = new SerialPort();
        // string recieved_data;

        // buffers
        public static byte[] recieveData;
        public static int recieveDataPosition;
        public static byte[] bufferForData;
        public static byte[] kadr;
        public static int bufferForDataCount;
        public static int bufferForDataCount_backup;
        private int receivedbytes;

        private static string DIR_path = @"";
        // private string DIR_path = @"e:\\RESULTS\\";
        private FileStream Frez_bin;
        BinaryWriter InputDataCopy;
        StreamWriter Frez_text;

        public static StringBuilder sb;
        public static StringBuilder sbdecoded;
        public static int bufferSize = 256;
        int block_number = 0;

        #endregion


        public MainWindow()
        {
            InitializeComponent();
            InitializeComponent();
            //overwite to ensure state
            Connect_btn.Content = "Connect";

            // buffers for data
            recieveData = new byte[0xFFFF];
            kadr = new byte[128];
            recieveDataPosition = 0;
            bufferForData = new byte[0xFFFF];
            bufferForDataCount = 0;



            sb = new StringBuilder(bufferSize);
            sbdecoded = new StringBuilder(bufferSize);

            // Opening of files
            {
                string file_path = DIR_path + DateTime.Today.ToShortDateString() + @"_" + DateTime.Now.Hour + @"-" + DateTime.Now.Minute;


                // Create a files to write to.
                try
                {
                    Frez_text = new StreamWriter(file_path + @".txt"); //, FileMode.Create
                }
                catch
                {
                    MessageBox.Show("Can not create File \n" + file_path + @".txt");
                    return;
                }

                try
                { //Frez_bin
                    Frez_bin = new FileStream(file_path + @".bin", FileMode.Create);
                    InputDataCopy = new BinaryWriter(Frez_bin);
                }
                catch
                {
                    MessageBox.Show("Can not create File \n" + file_path + @".bin");
                    return;
                }
            }

            Frez_text.Write("Sib\t Time");
            DateTime DT = DateTime.Now;

        }
        ~MainWindow()
        {
            InputDataCopy.Close();
            Frez_bin.Close();
            
            //Frez_text.Close();
            

            if (serial.IsOpen)
                serial.Close();
        }

        private void Connect_Comms(object sender, RoutedEventArgs e)
        {
            if (Connect_btn.Content == "Connect")
            {
                //Sets up serial port
                serial.PortName = Comm_Port_Names.Text;
                serial.BaudRate = Convert.ToInt32(Baud_Rates.Text);
                serial.Handshake = System.IO.Ports.Handshake.None;
                serial.Parity = Parity.None;
                serial.DataBits = 8;
                serial.StopBits = StopBits.One;
                serial.ReadTimeout = 400;
                serial.WriteTimeout = 50;
                serial.Open();

                //Sets button State and Creates function call on data recieved
                Connect_btn.Content = "Disconnect";
                serial.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(Recieve);

            }
            else
            {
                try // just in case serial port is not open could also be acheved using if(serial.IsOpen)
                {
                    serial.Close();
                    Connect_btn.Content = "Connect";
                }
                catch
                {
                    MessageBox.Show("Can not close COM port"); 
                }
            }
        }

        #region Recieving

        private delegate void UpdateUiTextDelegate(byte [] recieveData);
        private void Recieve(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            // Collecting the characters received to our 'buffer' (string).
            //recieved_data = serial.ReadExisting(); 
            int count = 0;
            int index = serial.BytesToRead;
            if (index < recieveData.Length)
            {
                try
                {
                    count = serial.Read(recieveData, 0, index);
                    if (count != index)
                        MessageBox.Show("Data loss: " + (count + index).ToString());
                    	else {	// Вариант, когда всё в порядке
                            receivedbytes = count; 
                        }

                }
                catch (TimeoutException) { MessageBox.Show("Time out"); }
            }
            else
            {
                serial.DiscardInBuffer();
                MessageBox.Show("Too long data block: " + index.ToString());
            }

            Dispatcher.Invoke(DispatcherPriority.Send, new UpdateUiTextDelegate(WriteData), recieveData);
        }
        private void WriteData(byte [] recieveData)
        {
            // write to disk
            InputDataCopy.Write(recieveData, 0, receivedbytes);


            Array.Copy(recieveData, 0, bufferForData, bufferForDataCount, receivedbytes);
            bufferForDataCount += receivedbytes;

            Show_block();


            // Assign the value of the recieved_data to the RichTextBox.
            para.Inlines.Clear();
            para.Inlines.Add(sb.ToString());

            mcFlowDoc.Blocks.Clear();
            mcFlowDoc.Blocks.Add(para);

            Commdata.Document = mcFlowDoc;
            Commdata.ScrollToEnd();

            para1.Inlines.Clear();
            para1.Inlines.Add(sbdecoded.ToString());
            mcFlowDoc1.Blocks.Clear();
            mcFlowDoc1.Blocks.Add(para1);
            Parsedata.Document = mcFlowDoc1;

            Parsedata.ScrollToEnd();
            // Get the current caret position.
            TextPointer caretPos = Parsedata.CaretPosition;

            // Set the TextPointer to the end of the current document.
            caretPos = caretPos.DocumentEnd;

            // Specify the new caret position at the end of the current document.
            Parsedata.CaretPosition = caretPos;



            Received_txt.Text = bufferForDataCount_backup.ToString();
        }

        private void Show_block()
        {
            if (receivedbytes == 0)
                return;

            bufferForDataCount_backup = bufferForDataCount_backup + bufferForDataCount;

            sb.Clear();
            block_number++;

            if (bufferForDataCount > 1000)
            {
                sb.Clear();
                sbdecoded.Clear();
                bufferForDataCount = 0;
            }
            
            

                // raw hex - left window
            for (int j = 0; j < receivedbytes; j++)
                {
                    if (j % 16 == 0)
                        sb.AppendLine(" ");

                    sb.AppendFormat(" {0,2:x2} ", recieveData[j]);
                }
                sb.AppendLine(" ");

                sbdecoded.AppendFormat("Принят блок номер: {0,6:d2} Длина блока: {1,6:d2}", block_number, receivedbytes);
                sbdecoded.AppendLine(" ");






                if (bufferForDataCount < 128)
                    return;

                    // metka 1 metka 2 поиск метки
                    for (int i = 0; i < bufferForDataCount; i++)
                    {
                        if (bufferForData[i] == 0xCC &&
                            bufferForData[i + 1] == 0x55 &&
                            bufferForDataCount - i >= 128)
                        {
                            Array.Copy(bufferForData, i, kadr, 0, 128);
                            Array.Copy(bufferForData, i + 128, bufferForData, 0, bufferForDataCount-(i+128));
                            bufferForDataCount = 0;
                        }

                    }

                





                int time = kadr[4] + kadr[5] * 256 + kadr[6] * 256 * 256 + kadr[7] * 256 * 256 * 256;

                switch (kadr[2])
                {
                    // packet type тип массива 0-5
                    // 0	- потоки и дозы
                    // 1	– спектр 1-го АЦП
                    // 2	– спектр 2-го АЦП
                    // 3	– спектр 1-го АЦП при наличии совпадений.
                    // 4	– спектр 2-го АЦП при наличии совпадений.
                    // 5	– массив кодов АЦП
                    // 16   - приветствие  
                    case 16:
                        sbdecoded.AppendLine("Принят блок приветствие");

                        sbdecoded.AppendFormat("Режим работы прибора {0}", kadr[3]);
                        sbdecoded.AppendFormat("Время прибора {0}", time);

                        for (int j = 0; j < 128; j++)
                        {
                            if (j % 16 == 0)
                                sbdecoded.AppendLine(" ");

                            sbdecoded.AppendFormat(" {0} ", (char)kadr[j]);
                        }

                        sbdecoded.AppendLine("");
                        
                        break;
                    case 0:
                        sbdecoded.AppendLine("Принят блок потоки и дозы");
                        sbdecoded.AppendFormat("Режим работы прибора {0}", kadr[3]);
                        sbdecoded.AppendFormat("Время прибора {0}", time);
                        sbdecoded.AppendLine("");

                       for (int j = 0; j < 30; j++)
                        {
                            if (j % 5 == 0)
                                sbdecoded.AppendLine(" ");

                            sbdecoded.AppendFormat(" {0,6:d6} ", 
                                kadr[4 * j + 8] + 
                                kadr[4 * j + 9] * 256 + 
                                kadr[4 * j + 10] * 256 * 256 + 
                                kadr[4 * j + 11] * 256 * 256 * 256);
                        }
                        sbdecoded.AppendLine(" ");   
                        break;

                    case 1:
                        sbdecoded.AppendLine("Принят блок спектр 1-го АЦП");
                        sbdecoded.AppendFormat("Режим работы прибора {0}", kadr[3]);
                        sbdecoded.AppendFormat("Время прибора {0}", time);
                        sbdecoded.AppendLine("");
                        for (int j = 0; j < 30; j++)
                        {
                            if (j % 5 == 0)
                                sbdecoded.AppendLine(" ");

                            sbdecoded.AppendFormat(" {0,6:d6} ", 
                                kadr[4 * j + 8] + 
                                kadr[4 * j + 9] * 256 + 
                                kadr[4 * j + 10] * 256 * 256 + 
                                kadr[4 * j + 11] * 256 * 256 * 256);
                        }
                        sbdecoded.AppendLine(" ");
                                             

                        break;

                    case 2:
                        sbdecoded.AppendLine("Принят блок спектр 2-го АЦП");
                        sbdecoded.AppendFormat("Режим работы прибора {0}", kadr[3]);
                        sbdecoded.AppendFormat("Время прибора {0}", time);
                        sbdecoded.AppendLine("");
                        for (int j = 0; j < 30; j++)
                        {
                            if (j % 5 == 0)
                                sbdecoded.AppendLine(" ");

                            sbdecoded.AppendFormat(" {0,6:d6} ", 
                                kadr[4 * j + 8] + 
                                kadr[4 * j + 9] * 256 + 
                                kadr[4 * j + 10] * 256 * 256 + 
                                kadr[4 * j + 11] * 256 * 256 * 256);
                        }
                        sbdecoded.AppendLine(" ");
                        break;
                    case 3:
                        sbdecoded.AppendLine("Принят блок спектр 1-го АЦП при наличии совпадений");
                        sbdecoded.AppendFormat("Режим работы прибора {0}", kadr[3]);
                        sbdecoded.AppendFormat("Время прибора {0}", time);
                        sbdecoded.AppendLine("");
                        for (int j = 0; j < 30; j++)
                        {
                            if (j % 5 == 0)
                                sbdecoded.AppendLine(" ");

                            sbdecoded.AppendFormat(" {0,6:d6} ", 
                                kadr[4 * j + 8] + 
                                kadr[4 * j + 9] * 256 + 
                                kadr[4 * j + 10] * 256 * 256 + 
                                kadr[4 * j + 11] * 256 * 256 * 256);
                        }
                        sbdecoded.AppendLine(" ");  
                        break;
                    case 4:
                        sbdecoded.AppendLine("Принят блок спектр 2-го АЦП при наличии совпадений");
                        sbdecoded.AppendFormat("Время прибора {0}", time);
                        sbdecoded.AppendLine("");
                        for (int j = 0; j < 30; j++)
                        {
                            if (j % 5 == 0)
                                sbdecoded.AppendLine(" ");

                            sbdecoded.AppendFormat(" {0,6:d6} ", 
                                kadr[4 * j + 8] + 
                                kadr[4 * j + 9] * 256 + 
                                kadr[4 * j + 10] * 256 * 256 + 
                                kadr[4 * j + 11] * 256 * 256 * 256);
                        }
                        sbdecoded.AppendLine(" ");  
                        break;
                    case 5:
                        sbdecoded.AppendLine("Принят блок массив кодов АЦП");
                        sbdecoded.AppendFormat("Время прибора {0}", time);
                        sbdecoded.AppendLine("");

                        for (int j = 0; j < 30; j++)
                        {
                            if (j % 2 == 0)
                                sbdecoded.AppendLine(" ");

                            sbdecoded.AppendFormat("   {0,1:x1} {1,4:d4}  {2,1:x1} {3,4:d4} ",
                                kadr[4 * j + 9] & 0xf0,
                                kadr[4 * j + 8] + (kadr[4 * j + 9] & 0x0f) * 256,
                                kadr[4 * j + 11] & 0xf0,
                                kadr[4 * j + 10] + (kadr[4 * j + 11] & 0x0f) * 256);
                        }
                        sbdecoded.AppendLine(" ");  

                        break;
                    default:
                        sbdecoded.AppendLine("Принят непредусмотренный блок");
                        break;
                }

            


            
        }
        #endregion

        bool AreEqual(byte[] a, byte[] b, byte[] mask)
        {
            for (int i = 0; i < a.Length; i++)
            {
                if ((a[i] & mask[i]) != (b[i] & mask[i])) return false;
            }
            return true;
        }
        #region Sending        
        
        private void Send_Data(object sender, RoutedEventArgs e)
        {
            SerialCmdSend(SerialData.Text);
            //SerialData.Text = "";
        }
        public void SerialCmdSend(string data)
        {
            if (serial.IsOpen)
            {
                try
                {
                    // Send the binary data out the port
                    byte[] hexstring = Encoding.ASCII.GetBytes(data);
                    //There is a intermitant problem that I came across
                    //If I write more than one byte in succesion without a 
                    //delay the PIC i'm communicating with will Crash
                    //I expect this id due to PC timing issues ad they are
                    //not directley connected to the COM port the solution
                    //Is a ver small 1 millisecound delay between chracters
                    foreach (byte hexval in hexstring)
                    {
                        byte[] _hexval = new byte[] { hexval }; // need to convert byte to byte[] to write
                        serial.Write(_hexval, 0, 1);
                        Thread.Sleep(1);
                    }
                }
                catch (Exception ex)
                {
                    para.Inlines.Add("Failed to SEND" + data + "\n" + ex + "\n");
                    mcFlowDoc.Blocks.Add(para);
                    Commdata.Document = mcFlowDoc;
                }
            }
            else
            {
            }
        }

        #endregion


        #region Form Controls

        private void Close_Form(object sender, RoutedEventArgs e)
        {
            if (serial.IsOpen) serial.Close();
            this.Close();
        }
        private void Max_size(object sender, RoutedEventArgs e)
        {
            if (this.WindowState != WindowState.Maximized) this.WindowState = WindowState.Maximized;
            else this.WindowState = WindowState.Normal;
        }
        private void Min_size(object sender, RoutedEventArgs e)
        {
            if (this.WindowState != WindowState.Minimized) this.WindowState = WindowState.Minimized;
            else this.WindowState = WindowState.Normal;
        }
        private void Move_Window(object sender, MouseButtonEventArgs e)
        {
            this.DragMove();
        }

        #endregion

        private void Baud_Rates_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            try // just in case serial port is not open could also be acheved using if(serial.IsOpen)
            {
                serial.Close();
                Connect_btn.Content = "Connect";
            }
            catch
            {
            }

        }

        private void Hyperlink_RequestNavigate(object sender, RequestNavigateEventArgs e)
        {
            // for .NET Core you need to add UseShellExecute = true
            // see https://docs.microsoft.com/dotnet/api/system.diagnostics.processstartinfo.useshellexecute#property-value
            Process.Start(new ProcessStartInfo(e.Uri.AbsoluteUri));
            e.Handled = true;
        }

        private void Up_Font(object sender, RoutedEventArgs e)
        {
            Parsedata.FontSize = Parsedata.FontSize++;
            Commdata.FontSize = Parsedata.FontSize++;
        }

        private void Down_Font(object sender, RoutedEventArgs e)
        {
            Parsedata.FontSize = Parsedata.FontSize--;
            Commdata.FontSize = Parsedata.FontSize--;
        }


    }
}

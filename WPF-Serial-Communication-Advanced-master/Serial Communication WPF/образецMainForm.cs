using System;

using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using System.IO;
using System.Collections.Specialized;
//using ZedGraph;
using System.Threading;
using System.Text.RegularExpressions;


namespace Db8m
{
  public partial class Terminal : Form
  {
  
    /// <summary>
    /// global variables
    /// </summary>
    public static SerialPort OnlySerialPort;

    // default connection 
    public int ReadTimeout = 1000;
    public int WriteTimeout = 500;
    public int defaultbaudrate = 19200;
    public int defaultdatabits = 8;
    public string defaultportnumber = "COM6";
    public Parity defaulparity = Parity.None;
    public StopBits defaultstopbits = StopBits.One;



    Thread AddToUICaller;
    private static System.Threading.Mutex mut = new System.Threading.Mutex();
      // buffers
    public static byte[] recieveData;
    public static int recieveDataPosition;
    public static byte[] bufferForData;
    public static int bufferForDataCount;       // Позиция буфера, перед которой заканчиваются полученные из порта данные
    public static int bufferForDataPosition;    // Позиция буфера, с которой начинаются данные, подлежащие анализу и отображению 
    public static int bufferForDataCheckedBytes;
    public static int blockLength = 48;
    public static byte[] buffer_Old_k_Data;

    public static int block_N0 = 0;
    public static int Expected_Length = 60;
    public static char Sended_Byte='\0';

    public static uint[] xi ;
    public static char[] sendData = {'U'};
      // string variables
    public static StringBuilder sb;
    public static StringBuilder sb_temp;
    public static StringBuilder sb1;
    public int[] SpectrADC1 = new int[4096];
    public int[] SpectrADC2 = new int[4096];
    string st1 = @"";
    public string[] row0;
    public string[] row1;
    public DataTable table = new DataTable();

    public static int[] Cumul_Spectr = new int[100]; 

    public static int bufferSize = 256;
    public static int numeral_base = 10;
    public static uint Metka = 61695; // 4279234815; f0ff
    public System.Windows.Forms.Timer tim;
    public System.Windows.Forms.Timer tim1;
    public System.Windows.Forms.Timer tim2;
        // table data
    public int tableindex;
    public int receivedBlocks;
    public bool neednewline;
      //
    public int receivedSpectrs;

    public int receivedbytes;

    //LineItem adc1Curve = new LineItem("Спектр ADC1");
    //LineItem adc2Curve = new LineItem("Спектр ADC2");
    // public DataBytes temporal_buffer;

    private static string DIR_path = @"..\\..\\results\\";
    // private string DIR_path = @"e:\\RESULTS\\";
    private FileStream Frez_bin;
    BinaryWriter InputDataCopy;
    StreamWriter Frez_text;
    private int SleepTime=20;
    
    private bool toggle = false;
    private bool toggleK = false;
 /*   private int Number_of_Mess= 5;  //  Число видов сообщений (информационных массивов) получаемых из Дэпрона/
    private int Mess_Tipe_Index=0;  //  Тип текущего информационного сообщения 
                                    //  0 - сообщения данного типа не предусмотрены; размер  2 байта (чтобы сдвинуть на длину "метки")
                                    //  1 - Эхо принятого символа;          размер сообщения 4 байта
                                    //  2 - Информация тестового режима;    размер сообщения 48 байт
                                    //  3 - Информация о потоках и дозах;   размер сообщения 40 байт
                                    //  4 - Информация  о спектрах;         размер сообщения 512 байт
                                    //  5 - информация
    private int[] Mess_Tipe_Size = { 2, 4, 48,40, 128 }; 
*/

    /// <summary>
    /// Констуктор
    /// </summary>
    public Terminal()
    {
      InitializeComponent();
      MyInitializeComponent();

      // this.rows = new System.Windows.Forms.DataGridViewRow[10];
      

      
      //StringCollection stringlines;
      
      OnlySerialPort = new SerialPort(defaultportnumber, defaultbaudrate, defaulparity, defaultdatabits, defaultstopbits);
      OnlySerialPort.WriteTimeout = WriteTimeout;
      OnlySerialPort.ReadTimeout = ReadTimeout;

        // buffers for data
      recieveData = new byte[0xFFFF];
      recieveDataPosition = 0;
      bufferForData = new byte[0xFFFF];
      buffer_Old_k_Data = new byte[0x7F];

      bufferForDataCount = 0;
      bufferForDataPosition = 0;
      bufferForDataCheckedBytes = 0;
      
      xi = new uint[0xFF];
      // temporal_buffer = new DataBytes();

        // string vars
      sb =      new StringBuilder(bufferSize);
      sb1 = new StringBuilder(bufferSize);
      sb_temp = new StringBuilder(bufferSize);
      row0 = new string[70];
      row1 = new string[2];
      tableindex = 0;
      neednewline = true;
      receivedBlocks = 0;
      receivedSpectrs = 0;

      for (int i = 0; i < 100; i++) Cumul_Spectr[i] = 0;
      
        
      //table.Columns.Add();//удалить
      //table.Columns.Add();

      tim = new System.Windows.Forms.Timer();
      tim1 = new System.Windows.Forms.Timer();
      tim2 = new System.Windows.Forms.Timer();
      tim.Interval = 10000;
      tim1.Interval = 1000;
      tim2.Interval = 10000;

        // Events
      tim.Tick += new EventHandler(tim_Tick);
      //tim2.Tick += new EventHandler(tim2_Tick);
      tim1.Tick += new EventHandler(tim1_Tick);
      OnlySerialPort.DataReceived += new SerialDataReceivedEventHandler(DataReceviedHandler);

      tim2.Start();

      // Opening of files
      {
          string file_path = DIR_path + DateTime.Today.ToShortDateString() + @"_" + DateTime.Now.Hour + @"-" + DateTime.Now.Minute;

          
          // Create a files to write to.
          try
          {
              Frez_text = new StreamWriter(file_path  + @".txt"); //, FileMode.Create
          }
          catch
          {
              MessageBox.Show("Can not create File \n" + file_path  + @".txt");
              return;
          }

          try
          { //Frez_bin
              Frez_bin = new FileStream(file_path +  @".bin", FileMode.Create);
              InputDataCopy = new BinaryWriter(Frez_bin);
          }
          catch
          {
              MessageBox.Show("Can not create File \n" + file_path  + @".bin");
              return;
          }
      }

      Frez_text.Write("Sib\t Time");
      DateTime DT = DateTime.Now;
      Frez_text.Write("\n g\t" + DT.ToLongTimeString() + "\tZav_No\t block\tIA1\tIC1\tDR1"); // 
      Frez_text.Write("\tIA2\tIC2\tDR2\tICS\tDS1\tEDS1"); // 2 byte
      Frez_text.Write("\tSI1\tSD1\tSI2\tSD2\tSIS\tSDS1\tEDS1\tDT"); // 4 byte
      Frez_text.Write("\tTemperature\tChSum"); // End of message
      Frez_text.Write("\tN_zero_ADS_1\tN_FF_ADS_1"); // For testing
      Frez_text.Write("\n k\t" + DT.ToLongTimeString() + "\tZav_No\t block\tKEY\tADC"); //     
      Frez_text.Write("\n s\t" + DT.ToLongTimeString() + "\tSpectr\t\tChennal No\tParticles"); // 
    }
     ~Terminal()
    {
        InputDataCopy.Close();
        Frez_text.Close();
        Frez_bin.Close();

        if (OnlySerialPort.IsOpen)
            OnlySerialPort.Close();
        Dispose();
    }

     

      /// <summary>
      /// Brand new hanler for incoming messeges on COM port
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
    private  void DataReceviedHandler(
                      object sender,
                      SerialDataReceivedEventArgs e)
    {
        SerialPort sp = (SerialPort)sender;
        System.Threading.Thread.Sleep(SleepTime);
        int count = 0;
        int index = sp.BytesToRead;
        if (index < recieveData.Length)
        {
            try
            {
                count = OnlySerialPort.Read(recieveData, 0, index);
                if (count != index)
                    MessageBox.Show("Data loss: " + (count + index).ToString());
                		//	else {	// Вариант, когда всё в порядке
      		            //        receivedbytes = count; Block_of_Data_Show(count);
			            //    }

            }
            catch (TimeoutException) { MessageBox.Show("Time out"); }
        }
        else
        {            
            OnlySerialPort.DiscardInBuffer();
            MessageBox.Show("Too long data block: " + index.ToString()); 
        }

        /* Try Circle buffer
        if (recieveDataPosition + index < recieveData.Length)
        {
            try
            {    
                count = OnlySerialPort.Read(recieveData, recieveDataPosition, index);
                if (count != index)
                    MessageBox.Show("Data loss: " + (count + index).ToString()); 
                recieveDataPosition += count;
            }
            catch (TimeoutException) { MessageBox.Show("Time out"); }
        }
        else 
        {
            try
            {
                count = OnlySerialPort.Read(recieveData, recieveDataPosition, recieveData.Length - recieveDataPosition);
                
                recieveDataPosition = OnlySerialPort.Read(recieveData, 0, index - count);
                
            }
            catch (TimeoutException) { MessageBox.Show("Time out"); }
            // OnlySerialPort.DiscardInBuffer();
            // MessageBox.Show("Too long data block: " + index.ToString()); 
        }
        */


        InputDataCopy.Write(recieveData, 0, count);
        receivedbytes = count;
        if (count > 1) Block_of_Data_Show(count); else bufferForDataCheckedBytes += count;
    }

    
// ==================================================================  
    //    unsafe private void Block_of_Data_Show(int count)
    unsafe private void Block_of_Data_Show(int count)
    {
        if (sb.Length > 10000) // shows two data blocks!!!  recieveTextBox.MaxLength / 2)
            sb.Clear();

        Array.Copy(recieveData, 0, bufferForData, bufferForDataCount, count); 
        bufferForDataCount += count;
      //  if(count<2) return;
		block_N0++;
        		switch (Sended_Byte){
                    case 'U': Expected_Length = 234; break;
                    case 'k': Expected_Length = 66; break; //121
                    case 'c': Expected_Length = 6; break;
                    case 't': Expected_Length = 20; break;
                    case '\0': Expected_Length = 24; break;
                    case 'r': Expected_Length = 24; break;
                    case 's': Expected_Length = 80; break;
                    case 'p': Expected_Length = 1; break;

                    default: Expected_Length = 6; break;
                }
      int i;
         if ((Sended_Byte == '\0')||(Sended_Byte == 'r'))
         {
                    sb.AppendLine(" ");
                    // Пытаемся отобразить строку в текстовом виде
                    int K = count;
                    if (K > 80) K = 80;
                    string info_str = "                                                                                ";
                    string temp = new string(info_str.ToCharArray());
             
 
             //string info_str = Encoding.Unicode.GetString(recieveData);
                        //new string(recieveData, 0, count, Encoding.ASCII);
                    fixed (char* pc = temp)
                    {
                        fixed (byte* pb = &recieveData[0])
                        {
                            byte* pb1; // = &recieveData[0]);
                            char* pc1;
                            pc1 = pc;
                          //  for (i = 0; i < 20; i++) *pc1++ = ' '; 
                          //  pc1 = pc;
                            pb1 = pb;
                            for (i = 0; i < K; i++)
                            {
                                *(byte*)pc1++ = *pb1++;
                               // pc1++;
                            }
                                //*(byte*)pc1++ = *pb1++;
                     //       *pc1 = '\0';
                        }
                        if (Sended_Byte == '\0')
                        {
                            sb.AppendLine(@"Resived unrequest block. Length= " + count.ToString() + "  " + temp);
                            sb.AppendFormat("ZAV No {0,02}    (0x{0,2:x2})    Start code ={1,2:x2}", recieveData[26], recieveData[27]);
                        }
                        else
                            sb.AppendLine(@"Responce on restart code. Length= " + count.ToString() + "  " + temp);
                        // sb.AppendLine(info_str.Trim());
                        sb.AppendLine(" ");
                        Sended_Byte = '\0'; // Признак того, что нового запроса на передачу данных не было
                        // *pc = '\0';
                    }
             
                    for (i = 0; i < count; i++) sb.AppendFormat(" {0,2:x2} ", recieveData[i]);
                    sb.AppendLine(" ");
                    SendToWindow();
                    bufferForDataCount = 0;
                    return;
         }

       // uint Metka = BitConverter.ToUInt16(bufferForData, 0);
         if (Sended_Byte == 'U' ) //&& bufferForData[0] == 0xa1)//Metka == 4080)
        {
        //    if (bufferForDataCount == Expected_Length || bufferForDataCount == (Expected_Length + 1))
                if (bufferForDataCount >= Expected_Length )

                {
                //sb.AppendLine(" ");
                //sb.AppendLine(@"Resived last part of g-block with length= " + count.ToString());
                //for (i = 0; i < count; i++) sb.AppendFormat(" {0,2:x2} ", recieveData[i]);
                //SendToWindow();
                Show_g_block(); bufferForDataCount = 0;
                Sended_Byte = '\0'; // Признак того, что нового запроса на передачу данных не было
                SleepTime=20;
                return; 
            }
            else
            {
                //sb.AppendLine(" ");
                //sb.AppendLine(@"Resived part of g-block with length= " + count.ToString());
                //for (i = 0; i < count; i++) sb.AppendFormat(" {0,2:x2} ", recieveData[i]);
                //SendToWindow();
                return;
            }
        }

         if (Sended_Byte == 'k') // && bufferForData[0] == 0xb1)//Metka == 4080)
         {
//             if (bufferForDataCount == Expected_Length || bufferForDataCount == (Expected_Length + 1))
             if (bufferForDataCount >= Expected_Length)
             {
                 sb.AppendLine(" ");
                 sb.AppendLine(@"Resived last part of k-block with length= " + count.ToString());
                 for (i = 0; i < count; i++) sb.AppendFormat(" {0,2:x2} ", recieveData[i]);
                 SendToWindow();
                 Show_k_block(); bufferForDataCount = 0;
                 Sended_Byte = '\0'; // Признак того, что нового запроса на передачу данных не было
                 SleepTime=20;
                 return;
             }
             else
             {
                 sb.AppendLine(" ");
                 sb.AppendLine(@"Resived part of k-block with length= " + count.ToString());
                 for (i = 0; i < count; i++) sb.AppendFormat(" {0,2:x2} ", recieveData[i]);
                 SendToWindow();
                 return;
             }
         }
         if (Sended_Byte == 's')
         {
             if (bufferForDataCount == Expected_Length || bufferForDataCount == (Expected_Length + 1))
             {
                 sb.AppendLine(" ");
                 sb.AppendLine(@"Resived last part of s-block with length= " + count.ToString());
                 for (i = 0; i < count; i++) sb.AppendFormat(" {0,2:x2} ", recieveData[i]);
                 SendToWindow();
                 Show_s_block(); bufferForDataCount = 0;
                 Sended_Byte = '\0'; 
                 return;
             }
             else
             {
                 sb.AppendLine(" ");
                 sb.AppendLine(@"Resived part of s-block with length= " + count.ToString());
                 for (i = 0; i < count; i++) sb.AppendFormat(" {0,2:x2} ", recieveData[i]);
                 SendToWindow();
                 return;
             }
         }
         if (Sended_Byte == 'p')
         {
             sb.AppendLine(" ");
             sb.AppendLine(@"Received p-block with length= " + count.ToString());
             for (i = 0; i < count; i++) sb.AppendFormat(" {0,2:x2} ", recieveData[i]);
             SendToWindow();
             Sended_Byte = '\0';
             return;
         }
        
        sb.AppendLine(" ");
        sb.AppendLine(@"Received block No"+block_N0.ToString()+"  with length= " + count.ToString() + @" Expected " + Expected_Length.ToString());
        for (i = 0; i < count; i++) sb.AppendFormat(" {0,2:x2} ", recieveData[i]);
        sb.AppendLine(" ");
       // for(i=0; i< 6; i++) sb.Append (recieveData[i].ToString());
        SendToWindow();
        bufferForDataCount = 0;
	}
// ==================================================================    
    private  void Show_g_block()
    {
        sb.Clear();
        int i,k; //,m;
        int Sp1_Summ = 0, Sp2_Summ = 0;
        DateTime DT = DateTime.Now;
        sb.AppendLine(" ");
        sb.AppendFormat("Get data DB    Zav. No {0,2}   (0x{0,2:x2})", bufferForData[0]);
        k = 0; for (i = 0; i < 233; i++) k += bufferForData[i];
        sb.AppendFormat("\t CHECK_SUMM {0,2:x2}   ", k &0x00ff);
/*        k = (bufferForData[1] << 8) | bufferForData[0];
        sb.AppendFormat("Metka={0,10}   ", k); // decimal
 */ 
        k = bufferForData[1];
        sb.AppendFormat("block {0,10} ", k); // decimal
        Frez_text.Write("\n g\t" + DT.ToLongTimeString()); //            .ToShortTimeString());  DateTime.Today
//        Frez_text.Write("\t block=\t{0,10}\t", k);
        Frez_text.Write("\t{0,2:x2}\t{1,10}", bufferForData[0], bufferForData[1]); // 
        k =  (bufferForData[3] << 8) | bufferForData[2];
        sb.AppendLine(" ");
        sb.AppendFormat("IA1 counter = {0,10} ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        k =  (bufferForData[5] << 8) | bufferForData[4];
        sb.AppendFormat("IC1 counter = {0,10} ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        k = (bufferForData[7] << 8) | bufferForData[6];
        sb.AppendFormat("DR1 counter = {0,10} ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        sb.AppendLine(" ");

        k = (bufferForData[9] << 8) | bufferForData[8];
        sb.AppendFormat("IA2 counter = {0,10} ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        k = (bufferForData[11] << 8) | bufferForData[10];
        sb.AppendFormat("IC2 counter = {0,10} ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        k = (bufferForData[13] << 8) | bufferForData[12];   //(bufferForData[15] << 24) | (bufferForData[14] << 16)|
        sb.AppendFormat("DR2 counter = {0,10} ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        k = (bufferForData[15] << 8) | bufferForData[14];   //(bufferForData[15] << 24) | (bufferForData[14] << 16)|
        sb.AppendLine(" ");

        sb.AppendFormat("ICS counter = {0,10} ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        k = (bufferForData[17] << 8) | bufferForData[16];   //(bufferForData[15] << 24) | (bufferForData[14] << 16)|
        sb.AppendFormat("DS1 counter = {0,10} ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        k = (bufferForData[19] << 8) | bufferForData[18];   //(bufferForData[15] << 24) | (bufferForData[14] << 16)|
        sb.AppendFormat("EDS1 counter = {0,9} ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        sb.AppendLine(" ");

        k = (bufferForData[20] | bufferForData[21] << 8 | bufferForData[22] << 16 | bufferForData[23] << 24);
        sb.AppendFormat("SI1 = {0,10}  ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        k = (bufferForData[24] | bufferForData[25] << 8 | bufferForData[26] << 16 | bufferForData[27] << 24);
        sb.AppendFormat("SD1 = {0,10}  ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        k = (bufferForData[28] | bufferForData[29] << 8 | bufferForData[30] << 16 | bufferForData[31] << 24);
        sb.AppendFormat("SI2 = {0,10}  ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        k = (bufferForData[32] | bufferForData[33] << 8 | bufferForData[34] << 16 | bufferForData[35] << 24);
        sb.AppendFormat("SD2 = {0,10}  ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        sb.AppendLine(" ");

        k = (bufferForData[36] | bufferForData[37] << 8 | bufferForData[38] << 16 | bufferForData[39] << 24);
        sb.AppendFormat("SIS = {0,10}  ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        k = (bufferForData[40] | bufferForData[41] << 8 | bufferForData[42] << 16 | bufferForData[43] << 24);
        sb.AppendFormat("SDS1 = {0,10}  ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        k = (bufferForData[44] | bufferForData[45] << 8 | bufferForData[46] << 16 | bufferForData[47] << 24);
        sb.AppendFormat("EDS1 = {0,10}  ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        k = ( bufferForData[228] | bufferForData[229] << 8 | bufferForData[230] << 16 | bufferForData[231] << 24);
        sb.AppendFormat("DT = {0,10}  ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        sb.AppendLine(" ");
        // --------------------------------------------------------
        k = bufferForData[232];
        sb.AppendFormat("Temperature = {0,10}  ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        k = bufferForData[233];
        sb.AppendFormat("ChSum = {0,2:x2}", k); // decimal
        Frez_text.Write("\t{0,2:x2}  ", k);
        sb.AppendLine(" ");
/*        // --------------------------------------------------------
        k = (bufferForData[53] << 8) | bufferForData[52];
        sb.AppendFormat("ZERO counter= {0,10}  ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
        k = (bufferForData[55] << 8) | bufferForData[54];
        sb.AppendFormat("FF counter= {0,10}  ", k); // decimal
        Frez_text.Write("\t{0,10}", k);
*/        // --------------------------------------------------------
        sb.AppendLine(" ");
        sb.AppendLine("Spectrums ");
        Frez_text.Write("\n S\t" + DT.ToLongTimeString() + "\t Spectr");
        for (i = 0; i < 30; i++)
        {
            sb.AppendLine(" ");
            sb.AppendFormat("Channel No=\t{0,2}   | ", i); // decimal
            Frez_text.Write("\n S\t\t\t\t{0,10}", i);

            for (int j = 0; j < 3; j++)
            {
                k = (bufferForData[49 + 2 * i + 60 * j] << 8) | bufferForData[48 + 2 * i + 60 * j];
                sb.AppendFormat(" {0,10}  ",  k); // decimal
                Frez_text.Write("\t{0,10}", k);
                Cumul_Spectr[i + 30 * j] += k;
                Sp1_Summ += k;
                Sp2_Summ += Cumul_Spectr[i + 30 * j];
            }
            sb.AppendFormat(" |  ");
            Frez_text.Write("\t");
            for (int j = 0; j < 3; j++)
            {
                sb.AppendFormat(" {0,10}  ", Cumul_Spectr[i + 30 * j]); // decimal
                Frez_text.Write("\t{0,10}", Cumul_Spectr[i + 30 * j]);
            }
            sb.AppendFormat("  |");
        }
        sb.AppendLine(" ");
        sb.AppendLine(" ");
        sb.AppendFormat("\t\t Sp1_Summ = {0,10}\t\t\t\t\t Sp2_Summ =  {1,10} ", Sp1_Summ, Sp2_Summ);
        sb.AppendLine(" ");

        Frez_text.Write("\n S\t-----------------------------------------------------");
        SendToWindow();
        


    }
// ==================================================================    
    private  void Show_k_block()
    {
        int i, k, m, L;
        DateTime DT = DateTime.Now;
//        sb.AppendLine("Amplitudes");
        sb.AppendLine(" ");
        sb.AppendFormat("Amplitudes  block counter = {0,2}", bufferForData[bufferForDataPosition + 1]);
        sb.AppendLine(" ");
        Frez_text.Write("\n k\t" + DT.ToLongTimeString());
        Frez_text.Write("\t{0,2:x2}\t{1,4:10}", bufferForData[0], bufferForData[1]); // 
        for (i = 2; i < 66; i += 2) // 120
            if ((bufferForData[bufferForDataPosition + i ] != buffer_Old_k_Data[ i ])
                || (bufferForData[bufferForDataPosition + i + 1] != buffer_Old_k_Data[ i + 1]))
        {
            buffer_Old_k_Data[i] = bufferForData[bufferForDataPosition + i];
            buffer_Old_k_Data[i+1] = bufferForData[bufferForDataPosition + i+1];
            m = ((bufferForData[bufferForDataPosition + i + 1] & 0x70) >> 4);
            L = (bufferForData[bufferForDataPosition + i + 1] >>7);
            k = ((bufferForData[bufferForDataPosition + i + 1] & 0x0f) << 8) | bufferForData[bufferForDataPosition + i];
            if ((bufferForData[bufferForDataPosition + i + 1] & 0x80) ==0x00) sb.AppendFormat("  "); else sb.AppendFormat("| ");
            sb.AppendFormat("ADC No=\t{0,2}\t", m); // decimal
            sb.AppendFormat(" Code=\t{0,10}\r\n", k); // decimal
//            Frez_text.Write("{0,10}\t", m);
//            Frez_text.Write("{0,10}\t", k);
            Frez_text.Write("\n k\t\t\t\t{0,10}\t{1,10}", m, k);
//            Frez_text.Write("\n k\t\t\t\t{0,4:D}\t{1,4:D}", m, k);
            
            if ((m & 1)!=0)
            {

                if ((k >= 0) && (k < SpectrADC1.Length)) SpectrADC1[k]++;
            }
            if ((m & 2) != 0)
            {
                if ((k >= 0) && (k < SpectrADC2.Length)) SpectrADC2[k]++;
            }
        }
        SendToWindow();
    }
    // ==================================================================    
    private void Show_s_block()
    {
        int i, k; //, m;
        DateTime DT = DateTime.Now;
        sb.AppendLine(" ");
        sb.AppendLine("Spectr ");
        Frez_text.Write("\n s\t" + DT.ToLongTimeString() + "\t Spectr");
        for (i = 0; i < 30; i++)
        {
            sb.AppendLine(" ");
            k = (bufferForData[bufferForDataPosition + 2 * i + 1] << 8) | bufferForData[bufferForDataPosition + 2*i];
            sb.AppendFormat("Channel No=\t{0,2}   ", i); // decimal
            sb.AppendFormat("{0,10}", k); // decimal
            Frez_text.Write("\n s\t\t\t\t{0,10}\t{1,10}", i, k);

        }
        SendToWindow();
    }

    // ==================================================================    


    /// <summary>
    /// Copeing received data to global array
    /// it ssems that it's not used 
    /// </summary>
    /// <param name="index"></param>
    private  void CopyRaw(int count)
    {
        InputDataCopy.Write(recieveData, 0, count);   // Запись принятых байтов в файл
        if (count < bufferForData.Length - bufferForDataCount)
        {
            // recieveData -> bufferForData
            Array.Copy(recieveData , 0 , bufferForData, bufferForDataCount, count); 
            bufferForDataCount += count;
        }
        else 
        {
            SaveToFile(true);

            Array.Copy(bufferForData, bufferForDataPosition, bufferForData, 0, bufferForDataCount - bufferForDataPosition);
            bufferForDataCount = bufferForDataCount - bufferForDataPosition;
            bufferForDataPosition = 0;

            // recieveData -> bufferForData
            Array.Copy(recieveData, 0, bufferForData, bufferForDataCount, count);
            bufferForDataCount += count;

            // MessageBox.Show("Too long data stream received\r\n While program was saving file"); 
        }

    }

      /// <summary>
      /// get incoming stream as 32-byte array and present to textbox
      /// </summary>
      /// <param name="index"></param>
    private  void AddToUI()
    {
        
        if (bufferForDataPosition > bufferForDataCount)
            bufferForDataPosition = 0;

        
        
        if(bufferForDataCount - bufferForDataPosition < 60) 
                    return;// ожидание окончания посылки
        
        int j = 0;
        // searching for Metka
        while (j < bufferForDataCount - bufferForDataPosition - 4)
        {
            if (bufferForDataCount - bufferForDataPosition - j < 60) return;// ожидание окончания посылки

            if (((BitConverter.ToUInt16(bufferForData, bufferForDataPosition + j)) == 4080)   )
            {
                AddToTextBox();
                bufferForDataPosition += 60; 
            }
            else j++;
        }
        bufferForDataPosition += j; // Не нашли метку до конца имеющихся данных
        return;
        
        
    }

    private void AddToTextBox() 
    {
   
        int k;
        int i;
        sb.AppendLine(" ");
        sb.AppendLine("Get data");
        for (i = 0; i < bufferForDataCount - bufferForDataPosition; i++)
        {
            sb.AppendFormat("0x{0,2:x2} ", bufferForData[i + bufferForDataPosition]);
        }
        sb.AppendLine(" ");
        sb.AppendLine("Int value");

        k = (bufferForData[1 + bufferForDataPosition] << 8) | bufferForData[bufferForDataPosition];
        sb.AppendFormat("Metka={0,10} ", k); // decimal
        k = bufferForData[2 + bufferForDataPosition];
        sb.AppendFormat("block {0,10} ", k); // decimal
        k = (bufferForData[3 + bufferForDataPosition] << 16) | (bufferForData[5 + bufferForDataPosition] << 8) | bufferForData[4 + bufferForDataPosition];
        sb.AppendFormat("{0,10}", k); // decimal

        for (i = 6; i < bufferForDataCount - bufferForDataPosition; i += 2)
        {
            k = (bufferForData[bufferForDataPosition + i + 1] << 8) | bufferForData[bufferForDataPosition + i];
            sb.AppendFormat("{0,10}", k); // decimal
            
            if ((k >= 0) && (k < SpectrADC1.Length))
                SpectrADC1[k]++;

        }

        SendToWindow();

    }

      /// <summary>
      /// Функция для выделения значения кода АЦП из пришедшей строки
      /// </summary>
      /// <param name="count"></param>
    private void CopyToSpectr(string count)
    {
        string[] temp = st1.Split(new Char[] { ':' });
        int k = 0;
        if (temp[0] == "ADC1")
        {
            k = int.Parse(temp[1]);
            if( (k >= 0) && (k < SpectrADC1.Length))
                SpectrADC1[k]++;
        }

        if (temp[0] == "ADC2")
        {
            k = int.Parse(temp[1]);
            if( (k >= 0) && (k < SpectrADC2.Length))
                SpectrADC2[k]++;
        }
    }



    /// <summary>
    /// Show echo Litteral
    /// </summary>
    private void ECHO_Show() 
    {
        if (sb.Length > 10000)
            sb.Clear();
        sb.AppendLine();
        xi.Initialize(); //zeroes
        sb.Append("ECHO " + "\t");
        sb.Append((char)bufferForData[bufferForDataPosition + 2] + "\t");
        sb.Append((char)bufferForData[bufferForDataPosition + 3] + "\t");

        SendToWindow();
    }
    /// <summary>
    /// Show Test_Data
    /// </summary>
    private void Test_Data_Show()
    {
        if (sb.Length > 10000)
            sb.Clear();
        sb.AppendLine();
        xi.Initialize(); //zeroes
        sb.Append("Test_Data_ " + "\t");
        sb.Append((char)bufferForData[bufferForDataPosition + 2] + "\t");
        sb.Append((char)bufferForData[bufferForDataPosition + 3] + "\t");

        SendToWindow();
    }
    /// <summary>
    /// Show Flux
    /// </summary>
    private void Flux_Show()
    {
        if (sb.Length > 10000)
            sb.Clear();
        sb.AppendLine();
        xi.Initialize(); //zeroes
        sb.Append("Flux_Show " + "\t");
        sb.Append((char)bufferForData[bufferForDataPosition + 2] + "\t");
        sb.Append((char)bufferForData[bufferForDataPosition + 3] + "\t");

        SendToWindow();
    }
    /// <summary>
    /// Show Spectr
    /// </summary>
    private void Spectr_Show()
    {
        if (sb.Length > 10000)
            sb.Clear();
        sb.AppendLine();
        xi.Initialize(); //zeroes
        sb.Append("Spectr_Show " + "\t");
        sb.Append((char)bufferForData[bufferForDataPosition + 2] + "\t");
        sb.Append((char)bufferForData[bufferForDataPosition + 3] + "\t");

        SendToWindow();
    }

    private static void ConvertToBase(uint i, int numeral_base_int)
    {
        sb_temp.Clear();

        switch (numeral_base_int)
        {
            case 10:
                sb_temp.AppendFormat("{0,10}", i); // decimal
                break;
            case 2:
                sb_temp.AppendFormat("{0,32} ", Convert.ToString(i, 2));   // binary format, need test!
                // MessageBox.Show("Is conversion correct?");
                break;
            case 16:
                sb_temp.AppendFormat("0x{0,8:x8} ", i); // hexademical
                break;
            default:
                MessageBox.Show("Cannot convert to " + numeral_base_int.ToString() + " numeral system, use 2, 10 or 16");
                break;

            // {индекс[,длина][:строкаФормата]} 

        }

    }


      /// <summary>
      /// Writing received infornation to UI
      /// </summary>
    [STAThread]
    public void SendToWindow()
    {

        recieveTextBox.Invoke(new EventHandler(delegate
            {
                recieveTextBox.Text = sb.ToString();
                recieveTextBox.ScrollToCaret();
       //         sendTextBox.Text = sb1.ToString();
                receivedBytesTextBox1.Text = bufferForDataCount.ToString();
                checkedBytesTextBox.Text = bufferForDataCheckedBytes.ToString();
            }));
    }


    private void AddToTable()
    {
 
        row0.Initialize();

        for (int i = 0; i < blockLength / 4 ; i++)
        {
            if (BitConverter.ToUInt16(bufferForData, bufferForDataPosition + 4 * i) == Metka)
            {
                tableindex = 0;
                neednewline = true; 
            }

            switch (tableindex)
            {
                case 0:
                    ConvertToBase(BitConverter.ToUInt16(bufferForData, bufferForDataPosition + 4 * i), 10);
                    sb_temp.Append("  " + (char)bufferForData[bufferForDataPosition + 4 * i +2]);                    
                    row0[tableindex] = sb_temp.ToString();
                    tableindex++;
                    sb_temp.Clear();
                    sb_temp.Append("  " + ((int)bufferForData[bufferForDataPosition + 4 * i + 3]).ToString());
                    break;                
                case 3:
                    ConvertToBase(BitConverter.ToUInt32(bufferForData, bufferForDataPosition + 4 * i), 2);
                    break;
                case 4:
                    ConvertToBase(BitConverter.ToUInt32(bufferForData, bufferForDataPosition + 4 * i), 2);
                    break;
                case 5:
                    ConvertToBase(BitConverter.ToUInt16(bufferForData, bufferForDataPosition + 4 * i), 10);
                    row0[tableindex] = sb_temp.ToString();
                    tableindex++;
                    ConvertToBase(BitConverter.ToUInt16(bufferForData, bufferForDataPosition + 4 * i + 2), 10);
                    break;
                default:
                    ConvertToBase(BitConverter.ToUInt32(bufferForData, bufferForDataPosition + 4 * i), 10);
                    break;
            }
            row0[tableindex] = sb_temp.ToString();
            tableindex++;

            if (tableindex > 13){
                receivedBlocks++;
                SendToTable();
                tableindex = 0;
            }
        }         
        
    }

    public void SendToTable()
    {
        if (InvokeRequired)
            Invoke(new SendToTableDelegate(SendToTable));
        else
            WriteRowToTable();
    }
    delegate void SendToTableDelegate();


      /// <summary>
      /// writing data to table cells
      /// </summary>
    public void WriteRowToTable()
    {

        if (neednewline)
            this.dataGridView1.Rows.Insert(0, row0);
        this.dataGridView1.Rows[0].HeaderCell.Value = receivedBlocks.ToString();
        this.dataGridView1.Rows[0].SetValues(row0);

    }

    private void AddToSpectr()
    {
        

        row1.Initialize();
        
        for (int i = 0; i < 4096; i++)
        {
            row1[0] = i.ToString();
            row1[1] = SpectrADC1[i].ToString();

            table.Rows.Add(row1);
        }
        // This was excluded because slow the program
        // SendToSpectr();
    }
    public void SendToSpectr()
    {
        if (InvokeRequired)
            Invoke(new SendToSpectrDelegate(SendToSpectr));
        else
            WriteToSpectr();
    }
    delegate void SendToSpectrDelegate();


    /// <summary>
    /// writing data to table cells, not work now
    /// </summary>
    public void WriteToSpectr()
    {

        this.dataGridView2.DataSource = table; 
        this.dataGridView2.Refresh();
        
    }


      /// <summary>
      /// sending one byte on COM port
      /// </summary>
    public static void Write()
    {
        if (OnlySerialPort.IsOpen)
        {
            try
            {
                OnlySerialPort.Write(sendData, 0, 1);
            }
            catch (TimeoutException) { MessageBox.Show("Time out"); }
        }
        else { MessageBox.Show("COM port is closed"); }
      
     }

      /// <summary>
      /// actually not used, see DataReceivedHandler
      /// </summary>
    public static void Read()
    {
        try
        {
          int count = OnlySerialPort.Read(recieveData, 0, 10);
        }
        catch (TimeoutException) { MessageBox.Show("Time out"); }
        
        
        for (int i = 0; i<=(recieveData.Length/2 - 1); i++)
        {
            // Old Small-Endian Type data sending
            // xi[i] = (recieveData[2 * i] << 8) | recieveData[2 * i + 1];
        }

    }




    private void connectButton_Click(object sender, EventArgs e)
    {
      try
      {
          if (OnlySerialPort.IsOpen == false)
          {
              OnlySerialPort.DataReceived += new SerialDataReceivedEventHandler(DataReceviedHandler);
              try
              {
                  OnlySerialPort.Open();
              }
              catch (Exception ex)
              {
                  MessageBox.Show("Ошибка открытия последовательного порта: \n" + ex.ToString());
                  
              }
              connectButton.Text = "Disconnect";
              connectButton.Refresh();
          }
          else
          {

              OnlySerialPort.DataReceived -= new SerialDataReceivedEventHandler(DataReceviedHandler);
              OnlySerialPort.DiscardInBuffer();
              OnlySerialPort.DiscardOutBuffer();
              OnlySerialPort.Close();
              while (OnlySerialPort.IsOpen == true)
              {
                  connectButton.Text = "Disconnecting...";
                  connectButton.Refresh();
                  // Thread.Sleep(1000);
                  
              }
              connectButton.Text = "Connect";
              connectButton.Refresh();
          }

      }
      catch (UnauthorizedAccessException)
      {
          MessageBox.Show("COM5 port is Already used by another Application"); 
      }

   
    }
      /// <summary>
      /// used for testing purposes - converting to other formats
      /// </summary>
      /// <param name="sender"></param>
      /// <param name="e"></param>
    private void Terminal_Load(object sender, EventArgs e)
    {
        try
        {
            if (!OnlySerialPort.IsOpen)
            {
                OnlySerialPort.Open();

            }
        }
        catch (UnauthorizedAccessException)
        {
            MessageBox.Show("COM port is Already used by another Application");
        }
        catch (IOException)
        {
            MessageBox.Show("COM Порт находится в недействительном состоянии.");
        }
        

        if (OnlySerialPort.IsOpen)
        {
            connectButton.Text = "Disconnect";
            connectButton.Refresh();
        }
        //CreateGraph( zedGraphControl1 );
        
    }
    private void OnTerminalClosing(Object sender, FormClosingEventArgs e)
    {
        tim.Stop();

        //SaveToFile(false);
       // SaveToFile(true);

        // InputDataCopy.Close();
        if (Frez_text !=null)
        {
            Frez_text.Close();
            Frez_bin.Close(); 
        }

        if (OnlySerialPort.IsOpen)
            OnlySerialPort.Close();
    }

    private void sendTextBox_TextChanged(object sender, EventArgs e)
    {

      char b = sendTextBox.Text.LastOrDefault(); //copeing last char
      Sended_Byte =  b;
      sendData[0] = b;
      if ((b == 'U') && (sb.Length > 4000))
      {
          sb.Clear();
          SleepTime=100;
      }
      if ((b == 'k') && (sb.Length > 4000))
      {
          sb.Clear();
          SleepTime = 70;
      }

      //if ((b == 'U') && (Regex.Matches(sb.ToString(), Environment.NewLine).Count> 40))
      //    sb.Clear();
      sb.AppendLine(" ");
      sb.AppendLine("Sending_Byte: " + b);
      sb.AppendFormat(" {0,2:x2} ", b); 
      sb.AppendLine("Sending_sendData[0]: " + sendData[0]);

      DateTime DT = DateTime.Now;
      Frez_text.Write("\nSend\t" + DT.ToLongTimeString() + "\t" + b + "\t 0x {0,2:x2}\t = {0,10}", (int)b); //    " {0,2:x2} " +        .ToShortTimeString());  DateTime.Today

      bufferForDataCount = 0;
      bufferForDataPosition = 0;
      bufferForDataCheckedBytes = 0;
      block_N0 = 0;
      SendToWindow();

      //sendTextBox.Text = sendTextBox.Text.Substring(0, sendTextBox.Text.Length); //deleting last char
      
      Write();
        

    }


    private void repeatButton_Click(object sender, EventArgs e)
    {
      if (!toggle)
      {
    //    sb.Clear();
        repeatButton.Text = "STOP";
        repeatButton.Refresh();
        toggle = true;
        tim.Start();
      }
      else
      {
        repeatButton.Text = "RUN";
        repeatButton.Refresh();
        toggle = false;
        tim.Stop();
      }
      
    }


     
    private void tim_Tick(object sender, EventArgs e)
    {
      //Sended_Byte=sendData[0] ;
      sb.AppendLine(" ");
      sb.AppendLine("Repeated Sending_Byte: " + (char)Sended_Byte);
      Sended_Byte = 'U';
      sendData[0] = 'U';

      if (sb.Length > 4000)
          sb.Clear();

      bufferForDataCount = 0;
      bufferForDataPosition = 0;
      bufferForDataCheckedBytes = 0;
      block_N0 = 0;
      SendToWindow();

        Write();
        DateTime DT = DateTime.Now;
        Frez_text.Write("\nSend\t" + DT.ToLongTimeString() + "\t\t" + Sended_Byte); //            .ToShortTimeString());  DateTime.Today

    }

    private void tim1_Tick(object sender, EventArgs e)
    {
        //Sended_Byte=sendData[0] ;

        Sended_Byte = 'k';
        sendData[0] = 'k';
  //      if (sb.Length > 4000)
            sb.Clear();
        sb.AppendLine(" Spectr accumulation         ");
        sb.AppendLine("Repeated Sending_Byte: " + (char)Sended_Byte);

        bufferForDataCount = 0;
        bufferForDataPosition = 0;
        bufferForDataCheckedBytes = 0;
        block_N0 = 0;

        SendToWindow();
        Write(); 

        DateTime DT = DateTime.Now;
        Frez_text.Write("\nSend\t" + DT.ToLongTimeString() + "\t\t" + Sended_Byte);


        
    }

    private void tim2_Tick(object sender, EventArgs e)
    {
        // AddToSpectr();
        //if (zedGraphControl1 == null)
        //    zedGraphControl1 = new ZedGraphControl();
        //RefreshGraph(zedGraphControl1);    
    }

    


    private void comboBox1_SelectedIndexChanged(object sender, EventArgs e)
    {
        if (comboBox1.SelectedIndex == 0)
            numeral_base = 2;
        if (comboBox1.SelectedIndex == 1)
            numeral_base = 10; 
        if (comboBox1.SelectedIndex == 2)
            numeral_base = 16;
    }

    private void clear_button_Click(object sender, EventArgs e)
    {
        // buffers for data
        recieveData = new byte[0xFFFF];
        recieveDataPosition = 0;
        bufferForData = new byte[0xFFFF];
        bufferForDataCount = 0;
        bufferForDataPosition = 0;
        bufferForDataCheckedBytes = 0;

        xi = new uint[0xFF];

        sb.Clear();
        recieveTextBox.Text = "";
        this.dataGridView1.RowCount = 0;
        for (int i = 0; i < 100; i++) Cumul_Spectr[i] = 0;
        //zedGraphControl1.Dispose();
        //adc1Curve.Clear();
        //adc2Curve.Clear();
        
    }
      



    private void saveButton_Click(object sender, EventArgs e)
    {
        SaveToFile(false);
        SaveToFile(true);
    }

      /// <summary>
      /// saving to file all data
      /// </summary>
      /// <param name="writeBinary">true to write bynary, false to wtite txt</param>
    private static void SaveToFile(bool writeBinary)
    {
        string path = @"";
         path = DIR_path + DateTime.Today.ToShortDateString() + @"_" + DateTime.Now.Hour + @"-" + DateTime.Now.Minute;
        //path += DateTime.Today.ToShortDateString() + DateTime.Today.ToShortTimeString() + @".bin"; // or @"c:\DEPRON\DEPRON\bin\Debug\";

        if (writeBinary)
        {
            //mut.WaitOne();
            if (bufferForDataCount != 0)
            {
                
                int i = 0;
                while (File.Exists(path + @"no" + i.ToString() + @".bin"))
                {
                    i++;
                }
                // Create a file to write to.
                //File.WriteAllBytes(path + @"no" + i.ToString() + @".bin", bufferForData);
            }
            //mut.ReleaseMutex();
            
        }
        else 
        {            
            path += @".txt";
            
            // This text is added only once to the file.
            if (!File.Exists(path))
            {
                // Create a file to write to.
                string createText = "Wtite at " + DateTime.UtcNow.ToString()+ Environment.NewLine;
                File.WriteAllText(path, createText);
            }

            // This text is always added, making the file longer over time
            // if it is not deleted.
            string appendText = sb.ToString();
            File.AppendAllText(path, appendText);

            // Open the file to read from.
            // string readText = File.ReadAllText(path);
            //Console.WriteLine(readText); 
        }
    }


    private void openButton_Click(object sender, EventArgs e)
    {
        Stream myStream = null;
        OpenFileDialog openFileDialog1 = new OpenFileDialog();

        openFileDialog1.InitialDirectory = Application.StartupPath;// "c:\\";
        openFileDialog1.Filter = "bynary files (*.bin)|*.bin|All files (*.*)|*.*";
        openFileDialog1.FilterIndex = 1;
        openFileDialog1.RestoreDirectory = true;

        if (openFileDialog1.ShowDialog() == DialogResult.OK)
        {
            try
            {
                if ((myStream = openFileDialog1.OpenFile()) != null)
                {
                    using (myStream)
                    {
                        myStream.Read(bufferForData, 0, bufferForData.Length);
                    }
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error: Could not read file from disk. Original error: " + ex.Message);
            }
            /*
            PositionDialog dialog = new PositionDialog();
            if (dialog.ShowDialog() == DialogResult.OK)
                AddToUI();//bufferForDataCount - bufferForDataPosition
            dialog.Dispose();
            openFileDialog1.Dispose();*/

        }

    }

    private void SaveSpectrabutton_Click(object sender, EventArgs e)
    {
        string path = @"";
        path = DIR_path + DateTime.Today.ToShortDateString() + @"_" + DateTime.Now.Hour + @"-" + DateTime.Now.Minute;
        //path += DateTime.Today.ToShortDateString() + DateTime.Today.ToShortTimeString() + @".bin"; // or @"c:\DEPRON\DEPRON\bin\Debug\";

        
        path += @".spe";

        // This text is added only once to the file.
        if (!File.Exists(path))
        {
            // Create a file to write to.
            string createText = "Spectr1 write at " + DateTime.UtcNow.ToString() + Environment.NewLine;
            File.WriteAllText(path, createText);
        }

        // This text is always added, making the file longer over time
        // if it is not deleted.
        StringBuilder appText2  = new StringBuilder();

        foreach (var item in SpectrADC1)
            appText2.AppendLine( item.ToString());


        string createText2 = Environment.NewLine +"Spectr2 write at " + DateTime.UtcNow.ToString() ;
            appText2.AppendLine( createText2);

        foreach (var item in SpectrADC2)
            appText2.AppendLine(item.ToString());


        File.AppendAllText(path, appText2.ToString());

        Array.Clear(SpectrADC1, 0, SpectrADC1.Length);
        Array.Clear(SpectrADC2, 0, SpectrADC2.Length);

        tim1.Stop();
        repeatKbutton.Text = "Repeat k";
        repeatKbutton.Refresh();
        toggleK = false;
        
        
    }

    private void repeatKbutton_Click(object sender, EventArgs e)
    {
        if (!toggleK)
        {
            //    sb.Clear();
            repeatKbutton.Text = "Stop K";
            repeatKbutton.Refresh();
            toggleK = true;
            tim1.Start();
        }
        else
        {
            repeatKbutton.Text = "Repeat k";
            repeatKbutton.Refresh();
            toggleK = false;
            tim1.Stop();
        }
    }


    //private void buttonGraph_Click(object sender, EventArgs e)
    //{
    //    if (zedGraphControl1 == null)
    //        zedGraphControl1 = new ZedGraphControl();
    //    CreateGraph(zedGraphControl1);
    //}

//private void CreateGraph( ZedGraphControl zgc )
//{
//   // get a reference to the GraphPane

//   GraphPane myPane = zgc.GraphPane;

//   // Set the Titles

//   myPane.Title.Text = "Спектр АЦП";
//   myPane.XAxis.Title.Text = "Номер канала";
//   myPane.YAxis.Title.Text = "Счет";

       
    
    
//    //myPane.XAxis.
//   // Make up some data arrays based on the Sine function

//   double x, y1, y2;
//   PointPairList list1 = new PointPairList();
//   PointPairList list2 = new PointPairList();

//   for ( int i = 0; i < 4096; i++ )
//   {
//      x = (double)i;      
//      y1 = SpectrADC1[i];
//      y2 = SpectrADC2[i];
      
//      list1.Add( x, y1 );
//      list2.Add(x, y2);
      
//   }

//   // Generate a red curve with diamond

//   // symbols, in the legend
//   myPane.CurveList.Clear();

//   adc1Curve = myPane.AddCurve("Спектр ADC1",
//         list1, Color.Red, SymbolType.Diamond );
//   adc2Curve = myPane.AddCurve("Спектр ADC2",
//        list2, Color.Blue, SymbolType.Circle);

//   list1.Clear();
//   list2.Clear();
//   // Generate a blue curve with circle

//   // symbols, and "Piper" in the legend

//   //LineItem myCurve2 = myPane.AddCurve( "Piper",
//   //      list2, Color.Blue, SymbolType.Circle );


//   // Tell ZedGraph to refigure the

//   // axes since the data have changed

//   zgc.AxisChange();
//}
//private void RefreshGraph(ZedGraphControl zgc)
//{
//    // get a reference to the GraphPane

//    GraphPane myPane = zgc.GraphPane;

//    if (adc1Curve != null)
//    {
//        adc1Curve.Clear();
//        adc2Curve.Clear();
//    }
//    else
//    { }
    
//    double x, y1, y2;
//    PointPairList list1 = new PointPairList();
//    PointPairList list2 = new PointPairList();

//    for (int i = 0; i < 4096; i++)
//    {
//        x = (double)i;
//        y1 = SpectrADC1[i];
//        y2 = SpectrADC2[i];

//        list1.Add(x, y1);
//        list2.Add(x, y2);
//        adc1Curve.AddPoint(list1[i]);
//        adc2Curve.AddPoint(list2[i]);
//    }

//    // Generate a red curve with diamond
        
//    // Generate a blue curve with circle

//    // symbols, and "Piper" in the legend

//    //LineItem myCurve2 = myPane.AddCurve( "Piper",
//    //      list2, Color.Blue, SymbolType.Circle );


//    // Tell ZedGraph to refigure the

//    // axes since the data have changed

//    zgc.AxisChange();
//    zgc.Refresh();
//}

//private void button1_Click(object sender, EventArgs e)
//{
//    if (zedGraphControl1 == null)
//        zedGraphControl1 = new ZedGraphControl();
//    GraphPane myPane = zedGraphControl1.GraphPane;
//    //Логарифмический масштаб по Y
//    if (myPane.YAxis.Type == AxisType.Linear)
//    {
//        myPane.YAxis.Type = AxisType.Log;
//        myPane.YAxis.Scale.Max = 1e4;
//        myPane.YAxis.Scale.Min = 1.0;
//    }
//    else 
//    {
//        myPane.YAxis.Type = AxisType.Linear;
//        // Установим масштаб по умолчанию для оси X
//        myPane.XAxis.Scale.MinAuto = true;
//        myPane.XAxis.Scale.MaxAuto = true;

//        // Установим масштаб по умолчанию для оси Y
//        myPane.YAxis.Scale.MinAuto = true;
//        myPane.YAxis.Scale.MaxAuto = true;
 
//    }
//}

}
    
}

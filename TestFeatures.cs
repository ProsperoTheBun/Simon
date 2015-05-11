using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Connectors;
using System.Threading;

namespace Simon_3
{
    class TestFeatures
    {
        public static void SoundTest()
        {
            Console.WriteLine("Testing sound");


            int sampleRate = 48000;
            short[] sBuffer = new short[sampleRate];

            double amplitude = 0.5;
            double frequency = 440; // A4

            for (int n = 0; n < sBuffer.Length; n++)
            {
                sBuffer[n] = (short)(amplitude * short.MaxValue * Math.Sin((2 * Math.PI * n * frequency) / sampleRate));
            }

            Console.WriteLine("initializing device");
            Mono.Audio.AlsaDevice ad = new Mono.Audio.AlsaDevice("default");
            ad.SetFormat(Mono.Audio.AudioFormat.S16_LE, 1, sampleRate);

            int result = 0;
            Console.WriteLine("sending sample");
            int frames = 480; // 1/100th of a second
            short[] partBuffer = new short[frames];
            for (int x = 0; x < 100; x++)
            {
                Array.Copy(sBuffer, frames * x, partBuffer, 0, frames);
                result = ad.PlaySample(partBuffer, frames);

            }
            Console.WriteLine("done");

            ad.Dispose();



        }

        public static void SoundTestWindows()
        {
            PlayBeep(440, 1000);
            PlayBeep(660, 1000);

            // pass an array

            int sampleRate = 48000;
            short[] sBuffer = new short[sampleRate];

            double amplitude = 0.5;
            double frequency = 440; // A4

            // todo: fix sine wave function as it is "shifting" values (the original short array was signed)
            // check if ALSA device takes signed bytes or how it signs bytes
            for (int n = 0; n < sBuffer.Length; n++)
            {
                sBuffer[n] = (short)(amplitude * short.MaxValue * Math.Sin((2 * Math.PI * n * frequency) / sampleRate));
            }

            byte[] sbufferConv = new byte[sampleRate * 2];
            //convert ..
            for (int i = 0; i < sBuffer.Length; i++)
            {
                sbufferConv[i] = (byte)sBuffer[i];
                sbufferConv[i + 1] = (byte)(sBuffer[i] >> 8 & 0xFF);
            }


            PlayBeep(1000, sBuffer);

        }

        public static void PlayBeep(UInt16 frequency, int msDuration, UInt16 volume = 16383)
        {
            var mStrm = new MemoryStream();
            BinaryWriter writer = new BinaryWriter(mStrm);

            const double TAU = 2 * Math.PI;
            int formatChunkSize = 16;
            int headerSize = 8;
            short formatType = 1;
            short tracks = 1;
            int samplesPerSecond = 48000;
            short bitsPerSample = 16;
            short frameSize = (short)(tracks * ((bitsPerSample + 7) / 8));
            int bytesPerSecond = samplesPerSecond * frameSize;
            int waveSize = 4;
            int samples = (int)((decimal)samplesPerSecond * msDuration / 1000);
            int dataChunkSize = samples * frameSize;
            int fileSize = waveSize + headerSize + formatChunkSize + headerSize + dataChunkSize;
            // var encoding = new System.Text.UTF8Encoding();
            writer.Write(0x46464952); // = encoding.GetBytes("RIFF")
            writer.Write(fileSize);
            writer.Write(0x45564157); // = encoding.GetBytes("WAVE")
            writer.Write(0x20746D66); // = encoding.GetBytes("fmt ")
            writer.Write(formatChunkSize);
            writer.Write(formatType);
            writer.Write(tracks);
            writer.Write(samplesPerSecond);
            writer.Write(bytesPerSecond);
            writer.Write(frameSize);
            writer.Write(bitsPerSample);
            writer.Write(0x61746164); // = encoding.GetBytes("data")
            writer.Write(dataChunkSize);
            {
                double theta = frequency * TAU / (double)samplesPerSecond;
                // 'volume' is UInt16 with range 0 thru Uint16.MaxValue ( = 65 535)
                // we need 'amp' to have the range of 0 thru Int16.MaxValue ( = 32 767)
                double amp = volume >> 2; // so we simply set amp = volume / 2
                for (int step = 0; step < samples; step++)
                {
                    short s = (short)(amp * Math.Sin(theta * (double)step));
                    writer.Write(s);
                }
            }

            mStrm.Seek(0, SeekOrigin.Begin);
            var player = new System.Media.SoundPlayer(mStrm);
            player.PlaySync();
            writer.Close();
            mStrm.Close();
        }

        public static void PlayBeep(int msDuration, short[] buffer)
        {
            var mStrm = new MemoryStream();
            BinaryWriter writer = new BinaryWriter(mStrm);

            const double TAU = 2 * Math.PI;
            int formatChunkSize = 16;
            int headerSize = 8;
            short formatType = 1;
            short tracks = 1;
            int samplesPerSecond = 48000;
            short bitsPerSample = 16;
            short frameSize = (short)(tracks * ((bitsPerSample + 7) / 8));
            int bytesPerSecond = samplesPerSecond * frameSize;
            int waveSize = 4;
            int samples = (int)((decimal)samplesPerSecond * msDuration / 1000);
            int dataChunkSize = samples * frameSize;
            int fileSize = waveSize + headerSize + formatChunkSize + headerSize + dataChunkSize;
            // var encoding = new System.Text.UTF8Encoding();
            writer.Write(0x46464952); // = encoding.GetBytes("RIFF")
            writer.Write(fileSize);
            writer.Write(0x45564157); // = encoding.GetBytes("WAVE")
            writer.Write(0x20746D66); // = encoding.GetBytes("fmt ")
            writer.Write(formatChunkSize);
            writer.Write(formatType);
            writer.Write(tracks);
            writer.Write(samplesPerSecond);
            writer.Write(bytesPerSecond);
            writer.Write(frameSize);
            writer.Write(bitsPerSample);
            writer.Write(0x61746164); // = encoding.GetBytes("data")
            writer.Write(dataChunkSize);
            //{
            //    double theta = frequency * TAU / (double)samplesPerSecond;
            //    // 'volume' is UInt16 with range 0 thru Uint16.MaxValue ( = 65 535)
            //    // we need 'amp' to have the range of 0 thru Int16.MaxValue ( = 32 767)
            //    double amp = volume >> 2; // so we simply set amp = volume / 2
            //    for (int step = 0; step < samples; step++)
            //    {
            //        short s = (short)(amp * Math.Sin(theta * (double)step));
            //        writer.Write(s);
            //    }
            //}

            foreach (short s in buffer)
                writer.Write(s);

            mStrm.Seek(0, SeekOrigin.Begin);
            var player = new System.Media.SoundPlayer(mStrm);
            player.PlaySync();
            writer.Close();
            mStrm.Close();
        }

        public static void TurnOnGreenLed()
        {
            const ConnectorPin sdaPin = ConnectorPin.P1Pin03;
            const ConnectorPin sclPin = ConnectorPin.P1Pin05;

            const Mcp23017Pin ledGreen = Mcp23017Pin.B0;


            using (var driver = new I2cDriver(sdaPin.ToProcessor(), sclPin.ToProcessor()))
            {
                var deviceConnection = new Mcp23017I2cConnection(driver.Connect(0x20));
                Console.WriteLine("Connected");

                deviceConnection.SetDirection(ledGreen, Mcp23017PinDirection.Output);
                // green light off

                deviceConnection.SetPinStatus(ledGreen, true);

                System.Threading.Thread.Sleep(1000); // wait  a sec

                deviceConnection.SetPinStatus(ledGreen, false);



            }
        }



        public static void WriteToLcd()
        {
            Console.WriteLine("writing to the lcd");

            const ConnectorPin sdaPin = ConnectorPin.P1Pin03;
            const ConnectorPin sclPin = ConnectorPin.P1Pin05;

            using (var driver = new I2cDriver(sdaPin.ToProcessor(), sclPin.ToProcessor()))
            {
                var deviceConnection = new Mcp23017I2cConnection(driver.Connect(0x20));
                Console.WriteLine("Connected");

                // turn on LCD backlight (on/off/on)
                deviceConnection.SetDirection(Mcp23017Pin.A0, Mcp23017PinDirection.Output);

                deviceConnection.SetPinStatus(Mcp23017Pin.A0, true);
                Thread.Sleep(500); // wait  
                deviceConnection.SetPinStatus(Mcp23017Pin.A0, false);
                Thread.Sleep(500); // wait  
                deviceConnection.SetPinStatus(Mcp23017Pin.A0, true);
            }

            // light is on, let's write
            var settings = new Hd44780LcdConnectionSettings
            {
                ScreenWidth = 16,
                ScreenHeight = 2
            };

            settings.Encoding = Encoding.ASCII;

            using (Hd44780Configuration configuration = Hd44780Configuration.LoadGpioConfiguration())
            using (var connection = new Hd44780LcdConnection(settings, configuration.Pins))
            {
                // connection.SetCustomCharacter(1, new byte[] { 0x0, 0x0, 0x04, 0xe, 0x1f, 0x0, 0x0 });
                //  connection.SetCustomCharacter(2, new byte[] { 0x0, 0x0, 0x1f, 0xe, 0x04, 0x0, 0x0 });

                connection.Clear();
                connection.WriteLine("Pi & Bash>_");
                Thread.Sleep(750);
                connection.WriteLine("Test");
                Thread.Sleep(750);
                connection.WriteLine("Thank You");
                Thread.Sleep(750);
                connection.WriteLine("more text");
                Thread.Sleep(750);
                connection.WriteLine("and another bit");

                Thread.Sleep(2000);
            }

        }

        public static void WaitForAllButtons()
        {


            const ConnectorPin sdaPin = ConnectorPin.P1Pin03;
            const ConnectorPin sclPin = ConnectorPin.P1Pin05;

            using (var driver = new I2cDriver(sdaPin.ToProcessor(), sclPin.ToProcessor()))
            {
                var deviceConnection = new Mcp23017I2cConnection(driver.Connect(0x20));
                Console.WriteLine("Connected");
                Console.WriteLine("press all 5 of the buttons");

                deviceConnection.SetDirection(Mcp23017Pin.B1, Mcp23017PinDirection.Input);
                deviceConnection.SetDirection(Mcp23017Pin.B3, Mcp23017PinDirection.Input);
                deviceConnection.SetDirection(Mcp23017Pin.B5, Mcp23017PinDirection.Input);
                deviceConnection.SetDirection(Mcp23017Pin.B6, Mcp23017PinDirection.Input);
                deviceConnection.SetDirection(Mcp23017Pin.B7, Mcp23017PinDirection.Input);

                deviceConnection.SetResistor(Mcp23017Pin.B1, Mcp23017PinResistor.PullUp);
                deviceConnection.SetResistor(Mcp23017Pin.B3, Mcp23017PinResistor.PullUp);
                deviceConnection.SetResistor(Mcp23017Pin.B5, Mcp23017PinResistor.PullUp);
                deviceConnection.SetResistor(Mcp23017Pin.B6, Mcp23017PinResistor.PullUp);
                deviceConnection.SetResistor(Mcp23017Pin.B7, Mcp23017PinResistor.PullUp);


                bool[] btns = { false, false, false, false, false };

                bool btnsPushed = false;
                while (!btnsPushed && !Console.KeyAvailable)
                {
                    Thread.Sleep(100);
                    if (!deviceConnection.GetPinStatus(Mcp23017Pin.B1)) { btns[0] = true; Console.Write("UP "); }
                    if (!deviceConnection.GetPinStatus(Mcp23017Pin.B3)) { btns[1] = true; Console.Write("ENTER "); }
                    if (!deviceConnection.GetPinStatus(Mcp23017Pin.B5)) { btns[2] = true; Console.Write("DOWN "); }
                    if (!deviceConnection.GetPinStatus(Mcp23017Pin.B6)) { btns[3] = true; Console.Write("TOPSEL "); }
                    if (!deviceConnection.GetPinStatus(Mcp23017Pin.B7)) { btns[4] = true; Console.Write("BOTSEL "); }

                    btnsPushed = btns[0] && btns[1] && btns[2] && btns[3] && btns[4];


                }


            }

            Console.WriteLine();
            Console.WriteLine("All buttons pressed");
        }

        public static void WaitForAllButtonsAsync()
        {

            // http://stackoverflow.com/questions/3390286/making-a-console-application-like-a-windows-application

            PiAndBash.Display disp = new PiAndBash.Display();
            disp.TopLine = "Simon";
            disp.BottomLine = "< EXIT";


            notePlayer = new NotePlayer();
            notePlayer.CreateSamples(new double[] { 440, 523 });
            PiAndBash.ButtonCatcher bc = new PiAndBash.ButtonCatcher();
            bc.ButtonEvent += bc_ButtonEvent2;



            bc.Start();




        }

        private static NotePlayer notePlayer;

        static void bc_ButtonEvent(object sender, PiAndBash.ButtonCatcher.ButtonArgs e)
        {

            if (e.ButtonEvent == PiAndBash.ButtonCatcher.ButtonEventType.Up)
            {
                Console.WriteLine(e.Button.ToString() + " pressed and released");

                if (e.Button == PiAndBash.ButtonCatcher.ButtonType.Down)
                {
                    notePlayer.PlayNote(440, 1);
                }
                if (e.Button == PiAndBash.ButtonCatcher.ButtonType.Enter)
                {
                    notePlayer.PlayNote(523, 1);
                }
                if (e.Button == PiAndBash.ButtonCatcher.ButtonType.BotSel)
                {
                    Console.WriteLine("Exiting");
                    (sender as PiAndBash.ButtonCatcher).Stop();
                }
            }

        }

        static void bc_ButtonEvent2(object sender, PiAndBash.ButtonCatcher.ButtonArgs e)
        {

            // play note while down, stop when released

            if (e.ButtonEvent == PiAndBash.ButtonCatcher.ButtonEventType.Down)
            {
                switch (e.Button)
                {
                    case PiAndBash.ButtonCatcher.ButtonType.Up:
                        // play A
                        notePlayer.PlayNote(440,500);
                        break;

                    case PiAndBash.ButtonCatcher.ButtonType.Down:
                        // play C
                        notePlayer.PlayNote(523,500);
                        break;

                    case PiAndBash.ButtonCatcher.ButtonType.Enter:
                        // play F
                        notePlayer.PlayNote(349, 500);
                        break;

                    default:
                        break;
                }
            }
            else // button released
            {
                // stop all notes
                notePlayer.Silence();
                if (e.Button == PiAndBash.ButtonCatcher.ButtonType.BotSel)
                {
                    Console.WriteLine("Exiting");
                    (sender as PiAndBash.ButtonCatcher).Stop();
                }
            }
        }

        internal static void WindowsInterruptTest()
        {

            Console.WriteLine("press any key to play sound");
            bool isPlaying = false;
            var ad = Mono.Audio.AudioDevice.CreateDevice("sss") ;
           
            var np = new NotePlayer();

         

            short[] buffer = new short[480];
            while (true)
            {

                if (!isPlaying)
                {
                    Console.ReadKey(false);
                    isPlaying = true;
                }

                if (isPlaying && !Console.KeyAvailable)
                {
                   // ad.PlaySample(buffer, 480);
                    //Thread.Sleep(100);
                    //Task.Factory.StartNew(() => np.PlayNote(440, 1));
                    //Thread t = new Thread(() => np.PlayNote(440,1));
                    //t.Start();
                    //while (!t.IsAlive) ;
                    np.PlayNote(440, 1);
                    Console.Write(".");
                }
                else
                {
                    
                    Console.Write("£");
                    np.Silence();
                    isPlaying = false;
                    Console.ReadKey(true);
                }


            }



        }

        private static Mono.Audio.AudioDevice _dev;
        private static short[] _sample;

        internal static void LatencyTest()
        {

            var ac = Mono.Audio.AudioDevice.CreateDevice("default");
            ac.SetFormat(Mono.Audio.AudioFormat.S16_LE, 1, 48000);
            short[] sample = new short[48000];
            for (int n = 0; n < sample.Length; n++)
            {
                sample[n] = (short)(0.5 * short.MaxValue * Math.Sin((2 * Math.PI * n * 440) / 48000));
            }

            Console.WriteLine("Ready. Press key to play note");
            Console.ReadKey();
          //  ac.PlaySample(sample, 48000);
            Console.WriteLine("done");

            _dev = ac;
            _sample = sample;

            Console.WriteLine("now try a pi button");
            var bc = new PiAndBash.ButtonCatcher();
            bc.ButtonEvent += bc_ButtonEvent3;
            bc.Start();


        }

        static void bc_ButtonEvent3(object sender, PiAndBash.ButtonCatcher.ButtonArgs e)
        {
            Console.WriteLine("event");
            var bc = sender as PiAndBash.ButtonCatcher;
            bc.ButtonEvent -= bc_ButtonEvent3;

            _dev.PlaySample(_sample, 48000);

         //   bc.Stop();

        }
    }
}

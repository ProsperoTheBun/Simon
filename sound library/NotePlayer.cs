using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Simon_3
{
    public class NotePlayer
    {
        /// <summary>
        /// Sample rate defaults to 48000
        /// </summary>
        public int SampleRate { get; set; }

        /// <summary>
        /// Volume Percentage (0-100) defaults to 50%
        /// </summary>
        public int Volume { get; set; }

        private double amplitude
        {
            get
            {
                if (Volume <= 0) return 0;
                if (Volume >= 100) return 1;

                return (double)Volume / 100;

            }
        }
        public NotePlayer()
        {
            Console.WriteLine("Initialising Sound Driver");
            // defaults
            SampleRate = 48000;
            Volume = 50;
            //currentDevice = Mono.Audio.AudioDevice.CreateDevice("default");
            //currentDevice.SetFormat(Mono.Audio.AudioFormat.S16_LE, 1, this.SampleRate);
            Console.WriteLine("Sound driver Initialised");
        }

        /// <summary>
        /// Play a sine wave tone
        /// </summary>
        /// <param name="Frequency">The frequency of the note</param>
        /// <param name="Duration">The duration of the note in milliseconds </param>
        public void PlayNote(double Frequency, int Duration)
        {
            //if (currentDevice == null)
            //{
            //    throw new Exception("Audio device is NULL");
            //}

            int sampleRate = this.SampleRate;
            short[] sBuffer = new short[sampleRate];


            double frequency = Frequency;


            for (int n = 0; n < sBuffer.Length; n++)
            {
                sBuffer[n] = (short)(amplitude * short.MaxValue * Math.Sin((2 * Math.PI * n * frequency) / sampleRate));
            }


            // generate the full waveform from the 1s sample
            double seconds = (double)Duration / 1000;
            int totalSamples = (int)((double)this.SampleRate * seconds);
            short[] waveForm = new short[totalSamples];
            if (totalSamples == sBuffer.Length)
                waveForm = sBuffer;
            else if (totalSamples < sBuffer.Length)
            {
                Array.Copy(sBuffer, 0, waveForm, 0, totalSamples);
            }
            else
            {
                int wholeSeconds = (int)Math.Floor((double)totalSamples / (double)sampleRate);
                double remainder = seconds - wholeSeconds;
                int extraSamples = (int)(remainder * sampleRate);
                for (int x = 0; x < wholeSeconds; x++)
                    Array.Copy(sBuffer, 0, waveForm, sampleRate * x, sampleRate);
                Array.Copy(sBuffer, 0, waveForm, sampleRate * wholeSeconds, extraSamples);

            }

            currentDevice = Mono.Audio.AudioDevice.CreateDevice("default");
            currentDevice.SetFormat(Mono.Audio.AudioFormat.S16_LE, 1, sampleRate);

        //    Console.WriteLine("playing sample");
            int result = 0;


            result = currentDevice.PlaySample(waveForm, totalSamples);

            currentDevice.Wait();
            //currentDevice.Stop();
       //    Console.WriteLine("waited for sound device after {0}", result);
           ((Mono.Audio.AlsaDevice)currentDevice).Dispose();
        }

        private Dictionary<double, short[]> notes = new Dictionary<double, short[]>();

        public void CreateSamples(params double[] frequencies)
        {

            foreach (double freq in frequencies)
            {
                if (!notes.Keys.Contains(freq))
                {
                    Console.Write("+");
                    short[] sample = new short[this.SampleRate];
                    for (int n = 0; n < sample.Length; n++)
                    {
                        sample[n] = (short)(amplitude * short.MaxValue * Math.Sin((2 * Math.PI * n * freq) / this.SampleRate));
                    }
                    notes.Add(freq, sample);
                }
            }

        }


        public void PlayPartNote(double Frequency)
        {
            //  Console.Write("|"); return;

            if (!isPlaying)
            {
                Console.WriteLine("<");
                localBuffer = notes[Frequency];
                currentBufferPos = 0;
                isPlaying = true;
            }

            int frames = 480; // 1/100th of a second
            short[] partBuffer = new short[frames];


            Array.Copy(localBuffer, frames * currentBufferPos, partBuffer, 0, frames);
            Console.Write(currentBufferPos.ToString() + " ");
            currentDevice.PlaySample(partBuffer, frames);
            currentBufferPos++;
            if (currentBufferPos == 100) currentBufferPos = 0; // reset

        }

        private short[] localBuffer;
        private int currentBufferPos = 0;

        private Mono.Audio.AudioDevice currentDevice = null;

        private bool interruptSound = false;
        private bool isPlaying = false;
        internal void Silence()
        {



        }
    }
}

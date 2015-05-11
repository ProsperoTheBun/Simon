using System;
using System.IO;
using System.Runtime.InteropServices;





// // source : https://github.com/mosa/Mono-Class-Libraries/blob/master/mcs/class/System/System.Media/AudioDevice.Original.cs



namespace Mono.Audio
{

    /* these are the values used by alsa */

    public enum AudioFormat
    {
        S8,
        U8,
        S16_LE,
        S16_BE,
        U16_LE,
        U16_BE,
        S24_LE,
        S24_BE,
        U24_LE,
        U24_BE,
        S32_LE,
        S32_BE,
        U32_LE,
        U32_BE,
        FLOAT_LE,
        FLOAT_BE,
        FLOAT64_LE,
        FLOAT64_BE,
        IEC958_SUBFRAME_LE,
        IEC958_SUBFRAME_BE,
        MU_LAW,
        A_LAW,
        IMA_ADPCM,
        MPEG,
        GSM
    }


    public class AudioDevice
    {

        static AudioDevice TryAlsa(string name)
        {
            AudioDevice dev;
            try
            {
                dev = new AlsaDevice(name);
                return dev;
            }
            catch
            {
                return null;
            }
        }

        public static AudioDevice CreateDevice(string name)
        {
            AudioDevice dev;

            dev = TryAlsa(name);
            /* if no option is found, return a silent device */
            if (dev == null)
                dev = new AudioDevice();
            return dev;
        }

        public virtual bool SetFormat(AudioFormat format, int channels, int rate)
        {
            return true;
        }

        public virtual long PlaySample(short[] buffer, long num_frames)
        {

            return num_frames;
        }

        public virtual int PlaySample(byte[] buffer, int num_frames)
        {

            return num_frames;
        }
        public virtual int PlaySample(short[] buffer, int num_frames)
        {
            // for a null device just wait for a bit
            System.Threading.Thread.Sleep(10);
            return num_frames;
        }
        public virtual void Wait()
        {
           
        }
        //public virtual void Dispose()
        //{
        //    Console.WriteLine("disp A");

        //}
        public virtual void Stop()
        {

        }
    }

    public class AlsaDevice : AudioDevice, IDisposable
    {
        IntPtr handle;

        [DllImport("libasound.so.2")]
        static extern int snd_pcm_open(ref IntPtr handle, string pcm_name, int stream, int mode);

        [DllImport("libasound.so.2")]
        static extern int snd_pcm_close(IntPtr handle);

        [DllImport("libasound.so.2")]
        static extern int snd_pcm_drain(IntPtr handle);

        [DllImport("libasound.so.2")]
        static extern int snd_pcm_writei(IntPtr handle, byte[] buf, int size);

        [DllImport("libasound.so.2")]
        static extern int snd_pcm_writei(IntPtr handle, short[] buf, int size);

        [DllImport("libasound.so.2")]
        static extern long snd_pcm_writei(IntPtr handle, short[] buf, long size);


        [DllImport("libasound.so.2")]
        static extern int snd_pcm_set_params(IntPtr handle, int format, int access, int channels, int rate, int soft_resample, int latency);

        [DllImport("libasound.so.2")]
        static extern int snd_pcm_drop(IntPtr handle);


        public AlsaDevice(string name)
        {
            if (name == null)
                name = "default";
            int err = snd_pcm_open(ref handle, name, 0, 0);
            if (err < 0)
                throw new Exception("no open " + err);
        }

        ~AlsaDevice()
        {
            Dispose(false);
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {

            if (disposing)
            {

            }
            if (handle != IntPtr.Zero)
                snd_pcm_close(handle);
            handle = IntPtr.Zero;
        }

        public override bool SetFormat(AudioFormat format, int channels, int rate)
        {
            int err = snd_pcm_set_params(handle, (int)format, 3, channels, rate, 1, 500000);
            return err == 0;
        }

        public override int PlaySample(byte[] buffer, int num_frames)
        {
            int frames = snd_pcm_writei(handle, buffer, num_frames);
            return frames;
        }

        public override int PlaySample(short[] buffer, int num_frames)
        {
            int frames = snd_pcm_writei(handle, buffer, num_frames);
            return frames;
        }

        public override long PlaySample(short[] buffer, long num_frames)
        {
            long frames = snd_pcm_writei(handle, buffer, num_frames);
            return frames;
        }

        public override void Wait()
        {
            snd_pcm_drain(handle);
        }
        public override void Stop()
        {
            snd_pcm_drop(handle);
        }
    }

}



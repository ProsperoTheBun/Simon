using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Runtime.Serialization;
using System.Text;
using System.Threading.Tasks;

namespace Connectors
{

    public enum ConnectorPin
    {
        /// <summary>
        /// Connector P1, pin 3.
        /// </summary>
        P1Pin3,

        /// <summary>
        /// Connector P1, pin 3.
        /// </summary>
        P1Pin03 = P1Pin3,

        /// <summary>
        /// Connector P1, pin 5.
        /// </summary>
        P1Pin5,

        /// <summary>
        /// Connector P1, pin 5.
        /// </summary>
        P1Pin05 = P1Pin5,

        /// <summary>
        /// Connector P1, pin 7.
        /// </summary>
        P1Pin7,

        /// <summary>
        /// Connector P1, pin 7.
        /// </summary>
        P1Pin07 = P1Pin7,

        /// <summary>
        /// Connector P1, pin 8.
        /// </summary>
        P1Pin8,

        /// <summary>
        /// Connector P1, pin 8.
        /// </summary>
        P1Pin08 = P1Pin8,

        /// <summary>
        /// Connector P1, pin 10.
        /// </summary>
        P1Pin10,

        /// <summary>
        /// Connector P1, pin 11.
        /// </summary>
        P1Pin11,

        /// <summary>
        /// Connector P1, pin 12.
        /// </summary>
        P1Pin12,

        /// <summary>
        /// Connector P1, pin 13.
        /// </summary>
        P1Pin13,

        /// <summary>
        /// Connector P1, pin 15.
        /// </summary>
        P1Pin15,

        /// <summary>
        /// Connector P1, pin 16.
        /// </summary>
        P1Pin16,

        /// <summary>
        /// Connector P1, pin 18.
        /// </summary>
        P1Pin18,

        /// <summary>
        /// Connector P1, pin 19.
        /// </summary>
        P1Pin19,

        /// <summary>
        /// Connector P1, pin 21.
        /// </summary>
        P1Pin21,

        /// <summary>
        /// Connector P1, pin 22.
        /// </summary>
        P1Pin22,

        /// <summary>
        /// Connector P1, pin 23.
        /// </summary>
        P1Pin23,

        /// <summary>
        /// Connector P1, pin 24.
        /// </summary>
        P1Pin24,

        /// <summary>
        /// Connector P1, pin 26.
        /// </summary>
        P1Pin26,


        // Pins 27+ exist starting from Model B+

        /// <summary>
        /// Connector P1, pin 27.
        /// </summary>
        P1Pin27,

        /// <summary>
        /// Connector P1, pin 28.
        /// </summary>
        P1Pin28,

        /// <summary>
        /// Connector P1, pin 29.
        /// </summary>
        P1Pin29,

        /// <summary>
        /// Connector P1, pin 31.
        /// </summary>
        P1Pin31,

        /// <summary>
        /// Connector P1, pin 32.
        /// </summary>
        P1Pin32,

        /// <summary>
        /// Connector P1, pin 33.
        /// </summary>
        P1Pin33,

        /// <summary>
        /// Connector P1, pin 35.
        /// </summary>
        P1Pin35,

        /// <summary>
        /// Connector P1, pin 36.
        /// </summary>
        P1Pin36,

        /// <summary>
        /// Connector P1, pin 37.
        /// </summary>
        P1Pin37,

        /// <summary>
        /// Connector P1, pin 38.
        /// </summary>
        P1Pin38,

        /// <summary>
        /// Connector P1, pin 40.
        /// </summary>
        P1Pin40,


        // P5 Connector exist on Rev2 boards (no longer on B+)

        /// <summary>
        /// Connector P5, pin 3.
        /// </summary>
        P5Pin3,

        /// <summary>
        /// Connector P5, pin 3.
        /// </summary>
        P5Pin03 = P5Pin3,

        /// <summary>
        /// Connector P5, pin 4.
        /// </summary>
        P5Pin4,

        /// <summary>
        /// Connector P5, pin 4.
        /// </summary>
        P5Pin04 = P5Pin4,

        /// <summary>
        /// Connector P5, pin 5.
        /// </summary>
        P5Pin5,

        /// <summary>
        /// Connector P5, pin 5.
        /// </summary>
        P5Pin05 = P5Pin5,

        /// <summary>
        /// Connector P5, pin 6.
        /// </summary>
        P5Pin6,

        /// <summary>
        /// Connector P5, pin 6.
        /// </summary>
        P5Pin06 = P5Pin6,

    }

    public enum ProcessorPin
    {
        /// <summary>
        /// Pin 0.
        /// </summary>
        Pin0 = 0,

        /// <summary>
        /// Pin 0.
        /// </summary>
        Pin00 = Pin0,

        /// <summary>
        /// Pin 1.
        /// </summary>
        Pin1 = 1,

        /// <summary>
        /// Pin 1.
        /// </summary>
        Pin01 = Pin1,

        /// <summary>
        /// Pin 2.
        /// </summary>
        Pin2 = 2,

        /// <summary>
        /// Pin 2.
        /// </summary>
        Pin02 = Pin2,

        /// <summary>
        /// Pin 3.
        /// </summary>
        Pin3 = 3,

        /// <summary>
        /// Pin 3.
        /// </summary>
        Pin03 = Pin3,

        /// <summary>
        /// Pin 4.
        /// </summary>
        Pin4 = 4,

        /// <summary>
        /// Pin 4.
        /// </summary>
        Pin04 = Pin4,

        /// <summary>
        /// Pin 5.
        /// </summary>
        Pin5 = 5,

        /// <summary>
        /// Pin 5.
        /// </summary>
        Pin05 = Pin5,

        /// <summary>
        /// Pin 6.
        /// </summary>
        Pin6 = 6,

        /// <summary>
        /// Pin 6.
        /// </summary>
        Pin06 = Pin6,

        /// <summary>
        /// Pin 7.
        /// </summary>
        Pin7 = 7,

        /// <summary>
        /// Pin 7.
        /// </summary>
        Pin07 = Pin7,

        /// <summary>
        /// Pin 8.
        /// </summary>
        Pin8 = 8,

        /// <summary>
        /// Pin 8.
        /// </summary>
        Pin08 = Pin8,

        /// <summary>
        /// Pin 9.
        /// </summary>
        Pin9 = 9,

        /// <summary>
        /// Pin 9.
        /// </summary>
        Pin09 = Pin9,

        /// <summary>
        /// Pin 10.
        /// </summary>
        Pin10 = 10,

        /// <summary>
        /// Pin 11.
        /// </summary>
        Pin11 = 11,

        /// <summary>
        /// Pin 12.
        /// </summary>
        Pin12 = 12,

        /// <summary>
        /// Pin 13.
        /// </summary>
        Pin13 = 13,

        /// <summary>
        /// Pin 14.
        /// </summary>
        Pin14 = 14,

        /// <summary>
        /// Pin 15.
        /// </summary>
        Pin15 = 15,

        /// <summary>
        /// Pin 16.
        /// </summary>
        Pin16 = 16,

        /// <summary>
        /// Pin 17.
        /// </summary>
        Pin17 = 17,

        /// <summary>
        /// Pin 18.
        /// </summary>
        Pin18 = 18,

        /// <summary>
        /// Pin 19.
        /// </summary>
        Pin19 = 19,

        /// <summary>
        /// Pin 20.
        /// </summary>
        Pin20 = 20,

        /// <summary>
        /// Pin 21.
        /// </summary>
        Pin21 = 21,

        /// <summary>
        /// Pin 22.
        /// </summary>
        Pin22 = 22,

        /// <summary>
        /// Pin 23.
        /// </summary>
        Pin23 = 23,

        /// <summary>
        /// Pin 24.
        /// </summary>
        Pin24 = 24,

        /// <summary>
        /// Pin 25.
        /// </summary>
        Pin25 = 25,

        /// <summary>
        /// Pin 26.
        /// </summary>
        Pin26 = 26,

        /// <summary>
        /// Pin 27.
        /// </summary>
        Pin27 = 27,

        /// <summary>
        /// Pin 28.
        /// </summary>
        Pin28 = 28,

        /// <summary>
        /// Pin 29.
        /// </summary>
        Pin29 = 29,

        /// <summary>
        /// Pin 30.
        /// </summary>
        Pin30 = 30,

        /// <summary>
        /// Pin 31.
        /// </summary>
        Pin31 = 31
    }

    public static class PinMapping
    {


        private static readonly Dictionary<ProcessorPin, ConnectorPin> connectorMappings;
        private static readonly Dictionary<ConnectorPin, ProcessorPin> processorMappings;




        static PinMapping()
        {
            var mapping = /* Value is not used but required for anonymous type */ new[] { new { Processor = ProcessorPin.Pin0, Connector = ConnectorPin.P1Pin03 } };

            if (GpioConnectionSettings.BoardConnectorRevision == 1)
                mapping = new[]
                {
                    new {Processor = ProcessorPin.Pin0, Connector = ConnectorPin.P1Pin3},
                    new {Processor = ProcessorPin.Pin1, Connector = ConnectorPin.P1Pin5},
                    new {Processor = ProcessorPin.Pin4, Connector = ConnectorPin.P1Pin7},
                    new {Processor = ProcessorPin.Pin7, Connector = ConnectorPin.P1Pin26},
                    new {Processor = ProcessorPin.Pin8, Connector = ConnectorPin.P1Pin24},
                    new {Processor = ProcessorPin.Pin9, Connector = ConnectorPin.P1Pin21},
                    new {Processor = ProcessorPin.Pin10, Connector = ConnectorPin.P1Pin19},
                    new {Processor = ProcessorPin.Pin11, Connector = ConnectorPin.P1Pin23},
                    new {Processor = ProcessorPin.Pin14, Connector = ConnectorPin.P1Pin8},
                    new {Processor = ProcessorPin.Pin15, Connector = ConnectorPin.P1Pin10},
                    new {Processor = ProcessorPin.Pin17, Connector = ConnectorPin.P1Pin11},
                    new {Processor = ProcessorPin.Pin18, Connector = ConnectorPin.P1Pin12},
                    new {Processor = ProcessorPin.Pin21, Connector = ConnectorPin.P1Pin13},
                    new {Processor = ProcessorPin.Pin22, Connector = ConnectorPin.P1Pin15},
                    new {Processor = ProcessorPin.Pin23, Connector = ConnectorPin.P1Pin16},
                    new {Processor = ProcessorPin.Pin24, Connector = ConnectorPin.P1Pin18},
                    new {Processor = ProcessorPin.Pin25, Connector = ConnectorPin.P1Pin22}
                };

            else if (GpioConnectionSettings.BoardConnectorRevision == 2)
                mapping = new[]
                {
                    new {Processor = ProcessorPin.Pin2, Connector = ConnectorPin.P1Pin3},
                    new {Processor = ProcessorPin.Pin3, Connector = ConnectorPin.P1Pin5},
                    new {Processor = ProcessorPin.Pin4, Connector = ConnectorPin.P1Pin7},
                    new {Processor = ProcessorPin.Pin7, Connector = ConnectorPin.P1Pin26},
                    new {Processor = ProcessorPin.Pin8, Connector = ConnectorPin.P1Pin24},
                    new {Processor = ProcessorPin.Pin9, Connector = ConnectorPin.P1Pin21},
                    new {Processor = ProcessorPin.Pin10, Connector = ConnectorPin.P1Pin19},
                    new {Processor = ProcessorPin.Pin11, Connector = ConnectorPin.P1Pin23},
                    new {Processor = ProcessorPin.Pin14, Connector = ConnectorPin.P1Pin8},
                    new {Processor = ProcessorPin.Pin15, Connector = ConnectorPin.P1Pin10},
                    new {Processor = ProcessorPin.Pin17, Connector = ConnectorPin.P1Pin11},
                    new {Processor = ProcessorPin.Pin18, Connector = ConnectorPin.P1Pin12},
                    new {Processor = ProcessorPin.Pin27, Connector = ConnectorPin.P1Pin13},
                    new {Processor = ProcessorPin.Pin22, Connector = ConnectorPin.P1Pin15},
                    new {Processor = ProcessorPin.Pin23, Connector = ConnectorPin.P1Pin16},
                    new {Processor = ProcessorPin.Pin24, Connector = ConnectorPin.P1Pin18},
                    new {Processor = ProcessorPin.Pin25, Connector = ConnectorPin.P1Pin22},
                    new {Processor = ProcessorPin.Pin28, Connector = ConnectorPin.P5Pin3},
                    new {Processor = ProcessorPin.Pin29, Connector = ConnectorPin.P5Pin4},
                    new {Processor = ProcessorPin.Pin30, Connector = ConnectorPin.P5Pin5},
                    new {Processor = ProcessorPin.Pin31, Connector = ConnectorPin.P5Pin6}
                };

            else //if (GpioConnectionSettings.BoardConnectorRevision == 3)
                mapping = new[]
                {
                    new {Processor = ProcessorPin.Pin2, Connector = ConnectorPin.P1Pin3},
                    new {Processor = ProcessorPin.Pin3, Connector = ConnectorPin.P1Pin5},
                    new {Processor = ProcessorPin.Pin4, Connector = ConnectorPin.P1Pin7},
                    new {Processor = ProcessorPin.Pin5, Connector = ConnectorPin.P1Pin29},
                    new {Processor = ProcessorPin.Pin6, Connector = ConnectorPin.P1Pin31},
                    new {Processor = ProcessorPin.Pin7, Connector = ConnectorPin.P1Pin26},
                    new {Processor = ProcessorPin.Pin8, Connector = ConnectorPin.P1Pin24},
                    new {Processor = ProcessorPin.Pin9, Connector = ConnectorPin.P1Pin21},
                    new {Processor = ProcessorPin.Pin10, Connector = ConnectorPin.P1Pin19},
                    new {Processor = ProcessorPin.Pin11, Connector = ConnectorPin.P1Pin23},
                    new {Processor = ProcessorPin.Pin12, Connector = ConnectorPin.P1Pin32},
                    new {Processor = ProcessorPin.Pin13, Connector = ConnectorPin.P1Pin33},
                    new {Processor = ProcessorPin.Pin14, Connector = ConnectorPin.P1Pin8},
                    new {Processor = ProcessorPin.Pin15, Connector = ConnectorPin.P1Pin10},
                    new {Processor = ProcessorPin.Pin16, Connector = ConnectorPin.P1Pin36},
                    new {Processor = ProcessorPin.Pin17, Connector = ConnectorPin.P1Pin11},
                    new {Processor = ProcessorPin.Pin18, Connector = ConnectorPin.P1Pin12},
                    new {Processor = ProcessorPin.Pin19, Connector = ConnectorPin.P1Pin35},
                    new {Processor = ProcessorPin.Pin20, Connector = ConnectorPin.P1Pin38},
                    new {Processor = ProcessorPin.Pin21, Connector = ConnectorPin.P1Pin40},
                    new {Processor = ProcessorPin.Pin22, Connector = ConnectorPin.P1Pin15},
                    new {Processor = ProcessorPin.Pin23, Connector = ConnectorPin.P1Pin16},
                    new {Processor = ProcessorPin.Pin24, Connector = ConnectorPin.P1Pin18},
                    new {Processor = ProcessorPin.Pin25, Connector = ConnectorPin.P1Pin22},
                    new {Processor = ProcessorPin.Pin26, Connector = ConnectorPin.P1Pin37},
                    new {Processor = ProcessorPin.Pin27, Connector = ConnectorPin.P1Pin13},
                };

            processorMappings = mapping.ToDictionary(p => p.Connector, p => p.Processor);
            connectorMappings = mapping.ToDictionary(p => p.Processor, p => p.Connector);
        }




        /// <summary>
        /// Convert the specified connector pin to a processor pin.
        /// </summary>
        /// <param name="pin">The connector pin.</param>
        /// <returns>The processor pin.</returns>
        public static ProcessorPin ToProcessor(this ConnectorPin pin)
        {
            ProcessorPin processorPin;
            if (!processorMappings.TryGetValue(pin, out processorPin))
                throw new InvalidOperationException(string.Format(CultureInfo.InvariantCulture, "Connector pin {0} is not mapped to processor with pin layout revision {1}", pin.ToString().Replace("Pin", "-"), GpioConnectionSettings.BoardConnectorRevision));

            return processorPin;
        }

        ///// <summary>
        ///// Convert the specified processor pin to a connector pin.
        ///// </summary>
        ///// <param name="pin">The processor pin.</param>
        ///// <returns>The connector pin.</returns>
        //public static ConnectorPin ToConnector(this ProcessorPin pin)
        //{
        //    ConnectorPin connectorPin;
        //    if (!connectorMappings.TryGetValue(pin, out connectorPin))
        //        throw new InvalidOperationException(string.Format(CultureInfo.InvariantCulture, "Processor pin {0} is not mapped to processor with pin layout revision {1}", (int)pin, GpioConnectionSettings.BoardConnectorRevision));

        //    return connectorPin;
        //}

    }

    public class GpioConnectionSettings
    {

        private static int _revision = 2;
        public static int BoardConnectorRevision
        {
            get
            {

                return _revision;
            }
            set
            {
                _revision = value;
            }
        }

        public static IGpioConnectionDriver DefaultDriver
        {
            get
            {
                return (IGpioConnectionDriver)new GpioConnectionDriver();
            }
        }

    }

    #region Mcp23017

    public enum Mcp23017Pin
    {
        A0 = 0x0001,
        A1 = 0x0002,
        A2 = 0x0004,
        A3 = 0x0008,
        A4 = 0x0010,
        A5 = 0x0020,
        A6 = 0x0040,
        A7 = 0x0080,

        B0 = 0x0101,
        B1 = 0x0102,
        B2 = 0x0104,
        B3 = 0x0108,
        B4 = 0x0110,
        B5 = 0x0120,
        B6 = 0x0140,
        B7 = 0x0180,
    }
    public enum Mcp23017PinDirection
    {
        Input,
        Output
    }
    public enum Mcp23017PinPolarity
    {
        Normal,
        Inverted
    }
    public enum Mcp23017PinResistor
    {
        None,
        PullUp
    }

    internal static class Interop
    {
        #region BCM2835

        #region Constants

        public const uint BCM2835_PERI_BASE = 0x20000000;
        public const uint BCM2835_GPIO_BASE = (BCM2835_PERI_BASE + 0x200000);
        public const uint BCM2835_BSC0_BASE = (BCM2835_PERI_BASE + 0x205000);
        public const uint BCM2835_BSC1_BASE = (BCM2835_PERI_BASE + 0x804000);

        public const uint BCM2835_BLOCK_SIZE = (4 * 1024);

        public const uint BCM2835_BSC_C = 0x0000;
        public const uint BCM2835_BSC_S = 0x0004;
        public const uint BCM2835_BSC_DLEN = 0x0008;
        public const uint BCM2835_BSC_A = 0x000c;
        public const uint BCM2835_BSC_FIFO = 0x0010;
        public const uint BCM2835_BSC_DIV = 0x0014;

        public const uint BCM2835_BSC_C_CLEAR_1 = 0x00000020;
        public const uint BCM2835_BSC_C_CLEAR_2 = 0x00000010;
        public const uint BCM2835_BSC_C_I2CEN = 0x00008000;
        public const uint BCM2835_BSC_C_ST = 0x00000080;
        public const uint BCM2835_BSC_C_READ = 0x00000001;

        public const uint BCM2835_BSC_S_CLKT = 0x00000200;
        public const uint BCM2835_BSC_S_ERR = 0x00000100;
        public const uint BCM2835_BSC_S_DONE = 0x00000002;
        public const uint BCM2835_BSC_S_TXD = 0x00000010;
        public const uint BCM2835_BSC_S_RXD = 0x00000020;

        public const uint BCM2835_BSC_FIFO_SIZE = 16;

        public const uint BCM2835_CORE_CLK_HZ = 250000000;

        public const uint BCM2835_GPIO_FSEL_INPT = 0;
        public const uint BCM2835_GPIO_FSEL_OUTP = 1;
        public const uint BCM2835_GPIO_FSEL_ALT0 = 4;
        public const uint BCM2835_GPIO_FSEL_MASK = 7;

        public const uint BCM2835_GPFSEL0 = 0x0000;
        public const uint BCM2835_GPPUD = 0x0094;
        public const uint BCM2835_GPPUDCLK0 = 0x0098;
        public const uint BCM2835_GPSET0 = 0x001c;
        public const uint BCM2835_GPCLR0 = 0x0028;
        public const uint BCM2835_GPLEV0 = 0x0034;

        public const uint BCM2835_GPIO_PUD_OFF = 0;
        public const uint BCM2835_GPIO_PUD_DOWN = 1;
        public const uint BCM2835_GPIO_PUD_UP = 2;


        public const int EPOLLIN = 1;
        public const int EPOLLPRI = 2;
        public const int EPOLLET = (1 << 31);

        public const int EPOLL_CTL_ADD = 0x1;
        public const int EPOLL_CTL_DEL = 0x2;



        #endregion

        #endregion

        #region Libc

        #region Constants

        public const int O_RDWR = 2;
        public const int O_SYNC = 10000;

        public const int PROT_READ = 1;
        public const int PROT_WRITE = 2;

        public const int MAP_SHARED = 1;
        public const int MAP_FAILED = -1;

        #endregion

        #region Methods

        [DllImport("libc.so.6", EntryPoint = "open")]
        public static extern IntPtr open(string fileName, int mode);

        [DllImport("libc.so.6", EntryPoint = "close")]
        public static extern void close(IntPtr file);

        [DllImport("libc.so.6", EntryPoint = "mmap")]
        public static extern IntPtr mmap(IntPtr address, uint size, int protect, int flags, IntPtr file, uint offset);

        [DllImport("libc.so.6", EntryPoint = "munmap")]
        public static extern IntPtr munmap(IntPtr address, uint size);

        [DllImport("libc.so.6", EntryPoint = "epoll_create")]
        public static extern int epoll_create(int size);

        [DllImport("libc.so.6", EntryPoint = "epoll_ctl")]
        public static extern int epoll_ctl(int epfd, int op, int fd, IntPtr epevent);

        [DllImport("libc.so.6", EntryPoint = "epoll_wait")]
        public static extern int epoll_wait(int epfd, IntPtr events, int maxevents, int timeout);

        [StructLayout(LayoutKind.Explicit)]
        public struct epoll_data
        {
            [FieldOffset(0)]
            public IntPtr ptr;
            [FieldOffset(0)]
            public int fd;
            [FieldOffset(0)]
            public UInt32 u32;
            [FieldOffset(0)]
            public UInt64 u64;
        };

        [StructLayout(LayoutKind.Explicit)]
        public struct epoll_event
        {
            [FieldOffset(0)]
            public int events;
            [FieldOffset(4)]
            public epoll_data data;
        };
        #endregion

        #endregion
    }

    public class I2cDriver : IDisposable
    {
        #region Fields

        private readonly object driverLock = new object();

        private readonly ProcessorPin sdaPin;
        private readonly ProcessorPin sclPin;

        private readonly IntPtr gpioAddress;
        private readonly IntPtr bscAddress;

        private int currentDeviceAddress;
        private int waitInterval;

        #endregion

        #region Instance Management

        /// <summary>
        /// Initializes a new instance of the <see cref="I2cDriver"/> class.
        /// </summary>
        /// <param name="sdaPin">The SDA pin.</param>
        /// <param name="sclPin">The SCL pin.</param>
        public I2cDriver(ProcessorPin sdaPin, ProcessorPin sclPin)
        {
            this.sdaPin = sdaPin;
            this.sclPin = sclPin;

            var bscBase = GetBscBase(sdaPin, sclPin);

            var memoryFile = Interop.open("/dev/mem", Interop.O_RDWR + Interop.O_SYNC);
            try
            {
                gpioAddress = Interop.mmap(IntPtr.Zero, Interop.BCM2835_BLOCK_SIZE, Interop.PROT_READ | Interop.PROT_WRITE, Interop.MAP_SHARED, memoryFile, Interop.BCM2835_GPIO_BASE);
                bscAddress = Interop.mmap(IntPtr.Zero, Interop.BCM2835_BLOCK_SIZE, Interop.PROT_READ | Interop.PROT_WRITE, Interop.MAP_SHARED, memoryFile, bscBase);
            }
            finally
            {
                Interop.close(memoryFile);
            }

            if (bscAddress == (IntPtr)Interop.MAP_FAILED)
                throw new InvalidOperationException("Unable to access device memory");

            // Set the I2C pins to the Alt 0 function to enable I2C access on them
            SetPinMode((uint)(int)sdaPin, Interop.BCM2835_GPIO_FSEL_ALT0); // SDA
            SetPinMode((uint)(int)sclPin, Interop.BCM2835_GPIO_FSEL_ALT0); // SCL

            // Read the clock divider register
            var dividerAddress = bscAddress + (int)Interop.BCM2835_BSC_DIV;
            var divider = (ushort)SafeReadUInt32(dividerAddress);
            waitInterval = GetWaitInterval(divider);

            var addressAddress = bscAddress + (int)Interop.BCM2835_BSC_A;
            SafeWriteUInt32(addressAddress, (uint)currentDeviceAddress);
        }

        /// <summary>
        /// Performs application-defined tasks associated with freeing, releasing, or resetting unmanaged resources.
        /// </summary>
        public void Dispose()
        {
            // Set all the I2C/BSC1 pins back to input
            SetPinMode((uint)(int)sdaPin, Interop.BCM2835_GPIO_FSEL_INPT); // SDA
            SetPinMode((uint)(int)sclPin, Interop.BCM2835_GPIO_FSEL_INPT); // SCL

            Interop.munmap(gpioAddress, Interop.BCM2835_BLOCK_SIZE);
            Interop.munmap(bscAddress, Interop.BCM2835_BLOCK_SIZE);
        }

        #endregion

        #region Properties

        /// <summary>
        /// Gets or sets the clock divider.
        /// </summary>
        /// <value>
        /// The clock divider.
        /// </value>
        public int ClockDivider
        {
            get
            {
                var dividerAddress = bscAddress + (int)Interop.BCM2835_BSC_DIV;
                return (ushort)SafeReadUInt32(dividerAddress);
            }
            set
            {
                var dividerAddress = bscAddress + (int)Interop.BCM2835_BSC_DIV;
                SafeWriteUInt32(dividerAddress, (uint)value);

                var actualDivider = (ushort)SafeReadUInt32(dividerAddress);
                waitInterval = GetWaitInterval(actualDivider);
            }
        }

        #endregion

        #region Methods

        /// <summary>
        /// Connects the specified device address.
        /// </summary>
        /// <param name="deviceAddress">The device address.</param>
        /// <returns>The device connection</returns>
        public I2cDeviceConnection Connect(int deviceAddress)
        {
            return new I2cDeviceConnection(this, deviceAddress);
        }

        #endregion

        #region Internal Methods

        internal void Write(int deviceAddress, byte[] buffer)
        {
            lock (driverLock)
            {
                EnsureDeviceAddress(deviceAddress);

                var len = (uint)buffer.Length;

                var dlen = bscAddress + (int)Interop.BCM2835_BSC_DLEN;
                var fifo = bscAddress + (int)Interop.BCM2835_BSC_FIFO;
                var status = bscAddress + (int)Interop.BCM2835_BSC_S;
                var control = bscAddress + (int)Interop.BCM2835_BSC_C;

                var remaining = len;
                var i = 0;

                // Clear FIFO
                WriteUInt32Mask(control, Interop.BCM2835_BSC_C_CLEAR_1, Interop.BCM2835_BSC_C_CLEAR_1);

                // Clear Status
                WriteUInt32(status, Interop.BCM2835_BSC_S_CLKT | Interop.BCM2835_BSC_S_ERR | Interop.BCM2835_BSC_S_DONE);

                // Set Data Length
                WriteUInt32(dlen, len);

                while (remaining != 0 && i < Interop.BCM2835_BSC_FIFO_SIZE)
                {
                    WriteUInt32(fifo, buffer[i]);
                    i++;
                    remaining--;
                }

                // Enable device and start transfer
                WriteUInt32(control, Interop.BCM2835_BSC_C_I2CEN | Interop.BCM2835_BSC_C_ST);

                while ((ReadUInt32(status) & Interop.BCM2835_BSC_S_DONE) == 0)
                {
                    while (remaining != 0 && (ReadUInt32(status) & Interop.BCM2835_BSC_S_TXD) != 0)
                    {
                        // Write to FIFO, no barrier
                        WriteUInt32(fifo, buffer[i]);
                        i++;
                        remaining--;
                    }

                    Wait(remaining);
                }

                if ((SafeReadUInt32(status) & Interop.BCM2835_BSC_S_ERR) != 0) // Received a NACK
                    throw new InvalidOperationException("Read operation failed with BCM2835_I2C_REASON_ERROR_NACK status");
                if ((SafeReadUInt32(status) & Interop.BCM2835_BSC_S_CLKT) != 0) // Received Clock Stretch Timeout
                    throw new InvalidOperationException("Read operation failed with BCM2835_I2C_REASON_ERROR_CLKT status");
                if (remaining != 0) // Not all data is sent
                    throw new InvalidOperationException(string.Format("Read operation failed with BCM2835_I2C_REASON_ERROR_DATA status, missing {0} bytes", remaining));

                WriteUInt32Mask(control, Interop.BCM2835_BSC_S_DONE, Interop.BCM2835_BSC_S_DONE);
            }
        }

        internal byte[] Read(int deviceAddress, int byteCount)
        {
            lock (driverLock)
            {
                EnsureDeviceAddress(deviceAddress);

                var dlen = bscAddress + (int)Interop.BCM2835_BSC_DLEN;
                var fifo = bscAddress + (int)Interop.BCM2835_BSC_FIFO;
                var status = bscAddress + (int)Interop.BCM2835_BSC_S;
                var control = bscAddress + (int)Interop.BCM2835_BSC_C;

                var remaining = (uint)byteCount;
                uint i = 0;

                // Clear FIFO
                WriteUInt32Mask(control, Interop.BCM2835_BSC_C_CLEAR_1, Interop.BCM2835_BSC_C_CLEAR_1);

                // Clear Status
                WriteUInt32(status, Interop.BCM2835_BSC_S_CLKT | Interop.BCM2835_BSC_S_ERR | Interop.BCM2835_BSC_S_DONE);

                // Set Data Length
                WriteUInt32(dlen, (uint)byteCount);

                // Start read
                WriteUInt32(control, Interop.BCM2835_BSC_C_I2CEN | Interop.BCM2835_BSC_C_ST | Interop.BCM2835_BSC_C_READ);

                var buffer = new byte[byteCount];
                while ((ReadUInt32(status) & Interop.BCM2835_BSC_S_DONE) == 0)
                {
                    while ((ReadUInt32(status) & Interop.BCM2835_BSC_S_RXD) != 0)
                    {
                        // Read from FIFO, no barrier
                        buffer[i] = (byte)ReadUInt32(fifo);

                        i++;
                        remaining--;
                    }

                    Wait(remaining);
                }

                while (remaining != 0 && (ReadUInt32(status) & Interop.BCM2835_BSC_S_RXD) != 0)
                {
                    buffer[i] = (byte)ReadUInt32(fifo);
                    i++;
                    remaining--;
                }

                if ((SafeReadUInt32(status) & Interop.BCM2835_BSC_S_ERR) != 0) // Received a NACK
                    throw new InvalidOperationException("Read operation failed with BCM2835_I2C_REASON_ERROR_NACK status");
                if ((SafeReadUInt32(status) & Interop.BCM2835_BSC_S_CLKT) != 0) // Received Clock Stretch Timeout
                    throw new InvalidOperationException("Read operation failed with BCM2835_I2C_REASON_ERROR_CLKT status");
                if (remaining != 0) // Not all data is received
                    throw new InvalidOperationException(string.Format("Read operation failed with BCM2835_I2C_REASON_ERROR_DATA status, missing {0} bytes", remaining));

                WriteUInt32Mask(control, Interop.BCM2835_BSC_S_DONE, Interop.BCM2835_BSC_S_DONE);

                return buffer;
            }
        }

        #endregion

        #region Private Helpers

        private void EnsureDeviceAddress(int deviceAddress)
        {
            if (deviceAddress != currentDeviceAddress)
            {
                var addressAddress = bscAddress + (int)Interop.BCM2835_BSC_A;
                SafeWriteUInt32(addressAddress, (uint)deviceAddress);

                currentDeviceAddress = deviceAddress;
            }
        }

        private void Wait(uint remaining)
        {
            // When remaining data is to be received, then wait for a fully FIFO
            if (remaining != 0)
            {
                //   Timer.Sleep(waitInterval * (remaining >= Interop.BCM2835_BSC_FIFO_SIZE ? Interop.BCM2835_BSC_FIFO_SIZE : remaining) / 1000m);
                System.Threading.Thread.Sleep((int)(waitInterval * (remaining >= Interop.BCM2835_BSC_FIFO_SIZE ? Interop.BCM2835_BSC_FIFO_SIZE : remaining) / 1000m));
            }


        }

        private static int GetWaitInterval(ushort actualDivider)
        {
            // Calculate time for transmitting one byte
            // 1000000 = micros seconds in a second
            // 9 = Clocks per byte : 8 bits + ACK

            return (int)((decimal)actualDivider * 1000000 * 9 / Interop.BCM2835_CORE_CLK_HZ);
        }

        private static uint GetBscBase(ProcessorPin sdaPin, ProcessorPin sclPin)
        {
            switch (GpioConnectionSettings.BoardConnectorRevision)
            {
                case 1:
                    if (sdaPin == ProcessorPin.Pin0 && sclPin == ProcessorPin.Pin1)
                        return Interop.BCM2835_BSC0_BASE;
                    throw new InvalidOperationException("No I2C device exist on the specified pins");

                case 2:
                    if (sdaPin == ProcessorPin.Pin28 && sclPin == ProcessorPin.Pin29)
                        return Interop.BCM2835_BSC0_BASE;
                    if (sdaPin == ProcessorPin.Pin2 && sclPin == ProcessorPin.Pin3)
                        return Interop.BCM2835_BSC1_BASE;
                    throw new InvalidOperationException("No I2C device exist on the specified pins");

                case 3:
                    if (sdaPin == ProcessorPin.Pin2 && sclPin == ProcessorPin.Pin3)
                        return Interop.BCM2835_BSC1_BASE;
                    throw new InvalidOperationException("No I2C device exist on the specified pins");

                default:
                    throw new NotSupportedException(string.Format(CultureInfo.InvariantCulture, "Board revision {0} is not supported", GpioConnectionSettings.BoardConnectorRevision));
            }
        }

        private void SetPinMode(uint pin, uint mode)
        {
            // Function selects are 10 pins per 32 bit word, 3 bits per pin
            var paddr = gpioAddress + (int)(Interop.BCM2835_GPFSEL0 + 4 * (pin / 10));
            var shift = (pin % 10) * 3;
            var mask = Interop.BCM2835_GPIO_FSEL_MASK << (int)shift;
            var value = mode << (int)shift;

            WriteUInt32Mask(paddr, value, mask);
        }

        private static void WriteUInt32Mask(IntPtr address, uint value, uint mask)
        {
            var v = SafeReadUInt32(address);
            v = (v & ~mask) | (value & mask);
            SafeWriteUInt32(address, v);
        }

        private static uint SafeReadUInt32(IntPtr address)
        {
            // Make sure we dont return the _last_ read which might get lost
            // if subsequent code changes to a different peripheral
            unchecked
            {
                var returnValue = (uint)Marshal.ReadInt32(address);
                Marshal.ReadInt32(address);

                return returnValue;
            }
        }

        private static uint ReadUInt32(IntPtr address)
        {
            unchecked
            {
                return (uint)Marshal.ReadInt32(address);
            }
        }

        private static void SafeWriteUInt32(IntPtr address, uint value)
        {
            // Make sure we don't rely on the first write, which may get
            // lost if the previous access was to a different peripheral.
            unchecked
            {
                Marshal.WriteInt32(address, (int)value);
                Marshal.WriteInt32(address, (int)value);
            }
        }

        private static void WriteUInt32(IntPtr address, uint value)
        {
            unchecked
            {
                Marshal.WriteInt32(address, (int)value);
            }
        }

        #endregion
    }

    public class I2cDeviceConnection
    {
        #region Fields

        private readonly I2cDriver driver;
        private readonly int deviceAddress;

        #endregion

        #region Instance Management

        internal I2cDeviceConnection(I2cDriver driver, int deviceAddress)
        {
            this.driver = driver;
            this.deviceAddress = deviceAddress;
        }

        #endregion

        #region Properties

        /// <summary>
        /// Gets the device address.
        /// </summary>
        /// <value>
        /// The device address.
        /// </value>
        public int DeviceAddress
        {
            get { return deviceAddress; }
        }


        #endregion

        #region Methods

        /// <summary>
        /// Writes the specified buffer.
        /// </summary>
        /// <param name="buffer">The buffer.</param>
        public void Write(params byte[] buffer)
        {
            driver.Write(deviceAddress, buffer);
        }

        /// <summary>
        /// Writes the specified byte.
        /// </summary>
        /// <param name="value">The value.</param>
        public void WriteByte(byte value)
        {
            Write(value);
        }

        /// <summary>
        /// Reads the specified number of bytes.
        /// </summary>
        /// <param name="byteCount">The byte count.</param>
        /// <returns>The buffer.</returns>
        public byte[] Read(int byteCount)
        {
            return driver.Read(deviceAddress, byteCount);
        }

        /// <summary>
        /// Reads a byte.
        /// </summary>
        /// <returns>The byte.</returns>
        public byte ReadByte()
        {
            return Read(1)[0];
        }

        #endregion
    }

    public class Mcp23017I2cConnection
    {
        #region Fields

        private readonly I2cDeviceConnection connection;

        #endregion

        #region Instance Management

        /// <summary>
        /// Initializes a new instance of the <see cref="Mcp23017I2cConnection"/> class.
        /// </summary>
        /// <param name="connection">The connection.</param>
        public Mcp23017I2cConnection(I2cDeviceConnection connection)
        {
            this.connection = connection;
        }

        #endregion

        #region Methods

        /// <summary>
        /// Sets the direction.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <param name="direction">The direction.</param>
        public void SetDirection(Mcp23017Pin pin, Mcp23017PinDirection direction)
        {
            var register = ((int)pin & 0x0100) == 0x0000 ? Register.IODIRA : Register.IODIRB;

            connection.WriteByte((byte)register);
            var directions = connection.ReadByte();

            var bit = (byte)((int)pin & 0xFF);
            var newDirections = (direction == Mcp23017PinDirection.Input)
                                    ? directions | bit
                                    : directions & ~bit;

            connection.Write(new[] { (byte)register, (byte)newDirections });
        }

        /// <summary>
        /// Sets the polarity.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <param name="polarity">The polarity.</param>
        public void SetPolarity(Mcp23017Pin pin, Mcp23017PinPolarity polarity)
        {
            var register = ((int)pin & 0x0100) == 0x0000 ? Register.IPOLA : Register.IPOLB;

            connection.WriteByte((byte)register);
            var polarities = connection.ReadByte();

            var bit = (byte)((int)pin & 0xFF);
            var newPolarities = (polarity == Mcp23017PinPolarity.Inverted)
                                    ? polarities | bit
                                    : polarities & ~bit;

            connection.Write(new[] { (byte)register, (byte)newPolarities });
        }

        /// <summary>
        /// Sets the resistor.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <param name="resistor">The resistor.</param>
        public void SetResistor(Mcp23017Pin pin, Mcp23017PinResistor resistor)
        {
            var register = ((int)pin & 0x0100) == 0x0000 ? Register.GPPUA : Register.GPPUB;

            connection.WriteByte((byte)register);
            var resistors = connection.ReadByte();

            var bit = (byte)((int)pin & 0xFF);
            var newResistors = (resistor == Mcp23017PinResistor.PullUp)
                                   ? resistors | bit
                                   : resistors & ~bit;

            connection.Write(new[] { (byte)register, (byte)newResistors });
        }

        /// <summary>
        /// Sets the pin status.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <param name="enabled">if set to <c>true</c>, pin is enabled.</param>
        public void SetPinStatus(Mcp23017Pin pin, bool enabled)
        {
            var register = ((int)pin & 0x0100) == 0x0000 ? Register.GPIOA : Register.GPIOB;

            connection.WriteByte((byte)register);
            var status = connection.ReadByte();

            var bit = (byte)((int)pin & 0xFF);
            var newStatus = enabled
                                ? status | bit
                                : status & ~bit;

            connection.Write((byte)register, (byte)newStatus);
        }

        /// <summary>
        /// Gets the pin status.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <returns>The pin status.</returns>
        public bool GetPinStatus(Mcp23017Pin pin)
        {
            var register = ((int)pin & 0x0100) == 0x0000 ? Register.GPIOA : Register.GPIOB;

            connection.WriteByte((byte)register);
            var status = connection.ReadByte();

            var bit = (byte)((int)pin & 0xFF);
            return (status & bit) != 0x00;
        }

        /// <summary>
        /// Toogles the specified pin.
        /// </summary>
        /// <param name="pin">The pin.</param>
        public void Toogle(Mcp23017Pin pin)
        {
            var register = ((int)pin & 0x0100) == 0x0000 ? Register.GPIOA : Register.GPIOB;

            connection.WriteByte((byte)register);
            var status = connection.ReadByte();

            var bit = (byte)((int)pin & 0xFF);
            var bitEnabled = (status & bit) != 0x00;
            var newBitEnabled = !bitEnabled;

            var newStatus = newBitEnabled
                                ? status | bit
                                : status & ~bit;

            connection.Write((byte)register, (byte)newStatus);
        }

        #endregion

        #region Private Helpers

        private enum Register
        {
            IODIRA = 0x00,
            IODIRB = 0x01,
            IPOLA = 0x02,
            IPOLB = 0x03,
            GPPUA = 0x0c,
            GPPUB = 0x0d,
            GPIOA = 0x12,
            GPIOB = 0x13
        }

        #endregion
    }

    #endregion


    #region LCD Display
    [Flags]
    internal enum DisplayFlags
    {
        None = 0,

        BlinkOff = 0,
        CursorOff = 0,
        DisplayOff = 0,

        BlinkOn = 0x01,
        CursorOn = 0x02,
        DisplayOn = 0x04
    }
    [Flags]
    internal enum Functions
    {
        None = 0,

        Matrix5x8 = 0,
        Matrix5x10 = 0x04,

        OneLine = 0,
        TwoLines = 0x08,

        Data4bits = 0x0,
        Data8bits = 0x10,
    }
    [Flags]
    internal enum EntryModeFlags
    {
        None = 0,

        EntryRight = 0,
        EntryShiftDecrement = 0,

        EntryShiftIncrement = 0x01,
        EntryLeft = 0x02
    }
    internal enum Command
    {
        ClearDisplay = 0x01,
        ReturnHome = 0x02,
        SetEntryModeFlags = 0x04,
        SetDisplayFlags = 0x08,
        MoveCursor = 0x10,
        SetFunctions = 0x20,
        SetCGRamAddr = 0x40,
        //SetDDRamAddr = 0x80
    }
    [Flags]
    internal enum CursorShiftFlags
    {
        None = 0,

        CursorMove = 0,
        MoveLeft = 0,

        MoveRight = 0x04,
        DisplayMove = 0x08
    }
    public class Hd44780LcdConnectionSettings
    {
        #region Instance Management

        /// <summary>
        /// Initializes a new instance of the <see cref="Hd44780LcdConnectionSettings"/> class.
        /// </summary>
        public Hd44780LcdConnectionSettings()
        {
            ScreenWidth = 20;
            ScreenHeight = 2;
            PatternWidth = 5;
            PatternHeight = 8;

            Encoding = new Hd44780A00Encoding();
            //RightToLeft = false;
        }

        #endregion

        #region Properties

        /// <summary>
        /// Gets or sets the width of the screen.
        /// </summary>
        /// <value>
        /// The width of the screen.
        /// </value>
        public int ScreenWidth { get; set; }

        /// <summary>
        /// Gets or sets the height of the screen.
        /// </summary>
        /// <value>
        /// The height of the screen.
        /// </value>
        public int ScreenHeight { get; set; }

        /// <summary>
        /// Gets or sets the width of the pattern.
        /// </summary>
        /// <value>
        /// The width of the pattern.
        /// </value>
        public int PatternWidth { get; set; }

        /// <summary>
        /// Gets or sets the height of the pattern.
        /// </summary>
        /// <value>
        /// The height of the pattern.
        /// </value>
        public int PatternHeight { get; set; }

        /// <summary>
        /// Gets or sets the encoding.
        /// </summary>
        /// <value>
        /// The encoding.
        /// </value>
        public Encoding Encoding { get; set; }

        //public bool RightToLeft { get; set; }

        #endregion
    }

    public class Hd44780A00Encoding : Encoding
    {
        #region Fields

        private static readonly Dictionary<char, byte> charMap = GetMap().GroupBy(p => p.Key, p => p.Value).ToDictionary(g => g.Key, g => g.First());
        private static readonly Dictionary<byte, char> byteMap = GetMap().GroupBy(p => p.Value, p => p.Key).ToDictionary(g => g.Key, g => g.First());

        private const byte missingChar = 0x3F;
        private const char missingByte = '\uFFFD';

        #endregion

        #region Properties

        /// <summary>
        /// Gets the supported characters.
        /// </summary>
        public static IEnumerable<char> SupportedCharacters
        {
            get { return charMap.Keys.Except(new[] { '\r', '\n' }); }
        }

        #endregion

        #region Methods

        /// <summary>
        /// When overridden in a derived class, calculates the number of bytes produced by encoding a set of characters from the specified character array.
        /// </summary>
        /// <param name="chars">The character array containing the set of characters to encode.</param>
        /// <param name="index">The index of the first character to encode.</param>
        /// <param name="count">The number of characters to encode.</param>
        /// <returns>
        /// The number of bytes produced by encoding the specified characters.
        /// </returns>
        public override int GetByteCount(char[] chars, int index, int count)
        {
            return count;
        }

        /// <summary>
        /// When overridden in a derived class, encodes a set of characters from the specified character array into the specified byte array.
        /// </summary>
        /// <param name="chars">The character array containing the set of characters to encode.</param>
        /// <param name="charIndex">The index of the first character to encode.</param>
        /// <param name="charCount">The number of characters to encode.</param>
        /// <param name="bytes">The byte array to contain the resulting sequence of bytes.</param>
        /// <param name="byteIndex">The index at which to start writing the resulting sequence of bytes.</param>
        /// <returns>
        /// The actual number of bytes written into <paramref name="bytes" />.
        /// </returns>
        public override int GetBytes(char[] chars, int charIndex, int charCount, byte[] bytes, int byteIndex)
        {
            Array.Copy(
                chars
                    .Skip(charIndex)
                    .Take(charCount)
                    .Select(c =>
                    {
                        byte b;
                        return charMap.TryGetValue(c, out b) ? b : missingChar;
                    })
                    .ToArray(),
                0,
                bytes,
                byteIndex,
                charCount);

            return charCount;
        }

        /// <summary>
        /// When overridden in a derived class, calculates the number of characters produced by decoding a sequence of bytes from the specified byte array.
        /// </summary>
        /// <param name="bytes">The byte array containing the sequence of bytes to decode.</param>
        /// <param name="index">The index of the first byte to decode.</param>
        /// <param name="count">The number of bytes to decode.</param>
        /// <returns>
        /// The number of characters produced by decoding the specified sequence of bytes.
        /// </returns>
        public override int GetCharCount(byte[] bytes, int index, int count)
        {
            return count;
        }

        /// <summary>
        /// When overridden in a derived class, decodes a sequence of bytes from the specified byte array into the specified character array.
        /// </summary>
        /// <param name="bytes">The byte array containing the sequence of bytes to decode.</param>
        /// <param name="byteIndex">The index of the first byte to decode.</param>
        /// <param name="byteCount">The number of bytes to decode.</param>
        /// <param name="chars">The character array to contain the resulting set of characters.</param>
        /// <param name="charIndex">The index at which to start writing the resulting set of characters.</param>
        /// <returns>
        /// The actual number of characters written into <paramref name="chars" />.
        /// </returns>
        public override int GetChars(byte[] bytes, int byteIndex, int byteCount, char[] chars, int charIndex)
        {
            Array.Copy(
                bytes
                    .Skip(byteIndex)
                    .Take(byteCount)
                    .Select(b =>
                    {
                        char c;
                        return byteMap.TryGetValue(b, out c) ? c : missingByte;
                    })
                    .ToArray(),
                0,
                chars,
                charIndex,
                byteCount);

            return byteCount;
        }

        /// <summary>
        /// When overridden in a derived class, calculates the maximum number of bytes produced by encoding the specified number of characters.
        /// </summary>
        /// <param name="charCount">The number of characters to encode.</param>
        /// <returns>
        /// The maximum number of bytes produced by encoding the specified number of characters.
        /// </returns>
        public override int GetMaxByteCount(int charCount)
        {
            return charCount;
        }

        /// <summary>
        /// When overridden in a derived class, calculates the maximum number of characters produced by decoding the specified number of bytes.
        /// </summary>
        /// <param name="byteCount">The number of bytes to decode.</param>
        /// <returns>
        /// The maximum number of characters produced by decoding the specified number of bytes.
        /// </returns>
        public override int GetMaxCharCount(int byteCount)
        {
            return byteCount;
        }

        #endregion

        #region Private Helpers

        private static IEnumerable<KeyValuePair<char, byte>> GetMap()
        {
            // CR/LF
            yield return new KeyValuePair<char, byte>('\u000A', 0x0A);
            yield return new KeyValuePair<char, byte>('\u000D', 0x0A);

            // Custom characters
            yield return new KeyValuePair<char, byte>('\u0000', 0x00);
            yield return new KeyValuePair<char, byte>('\u0001', 0x01);
            yield return new KeyValuePair<char, byte>('\u0002', 0x02);
            yield return new KeyValuePair<char, byte>('\u0003', 0x03);
            yield return new KeyValuePair<char, byte>('\u0004', 0x04);
            yield return new KeyValuePair<char, byte>('\u0005', 0x05);
            yield return new KeyValuePair<char, byte>('\u0006', 0x06);
            yield return new KeyValuePair<char, byte>('\u0007', 0x07);
            /*
                        yield return new KeyValuePair<char, byte>(' ', 0x08);
                        yield return new KeyValuePair<char, byte>(' ', 0x09);
                        yield return new KeyValuePair<char, byte>(' ', 0x0A);
                        yield return new KeyValuePair<char, byte>(' ', 0x0B);
                        yield return new KeyValuePair<char, byte>(' ', 0x0C);
                        yield return new KeyValuePair<char, byte>(' ', 0x0D);
                        yield return new KeyValuePair<char, byte>(' ', 0x0E);
                        yield return new KeyValuePair<char, byte>(' ', 0x0F);

                        yield return new KeyValuePair<char, byte>(' ', 0x10);
                        yield return new KeyValuePair<char, byte>(' ', 0x11);
                        yield return new KeyValuePair<char, byte>(' ', 0x12);
                        yield return new KeyValuePair<char, byte>(' ', 0x13);
                        yield return new KeyValuePair<char, byte>(' ', 0x14);
                        yield return new KeyValuePair<char, byte>(' ', 0x15);
                        yield return new KeyValuePair<char, byte>(' ', 0x16);
                        yield return new KeyValuePair<char, byte>(' ', 0x17);
                        yield return new KeyValuePair<char, byte>(' ', 0x18);
                        yield return new KeyValuePair<char, byte>(' ', 0x19);
                        yield return new KeyValuePair<char, byte>(' ', 0x1A);
                        yield return new KeyValuePair<char, byte>(' ', 0x1B);
                        yield return new KeyValuePair<char, byte>(' ', 0x1C);
                        yield return new KeyValuePair<char, byte>(' ', 0x1D);
                        yield return new KeyValuePair<char, byte>(' ', 0x1E);
                        yield return new KeyValuePair<char, byte>(' ', 0x1F);
            */
            yield return new KeyValuePair<char, byte>(' ', 0x20);
            // Variants
            yield return new KeyValuePair<char, byte>('\u0009', 0x20);
            yield return new KeyValuePair<char, byte>('\u000B', 0x20);
            yield return new KeyValuePair<char, byte>('\u000C', 0x20);
            yield return new KeyValuePair<char, byte>('\u0085', 0x20);
            yield return new KeyValuePair<char, byte>('\u00A0', 0x20);
            yield return new KeyValuePair<char, byte>('\u1680', 0x20);
            yield return new KeyValuePair<char, byte>('\u180E', 0x20);
            yield return new KeyValuePair<char, byte>('\u2000', 0x20);
            yield return new KeyValuePair<char, byte>('\u2001', 0x20);
            yield return new KeyValuePair<char, byte>('\u2002', 0x20);
            yield return new KeyValuePair<char, byte>('\u2003', 0x20);
            yield return new KeyValuePair<char, byte>('\u2004', 0x20);
            yield return new KeyValuePair<char, byte>('\u2005', 0x20);
            yield return new KeyValuePair<char, byte>('\u2006', 0x20);
            yield return new KeyValuePair<char, byte>('\u2007', 0x20);
            yield return new KeyValuePair<char, byte>('\u2008', 0x20);
            yield return new KeyValuePair<char, byte>('\u2009', 0x20);
            yield return new KeyValuePair<char, byte>('\u200A', 0x20);
            yield return new KeyValuePair<char, byte>('\u2028', 0x20);
            yield return new KeyValuePair<char, byte>('\u2029', 0x20);
            yield return new KeyValuePair<char, byte>('\u202F', 0x20);
            yield return new KeyValuePair<char, byte>('\u205F', 0x20);
            yield return new KeyValuePair<char, byte>('\u3000', 0x20);

            yield return new KeyValuePair<char, byte>('!', 0x21);

            yield return new KeyValuePair<char, byte>('"', 0x22);
            //Variants
            yield return new KeyValuePair<char, byte>('“', 0x22);
            yield return new KeyValuePair<char, byte>('”', 0x22);
            yield return new KeyValuePair<char, byte>('„', 0x22);
            yield return new KeyValuePair<char, byte>('‟', 0x22);

            yield return new KeyValuePair<char, byte>('#', 0x23);
            yield return new KeyValuePair<char, byte>('$', 0x24);
            yield return new KeyValuePair<char, byte>('%', 0x25);
            yield return new KeyValuePair<char, byte>('&', 0x26);

            yield return new KeyValuePair<char, byte>('\'', 0x27);
            // Variants
            yield return new KeyValuePair<char, byte>('‘', 0x2F);
            yield return new KeyValuePair<char, byte>('’', 0x2F);
            yield return new KeyValuePair<char, byte>('‛', 0x2F);
            yield return new KeyValuePair<char, byte>('′', 0x2F);

            yield return new KeyValuePair<char, byte>('(', 0x28);
            yield return new KeyValuePair<char, byte>(')', 0x29);
            yield return new KeyValuePair<char, byte>('*', 0x2A);
            yield return new KeyValuePair<char, byte>('+', 0x2B);
            yield return new KeyValuePair<char, byte>(',', 0x2C);

            yield return new KeyValuePair<char, byte>('-', 0x2D);
            // Variants
            yield return new KeyValuePair<char, byte>('‐', 0x2D);
            yield return new KeyValuePair<char, byte>('‒', 0x2D);
            yield return new KeyValuePair<char, byte>('–', 0x2D);
            yield return new KeyValuePair<char, byte>('—', 0x2D);
            yield return new KeyValuePair<char, byte>('―', 0x2D);

            yield return new KeyValuePair<char, byte>('.', 0x2E);

            yield return new KeyValuePair<char, byte>('/', 0x2F);
            // Variants
            yield return new KeyValuePair<char, byte>('⁄', 0x2F);

            yield return new KeyValuePair<char, byte>('0', 0x30);
            yield return new KeyValuePair<char, byte>('1', 0x31);
            yield return new KeyValuePair<char, byte>('2', 0x32);
            yield return new KeyValuePair<char, byte>('3', 0x33);
            yield return new KeyValuePair<char, byte>('4', 0x34);
            yield return new KeyValuePair<char, byte>('5', 0x35);
            yield return new KeyValuePair<char, byte>('6', 0x36);
            yield return new KeyValuePair<char, byte>('7', 0x37);
            yield return new KeyValuePair<char, byte>('8', 0x38);
            yield return new KeyValuePair<char, byte>('9', 0x39);
            yield return new KeyValuePair<char, byte>(':', 0x3A);
            yield return new KeyValuePair<char, byte>(';', 0x3B);

            yield return new KeyValuePair<char, byte>('<', 0x3C);
            // Variant
            yield return new KeyValuePair<char, byte>('‹', 0x3C);

            yield return new KeyValuePair<char, byte>('=', 0x3D);
            // Variant
            yield return new KeyValuePair<char, byte>('゠', 0x3D);

            yield return new KeyValuePair<char, byte>('>', 0x3E);
            // Variant
            yield return new KeyValuePair<char, byte>('›', 0x3E);

            yield return new KeyValuePair<char, byte>('?', 0x3F);
            // Variant
            yield return new KeyValuePair<char, byte>('¿', 0x3F);

            yield return new KeyValuePair<char, byte>('@', 0x40);

            yield return new KeyValuePair<char, byte>('A', 0x41);
            // Variants
            yield return new KeyValuePair<char, byte>('À', 0x41);
            yield return new KeyValuePair<char, byte>('Á', 0x41);
            yield return new KeyValuePair<char, byte>('Â', 0x41);
            yield return new KeyValuePair<char, byte>('Ã', 0x41);
            yield return new KeyValuePair<char, byte>('Ä', 0x41);
            yield return new KeyValuePair<char, byte>('Å', 0x41);

            yield return new KeyValuePair<char, byte>('B', 0x42);

            yield return new KeyValuePair<char, byte>('C', 0x43);
            // Variant
            yield return new KeyValuePair<char, byte>('Ç', 0x43);

            yield return new KeyValuePair<char, byte>('D', 0x44);

            yield return new KeyValuePair<char, byte>('E', 0x45);
            // Variants
            yield return new KeyValuePair<char, byte>('È', 0x45);
            yield return new KeyValuePair<char, byte>('É', 0x45);
            yield return new KeyValuePair<char, byte>('Ê', 0x45);
            yield return new KeyValuePair<char, byte>('Ë', 0x45);

            yield return new KeyValuePair<char, byte>('F', 0x46);
            yield return new KeyValuePair<char, byte>('G', 0x47);
            yield return new KeyValuePair<char, byte>('H', 0x48);

            yield return new KeyValuePair<char, byte>('I', 0x49);
            // Variants
            yield return new KeyValuePair<char, byte>('Ì', 0x49);
            yield return new KeyValuePair<char, byte>('Í', 0x49);
            yield return new KeyValuePair<char, byte>('Î', 0x49);
            yield return new KeyValuePair<char, byte>('Ï', 0x49);

            yield return new KeyValuePair<char, byte>('J', 0x4A);
            yield return new KeyValuePair<char, byte>('K', 0x4B);
            yield return new KeyValuePair<char, byte>('L', 0x4C);
            yield return new KeyValuePair<char, byte>('M', 0x4D);

            yield return new KeyValuePair<char, byte>('N', 0x4E);
            // Variant
            yield return new KeyValuePair<char, byte>('Ñ', 0x4E);

            yield return new KeyValuePair<char, byte>('O', 0x4F);
            // Variants
            yield return new KeyValuePair<char, byte>('Ò', 0x4F);
            yield return new KeyValuePair<char, byte>('Ó', 0x4F);
            yield return new KeyValuePair<char, byte>('Ô', 0x4F);
            yield return new KeyValuePair<char, byte>('Õ', 0x4F);
            yield return new KeyValuePair<char, byte>('Ö', 0x4F);
            yield return new KeyValuePair<char, byte>('Ø', 0x4F);

            yield return new KeyValuePair<char, byte>('P', 0x50);
            yield return new KeyValuePair<char, byte>('Q', 0x51);
            yield return new KeyValuePair<char, byte>('R', 0x52);
            yield return new KeyValuePair<char, byte>('S', 0x53);
            yield return new KeyValuePair<char, byte>('T', 0x54);

            yield return new KeyValuePair<char, byte>('U', 0x55);
            // Variants
            yield return new KeyValuePair<char, byte>('Ù', 0x55);
            yield return new KeyValuePair<char, byte>('Ú', 0x55);
            yield return new KeyValuePair<char, byte>('Û', 0x55);
            yield return new KeyValuePair<char, byte>('Ü', 0x55);

            yield return new KeyValuePair<char, byte>('V', 0x56);
            yield return new KeyValuePair<char, byte>('W', 0x57);
            yield return new KeyValuePair<char, byte>('X', 0x58);

            yield return new KeyValuePair<char, byte>('Y', 0x59);
            // Variant
            yield return new KeyValuePair<char, byte>('Ý', 0x59);

            yield return new KeyValuePair<char, byte>('Z', 0x5A);
            yield return new KeyValuePair<char, byte>('[', 0x5B);
            yield return new KeyValuePair<char, byte>('¥', 0x5C);
            yield return new KeyValuePair<char, byte>(']', 0x5D);
            yield return new KeyValuePair<char, byte>('^', 0x5E);

            yield return new KeyValuePair<char, byte>('_', 0x5F);
            // Variant
            yield return new KeyValuePair<char, byte>('‗', 0x5F);

            yield return new KeyValuePair<char, byte>('`', 0x60);

            yield return new KeyValuePair<char, byte>('a', 0x61);
            // Variants
            yield return new KeyValuePair<char, byte>('à', 0x61);
            yield return new KeyValuePair<char, byte>('á', 0x61);
            yield return new KeyValuePair<char, byte>('â', 0x61);
            yield return new KeyValuePair<char, byte>('ã', 0x61);
            yield return new KeyValuePair<char, byte>('å', 0x61);

            yield return new KeyValuePair<char, byte>('b', 0x62);

            yield return new KeyValuePair<char, byte>('c', 0x63);
            // Variant
            yield return new KeyValuePair<char, byte>('ç', 0x63);

            yield return new KeyValuePair<char, byte>('d', 0x64);

            yield return new KeyValuePair<char, byte>('e', 0x65);
            // Variants
            yield return new KeyValuePair<char, byte>('è', 0x65);
            yield return new KeyValuePair<char, byte>('é', 0x65);
            yield return new KeyValuePair<char, byte>('ê', 0x65);
            yield return new KeyValuePair<char, byte>('ë', 0x65);

            yield return new KeyValuePair<char, byte>('f', 0x66);
            yield return new KeyValuePair<char, byte>('g', 0x67);
            yield return new KeyValuePair<char, byte>('h', 0x68);

            yield return new KeyValuePair<char, byte>('i', 0x69);
            // Variants
            yield return new KeyValuePair<char, byte>('ì', 0x69);
            yield return new KeyValuePair<char, byte>('í', 0x69);
            yield return new KeyValuePair<char, byte>('î', 0x69);
            yield return new KeyValuePair<char, byte>('ï', 0x69);

            yield return new KeyValuePair<char, byte>('j', 0x6A);
            yield return new KeyValuePair<char, byte>('k', 0x6B);
            yield return new KeyValuePair<char, byte>('l', 0x6C);
            yield return new KeyValuePair<char, byte>('m', 0x6D);

            yield return new KeyValuePair<char, byte>('n', 0x6E);
            // Variant
            yield return new KeyValuePair<char, byte>('ñ', 0x6E);

            yield return new KeyValuePair<char, byte>('o', 0x6F);
            // Variants
            yield return new KeyValuePair<char, byte>('ò', 0x6F);
            yield return new KeyValuePair<char, byte>('ó', 0x6F);
            yield return new KeyValuePair<char, byte>('ô', 0x6F);
            yield return new KeyValuePair<char, byte>('õ', 0x6F);
            yield return new KeyValuePair<char, byte>('ö', 0x6F);
            yield return new KeyValuePair<char, byte>('ø', 0x6F);

            yield return new KeyValuePair<char, byte>('p', 0x70);
            yield return new KeyValuePair<char, byte>('q', 0x71);
            yield return new KeyValuePair<char, byte>('r', 0x72);
            yield return new KeyValuePair<char, byte>('s', 0x73);
            yield return new KeyValuePair<char, byte>('t', 0x74);

            yield return new KeyValuePair<char, byte>('u', 0x75);
            // Variants
            yield return new KeyValuePair<char, byte>('ù', 0x75);
            yield return new KeyValuePair<char, byte>('ú', 0x75);
            yield return new KeyValuePair<char, byte>('û', 0x75);
            yield return new KeyValuePair<char, byte>('ü', 0x75);

            yield return new KeyValuePair<char, byte>('v', 0x76);
            yield return new KeyValuePair<char, byte>('w', 0x77);
            yield return new KeyValuePair<char, byte>('x', 0x78);

            yield return new KeyValuePair<char, byte>('y', 0x79);
            // Variants
            yield return new KeyValuePair<char, byte>('ý', 0x79);
            yield return new KeyValuePair<char, byte>('ÿ', 0x79);

            yield return new KeyValuePair<char, byte>('z', 0x7A);
            yield return new KeyValuePair<char, byte>('{', 0x7B);
            yield return new KeyValuePair<char, byte>('|', 0x7C);
            yield return new KeyValuePair<char, byte>('}', 0x7D);
            yield return new KeyValuePair<char, byte>('→', 0x7E);
            yield return new KeyValuePair<char, byte>('←', 0x7F);

            yield return new KeyValuePair<char, byte>(' ', 0x80);
            yield return new KeyValuePair<char, byte>(' ', 0x81);
            yield return new KeyValuePair<char, byte>(' ', 0x82);
            yield return new KeyValuePair<char, byte>(' ', 0x83);
            yield return new KeyValuePair<char, byte>(' ', 0x84);
            yield return new KeyValuePair<char, byte>(' ', 0x85);
            yield return new KeyValuePair<char, byte>(' ', 0x86);
            yield return new KeyValuePair<char, byte>(' ', 0x87);
            yield return new KeyValuePair<char, byte>(' ', 0x88);
            yield return new KeyValuePair<char, byte>(' ', 0x89);
            yield return new KeyValuePair<char, byte>(' ', 0x8A);
            yield return new KeyValuePair<char, byte>(' ', 0x8B);
            yield return new KeyValuePair<char, byte>(' ', 0x8C);
            yield return new KeyValuePair<char, byte>(' ', 0x8D);
            yield return new KeyValuePair<char, byte>(' ', 0x8E);
            yield return new KeyValuePair<char, byte>(' ', 0x8F);

            yield return new KeyValuePair<char, byte>(' ', 0x90);
            yield return new KeyValuePair<char, byte>(' ', 0x91);
            yield return new KeyValuePair<char, byte>(' ', 0x92);
            yield return new KeyValuePair<char, byte>(' ', 0x93);
            yield return new KeyValuePair<char, byte>(' ', 0x94);
            yield return new KeyValuePair<char, byte>(' ', 0x95);
            yield return new KeyValuePair<char, byte>(' ', 0x96);
            yield return new KeyValuePair<char, byte>(' ', 0x97);
            yield return new KeyValuePair<char, byte>(' ', 0x98);
            yield return new KeyValuePair<char, byte>(' ', 0x99);
            yield return new KeyValuePair<char, byte>(' ', 0x9A);
            yield return new KeyValuePair<char, byte>(' ', 0x9B);
            yield return new KeyValuePair<char, byte>(' ', 0x9C);
            yield return new KeyValuePair<char, byte>(' ', 0x9D);
            yield return new KeyValuePair<char, byte>(' ', 0x9E);
            yield return new KeyValuePair<char, byte>(' ', 0x9F);

            yield return new KeyValuePair<char, byte>(' ', 0xA0);
            yield return new KeyValuePair<char, byte>('▫', 0xA1);
            //            yield return new KeyValuePair<char, byte>('', 0xA2);
            //            yield return new KeyValuePair<char, byte>('', 0xA3);
            yield return new KeyValuePair<char, byte>('ヽ', 0xA4);
            yield return new KeyValuePair<char, byte>('・', 0xA5);
            yield return new KeyValuePair<char, byte>('ヲ', 0xA6);
            yield return new KeyValuePair<char, byte>('ァ', 0xA7);
            yield return new KeyValuePair<char, byte>('ィ', 0xA8);
            yield return new KeyValuePair<char, byte>('ゥ', 0xA9);
            yield return new KeyValuePair<char, byte>('ェ', 0xAA);
            yield return new KeyValuePair<char, byte>('ォ', 0xAB);
            yield return new KeyValuePair<char, byte>('ャ', 0xAC);
            yield return new KeyValuePair<char, byte>('ュ', 0xAD);
            yield return new KeyValuePair<char, byte>('ョ', 0xAE);
            yield return new KeyValuePair<char, byte>('ッ', 0xAF);

            yield return new KeyValuePair<char, byte>('ー', 0xB0);
            yield return new KeyValuePair<char, byte>('ア', 0xB1);
            yield return new KeyValuePair<char, byte>('イ', 0xB2);
            yield return new KeyValuePair<char, byte>('ウ', 0xB3);
            yield return new KeyValuePair<char, byte>('エ', 0xB4);
            yield return new KeyValuePair<char, byte>('オ', 0xB5);
            yield return new KeyValuePair<char, byte>('カ', 0xB6);
            yield return new KeyValuePair<char, byte>('キ', 0xB7);
            yield return new KeyValuePair<char, byte>('ク', 0xB8);
            yield return new KeyValuePair<char, byte>('ケ', 0xB9);
            yield return new KeyValuePair<char, byte>('コ', 0xBA);
            yield return new KeyValuePair<char, byte>('サ', 0xBB);
            yield return new KeyValuePair<char, byte>('シ', 0xBC);
            yield return new KeyValuePair<char, byte>('ス', 0xBD);
            yield return new KeyValuePair<char, byte>('セ', 0xBE);
            yield return new KeyValuePair<char, byte>('ソ', 0xBF);

            yield return new KeyValuePair<char, byte>('タ', 0xC0);
            yield return new KeyValuePair<char, byte>('チ', 0xC1);
            yield return new KeyValuePair<char, byte>('ツ', 0xC2);
            yield return new KeyValuePair<char, byte>('テ', 0xC3);
            yield return new KeyValuePair<char, byte>('ト', 0xC4);
            yield return new KeyValuePair<char, byte>('ナ', 0xC5);
            yield return new KeyValuePair<char, byte>('ニ', 0xC6);
            yield return new KeyValuePair<char, byte>('ヌ', 0xC7);
            yield return new KeyValuePair<char, byte>('ネ', 0xC8);
            yield return new KeyValuePair<char, byte>('ノ', 0xC9);
            yield return new KeyValuePair<char, byte>('ハ', 0xCA);
            yield return new KeyValuePair<char, byte>('ヒ', 0xCB);
            yield return new KeyValuePair<char, byte>('フ', 0xCC);
            yield return new KeyValuePair<char, byte>('ヘ', 0xCD);
            yield return new KeyValuePair<char, byte>('ホ', 0xCE);
            yield return new KeyValuePair<char, byte>('マ', 0xCF);

            yield return new KeyValuePair<char, byte>('ミ', 0xD0);
            yield return new KeyValuePair<char, byte>('ム', 0xD1);
            yield return new KeyValuePair<char, byte>('メ', 0xD2);
            yield return new KeyValuePair<char, byte>('モ', 0xD3);
            yield return new KeyValuePair<char, byte>('ヤ', 0xD4);
            yield return new KeyValuePair<char, byte>('ユ', 0xD5);
            yield return new KeyValuePair<char, byte>('ヨ', 0xD6);
            yield return new KeyValuePair<char, byte>('ラ', 0xD7);
            yield return new KeyValuePair<char, byte>('リ', 0xD8);
            yield return new KeyValuePair<char, byte>('ル', 0xD9);
            yield return new KeyValuePair<char, byte>('レ', 0xDA);
            yield return new KeyValuePair<char, byte>('ロ', 0xDB);
            yield return new KeyValuePair<char, byte>('ワ', 0xDC);
            yield return new KeyValuePair<char, byte>('ン', 0xDD);

            yield return new KeyValuePair<char, byte>('゛', 0xDE);

            yield return new KeyValuePair<char, byte>('゜', 0xDF);
            // Variant
            yield return new KeyValuePair<char, byte>('°', 0xDF);

            yield return new KeyValuePair<char, byte>('α', 0xE0);

            yield return new KeyValuePair<char, byte>('ä', 0xE1);
            // Variant
            yield return new KeyValuePair<char, byte>('ӓ', 0xE1);

            yield return new KeyValuePair<char, byte>('β', 0xE2);
            // Variant
            yield return new KeyValuePair<char, byte>('ß', 0xE2);

            yield return new KeyValuePair<char, byte>('ε', 0xE3);
            yield return new KeyValuePair<char, byte>('μ', 0xE4);
            yield return new KeyValuePair<char, byte>('σ', 0xE5);
            yield return new KeyValuePair<char, byte>('ρ', 0xE6);
            yield return new KeyValuePair<char, byte>('ɡ', 0xE7);
            yield return new KeyValuePair<char, byte>('√', 0xE8);
            //            yield return new KeyValuePair<char, byte>('', 0xE9);
            yield return new KeyValuePair<char, byte>('ј', 0xEA);
            yield return new KeyValuePair<char, byte>('\u033D', 0xEB);

            yield return new KeyValuePair<char, byte>('¢', 0xEC);
            // Variants
            yield return new KeyValuePair<char, byte>('\u023B', 0xEC);
            yield return new KeyValuePair<char, byte>('￠', 0xEC);

            //            yield return new KeyValuePair<char, byte>('', 0xED);
            yield return new KeyValuePair<char, byte>('ñ', 0xEE);
            yield return new KeyValuePair<char, byte>('ö', 0xEF);

            yield return new KeyValuePair<char, byte>('ρ', 0xF0);
            //            yield return new KeyValuePair<char, byte>('', 0xF1);
            yield return new KeyValuePair<char, byte>('θ', 0xF2);
            yield return new KeyValuePair<char, byte>('∞', 0xF3);
            yield return new KeyValuePair<char, byte>('Ω', 0xF4);
            yield return new KeyValuePair<char, byte>('ü', 0xF5);
            yield return new KeyValuePair<char, byte>('Σ', 0xF6);
            yield return new KeyValuePair<char, byte>('π', 0xF7);
            //            yield return new KeyValuePair<char, byte>('', 0xF8);

            yield return new KeyValuePair<char, byte>('У', 0xF9);
            // Variant
            yield return new KeyValuePair<char, byte>('у', 0xF9);

            //            yield return new KeyValuePair<char, byte>('', 0xFA);
            //            yield return new KeyValuePair<char, byte>('', 0xFB);
            //            yield return new KeyValuePair<char, byte>('', 0xFC);
            yield return new KeyValuePair<char, byte>('÷', 0xFD);
            yield return new KeyValuePair<char, byte>(' ', 0xFE);
            yield return new KeyValuePair<char, byte>('█', 0xFF);
        }

        #endregion
    }

    internal class Hd44780Configuration : IDisposable
    {
        private readonly IDisposable driver;

        public Hd44780Configuration(IDisposable driver = null)
        {
            this.driver = driver;
        }

        public void Dispose()
        {
            if (driver != null)
                driver.Dispose();
        }

        public Hd44780Pins Pins;

        public static Hd44780Configuration LoadGpioConfiguration()
        {
            const ConnectorPin registerSelectPin = ConnectorPin.P1Pin11; // ConnectorPin.P1Pin22;
            const ConnectorPin clockPin = ConnectorPin.P1Pin12; // ConnectorPin.P1Pin18;
            var dataPins = new[]
            {
                ConnectorPin.P1Pin15,
                ConnectorPin.P1Pin16,
                ConnectorPin.P1Pin18,
                ConnectorPin.P1Pin22
            };

            //Console.WriteLine();
            //Console.WriteLine("Using GPIO connection");
            //Console.WriteLine("\tRegister Select: {0}", registerSelectPin);
            //Console.WriteLine("\tClock: {0}", clockPin);
            //Console.WriteLine("\tData 1: {0}", dataPins[0]);
            //Console.WriteLine("\tData 2: {0}", dataPins[1]);
            //Console.WriteLine("\tData 3: {0}", dataPins[2]);
            //Console.WriteLine("\tData 4: {0}", dataPins[3]);
            //Console.WriteLine("\tBacklight: VCC");
            //Console.WriteLine("\tRead/write: GND");
            //Console.WriteLine();

            var driver = GpioConnectionSettings.DefaultDriver;
            return new Hd44780Configuration
            {
                Pins = new Hd44780Pins(
                    driver.Out(registerSelectPin),
                    driver.Out(clockPin),
                    dataPins.Select(p => (IOutputBinaryPin)driver.Out(p))
                    )

            };
        }

    }

    public class Hd44780Pins : IDisposable
    {
        #region Instance Management

        /// <summary>
        /// Initializes a new instance of the <see cref="Hd44780Pins"/> class.
        /// </summary>
        /// <param name="registerSelect">The register select.</param>
        /// <param name="clock">The clock.</param>
        /// <param name="data">The data.</param>
        public Hd44780Pins(IOutputBinaryPin registerSelect, IOutputBinaryPin clock, IEnumerable<IOutputBinaryPin> data)
            : this(registerSelect, clock, data.ToArray())
        {
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Hd44780Pins"/> class.
        /// </summary>
        /// <param name="registerSelect">The register select.</param>
        /// <param name="clock">The clock.</param>
        /// <param name="data">The data.</param>
        public Hd44780Pins(IOutputBinaryPin registerSelect, IOutputBinaryPin clock, params IOutputBinaryPin[] data)
        {
            RegisterSelect = registerSelect;
            Clock = clock;
            Data = data;
        }

        /// <summary>
        /// Performs application-defined tasks associated with freeing, releasing, or resetting unmanaged resources.
        /// </summary>
        void IDisposable.Dispose()
        {
            Close();
        }

        #endregion

        #region Properties

        /// <summary>
        /// The register select (RS) pin.
        /// </summary>
        public IOutputBinaryPin RegisterSelect { get; private set; }

        /// <summary>
        /// The clock (EN) pin.
        /// </summary>
        public IOutputBinaryPin Clock { get; private set; }

        /// <summary>
        /// The backlight pin.
        /// </summary>
        public IOutputBinaryPin Backlight;

        /// <summary>
        /// The read write (RW) pin.
        /// </summary>
        public IOutputBinaryPin ReadWrite;

        /// <summary>
        /// The data pins.
        /// </summary>
        public IOutputBinaryPin[] Data { get; private set; }

        #endregion

        #region Methods

        /// <summary>
        /// Closes this instance.
        /// </summary>
        public void Close()
        {

            RegisterSelect.Dispose();
            Clock.Dispose();

            if (Backlight != null)
                Backlight.Dispose();
            if (ReadWrite != null)
                ReadWrite.Dispose();

            foreach (var dataPin in Data)
                dataPin.Dispose();
        }

        #endregion

    }



    public class Hd44780LcdConnection : IDisposable
    {
        #region Fields

        private readonly Hd44780Pins pins;

        private readonly int width;
        private readonly int height;

        private readonly Functions functions;
        private readonly Encoding encoding;
        private readonly EntryModeFlags entryModeFlags;

        private DisplayFlags displayFlags = DisplayFlags.DisplayOn | DisplayFlags.BlinkOff | DisplayFlags.CursorOff;
        private int currentRow;
        private int currentColumn;

        private bool backlightEnabled;

        #endregion

        #region Instance Management

        /// <summary>
        /// Initializes a new instance of the <see cref="Hd44780LcdConnection" /> class.
        /// </summary>
        /// <param name="registerSelectPin">The register select pin.</param>
        /// <param name="clockPin">The clock pin.</param>
        /// <param name="dataPins">The data pins.</param>
        public Hd44780LcdConnection(IOutputBinaryPin registerSelectPin, IOutputBinaryPin clockPin, params IOutputBinaryPin[] dataPins) : this(null, new Hd44780Pins(registerSelectPin, clockPin, dataPins)) { }

        /// <summary>
        /// Initializes a new instance of the <see cref="Hd44780LcdConnection" /> class.
        /// </summary>
        /// <param name="registerSelectPin">The register select pin.</param>
        /// <param name="clockPin">The clock pin.</param>
        /// <param name="dataPins">The data pins.</param>
        public Hd44780LcdConnection(IOutputBinaryPin registerSelectPin, IOutputBinaryPin clockPin, IEnumerable<IOutputBinaryPin> dataPins) : this(null, new Hd44780Pins(registerSelectPin, clockPin, dataPins)) { }

        /// <summary>
        /// Initializes a new instance of the <see cref="Hd44780LcdConnection" /> class.
        /// </summary>
        /// <param name="settings">The settings.</param>
        /// <param name="pins">The pins.</param>
        /// <exception cref="System.ArgumentOutOfRangeException">
        /// dataPins;There must be either 4 or 8 data pins
        /// or
        /// settings;ScreenHeight must be either 1 or 2 rows
        /// or
        /// settings;PatternWidth must be 5 pixels
        /// or
        /// settings;PatternWidth must be either 7 or 10 pixels height
        /// </exception>
        /// <exception cref="System.ArgumentException">
        /// At most 80 characters are allowed
        /// or
        /// 10 pixels height pattern cannot be used with 2 rows
        /// </exception>
        public Hd44780LcdConnection(Hd44780LcdConnectionSettings settings, Hd44780Pins pins)
        {
            settings = settings ?? new Hd44780LcdConnectionSettings();
            this.pins = pins;

            if (pins.Data.Length != 4 && pins.Data.Length != 8)
                throw new ArgumentOutOfRangeException("pins", pins.Data.Length, "There must be either 4 or 8 data pins");

            width = settings.ScreenWidth;
            height = settings.ScreenHeight;
            if (height < 1 || height > 2)
                throw new ArgumentOutOfRangeException("settings", height, "ScreenHeight must be either 1 or 2 rows");
            if (width * height > 80)
                throw new ArgumentException("At most 80 characters are allowed");

            if (settings.PatternWidth != 5)
                throw new ArgumentOutOfRangeException("settings", settings.PatternWidth, "PatternWidth must be 5 pixels");
            if (settings.PatternHeight != 8 && settings.PatternHeight != 10)
                throw new ArgumentOutOfRangeException("settings", settings.PatternWidth, "PatternWidth must be either 7 or 10 pixels height");
            if (settings.PatternHeight == 10 && height == 2)
                throw new ArgumentException("10 pixels height pattern cannot be used with 2 rows");

            functions = (settings.PatternHeight == 8 ? Functions.Matrix5x8 : Functions.Matrix5x10)
                | (height == 1 ? Functions.OneLine : Functions.TwoLines)
                | (pins.Data.Length == 4 ? Functions.Data4bits : Functions.Data8bits);

            entryModeFlags = /*settings.RightToLeft 
                ? EntryModeFlags.EntryRight | EntryModeFlags.EntryShiftDecrement
                :*/ EntryModeFlags.EntryLeft | EntryModeFlags.EntryShiftDecrement;

            encoding = settings.Encoding;

            BacklightEnabled = false;

            if (pins.ReadWrite != null)
                pins.ReadWrite.Write(false);

            pins.RegisterSelect.Write(false);
            pins.Clock.Write(false);
            foreach (var dataPin in pins.Data)
                dataPin.Write(false);

            WriteByte(0x33, false); // Initialize
            WriteByte(0x32, false);

            WriteCommand(Command.SetFunctions, (int)functions);
            WriteCommand(Command.SetDisplayFlags, (int)displayFlags);
            WriteCommand(Command.SetEntryModeFlags, (int)entryModeFlags);

            Clear();
            BacklightEnabled = true;
        }

        void IDisposable.Dispose()
        {
            Close();
        }

        #endregion

        #region Properties

        /// <summary>
        /// Gets or sets a value indicating whether display is enabled.
        /// </summary>
        /// <value>
        ///   <c>true</c> if display is enabled; otherwise, <c>false</c>.
        /// </value>
        public bool DisplayEnabled
        {
            get { return (displayFlags & DisplayFlags.DisplayOn) == DisplayFlags.DisplayOn; }
            set
            {
                if (value)
                    displayFlags |= DisplayFlags.DisplayOn;
                else
                    displayFlags &= ~DisplayFlags.DisplayOn;

                WriteCommand(Command.SetDisplayFlags, (int)displayFlags);
            }
        }

        /// <summary>
        /// Gets or sets a value indicating whether backlight is enabled.
        /// </summary>
        /// <value>
        ///   <c>true</c> if backlight is enabled; otherwise, <c>false</c>.
        /// </value>
        public bool BacklightEnabled
        {
            get { return backlightEnabled; }
            set
            {
                if (pins.Backlight == null)
                    return;

                pins.Backlight.Write(value);
                backlightEnabled = value;
            }
        }

        /// <summary>
        /// Gets or sets a value indicating whether cursor is enabled.
        /// </summary>
        /// <value>
        ///   <c>true</c> if cursor is enabled; otherwise, <c>false</c>.
        /// </value>
        public bool CursorEnabled
        {
            get { return (displayFlags & DisplayFlags.CursorOn) == DisplayFlags.CursorOn; }
            set
            {
                if (value)
                    displayFlags |= DisplayFlags.CursorOn;
                else
                    displayFlags &= ~DisplayFlags.CursorOn;

                WriteCommand(Command.SetDisplayFlags, (int)displayFlags);
            }
        }

        /// <summary>
        /// Gets or sets a value indicating whether cursor is blinking.
        /// </summary>
        /// <value>
        ///   <c>true</c> if cursor is blinking; otherwise, <c>false</c>.
        /// </value>
        public bool CursorBlinking
        {
            get { return (displayFlags & DisplayFlags.BlinkOn) == DisplayFlags.BlinkOn; }
            set
            {
                if (value)
                    displayFlags |= DisplayFlags.BlinkOn;
                else
                    displayFlags &= ~DisplayFlags.BlinkOn;

                WriteCommand(Command.SetDisplayFlags, (int)displayFlags);
            }
        }

        #endregion

        #region Methods

        /// <summary>
        /// Closes this instance.
        /// </summary>
        public void Close()
        {
            Clear();
            pins.Close();
        }

        /// <summary>
        /// Set cursor to top left corner.
        /// </summary>
        public void Home()
        {
            WriteCommand(Command.ReturnHome);
            currentRow = 0;
            currentColumn = 0;

            Sleep(3);
        }

        /// <summary>
        /// Clears the display.
        /// </summary>
        public void Clear()
        {
            WriteCommand(Command.ClearDisplay);
            currentRow = 0;
            currentColumn = 0;

            Sleep(3); // Clearing the display takes a long time
        }

        /// <summary>
        /// Moves the cursor of the specified offset.
        /// </summary>
        /// <param name="offset">The offset.</param>
        public void Move(int offset)
        {
            var count = offset > 0 ? offset : -offset;
            for (var i = 0; i < count; i++)
                WriteCommand(Command.MoveCursor, (int)(CursorShiftFlags.DisplayMove | (offset < 0 ? CursorShiftFlags.MoveLeft : CursorShiftFlags.MoveRight)));
        }

        /// <summary>
        /// Sets the custom character.
        /// </summary>
        /// <param name="character">The character.</param>
        /// <param name="pattern">The pattern.</param>
        public void SetCustomCharacter(byte character, byte[] pattern)
        {
            if ((functions & Functions.Matrix5x8) == Functions.Matrix5x8)
                Set5x8CustomCharacter(character, pattern);
            else
                Set5x10CustomCharacter(character, pattern);
        }

        /// <summary>
        /// Writes the line.
        /// </summary>
        /// <param name="value">The value.</param>
        /// <param name="animationDelay">The animation delay.</param>
        public void WriteLine(object value, decimal animationDelay = 0m)
        {
            WriteLine("{0}", value, animationDelay);
        }

        /// <summary>
        /// Writes the line.
        /// </summary>
        /// <param name="text">The text.</param>
        /// <param name="animationDelay">The animation delay.</param>
        public void WriteLine(string text, decimal animationDelay = 0m)
        {
            Write(text + Environment.NewLine, animationDelay);
        }

        /// <summary>
        /// Writes the specified value.
        /// </summary>
        /// <param name="value">The value.</param>
        /// <param name="animationDelay">The animation delay.</param>
        public void Write(object value, decimal animationDelay = 0m)
        {
            Write("{0}", value, animationDelay);
        }

        /// <summary>
        /// Writes the line.
        /// </summary>
        /// <param name="format">The format.</param>
        /// <param name="values">The values.</param>
        public void WriteLine(string format, params object[] values)
        {
            WriteLine(string.Format(format, values));
        }

        /// <summary>
        /// Writes the specified format.
        /// </summary>
        /// <param name="format">The format.</param>
        /// <param name="values">The values.</param>
        public void Write(string format, params object[] values)
        {
            Write(string.Format(format, values));
        }

        /// <summary>
        /// Writes the line.
        /// </summary>
        /// <param name="format">The format.</param>
        /// <param name="animationDelay">The animation delay.</param>
        /// <param name="values">The values.</param>
        public void WriteLine(string format, decimal animationDelay, params object[] values)
        {
            WriteLine(string.Format(format, values), animationDelay);
        }

        /// <summary>
        /// Writes the specified format.
        /// </summary>
        /// <param name="format">The format.</param>
        /// <param name="animationDelay">The animation delay.</param>
        /// <param name="values">The values.</param>
        public void Write(string format, decimal animationDelay, params object[] values)
        {
            Write(string.Format(format, values), animationDelay);
        }

        /// <summary>
        /// Writes the specified text.
        /// </summary>
        /// <param name="text">The text.</param>
        /// <param name="animationDelay">The animation delay.</param>
        public void Write(string text, decimal animationDelay = 0m)
        {
            var lines = text.Split(new[] { Environment.NewLine }, StringSplitOptions.None);

            foreach (var line in lines)
            {
                if (string.IsNullOrEmpty(line))
                    continue;

               // Console.WriteLine(line);
                var bytes = encoding.GetBytes(line);
                foreach (var b in bytes)
                {
                    if (currentColumn < width)
                        WriteByte(b, true);

                    if (animationDelay > 0m)
                    {
                        //Timer.Sleep(animationDelay);
                        System.Threading.Thread.Sleep((int)animationDelay);
                    }

                    currentColumn++;
                }

                if (currentRow == 0 && height > 1)
                {
                    WriteByte(0xC0, false);
                    currentColumn = 0;
                    currentRow++;
                }
                else
                    break;
            }
        }

        #endregion

        #region Private Helpers

        private void Sleep(decimal delay)
        {
            //Timer.Sleep(delay);
            System.Threading.Thread.Sleep((int)delay);
        }

        private void WriteCommand(Command command, int parameter = 0)
        {
            var bits = (int)command | parameter;
            WriteByte(bits, false);
        }

        private void Set5x10CustomCharacter(byte character, byte[] pattern)
        {
            if (character > 7 || (character & 0x1) != 0x1)
                throw new ArgumentOutOfRangeException("character", character, "character must be lower or equal to 7, and not an odd number");
            if (pattern.Length != 10)
                throw new ArgumentOutOfRangeException("pattern", pattern, "pattern must be 10 rows long");

            WriteCommand(Command.SetCGRamAddr, character << 3);
            for (var i = 0; i < 10; i++)
                WriteByte(pattern[i], true);
            WriteByte(0, true);
        }

        private void Set5x8CustomCharacter(byte character, byte[] pattern)
        {
            if (character > 7)
                throw new ArgumentOutOfRangeException("character", character, "character must be lower or equal to 7");
            if (pattern.Length != 7)
                throw new ArgumentOutOfRangeException("pattern", pattern, "pattern must be 7 rows long");

            WriteCommand(Command.SetCGRamAddr, character << 3);
            for (var i = 0; i < 7; i++)
                WriteByte(pattern[i], true);
            WriteByte(0, true);
        }

        private void WriteByte(int bits, bool charMode)
        {
            if (pins.Data.Length == 4)
                WriteByte4Pins(bits, charMode);
            else
                throw new NotImplementedException("8 bits mode is currently not implemented");
        }

        private void WriteByte4Pins(int bits, bool charMode)
        {
            pins.RegisterSelect.Write(charMode);

            pins.Data[0].Write((bits & 0x10) != 0);
            pins.Data[1].Write((bits & 0x20) != 0);
            pins.Data[2].Write((bits & 0x40) != 0);
            pins.Data[3].Write((bits & 0x80) != 0);

            Synchronize();

            pins.Data[0].Write((bits & 0x01) != 0);
            pins.Data[1].Write((bits & 0x02) != 0);
            pins.Data[2].Write((bits & 0x04) != 0);
            pins.Data[3].Write((bits & 0x08) != 0);

            Synchronize();
        }

        private void Synchronize()
        {
            pins.Clock.Write(true);
            Sleep(0.001m); // 1 microsecond pause - enable pulse must be > 450ns 	

            pins.Clock.Write(false);
            Sleep(0.001m); // commands need > 37us to settle
        }

        #endregion
    }

    public static class GpioBinaryPinExtensionMethods
    {
        public static GpioOutputBinaryPin Out(this IGpioConnectionDriver driver, ConnectorPin pin)
        {
            return driver.Out(pin.ToProcessor());
        }

        public static GpioOutputBinaryPin Out(this IGpioConnectionDriver driver, ProcessorPin pin)
        {
            return new GpioOutputBinaryPin(driver, pin);
        }

    }

    public class GpioOutputBinaryPin : IOutputBinaryPin
    {


        private readonly IGpioConnectionDriver driver;
        private readonly ProcessorPin pin;




        /// <summary>
        /// Initializes a new instance of the <see cref="GpioOutputBinaryPin"/> class.
        /// </summary>
        /// <param name="driver">The driver.</param>
        /// <param name="pin">The pin.</param>
        /// <param name="resistor">The resistor.</param>
        public GpioOutputBinaryPin(IGpioConnectionDriver driver, ProcessorPin pin, PinResistor resistor = PinResistor.None)
        {
            this.driver = driver;
            this.pin = pin;

            driver.Allocate(pin, PinDirection.Output);
            driver.SetPinResistor(pin, resistor);
        }



        /// <summary>
        /// Performs application-defined tasks associated with freeing, releasing, or resetting unmanaged resources.
        /// </summary>
        public void Dispose()
        {
            driver.Release(pin);
        }

        /// <summary>
        /// Writes the specified state.
        /// </summary>
        /// <param name="state">The pin state.</param>
        public void Write(bool state)
        {
            driver.Write(pin, state);
        }


    }

    #endregion


    //#region Board Stuff

    // public class Board
    //{
    //    #region Fields

    //    private static readonly Lazy<Board> board = new Lazy<Board>(LoadBoard);

    //    private readonly Dictionary<string, string> settings;
    //    private readonly Lazy<Model> model;
    //    private readonly Lazy<ConnectorPinout> connectorPinout;

    //    #endregion

    //    #region Instance Management

    //    private Board(Dictionary<string, string> settings)
    //    {
    //        model = new Lazy<Model>(LoadModel);
    //        connectorPinout = new Lazy<ConnectorPinout>(LoadConnectorPinout);

    //        this.settings = settings;
    //    }

    //    #endregion

    //    #region Properties

    //    /// <summary>
    //    /// Gets the current mainboard configuration.
    //    /// </summary>
    //    public static Board Current
    //    {
    //        get { return board.Value; }
    //    }

    //    /// <summary>
    //    /// Gets a value indicating whether this instance is a Raspberry Pi.
    //    /// </summary>
    //    /// <value>
    //    /// 	<c>true</c> if this instance is a Raspberry Pi; otherwise, <c>false</c>.
    //    /// </value>
    //    public bool IsRaspberryPi
    //    {
    //        get
    //        {
    //            return Processor != Processor.Unknown;
    //        }
    //    }

    //    /// <summary>
    //    /// Gets the processor name.
    //    /// </summary>
    //    /// <value>
    //    /// The name of the processor.
    //    /// </value>
    //    public string ProcessorName
    //    {
    //        get
    //        {
    //            string hardware;
    //            return settings.TryGetValue("Hardware", out hardware) ? hardware : null;
    //        }
    //    }

    //    /// <summary>
    //    /// Gets the processor.
    //    /// </summary>
    //    /// <value>
    //    /// The processor.
    //    /// </value>
    //    public Processor Processor
    //    {
    //        get
    //        {
    //            Processor processor;
    //            return Enum.TryParse(ProcessorName, true, out processor) ? processor : Processor.Unknown;
    //        }
    //    }

    //    /// <summary>
    //    /// Gets the board firmware version.
    //    /// </summary>
    //    public int Firmware
    //    {
    //        get
    //        {
    //            string revision;
    //            int firmware;
    //            if (settings.TryGetValue("Revision", out revision) 
    //                && !string.IsNullOrEmpty(revision) 
    //                && int.TryParse(revision, NumberStyles.HexNumber, CultureInfo.InvariantCulture, out firmware))
    //                return firmware;

    //            return 0;
    //        }
    //    }

    //    /// <summary>
    //    /// Gets the serial number.
    //    /// </summary>
    //    public string SerialNumber
    //    {
    //        get { 
    //            string serial;
    //            if (settings.TryGetValue("Serial", out serial) 
    //                && !string.IsNullOrEmpty(serial))
    //                return serial;

    //            return null;
    //        }
    //    }

    //    /// <summary>
    //    /// Gets a value indicating whether Raspberry Pi board is overclocked.
    //    /// </summary>
    //    /// <value>
    //    ///   <c>true</c> if Raspberry Pi is overclocked; otherwise, <c>false</c>.
    //    /// </value>
    //    public bool IsOverclocked
    //    {
    //        get
    //        {
    //            var firmware = Firmware;
    //            return (firmware & 0xFFFF0000) != 0;
    //        }
    //    }

    //    /// <summary>
    //    /// Gets the model.
    //    /// </summary>
    //    /// <value>
    //    /// The model.
    //    /// </value>
    //    public Model Model
    //    {
    //        get { return model.Value; }
    //    }

    //    /// <summary>
    //    /// Gets the connector revision.
    //    /// </summary>
    //    /// <value>
    //    /// The connector revision.
    //    /// </value>
    //    /// <remarks>See <see cref="http://raspi.tv/2014/rpi-gpio-quick-reference-updated-for-raspberry-pi-b"/> for more information.</remarks>
    //    public ConnectorPinout ConnectorPinout
    //    {
    //        get { return connectorPinout.Value; }
    //    }

    //    #endregion

    //    #region Private Helpers

    //    private static Board LoadBoard()
    //    {
    //        try
    //        {
    //            const string filePath = "/proc/cpuinfo";

    //            var cpuInfo = File.ReadAllLines(filePath);
    //            var settings = new Dictionary<string, string>();
    //            var suffix = string.Empty;

    //            foreach(var l in cpuInfo)
    //            {
    //                var separator = l.IndexOf(':');

    //                if (!string.IsNullOrWhiteSpace(l) && separator > 0)
    //                {
    //                    var key = l.Substring(0, separator).Trim();
    //                    var val = l.Substring(separator + 1).Trim();
    //                    if (string.Equals(key, "processor", StringComparison.InvariantCultureIgnoreCase))
    //                        suffix = "." + val;

    //                    settings.Add(key + suffix, val);
    //                }
    //                else
    //                    suffix = "";
    //            }

    //            return new Board(settings);
    //        }
    //        catch
    //        {
    //            return new Board(new Dictionary<string, string>());
    //        }
    //    }

    //    private Model LoadModel()
    //    {
    //        var firmware = Firmware;
    //        switch (firmware & 0xFFFF)
    //        {
    //            case 0x2:
    //            case 0x3:
    //                return Model.BRev1;

    //            case 0x4:
    //            case 0x5:
    //            case 0x6:
    //            case 0xd:
    //            case 0xe:
    //            case 0xf:
    //                return Model.BRev2;

    //            case 0x7:
    //            case 0x8:
    //            case 0x9:
    //                return Model.A;

    //            case 0x10:
    //                return Model.BPlus;

    //            case 0x11:
    //                return Model.ComputeModule;

    //            case 0x12:
    //                return Model.APlus;

    //            case 0x1040:
    //            case 0x1041:
    //                return Model.B2;

    //            default:
    //                return Model.Unknown;
    //        }
    //    }

    //    private ConnectorPinout LoadConnectorPinout()
    //    {
    //        switch (Model)
    //        {
    //            case Model.BRev1:
    //                return ConnectorPinout.Rev1;

    //            case Model.BRev2:
    //            case Model.A:
    //                return ConnectorPinout.Rev2;

    //            case Model.BPlus:
    //            case Model.ComputeModule:
    //            case Model.APlus:
    //            case Model.B2:
    //                return ConnectorPinout.Plus;

    //            default:
    //                return ConnectorPinout.Unknown;
    //        }
    //    }

    //    #endregion
    //}

    // public enum Model
    // {
    //     /// <summary>
    //     /// Unknown model.
    //     /// </summary>
    //     Unknown,

    //     /// <summary>
    //     /// Model A.
    //     /// </summary>
    //     A,

    //     /// <summary>
    //     /// Model A+.
    //     /// </summary>
    //     APlus,

    //     /// <summary>
    //     /// Model B rev1.
    //     /// </summary>
    //     BRev1,

    //     /// <summary>
    //     /// Model B rev2.
    //     /// </summary>
    //     BRev2,

    //     /// <summary>
    //     /// Model B+.
    //     /// </summary>
    //     BPlus,

    //     /// <summary>
    //     /// Compute module.
    //     /// </summary>
    //     ComputeModule,

    //     /// <summary>
    //     /// Pi 2 Model B.
    //     /// </summary>
    //     B2
    // }
    // public enum ConnectorPinout
    // {
    //     /// <summary>
    //     /// Connector pinout is unknown.
    //     /// </summary>
    //     Unknown,

    //     /// <summary>
    //     /// The first revision, as of Model B rev1.
    //     /// </summary>
    //     Rev1,

    //     /// <summary>
    //     /// The second revision, as of Model B rev2.
    //     /// </summary>
    //     Rev2,

    //     /// <summary>
    //     /// The third revision, as of Model B+.
    //     /// </summary>
    //     Plus,
    // }
    // public static class ModelExtensionMethods
    // {
    //     /// <summary>
    //     /// Gets the model display name.
    //     /// </summary>
    //     /// <param name="model">The model.</param>
    //     /// <returns>The display name, if known; otherwise, <c>null</c>.</returns>
    //     public static string GetDisplayName(this Model model)
    //     {
    //         switch (model)
    //         {
    //             case Model.Unknown:
    //                 return null;
    //             case Model.A:
    //                 return "Raspberry Pi Model A";
    //             case Model.APlus:
    //                 return "Raspberry Pi Model A+";
    //             case Model.BRev1:
    //                 return "Raspberry Pi Model B rev1";
    //             case Model.BRev2:
    //                 return "Raspberry Pi Model B rev2";
    //             case Model.BPlus:
    //                 return "Raspberry Pi Model B+";
    //             case Model.ComputeModule:
    //                 return "Raspberry Pi Compute Module";
    //             case Model.B2:
    //                 return "Raspberry Pi 2 Model B";

    //             default:
    //                 throw new ArgumentOutOfRangeException("model");
    //         }
    //     }
    // }
    // public enum Processor
    // {
    //     /// <summary>
    //     /// Processor is unknown.
    //     /// </summary>
    //     Unknown,

    //     /// <summary>
    //     /// Processor is a BCM2708.
    //     /// </summary>
    //     Bcm2708,

    //     /// <summary>
    //     /// Processor is a BCM2709.
    //     /// </summary>
    //     Bcm2709
    // }
    //#endregion


    #region GPIO General Stuff

    // TODO: this is similar to other pin enum - refactor

    public enum PinDirection
    {
        /// <summary>
        /// Pin is an input pin.
        /// </summary>
        Input,

        /// <summary>
        /// Pin is an output pin.
        /// </summary>
        Output
    }
    public enum PinResistor
    {
        /// <summary>
        /// No resistor is enabled on the input.
        /// </summary>
        None,

        /// <summary>
        /// A pull-down resistor is enabled.
        /// </summary>
        PullDown,

        /// <summary>
        /// A pull-up resistor is enabled.
        /// </summary>
        PullUp
    }
    [Flags]
    public enum PinDetectedEdges
    {
        /// <summary>
        /// No changes are detected.
        /// </summary>
        None = 0,

        /// <summary>
        /// Rising edge changes are detected.
        /// </summary>
        Rising = 1,

        /// <summary>
        /// Falling edge changes are detected.
        /// </summary>
        Falling = 2,

        /// <summary>
        /// Both changes are detected.
        /// </summary>
        Both = Rising | Falling
    }
    [Flags]
    public enum ProcessorPins : uint
    {
        /// <summary>
        /// No pins selected.
        /// </summary>
        None = 0,

        /// <summary>
        /// Pin 0 selected.
        /// </summary>
        Pin0 = 1 << 0,

        /// <summary>
        /// Pin 0 selected.
        /// </summary>
        Pin00 = Pin0,

        /// <summary>
        /// Pin 1 selected.
        /// </summary>
        Pin1 = 1 << 1,

        /// <summary>
        /// Pin 1 selected.
        /// </summary>
        Pin01 = Pin1,

        /// <summary>
        /// Pin 2 selected.
        /// </summary>
        Pin2 = 1 << 2,

        /// <summary>
        /// Pin 2 selected.
        /// </summary>
        Pin02 = Pin2,

        /// <summary>
        /// Pin 3 selected.
        /// </summary>
        Pin3 = 1 << 3,

        /// <summary>
        /// Pin 3 selected.
        /// </summary>
        Pin03 = Pin3,

        /// <summary>
        /// Pin 4 selected.
        /// </summary>
        Pin4 = 1 << 4,

        /// <summary>
        /// Pin 4 selected.
        /// </summary>
        Pin04 = Pin4,

        /// <summary>
        /// Pin 7 selected.
        /// </summary>
        Pin7 = 1 << 7,

        /// <summary>
        /// Pin 7 selected.
        /// </summary>
        Pin07 = Pin7,

        /// <summary>
        /// Pin 8 selected.
        /// </summary>
        Pin8 = 1 << 8,

        /// <summary>
        /// Pin 8 selected.
        /// </summary>
        Pin08 = Pin8,

        /// <summary>
        /// Pin 9 selected.
        /// </summary>
        Pin9 = 1 << 9,

        /// <summary>
        /// Pin 9 selected.
        /// </summary>
        Pin09 = Pin9,

        /// <summary>
        /// Pin 10 selected.
        /// </summary>
        Pin10 = 1 << 10,

        /// <summary>
        /// Pin 11 selected.
        /// </summary>
        Pin11 = 1 << 11,

        /// <summary>
        /// Pin 14 selected.
        /// </summary>
        Pin14 = 1 << 14,

        /// <summary>
        /// Pin 15 selected.
        /// </summary>
        Pin15 = 1 << 15,

        /// <summary>
        /// Pin 17 selected.
        /// </summary>
        Pin17 = 1 << 17,

        /// <summary>
        /// Pin 18 selected.
        /// </summary>
        Pin18 = 1 << 18,

        /// <summary>
        /// Pin 21 selected.
        /// </summary>
        Pin21 = 1 << 21,

        /// <summary>
        /// Pin 22 selected.
        /// </summary>
        Pin22 = 1 << 22,

        /// <summary>
        /// Pin 23 selected.
        /// </summary>
        Pin23 = 1 << 23,

        /// <summary>
        /// Pin 24 selected.
        /// </summary>
        Pin24 = 1 << 24,

        /// <summary>
        /// Pin 25 selected.
        /// </summary>
        Pin25 = 1 << 25,

        /// <summary>
        /// Pin 27 selected.
        /// </summary>
        Pin27 = 1 << 27,

        /// <summary>
        /// Pin 28 selected.
        /// </summary>
        Pin28 = 1 << 28,

        /// <summary>
        /// Pin 29 selected.
        /// </summary>
        Pin29 = 1 << 29,

        /// <summary>
        /// Pin 30 selected.
        /// </summary>
        Pin30 = 1 << 30,

        /// <summary>
        /// Pin 31 selected.
        /// </summary>
        Pin31 = (uint)1 << 31
    }


    public interface IOutputBinaryPin : IDisposable
    {
        void Write(bool state);

    }

    public interface IGpioConnectionDriver
    {


        /// <summary>
        /// Allocates the specified pin.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <param name="direction">The direction.</param>
        void Allocate(ProcessorPin pin, PinDirection direction);

        /// <summary>
        /// Sets the pin resistor.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <param name="resistor">The resistor.</param>
        void SetPinResistor(ProcessorPin pin, PinResistor resistor);

        /// <summary>
        /// Sets the detected edges on an input pin.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <param name="edges">The edges.</param>
        /// <remarks>By default, both edges may be detected on input pins.</remarks>
        void SetPinDetectedEdges(ProcessorPin pin, PinDetectedEdges edges);

        /// <summary>
        /// Waits for the specified pin to be in the specified state.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <param name="waitForUp">if set to <c>true</c> waits for the pin to be up.</param>
        /// <param name="timeout">The timeout, in milliseconds.</param>
        void Wait(ProcessorPin pin, bool waitForUp = true, decimal timeout = 0);

        /// <summary>
        /// Releases the specified pin.
        /// </summary>
        /// <param name="pin">The pin.</param>
        void Release(ProcessorPin pin);

        /// <summary>
        /// Modified the status of a pin.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <param name="value">The pin status.</param>
        void Write(ProcessorPin pin, bool value);

        /// <summary>
        /// Reads the status of the specified pin.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <returns>The pin status.</returns>
        bool Read(ProcessorPin pin);

        /// <summary>
        /// Reads the status of the specified pins.
        /// </summary>
        /// <param name="pins">The pins.</param>
        /// <returns>The pins status.</returns>
        ProcessorPins Read(ProcessorPins pins);


    }

    public class GpioConnectionDriver : IGpioConnectionDriver
    {
        #region Fields

        private readonly IntPtr gpioAddress;
        private const string gpioPath = "/sys/class/gpio";

        private readonly Dictionary<ProcessorPin, PinPoll> polls = new Dictionary<ProcessorPin, PinPoll>();

        #endregion

        #region Instance Management

        /// <summary>
        /// Initializes a new instance of the <see cref="MemoryGpioConnectionDriver"/> class.
        /// </summary>
        public GpioConnectionDriver()
        {

            using (var memoryFile = UnixFile.Open("/dev/mem", UnixFileMode.ReadWrite | UnixFileMode.Synchronized))
            {
                gpioAddress = MemoryMap.Create(
                    IntPtr.Zero,
                    Interop.BCM2835_BLOCK_SIZE,
                    MemoryProtection.ReadWrite,
                    MemoryFlags.Shared,
                    memoryFile.Descriptor,
                    Interop.BCM2835_GPIO_BASE
                );
            }
        }

        /// <summary>
        /// Releases unmanaged resources and performs other cleanup operations before the
        /// <see cref="MemoryGpioConnectionDriver"/> is reclaimed by garbage collection.
        /// </summary>
        ~GpioConnectionDriver()
        {
            MemoryMap.Close(gpioAddress, Interop.BCM2835_BLOCK_SIZE);
        }

        #endregion

        #region Methods

        /// <summary>
        /// Allocates the specified pin.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <param name="direction">The direction.</param>
        public void Allocate(ProcessorPin pin, PinDirection direction)
        {
            var gpioId = string.Format("gpio{0}", (int)pin);
            if (Directory.Exists(Path.Combine(gpioPath, gpioId)))
            {
                // Reinitialize pin virtual file
                using (var streamWriter = new StreamWriter(Path.Combine(gpioPath, "unexport"), false))
                    streamWriter.Write((int)pin);
            }

            // Export pin for file mode
            using (var streamWriter = new StreamWriter(Path.Combine(gpioPath, "export"), false))
                streamWriter.Write((int)pin);

            // Set the direction on the pin and update the exported list
            SetPinMode(pin, direction == PinDirection.Input ? Interop.BCM2835_GPIO_FSEL_INPT : Interop.BCM2835_GPIO_FSEL_OUTP);

            // Set direction in pin virtual file
            var filePath = Path.Combine(gpioId, "direction");
            using (var streamWriter = new StreamWriter(Path.Combine(gpioPath, filePath), false))
                streamWriter.Write(direction == PinDirection.Input ? "in" : "out");

            if (direction == PinDirection.Input)
            {
                SetPinResistor(pin, PinResistor.None);
                SetPinDetectedEdges(pin, PinDetectedEdges.Both);
                InitializePoll(pin);
            }
        }

        /// <summary>
        /// Sets the pin resistor.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <param name="resistor">The resistor.</param>
        public void SetPinResistor(ProcessorPin pin, PinResistor resistor)
        {
            // Set the pullup/down resistor for a pin
            //
            // The GPIO Pull-up/down Clock Registers control the actuation of internal pull-downs on
            // the respective GPIO pins. These registers must be used in conjunction with the GPPUD
            // register to effect GPIO Pull-up/down changes. The following sequence of events is
            // required:
            // 1. Write to GPPUD to set the required control signal (i.e. Pull-up or Pull-Down or neither
            // to remove the current Pull-up/down)
            // 2. Wait 150 cycles ? this provides the required set-up time for the control signal
            // 3. Write to GPPUDCLK0/1 to clock the control signal into the GPIO pads you wish to
            // modify ? NOTE only the pads which receive a clock will be modified, all others will
            // retain their previous state.
            // 4. Wait 150 cycles ? this provides the required hold time for the control signal
            // 5. Write to GPPUD to remove the control signal
            // 6. Write to GPPUDCLK0/1 to remove the clock
            //
            // RPi has P1-03 and P1-05 with 1k8 pullup resistor

            uint pud;
            switch (resistor)
            {
                case PinResistor.None:
                    pud = Interop.BCM2835_GPIO_PUD_OFF;
                    break;
                case PinResistor.PullDown:
                    pud = Interop.BCM2835_GPIO_PUD_DOWN;
                    break;
                case PinResistor.PullUp:
                    pud = Interop.BCM2835_GPIO_PUD_UP;
                    break;

                default:
                    throw new ArgumentOutOfRangeException("resistor", resistor, string.Format(CultureInfo.InvariantCulture, "{0} is not a valid value for pin resistor", resistor));
            }

            WriteResistor(pud);
            //HighResolutionTimer.Sleep(0.005m);
            System.Threading.Thread.Sleep(5);
            SetPinResistorClock(pin, true);
            //HighResolutionTimer.Sleep(0.005m);
            System.Threading.Thread.Sleep(5);
            WriteResistor(Interop.BCM2835_GPIO_PUD_OFF);
            SetPinResistorClock(pin, false);
        }

        /// <summary>
        /// Sets the detected edges on an input pin.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <param name="edges">The edges.</param>
        /// <remarks>By default, both edges may be detected on input pins.</remarks>
        public void SetPinDetectedEdges(ProcessorPin pin, PinDetectedEdges edges)
        {
            var edgePath = Path.Combine(gpioPath, string.Format("gpio{0}/edge", (int)pin));
            using (var streamWriter = new StreamWriter(edgePath, false))
                streamWriter.Write(ToString(edges));
        }

        /// <summary>
        /// Waits for the specified pin to be in the specified state.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <param name="waitForUp">if set to <c>true</c> waits for the pin to be up.</param>
        /// <param name="timeout">The timeout, in milliseconds.</param>
        /// <exception cref="System.TimeoutException"></exception>
        /// <exception cref="System.IO.IOException">epoll_wait failed</exception>
        public void Wait(ProcessorPin pin, bool waitForUp = true, decimal timeout = 0)
        {
            var pinPoll = polls[pin];
            if (Read(pin) == waitForUp)
                return;

            var actualTimeout = GetActualTimeout(timeout);

            while (true)
            {
                // TODO: timeout after the remaining amount of time.
                var waitResult = Interop.epoll_wait(pinPoll.PollDescriptor, pinPoll.OutEventPtr, 1, actualTimeout);
                if (waitResult > 0)
                {
                    if (Read(pin) == waitForUp)
                        break;
                }
                else if (waitResult == 0)
                    throw new TimeoutException(string.Format(CultureInfo.InvariantCulture, "Operation timed out after waiting {0}ms for the pin {1} to be {2}", actualTimeout, pin, (waitForUp ? "up" : "down")));
                else
                    throw new IOException("Call to epoll_wait API failed");
            }
        }

        /// <summary>
        /// Releases the specified pin.
        /// </summary>
        /// <param name="pin">The pin.</param>
        public void Release(ProcessorPin pin)
        {
            UninitializePoll(pin);
            using (var streamWriter = new StreamWriter(Path.Combine(gpioPath, "unexport"), false))
                streamWriter.Write((int)pin);

            SetPinMode(pin, Interop.BCM2835_GPIO_FSEL_INPT);
        }

        /// <summary>
        /// Modified the status of a pin.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <param name="value">The pin status.</param>
        public void Write(ProcessorPin pin, bool value)
        {
            int shift;
            var offset = Math.DivRem((int)pin, 32, out shift);

            var pinGroupAddress = gpioAddress + (int)((value ? Interop.BCM2835_GPSET0 : Interop.BCM2835_GPCLR0) + offset);
            SafeWriteUInt32(pinGroupAddress, (uint)1 << shift);
        }

        /// <summary>
        /// Reads the status of the specified pin.
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <returns>
        /// The pin status.
        /// </returns>
        public bool Read(ProcessorPin pin)
        {
            int shift;
            var offset = Math.DivRem((int)pin, 32, out shift);

            var pinGroupAddress = gpioAddress + (int)(Interop.BCM2835_GPLEV0 + offset);
            var value = SafeReadUInt32(pinGroupAddress);

            return (value & (1 << shift)) != 0;
        }

        /// <summary>
        /// Reads the status of the specified pins.
        /// </summary>
        /// <param name="pins">The pins.</param>
        /// <returns>
        /// The pins status.
        /// </returns>
        public ProcessorPins Read(ProcessorPins pins)
        {
            var pinGroupAddress = gpioAddress + (int)(Interop.BCM2835_GPLEV0 + (uint)0 * 4);
            var value = SafeReadUInt32(pinGroupAddress);

            return (ProcessorPins)((uint)pins & value);
        }

        #endregion

        #region Private Methods

        private static int GetActualTimeout(decimal timeout)
        {
            if (timeout > 0)
                return (int)timeout;

            if (timeout > 0)
                return 1;

            return 5000;
        }

        private void InitializePoll(ProcessorPin pin)
        {
            lock (polls)
            {
                PinPoll poll;
                if (polls.TryGetValue(pin, out poll))
                    return;

                var pinPoll = new PinPoll();

                pinPoll.PollDescriptor = Interop.epoll_create(1);
                if (pinPoll.PollDescriptor < 0)
                    throw new IOException("Call to epoll_create(1) API failed with the following return value: " + pinPoll.PollDescriptor);

                var valuePath = Path.Combine(gpioPath, string.Format("gpio{0}/value", (int)pin));

                pinPoll.FileDescriptor = UnixFile.OpenFileDescriptor(valuePath, UnixFileMode.ReadOnly | UnixFileMode.NonBlocking);

                var ev = new Interop.epoll_event
                {
                    events = (Interop.EPOLLIN | Interop.EPOLLET | Interop.EPOLLPRI),
                    data = new Interop.epoll_data { fd = pinPoll.FileDescriptor }
                };

                pinPoll.InEventPtr = Marshal.AllocHGlobal(64);
                Marshal.StructureToPtr(ev, pinPoll.InEventPtr, false);

                var controlResult = Interop.epoll_ctl(pinPoll.PollDescriptor, Interop.EPOLL_CTL_ADD, pinPoll.FileDescriptor, pinPoll.InEventPtr);
                if (controlResult != 0)
                    throw new IOException("Call to epoll_ctl(EPOLL_CTL_ADD) API failed with the following return value: " + controlResult);

                pinPoll.OutEventPtr = Marshal.AllocHGlobal(64);
                polls[pin] = pinPoll;
            }
        }

        private void UninitializePoll(ProcessorPin pin)
        {
            PinPoll poll;
            if (polls.TryGetValue(pin, out poll))
            {
                polls.Remove(pin);

                var controlResult = poll.InEventPtr != IntPtr.Zero ? Interop.epoll_ctl(poll.PollDescriptor, Interop.EPOLL_CTL_DEL, poll.FileDescriptor, poll.InEventPtr) : 0;

                Marshal.FreeHGlobal(poll.InEventPtr);
                Marshal.FreeHGlobal(poll.OutEventPtr);

                UnixFile.CloseFileDescriptor(poll.PollDescriptor);
                UnixFile.CloseFileDescriptor(poll.FileDescriptor);

                if (controlResult != 0)
                    throw new IOException("Call to epoll_ctl(EPOLL_CTL_DEL) API failed with the following return value: " + controlResult);
            }
        }

        private static string ToString(PinDetectedEdges edges)
        {
            switch (edges)
            {
                case PinDetectedEdges.Both:
                    return "both";
                case PinDetectedEdges.Rising:
                    return "rising";
                case PinDetectedEdges.Falling:
                    return "falling";
                case PinDetectedEdges.None:
                    return "none";
                default:
                    throw new ArgumentOutOfRangeException("edges", edges, string.Format(CultureInfo.InvariantCulture, "{0} is not a valid value for edge detection", edges));
            }
        }

        private void SetPinResistorClock(ProcessorPin pin, bool on)
        {
            int shift;
            var offset = Math.DivRem((int)pin, 32, out shift);

            var clockAddress = gpioAddress + (int)(Interop.BCM2835_GPPUDCLK0 + offset);
            SafeWriteUInt32(clockAddress, (uint)(on ? 1 : 0) << shift);
        }

        private void WriteResistor(uint resistor)
        {
            var resistorPin = gpioAddress + (int)Interop.BCM2835_GPPUD;
            SafeWriteUInt32(resistorPin, resistor);
        }

        private void SetPinMode(ProcessorPin pin, uint mode)
        {
            // Function selects are 10 pins per 32 bit word, 3 bits per pin
            var pinModeAddress = gpioAddress + (int)(Interop.BCM2835_GPFSEL0 + 4 * ((int)pin / 10));

            var shift = 3 * ((int)pin % 10);
            var mask = Interop.BCM2835_GPIO_FSEL_MASK << shift;
            var value = mode << shift;

            WriteUInt32Mask(pinModeAddress, value, mask);
        }

        private static void WriteUInt32Mask(IntPtr address, uint value, uint mask)
        {
            var v = SafeReadUInt32(address);
            v = (v & ~mask) | (value & mask);
            SafeWriteUInt32(address, v);
        }

        private static uint SafeReadUInt32(IntPtr address)
        {
            // Make sure we dont return the _last_ read which might get lost
            // if subsequent code changes to a different peripheral
            var ret = ReadUInt32(address);
            ReadUInt32(address);

            return ret;
        }

        private static uint ReadUInt32(IntPtr address)
        {
            unchecked
            {
                return (uint)Marshal.ReadInt32(address);
            }
        }

        private static void SafeWriteUInt32(IntPtr address, uint value)
        {
            // Make sure we don't rely on the first write, which may get
            // lost if the previous access was to a different peripheral.
            WriteUInt32(address, value);
            WriteUInt32(address, value);
        }

        private static void WriteUInt32(IntPtr address, uint value)
        {
            unchecked
            {
                Marshal.WriteInt32(address, (int)value);
            }
        }

        private struct PinPoll
        {
            public int FileDescriptor;
            public int PollDescriptor;
            public IntPtr InEventPtr;
            public IntPtr OutEventPtr;
        }

        #endregion
    }

    #region GPIO Interop
    [Flags]
    public enum UnixFileMode
    {
        /// <summary>
        /// The file will be opened with read-only access.
        /// </summary>
        ReadOnly = 1,
        /// <summary>
        /// The file will be opened with read/write access.
        /// </summary>
        ReadWrite = 2,
        /// <summary>
        /// When possible, the file is opened in nonblocking mode.
        /// </summary>
        NonBlocking = 4,
        /// <summary>
        /// The file is opened for synchronous I/O.
        /// </summary>
        Synchronized = 10000
    }
    [Flags]
    public enum MemoryProtection
    {
        None = 0,
        Read = 1,
        Write = 2,
        ReadWrite = Read | Write
    }
    [Flags]
    public enum MemoryFlags
    {
        None = 0,
        Shared = 1
    }
    [Serializable]
    public class MemoryMapFailedException : Exception
    {
        public MemoryMapFailedException() { }
        public MemoryMapFailedException(string message) : base(message) { }
        public MemoryMapFailedException(string message, Exception innerException) : base(message, innerException) { }
        protected MemoryMapFailedException(SerializationInfo info, StreamingContext context) : base(info, context) { }
    }
    [Serializable]
    public class MemoryUnmapFailedException : Exception
    {
        public MemoryUnmapFailedException() { }
        public MemoryUnmapFailedException(string message) : base(message) { }
        public MemoryUnmapFailedException(string message, Exception innerException) : base(message, innerException) { }
        protected MemoryUnmapFailedException(SerializationInfo info, StreamingContext context) : base(info, context) { }
    }


    public interface IFile : IDisposable
    {
        /// <summary>
        /// The file descriptor
        /// </summary>
        int Descriptor { get; }

        /// <summary>
        /// The pathname to the file
        /// </summary>
        string Filename { get; }
    }

    public sealed class UnixFile : IFile
    {
        #region Libc imports
        [DllImport("libc.so.6", EntryPoint = "open")]
        private static extern int open(string fileName, int mode);
        [DllImport("libc.so.6", EntryPoint = "close")]
        private static extern int close(int file);
        #endregion

        #region Fields
        private int descriptor;
        private string filename;
        #endregion

        #region Instance Management
        private UnixFile(int descriptor)
        {
            this.descriptor = descriptor;
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="UnixFile"/> class.
        /// </summary>
        /// <param name="filename">A pathname for the file.</param>
        /// <param name="fileMode">The file access mode.</param>
        public UnixFile(string filename, UnixFileMode fileMode)
            : this(OpenFileDescriptor(filename, fileMode))
        {
            this.filename = filename;
        }

        ~UnixFile()
        {
            Dispose(false);
            GC.SuppressFinalize(this);
        }
        #endregion

        #region Properties
        /// <summary>
        ///  The file descriptor
        /// </summary>
        public int Descriptor
        {
            get { return descriptor; }
        }

        /// <summary>
        /// The pathname for the file.
        /// </summary>
        public string Filename
        {
            get { return filename; }
        }
        #endregion

        #region Methods
        /// <summary>
        /// Closes the file and frees all unmanaged system resources. See <see cref="CloseFileDescriptor"/> for more information.
        /// </summary>
        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// Opens a UNIX file.
        /// </summary>
        /// <param name="fileName">The filepath.</param>
        /// <param name="fileMode">The file access mode.</param>
        /// <returns>A opened file.</returns>
        public static IFile Open(string fileName, UnixFileMode fileMode)
        {
            return new UnixFile(fileName, fileMode);
        }

        /// <summary>
        /// Opens a UNIX file and returns the file descriptor.
        /// </summary>
        /// <param name="fileName">The filepath.</param>
        /// <param name="fileMode">The file access mode.</param>
        /// <returns>The file descriptor returned by a successful call will be the lowest-numbered file descriptor not currently open for the process.</returns>
        public static int OpenFileDescriptor(string fileName, UnixFileMode fileMode)
        {
            var mode = unchecked((int)fileMode);
            return open(fileName, mode);
        }

        /// <summary>
        /// Closes a file descriptor, so that it no longer refers to any file and may be reused. Any record locks held on the file it was associated with, and owned by
        /// the process, are removed (regardless of the file descriptor that was used to obtain the lock).
        /// </summary>
        /// <param name="fileDescriptor">The file descriptor the shall be closed.</param>
        /// <returns><c>true</c> on success</returns>
        /// <remarks> 
        /// If <paramref name="fileDescriptor"/> is the last file descriptor referring to the underlying open file description, the resources associated with 
        /// the open file description are freed; if the descriptor was the last reference to a file which has been removed using unlink the file is deleted.
        /// </remarks>
        public static bool CloseFileDescriptor(int fileDescriptor)
        {
            if (fileDescriptor != 0)
            {
                return close(fileDescriptor) == 0;
            }
            return false;
        }
        #endregion

        #region Private Helpers
        private void Dispose(bool disposing)
        {
            if (disposing)
            {
                // free managed here
            }

            //Trace.Assert(disposing,
            //    string.Format("ERROR: GC finalized a unix file '{0}' with open file descriptor {1} that was not disposed!",
            //        filename, descriptor));

            if (descriptor != 0)
            {
                // we need to free unmanaged resources to avoid memory leeks
                close(descriptor);
                descriptor = 0;
                filename = null;
            }
        }
        #endregion
    }

    public static class MemoryMap
    {

        private static readonly IntPtr FAILED = new IntPtr(-1);

        [DllImport("libc.so.6", EntryPoint = "mmap")]
        private static extern IntPtr mmap(IntPtr address, UIntPtr size, int protect, int flags, int file, UIntPtr offset);

        [DllImport("libc.so.6", EntryPoint = "munmap")]
        private static extern IntPtr munmap(IntPtr address, UIntPtr size);

        public static IntPtr Create(IntPtr address, ulong size, MemoryProtection protection, MemoryFlags memoryflags, int fileDescriptor, ulong offset)
        {
            var result = mmap(address, new UIntPtr(size), (int)protection, (int)memoryflags, fileDescriptor, new UIntPtr(offset));
            ThrowOnError<MemoryMapFailedException>(result);
            return result;
        }

        public static IntPtr Create(IntPtr address, uint size, MemoryProtection protection, MemoryFlags memoryflags, int fileDescriptor, uint offset)
        {
            var result = mmap(address, new UIntPtr(size), (int)protection, (int)memoryflags, fileDescriptor, new UIntPtr(offset));
            ThrowOnError<MemoryMapFailedException>(result);
            return result;
        }

        public static void Close(IntPtr address, ulong size)
        {
            var result = munmap(address, new UIntPtr(size));
            ThrowOnError<MemoryUnmapFailedException>(result);
        }

        public static void Close(IntPtr address, uint size)
        {
            var result = munmap(address, new UIntPtr(size));
            ThrowOnError<MemoryUnmapFailedException>(result);
        }

        private static void ThrowOnError<TException>(IntPtr result)
            where TException : Exception, new()
        {
            if (result == FAILED)
            {
                throw new TException();
            }
        }

    }

    #endregion

    #endregion
}




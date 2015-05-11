using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Connectors;
using System.Threading;

namespace PiAndBash
{


    public class ButtonCatcher
    {

        public enum ButtonType
        {
            Up, Enter, Down, TopSel, BotSel
        }
        public enum ButtonEventType { Down, Up }

        public class ButtonArgs : EventArgs
        {
            public ButtonType Button { get; set; }
            public ButtonEventType ButtonEvent { get; set; }
        }

        private class ButtonState
        {
            public ButtonType Button { get; set; }
            public bool IsPressed { get; set; }
            public Mcp23017Pin Pin { get; set; }
        }

        private List<ButtonState> _states;

        public delegate void ButtonEventHandler(object sender, ButtonArgs e);
        public event ButtonEventHandler ButtonEvent;
        private bool _isRunning;
        public bool IsRunning
        {
            get
            {
                return _isRunning;
            }
        }

        I2cDriver driver;
        Mcp23017I2cConnection deviceConnection;

        /// <summary>
        /// Constructor
        /// </summary>
        public ButtonCatcher(PiAndBash.Driver PiAndBashDriver)
        {
            _states = new List<ButtonState>();
            _states.Add(new ButtonState() { Button = ButtonType.Up, Pin = Mcp23017Pin.B1 });
            _states.Add(new ButtonState() { Button = ButtonType.Down, Pin = Mcp23017Pin.B5 });
            _states.Add(new ButtonState() { Button = ButtonType.Enter, Pin = Mcp23017Pin.B3 });
            _states.Add(new ButtonState() { Button = ButtonType.TopSel, Pin = Mcp23017Pin.B6 });
            _states.Add(new ButtonState() { Button = ButtonType.BotSel, Pin = Mcp23017Pin.B7 });



            deviceConnection = PiAndBashDriver.i2cConnection;

            // Always for PnB
            foreach (ButtonState btn in _states)
            {
                deviceConnection.SetDirection(btn.Pin, Mcp23017PinDirection.Input);
                deviceConnection.SetResistor(btn.Pin, Mcp23017PinResistor.PullUp);
            }
        }



        public void Start()
        {
            _isRunning = true;

            while (ButtonEvent != null && _isRunning)
            {
                foreach (ButtonState btn in _states)
                {
                    if (!deviceConnection.GetPinStatus(btn.Pin))
                    {

                        if (!btn.IsPressed)
                        {
                            // Console.WriteLine("raise down");
                            ButtonEvent(this, new ButtonArgs() { Button = btn.Button, ButtonEvent = ButtonEventType.Down });
                            btn.IsPressed = true;
                        }

                    }
                    else if (btn.IsPressed)
                    {
                        btn.IsPressed = false;
                        ButtonEvent(this, new ButtonArgs() { Button = btn.Button, ButtonEvent = ButtonEventType.Up });
                    }
                }
            }
        }

        public void Stop()
        {
            _isRunning = false;
        }

    }

    public class Display
    {


        private string _topLine = string.Empty;
        public string TopLine
        {
            get { return _topLine.Substring(0, Math.Min(16, _topLine.Length)); }
            set
            {
                _topLine = value;
                connection.Clear();
                connection.WriteLine(TopLine);
                connection.WriteLine(BottomLine);
            }
        }

        private string _bottomLine = string.Empty;
        public string BottomLine
        {
            get { return _bottomLine.Substring(0, Math.Min(16, _bottomLine.Length)); }
            set
            {
                _bottomLine = value;
                connection.Clear();
                connection.WriteLine(TopLine);
                connection.WriteLine(BottomLine);
            }
        }

        I2cDriver driver;
        Mcp23017I2cConnection deviceConnection;
        Hd44780LcdConnection connection;
        Hd44780Configuration configuration;

        /// <summary>
        /// ctor
        /// </summary>
        public Display(PiAndBash.Driver PiAndBashDriver)
        {


            deviceConnection = PiAndBashDriver.i2cConnection;

            // turn on LCD backlight 
            deviceConnection.SetDirection(Mcp23017Pin.A0, Mcp23017PinDirection.Output);
            SetBacklight(true);


            // light is on, let's write
            var settings = new Hd44780LcdConnectionSettings
            {
                ScreenWidth = 16,
                ScreenHeight = 2
            };

            settings.Encoding = Encoding.ASCII;

            configuration = Hd44780Configuration.LoadGpioConfiguration();
            connection = new Hd44780LcdConnection(settings, configuration.Pins);


            connection.Clear();

            Console.WriteLine("Display initialised");
        }

        public void SetBacklight(bool enabled)
        {
            deviceConnection.SetPinStatus(Mcp23017Pin.A0, enabled);
        }
    }

    public class LedController
    {
        public enum LightColor { Green, Yellow, Red };

        private PiAndBash.Driver driver;
        public LedController(PiAndBash.Driver PiAndBashDriver)
        {
            driver = PiAndBashDriver;
        }

        public void SetLight(LightColor Light, bool IsOn)
        {
            Mcp23017Pin pin;
            switch (Light)
            {
                case LightColor.Green:
                    pin = Driver.ledGreen;
                    break;
                case LightColor.Yellow:
                    pin = Driver.ledYellow;
                    break;
                case LightColor.Red:
                    pin = Driver.ledRed;
                    break;
                default: // never reached
                    pin = Driver.ledGreen;
                    break;
            }
            driver.i2cConnection.SetPinStatus(pin, IsOn);
        }

    }

    public class Driver : IDisposable
    {
        public I2cDriver i2cDriver { get; set; }
        public Mcp23017I2cConnection i2cConnection;

       public const Mcp23017Pin ledGreen = Mcp23017Pin.B0;
       public const Mcp23017Pin ledYellow = Mcp23017Pin.B2;
       public const Mcp23017Pin ledRed = Mcp23017Pin.B4;
        
        const ConnectorPin sdaPin = ConnectorPin.P1Pin03;
        const ConnectorPin sclPin = ConnectorPin.P1Pin05;

        public Driver()
        {
            Console.WriteLine("Initialising Pi & Bash");
            this.i2cDriver = new I2cDriver(sdaPin.ToProcessor(), sclPin.ToProcessor());
            this.i2cConnection = new Mcp23017I2cConnection(i2cDriver.Connect(0x20));

            i2cConnection.SetDirection(ledGreen, Mcp23017PinDirection.Output);
            i2cConnection.SetDirection(ledYellow, Mcp23017PinDirection.Output);
            i2cConnection.SetDirection(ledRed, Mcp23017PinDirection.Output);

            Console.WriteLine("LEDs initialised");
        }

        void IDisposable.Dispose()
        {
            i2cDriver.Dispose();
        }
    }
}

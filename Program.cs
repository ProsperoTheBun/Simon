using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Simon_3
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Simon for Pi & Bash_");


            Connectors.GpioConnectionSettings.BoardConnectorRevision = 1;

            // initialise Pi and Bash
            PiAndBash.Driver driver = new PiAndBash.Driver();

            // initialise Sound Engine
            NotePlayer notePlayer = new NotePlayer();

            // load game engine
            GameEngine gameEngine = new GameEngine(driver, notePlayer);
            gameEngine.Start();







        }

        void test()
        {

            // // TestFeatures.SoundTest();
            ////  TestFeatures.SoundTestWindows();


            // // TestFeatures.TurnOnGreenLed();
            ////  TestFeatures.WriteToLcd();


            // // TestFeatures.WaitForAllButtons();
            //  if (!System.Diagnostics.Debugger.IsAttached)
            //  {
            //      TestFeatures.WaitForAllButtonsAsync();


            //      //TestFeatures.LatencyTest();


            //      //var np = new NotePlayer();
            //      //np.PlayNote(440, 500);
            //      //np.PlayNote(523, 1500);
            //      //np.PlayNote(349, 2500);
            //  }
            //  else
            //  {

            //      var np = new NotePlayer();
            //      np.PlayNote(440, 1000);
            //      np.PlayNote(523, 1000);
            //      np.PlayNote(349, 1000);
            //      // TestFeatures.WindowsInterruptTest();

            //  }
        }
    }
}

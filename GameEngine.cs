using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace Simon_3
{
    public class GameEngine
    {

        private PiAndBash.Driver pnbDriver;
        private PiAndBash.Display pnbDisplay;
        private PiAndBash.ButtonCatcher buttonCatcher;
        private bool isPlaying;
        private PiAndBash.LedController leds;
        private NotePlayer notePlayer;

        public GameEngine(PiAndBash.Driver PiAndBashDriver, NotePlayer NotePlayer)
        {

            pnbDriver = PiAndBashDriver;
            pnbDisplay = new PiAndBash.Display(pnbDriver);
            leds = new PiAndBash.LedController(pnbDriver);
            buttonCatcher = new PiAndBash.ButtonCatcher(pnbDriver);
            notePlayer = NotePlayer;

            // set up the Notes and associations
            notes = new Dictionary<int, Note>();
            notes.Add(1, new Note() { Color = PiAndBash.LedController.LightColor.Green, Button = PiAndBash.ButtonCatcher.ButtonType.Down, Frequency = 349 });
            notes.Add(2, new Note() { Color = PiAndBash.LedController.LightColor.Yellow, Button = PiAndBash.ButtonCatcher.ButtonType.Enter, Frequency = 440 });
            notes.Add(3, new Note() { Color = PiAndBash.LedController.LightColor.Red, Button = PiAndBash.ButtonCatcher.ButtonType.Up, Frequency = 523 });

        }

        void buttonCatcher_ButtonEvent(object sender, PiAndBash.ButtonCatcher.ButtonArgs e)
        {

            if (isPlaying && e.ButtonEvent == PiAndBash.ButtonCatcher.ButtonEventType.Down)
            {
                
                if (e.Button == beeps[currentPress].Button) // correct button
                {
                    leds.SetLight(beeps[currentPress].Color, true);
                    notePlayer.PlayNote(beeps[currentPress].Frequency, 300);
                    leds.SetLight(beeps[currentPress].Color, false);
                    currentPress++;
                    
                    if (currentPress == beeps.Count) // all notes pressed
                    {
                        pnbDisplay.BottomLine = "Score = " + beeps.Count.ToString();
                        Sleep(250);
                        nextRound();
                    }
                    
                }
                else
                {      
                    isPlaying = false;
                    pnbDisplay.TopLine = "YOU LOSE!";
                    notePlayer.Volume = 100;
                    for (int i = 0; i < 5; i++)
                    {
                        foreach (var n in notes.OrderByDescending(n=>n.Key))
                        {
                            leds.SetLight(n.Value.Color, true);
                            notePlayer.PlayNote(n.Value.Frequency*0.7, 70);
                            leds.SetLight(n.Value.Color, false);

                        }
                    }
                    notePlayer.Volume = 50;
                    // wait for one second then reset
                    Sleep(1000);
              
                    pnbDisplay.TopLine = "< Press to exit";
                    pnbDisplay.BottomLine = "< Press to start";
                    buttonCatcher.Start();
                }
              
            }
            else
            {
                if (e.ButtonEvent == PiAndBash.ButtonCatcher.ButtonEventType.Up)
                {
                    if (e.Button == PiAndBash.ButtonCatcher.ButtonType.BotSel)
                    {
                        buttonCatcher.Stop();
                        isPlaying = true;
                        newGame();
                    }
                    if (e.Button == PiAndBash.ButtonCatcher.ButtonType.TopSel)
                    {
                        pnbDisplay.SetBacklight(false);
                        pnbDisplay.TopLine = "";
                        pnbDisplay.BottomLine = "";
                        buttonCatcher.Stop();
                        Console.WriteLine("Exiting");
                    }
                }
                
            }
        }

        public void Start()
        {

            // setup game parameters



            // show welcome message and start instruction
            pnbDisplay.TopLine = "Play Simon!";
            pnbDisplay.BottomLine = "< Press to start";

            // attach handler for buttons 
            buttonCatcher.ButtonEvent += buttonCatcher_ButtonEvent;
            //start the button listener
            buttonCatcher.Start();

        }

        private List<Note> beeps = new List<Note>();
        private Dictionary<int, Note> notes;
        private int currentPress = 0;
        private class Note
        {
            public PiAndBash.LedController.LightColor Color { get; set; }
            public PiAndBash.ButtonCatcher.ButtonType Button { get; set; }
            public double Frequency { get; set; }
        }

        private void newGame()
        {
            pnbDisplay.TopLine = "Ready ...";
            pnbDisplay.BottomLine = "";

            // flash 3 leds
            leds.SetLight(PiAndBash.LedController.LightColor.Green, true);
            leds.SetLight(PiAndBash.LedController.LightColor.Red, true);
            leds.SetLight(PiAndBash.LedController.LightColor.Yellow, true);

            // wait a bit
            Sleep(1500);

            // turn them off
            leds.SetLight(PiAndBash.LedController.LightColor.Green, false);
            leds.SetLight(PiAndBash.LedController.LightColor.Red, false);
            leds.SetLight(PiAndBash.LedController.LightColor.Yellow, false);

            // game variables
            beeps.Clear();


            // round 1
            pnbDisplay.TopLine = "GO!!!";
            pnbDisplay.BottomLine = "";


            nextRound();
        }

        private void nextRound()
        {
            buttonCatcher.Stop();
            Random rnd = new Random();
            int newBeep = rnd.Next(1, 4);
            beeps.Add(notes[newBeep]);

            foreach (Note n in beeps)
            {
                // delay before next note
                Sleep(200);
                leds.SetLight(n.Color, true);
                notePlayer.PlayNote(n.Frequency, 500);
                leds.SetLight(n.Color, false);
                
            }

            currentPress = 0;
            buttonCatcher.Start();
        }



        private void Sleep(int duration)
        {
            System.Threading.Thread.Sleep(duration);
        }
    }
}

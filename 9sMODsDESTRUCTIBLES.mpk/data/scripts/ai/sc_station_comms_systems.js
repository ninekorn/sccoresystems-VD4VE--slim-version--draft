//------------------------------------------------------
// This script governs the behavior of npc ships
//
//
using(ship);
using(console);
using(game);
using(npc);
using(generator);
using(items);
using(relations);
using(station);
using(storage);
using(player);
using(timer);

include(SC_AI_Drone_Manager_Utilities.js);


var _frameCounter = 0;
var _initiliazeOnce = true;
var _systemsList;

var init_start_variables = 0;
var tempDroneCombatArray = [];
var npcDecisionsPerSeconds = 1;
//var sys_idNPC;
var sys_idPlayer;

function OnAIInited() {

    npcDecisionsPerSeconds = 19;
    npc.SetDecisionsPerSecond(SHIP_ID, npcDecisionsPerSeconds);
    player_id = npc.GetTag(SHIP_ID, "ownerPlayerShipId");
    storage.SetGlobal("npcDecisionsPerSeconds" + player_id, npcDecisionsPerSeconds);

    //sys_idNPC = ship.GetSystemID(SHIP_ID);
    sys_idPlayer = ship.GetSystemID(player_id);

    /*if (sys_idNPC == sys_idPlayer)
    {

    }
    else
    {
        someTimer = timer.SetTimer(5, "PlayerIsNotInTheSameSystemKeepNpcBrainAlive", { what_to_say: "hello" }, 0);
    }*/

    //someTimer = timer.SetTimer(3, "PlayerIsNotInTheSameSystemKeepNpcBrainAlive", { what_to_say: "hello" }, 0);

    ccommand = { com: 0, ticks: 0, id: null, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0  };
    storage.SetGlobal("player_command_", ccommand); // + ccommand.id

    ccommandtwo = { com: 0, ticks: 0, id: null, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0 };
    storage.SetGlobal("player_command_two", ccommandtwo); // + ccommand.id

    console.PrintError("SPAWNED STATION COMMS");


    sys_id = ship.GetSystemID(SHIP_ID);
    sys_idNPC = ship.GetSystemID(SHIP_ID);

    maxFrameForWaitingGibberish = 60 * npcDecisionsPerSeconds;
    maxFrameForWaitingNotGibberish = 60 * npcDecisionsPerSeconds;
}
var player_id;
var sys_id;
//-----------------------------------------------------
// Name: Decision
// Desc: 
//-----------------------------------------------------
/* args:
 scope_ships:
 disposition
 id
 is_npc
 name
 distance
 */

var init_start_variables_swtch = 1;

var playerbasecommsswtch = -1;
var playerbasecommsindex = -1;

var indexOfStationDroneManager = -1;
var arrayOfLastDroneCommand = [];
var currentObjective = null;

var ccommand;
var ccommandtwo;
//MY ORIGINAL ATTEMPT AT IT WITH DECISION(ARGS) BUT I WASN'T LETTING ENOUGH FRAMES FOR THE COMPUTER MENU PORTION TO CAPTURE KEYSTROKES ON THE COMPUTER DEVICE ANYWAY. 
/*var maxFrameForCommandOption1 = 20;//COMPUTER
var maxFrameForCommandOption2 = 60;//RELEASE
var maxFrameForCommandOption3 = 60;//COMBAT/MINING/REPAIR
var maxFrameForCommandOption4 = 60;

var maxFrameForWaitingGibberish = 60;
var maxFrameForWaitingNotGibberish = 60;*/




var maxFrameForCommandOption1 = 100;//COMPUTER
var maxFrameForCommandOption2 = 200;//RELEASE
var maxFrameForCommandOption3 = 200;//COMBAT/MINING/REPAIR
var maxFrameForCommandOption4 = 200;

var maxFrameForWaitingGibberish = 250;
var maxFrameForWaitingNotGibberish = 500;






var FrameForCommand = 0;
var startFrameSwtc = 0;
var resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
var mainOptionsSelector = -1;
var hasReceivedCommand = 0;

var lastCommand = 0;




var arrayOfStringPos = [];

var frontR = "front right";
var frontL = "front left";
var backR = "back right";
var backL = "back left";
var middleB = "middle back";



var name = "";
var shipID = null;

var GetShipOwnerSwtch = 1;

var playerHasDied = 0;
var lastFramePlayerid;

var swtchTwo = 0;

var someTimer;



function PlayerIsNotInTheSameSystemKeepNpcBrainAlive(args)
{
    //console.PrintError(args.what_to_say);

    //
    sys_idPlayer = ship.GetSystemID(player_id);

    /*if (sys_idNPC == sys_idPlayer) {

    }
    else {
        someTimer = timer.SetTimer(5, "PlayerIsNotInTheSameSystemKeepNpcBrainAlive", { what_to_say: "hello" }, 0);
    }*/

    /*if (someTimer != null) {
        timer.ClearTimer(someTimer);
    }*/
    if (swtchTwo == 0) {
        playerHasDied = 0;
        swtchTwo = 1;
    }

    player_id = npc.GetTag(shipid, "ownerPlayerShipId");

    //console.PrintError(ship.GetCurrentValue(player_id, "structure"));

    if (ship.GetCurrentValue(player_id, "structure") < 750) {
        //console.PrintError("test0");
        if (playerHasDied == 0) {
            console.PrintError("test1");
            //var jumpgates = game.GetSystemJumpgates(sys_id);
            //var jumpgates = game.GetSystemJumpgates(sys_id);
            //var nextJumpgateId = utils.SelectRandom(jumpgates);
            //ship.SetShipAsArrivedFromJumpgate(player_id, nextJumpgateId);
            //var baseid = npc.GetTag(shipid, "baseID");
            //generator.DockShipToBase(player_id, baseid);
            //var baseid = npc.GetTag(shipid, "baseID");
            //generator.DockShipToBase(player_id, baseid);
            playerHasDied = 1;
        }
    }
    else {

        /*if (generator.ShipExists(player_id) && ship.GetCurrentValue(player_id, "structure") > 0) {

            playerHasDied = 0;
        }
        else {
            if (playerHasDied == 0) {
                //var jumpgates = game.GetSystemJumpgates(sys_id);
                var jumpgates = game.GetSystemJumpgates(sys_id);
                var nextJumpgateId = utils.SelectRandom(jumpgates);
                ship.SetShipAsArrivedFromJumpgate(player_id, nextJumpgateId);

                //var baseid = npc.GetTag(shipid, "baseID");
                //generator.DockShipToBase(player_id, baseid);
                playerHasDied = 1;
            }
        }*/
    }

    /*
    if (player_id == null || isNaN(player_id) || player_id == undefined) {

        if (ship.GetCurrentValue(lastFramePlayerid, "structure") < 1500) {
            if (playerHasDied == 0) {

                //var jumpgates = game.GetSystemJumpgates(sys_id);
                var jumpgates = game.GetSystemJumpgates(sys_id);
                var nextJumpgateId = utils.SelectRandom(jumpgates);
                //ship.SetShipAsArrivedFromJumpgate(player_id, nextJumpgateId);
                generator.DockShipToBase(lastFramePlayerid, nextJumpgateId);
                //var baseid = npc.GetTag(shipid, "baseID");
                //generator.DockShipToBase(player_id, baseid);
                playerHasDied = 1;
            }
        }
        else {

            /*if (generator.ShipExists(player_id) && ship.GetCurrentValue(player_id, "structure") > 0) {
    
                playerHasDied = 0;
            }
            else {
                if (playerHasDied == 0) {
                    //var jumpgates = game.GetSystemJumpgates(sys_id);
                    var jumpgates = game.GetSystemJumpgates(sys_id);
                    var nextJumpgateId = utils.SelectRandom(jumpgates);
                    ship.SetShipAsArrivedFromJumpgate(player_id, nextJumpgateId);
    
                    //var baseid = npc.GetTag(shipid, "baseID");
                    //generator.DockShipToBase(player_id, baseid);
                    playerHasDied = 1;
                }
            }
        }


    }
    else {

        if (ship.GetCurrentValue(player_id, "structure") < 1500) {
            if (playerHasDied == 0) {

                //var jumpgates = game.GetSystemJumpgates(sys_id);
                var jumpgates = game.GetSystemJumpgates(sys_id);
                var nextJumpgateId = utils.SelectRandom(jumpgates);
                //ship.SetShipAsArrivedFromJumpgate(player_id, nextJumpgateId);
                generator.DockShipToBase(player_id, nextJumpgateId);
                //var baseid = npc.GetTag(shipid, "baseID");
                //generator.DockShipToBase(player_id, baseid);
                playerHasDied = 1;
            }
        }
        else {

            /*if (generator.ShipExists(player_id) && ship.GetCurrentValue(player_id, "structure") > 0) {
    
                playerHasDied = 0;
            }
            else {
                if (playerHasDied == 0) {
                    //var jumpgates = game.GetSystemJumpgates(sys_id);
                    var jumpgates = game.GetSystemJumpgates(sys_id);
                    var nextJumpgateId = utils.SelectRandom(jumpgates);
                    ship.SetShipAsArrivedFromJumpgate(player_id, nextJumpgateId);
    
                    //var baseid = npc.GetTag(shipid, "baseID");
                    //generator.DockShipToBase(player_id, baseid);
                    playerHasDied = 1;
                }
            }
        }


}* /





     //spawn.GetSystemID();
    //spawn.SetSystemID(1);
    //spawn.GetCoordinates();
    //spawn.SetCoordinates(100, 150);


    //var multiplier = game.GetSecondsMultiplier();
    //script.RestrictFunctionByTime("some_function", 10);
    //script.RestrictFunctionFPS("some_function", 10);
    //var time = script.GetSecondsMultiplier();
    //var a = config.GetSetting("some_var");
    //config.Wait(1000); /

    //var obj = server.GetAllPlayers();
    //var obj = server.GetConnectedPlayers();
    //var obj = server.IsPlayerExists("dlirry");
    //var obj = server.IsPlayerOnline("dlirry");

    /*var timer = timer.SetTimer(5, "SomeFunction", { what_to_say: "hello" }, 1);
    function SomeFunction(args) {
        console.Print("I was told to say " + args.what_to_say);
    }*/

    //timer.ClearTimer(5);
    //timer.GetGameTime();


    //spawn.GetSystemID();
    //spawn.SetSystemID(1);
    //spawn.GetCoordinates();
    //spawn.SetCoordinates(100, 150);

    if (storage.IsSetGlobal("npcDecisionsPerSeconds" + shipid)) {

        var npcDecisionsPerSeconds = storage.GetGlobal("npcDecisionsPerSeconds" + shipid);

        if (npcDecisionsPerSeconds == 1) {

            maxFrameForCommandOption1 = 30;//COMPUTER
            maxFrameForCommandOption2 = 60;//RELEASE
            maxFrameForCommandOption3 = 60;//COMBAT/MINING/REPAIR
            maxFrameForCommandOption4 = 60;


        }
        else if (npcDecisionsPerSeconds > 1 )
        {
            var npcDecisionsPerSecondsAlt = npcDecisionsPerSeconds * 0.65;
            maxFrameForCommandOption1 = 30 * npcDecisionsPerSecondsAlt;//COMPUTER
            maxFrameForCommandOption2 = 60 * npcDecisionsPerSecondsAlt;//RELEASE
            maxFrameForCommandOption3 = 60 * npcDecisionsPerSecondsAlt;//COMBAT/MINING/REPAIR
            maxFrameForCommandOption4 = 60 * npcDecisionsPerSecondsAlt;

            //maxFrameForWaitingGibberish = 60 * npcDecisionsPerSeconds;
            //maxFrameForWaitingNotGibberish = 60 * npcDecisionsPerSeconds;

        }

        //storage.SetGlobal("npcDecisionsPerSeconds" + player_id, npcDecisionsPerSeconds);
    }



    /*if (FrameForCommand > maxFrameForCommandOption1 * 0.75 || FrameForCommand > maxFrameForCommandOption2 * 0.75 || FrameForCommand > maxFrameForCommandOption3 * 0.75 || FrameForCommand > maxFrameForCommandOption4 * 0.75)
    {
        if (storage.IsSetGlobal("player_command_"))
        {
            ccommand = storage.GetGlobal("player_command_");
            if (ccommand.ticks < 3 && ccommand.ticks != ccommand.lastticks)
            {
                console.PrintError("resetted coms because this script captured a tick");
                FrameForCommand = Math.floor(FrameForCommand * 0.5);
                ///ccommand = storage.GetGlobal("player_command_");
                ccommand = { com: ccommand.com, ticks: ccommand.ticks, id: ccommand.id, lastticks: 0, frameforcommand: ccommand.frameforcommand, lastframeforcommand: ccommand.lastframeforcommand };
            }
        }
    }*/



    if (GetShipOwnerSwtch == 1 || GetShipOwnerSwtch == 0)
    {
        if (storage.IsSetGlobal("player_command_"))
        {
            ccommand = storage.GetGlobal("player_command_");
            name = game.GetShipOwner(ccommand.id);
        }

        GetShipOwnerSwtch = 0;
    }


    console.PrintError("ticks:" + ccommand.ticks + "/lastticks:" + ccommand.lastticks);

    if (mainOptionsSelector == -1) {
        ccommand = { com: 0, ticks: 0, id: null, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0 };

        //var someticks = ccommand.ticks;
        //ccommand.lastframeforcommand = FrameForCommand;
        //ccommand.lastticks = someticks;
        storage.SetGlobal("player_command_", ccommand);

        resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
        //console.PrintError("reseting Counter Voice To Key Stroke To Void Expanse Device");
        mainOptionsSelector = 0;
        hasReceivedCommand = 0;
        FrameForCommand = 0;
        resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
    }
    //maxFrameForWaitingGibberish

    if (hasReceivedCommand == 1) {
        //console.PrintError("coms msg - timer: " + resetCounterForVoiceToKeyStrokeToVoidExpanseDevice);

        if (resetCounterForVoiceToKeyStrokeToVoidExpanseDevice >= maxFrameForWaitingNotGibberish) {
            console.PrintError("coms msg - closing coms");
            mainOptionsSelector = 0;
            hasReceivedCommand = 0;
            FrameForCommand = 0;
            resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
            startFrameSwtc = 0;
        }

        resetCounterForVoiceToKeyStrokeToVoidExpanseDevice++;
    }
    else if (hasReceivedCommand == 2) {
        //console.PrintError("coms msg - timer: " + resetCounterForVoiceToKeyStrokeToVoidExpanseDevice);

        if (resetCounterForVoiceToKeyStrokeToVoidExpanseDevice >= maxFrameForWaitingGibberish) {
            console.PrintError("coms msg - closing coms");
            mainOptionsSelector = 0;
            hasReceivedCommand = 0;
            FrameForCommand = 0;
            resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
            startFrameSwtc = 0;
        }

        resetCounterForVoiceToKeyStrokeToVoidExpanseDevice++;
    }

    if (startFrameSwtc == 0)
    {
        if (storage.IsSetGlobal("player_command_"))
        {
            ccommand = storage.GetGlobal("player_command_");

            if (ccommand.com == 1)
            {
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                console.PrintError("comms" + ccommand.com);

                startFrameSwtc = 1;
                lastCommand = ccommand.com;
            }
        }
    }



    if (startFrameSwtc == 1)
    {
        if (storage.IsSetGlobal("player_command_")) {
            ccommand = storage.GetGlobal("player_command_");
        }


        //|| FrameForCommand > maxFrameForCommandOption2 * 0.75 || FrameForCommand > maxFrameForCommandOption3 * 0.75 || FrameForCommand > maxFrameForCommandOption4 * 0.75
        if (FrameForCommand > maxFrameForCommandOption1 * 0.75)
        {
            if (storage.IsSetGlobal("player_command_"))
            {
                ccommand = storage.GetGlobal("player_command_");
                if (ccommand.ticks != ccommand.lastticks)
                {
                    console.PrintError("1 resetted coms because this script captured a tick");
                    FrameForCommand = 0;//Math.floor(FrameForCommand * 0.5);
                    ///ccommand = storage.GetGlobal("player_command_");
                    ccommand = { com: ccommand.com, ticks: ccommand.ticks, id: ccommand.id, lastticks: 0, frameforcommand: ccommand.frameforcommand, lastframeforcommand: ccommand.lastframeforcommand };
                    storage.SetGlobal("player_command_", ccommand);
                }
            }
        }

        if (FrameForCommand >= maxFrameForCommandOption1)
        {
            //ORIGINAL
            /*if (storage.IsSetGlobal("player_command_"))
            {
                ccommand = storage.GetGlobal("player_command_");
            }*/
            //ORIGINAL

            if (ccommand.ticks == 1) // open the market hub for the player.
            {
                ccommand = { com: 0, ticks: 0, id: ccommand.id, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0 };
                //var someticks = ccommand.ticks;
                //ccommand.lastframeforcommand = FrameForCommand;
                //ccommand.lastticks = someticks;
                storage.SetGlobal("player_command_", ccommand);
                console.PrintError("MARKET");
                hasReceivedCommand = 1;
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                FrameForCommand = 0;
                startFrameSwtc = 0;
                mainOptionsSelector = 1;

                var name = game.GetShipOwner(ccommand.id);
                player.StartDialogue(name, "OnBoardComputer");
            }
            else if (ccommand.ticks == 2) // 
            {

                //working, to check what else we can do with this.
                /*sys_id = ship.GetSystemID(shipid);
                var jumpgates = game.GetSystemJumpgates(sys_id);
                console.PrintError("jumpgateID: " + jumpgates[0]);
                ship.SetShipAsArrivedFromJumpgate(player_id, jumpgates[0]);*/
                //working, to check what else we can do with this.


                //var nextJumpgateId = utils.SelectRandom(jumpgates);
                //var baseid = npc.GetTag(shipid, "baseID");
                //generator.DockShipToBase(player_id, baseid);

                /*var jumpgates0 = game.GetSystemJumpgates(sys_idNPC);

                if (jumpgates0 != null) {
                    //generator.AddSystemsLink(correctSystem, sys_idNPC);
                    //console.PrintError("test5");

                    //generator.RemoveJumpgate(lastJumpGate);
                    ship.SetShipAsArrivedFromJumpgate(ships[i], jumpgates0[0]);
                    //npc.TravelThroughJumpgate(arrayOfShips[i].id, jumpgates[0]);

                    someOtherSwitch = 0;
                    mainFrameCounter = 0;
                }
                else {
                    console.PrintError("null jump");
                }*/






                //npc.Fire(shipid, 1);
                //npc.Fire(shipid, 2);
                //visual.DeviceActivateEffectOnTarget(132, 515, "SomeDeviceEffect", 100, 100, 105);



                ccommand = { com: 0, ticks: 0, id: ccommand.id, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0 };
                //var someticks = ccommand.ticks;
                //ccommand.lastframeforcommand = FrameForCommand;
                //ccommand.lastticks = someticks;
                storage.SetGlobal("player_command_", ccommand);
                console.PrintError("COMPUTER");

                hasReceivedCommand = 1;
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                FrameForCommand = 0;
                startFrameSwtc = 2;
                mainOptionsSelector = 2;
            }
            /*else if (ccommand.ticks == 3) // 
            {
                npc.Fire(shipid, 1);
                npc.Fire(shipid, 1);
                npc.Fire(shipid, 1);
                //visual.DeviceActivateEffectOnTarget(132, 515, "SomeDeviceEffect", 100, 100, 105);



                ccommand = { com: 0, ticks: 0, id: ccommand.id };
                storage.SetGlobal("player_command_", ccommand); // + ccommand.id
                console.PrintError("CANCEL");

                hasReceivedCommand = 1;
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                FrameForCommand = 0;
                startFrameSwtc = 2;
                mainOptionsSelector = 2;
            }*/
            else if (ccommand.ticks > 4 && ccommand.ticks < 10) {
                //player.StartDialogue(name, "OnBoardComputer"); 

                var name = game.GetShipOwner(ccommand.id);
                player.StartDialogue(name, "OnBoardComputer");


                ccommand = { com: 0, ticks: 0, id: ccommand.id,lastticks:0, frameforcommand: 0, lastframeforcommand:0 };
                //var someticks = ccommand.ticks;
                //ccommand.lastframeforcommand = FrameForCommand;
                //ccommand.lastticks = someticks;
                storage.SetGlobal("player_command_", ccommand);
                console.PrintError("accessing the onboard computer");
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                //the command was gibberish so you reset the frame counter for "listen to global variable switch and commands set from the script OnBoardComputer"
                hasReceivedCommand = 2;

                FrameForCommand = 0;
                startFrameSwtc = 0;
                mainOptionsSelector = -1;
            }
            else if (ccommand.ticks == 4 || ccommand.ticks >= 10)//if (ccommand.ticks > 4 && ccommand.ticks < 10)
            {
                ccommand = { com: 0, ticks: 0, id: ccommand.id,lastticks:0, frameforcommand: 0, lastframeforcommand:0 };
                //var someticks = ccommand.ticks;
                //ccommand.lastframeforcommand = FrameForCommand;
                //ccommand.lastticks = someticks;
                storage.SetGlobal("player_command_", ccommand);
                console.PrintError("RESETTING COMMS");
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                //the command was gibberish so you reset the frame counter for "listen to global variable switch and commands set from the script OnBoardComputer"
                hasReceivedCommand = 2;

                FrameForCommand = 0;
                startFrameSwtc = 0;
                mainOptionsSelector = -1;
            }
        }
        FrameForCommand++;
    }
    else if (startFrameSwtc == 2) {

        // || FrameForCommand > maxFrameForCommandOption2 * 0.75 || FrameForCommand > maxFrameForCommandOption3 * 0.75 || FrameForCommand > maxFrameForCommandOption4 * 0.75
        if (FrameForCommand > maxFrameForCommandOption2 * 0.75) {
            if (storage.IsSetGlobal("player_command_")) {
                //var someticks = ccommand.ticks;
                //ccommand.lastframeforcommand = FrameForCommand;
                //ccommand.lastticks = someticks;
                storage.GetGlobal("player_command_", ccommand);
                if (ccommand.ticks != ccommand.lastticks) {
                    console.PrintError("2 resetted coms because this script captured a tick");
                    FrameForCommand = Math.floor(FrameForCommand * 0.5);
                    ///ccommand = storage.GetGlobal("player_command_");
                    ccommand = { com: ccommand.com, ticks: ccommand.ticks, id: ccommand.id, lastticks: 0, frameforcommand: ccommand.frameforcommand, lastframeforcommand: ccommand.lastframeforcommand };
                }
            }
        }


        if (FrameForCommand >= maxFrameForCommandOption2) {
            ccommand = storage.GetGlobal("player_command_");

            if (ccommand.ticks == 1) // open the market hub for the player.
            {
                ccommand = { com: 0, ticks: 0, id: ccommand.id,lastticks:0, frameforcommand: 0, lastframeforcommand:0 };
                //var someticks = ccommand.ticks;
                //ccommand.lastframeforcommand = FrameForCommand;
                //ccommand.lastticks = someticks;
                storage.SetGlobal("player_command_", ccommand);
                console.PrintError("MARKET");
                hasReceivedCommand = 1;
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                FrameForCommand = 0;
                startFrameSwtc = 0;
                mainOptionsSelector = 1;
                var name = game.GetShipOwner(ccommand.id);
                player.StartDialogue(name, "OnBoardComputer");
            }
            else if (ccommand.ticks == 2) {
                /*ccommand = { com: 0, ticks: 0, id: ccommand.id };
                storage.SetGlobal("player_command_", ccommand); // + ccommand.id
                console.PrintError("RELEASE");

                hasReceivedCommand = 1;
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                FrameForCommand = 0;
                startFrameSwtc = 3;
                mainOptionsSelector = 2;*/


                var totalDronesQuant = 0;
                var droneAICombat = "SC_Drone_Combat_AI_";
                var selectedDroneAI = "";
                var selectedDroneType = "combat";
                var selectedDroneXML = "";
                var tempDroneCombatArray = [];
                var droneCounter = 0;
                var arrayOfFormationPosNIder = [];
                //var currentDroneCounter = storage.GetGlobal("data" + ccommand.id);
                //var arrayOfDroneData = SC_AI_Drone_Manager_Utilities.getIDSOfDrones(ccommand.id);

                var currentDroneCounter = SC_AI_Drone_Manager_Utilities.resetGlobalArray(ccommand.id);
                //var array = { pos: arrayOfFormationPosNId[i].pos, id: arrayOfFormationPosNId[i].id, formation: arrayOfFormationPosNId[i].formation };

                /*if (currentDroneCounter.droneCounter > 0 && currentDroneCounter.droneCounter < 5) {
                    topic.AddChoice(1, "Release Drone");
                    topic.AddChoice(2, "Recover Drone");
                    topic.AddChoice(11, "Go back to the OnBoard Computer");
                    //topic.AddChoice(801, "Recall Drone");
                }
                else if (currentDroneCounter.droneCounter == 0) {
                    topic.AddChoice(1, "Release Drone");
                    topic.AddChoice(11, "Go back to the OnBoard Computer");
                }
                else if (currentDroneCounter.droneCounter == 5) {
                    //topic.AddPhrase("You cannot release other Drones! The On-Board Computer doesn't have enough capacity to manage more Drones than what are currently released.");
                    topic.AddChoice(2, "Recover Drone");
                    topic.AddChoice(11, "Go back to the OnBoard Computer");
                }*/

                var playerContainerID = items.GetGameObjectContainerId(ccommand.id);
                var itemsAndCargo = items.GetItemsAndCargo(playerContainerID);

                var tempDroneCombatArray = [];
                var quantityOfDrones = 0;

                if (itemsAndCargo.length > 0) {
                    for (var i = 0; i < itemsAndCargo.length; i++) {
                        if (itemsAndCargo[i].xml_id.substring(0, 12) == "drone_combat") {
                            //mystring = mystring.replace(/\/r/g, '');
                            //var chosenDrone = itemsAndCargo[i].xml_id.replace('_', '');

                            var tempXMLID = itemsAndCargo[i].xml_id;

                            tempXMLID = tempXMLID.replace(/_/g, " ");

                            //topic.AddChoice(800 + totalDronesQuant, " Select " + tempXMLID); //+ itemsAndCargo[i].xml_id //totalDronesQuant
                            tempDroneCombatArray.push(itemsAndCargo[i].xml_id);
                            quantityOfDrones++;
                        }
                    }

                    /*for (var i = 0; i < itemsAndCargo.length; i++) {
                        if (itemsAndCargo[i].xml_id == selectedDroneXML) {
                            quantityOfDrones = itemsAndCargo[i].quantity;
                            break;
                        }
                    }*/
                    console.PrintError("quantityOfDrones: " + quantityOfDrones);
                    if (quantityOfDrones > 0) {
                        ccommand = { com: 0, ticks: 0, id: ccommand.id,lastticks:0, frameforcommand: 0, lastframeforcommand:0 };
                        //var someticks = ccommand.ticks;
                        //ccommand.lastframeforcommand = FrameForCommand;
                        //ccommand.lastticks = someticks;
                        storage.SetGlobal("player_command_", ccommand);
                        console.PrintError("RELEASE");

                        hasReceivedCommand = 1;
                        resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                        FrameForCommand = 0;
                        startFrameSwtc = 3;
                        mainOptionsSelector = 2;

                    }
                    else {
                        ccommand = { com: 0, ticks: 0, id: ccommand.id,lastticks:0, frameforcommand: 0, lastframeforcommand:0 };
                        //var someticks = ccommand.ticks;
                        //ccommand.lastframeforcommand = FrameForCommand;
                        //ccommand.lastticks = someticks;
                        storage.SetGlobal("player_command_", ccommand);
                        console.PrintError("RESETTING COMMS. Player has no drones.");

                        //the command was gibberish so you reset the frame counter for "listen to global variable switch and commands set from the script OnBoardComputer"
                        hasReceivedCommand = 2;
                        resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                        FrameForCommand = 0;
                        startFrameSwtc = 0;
                        mainOptionsSelector = -1;
                    }
                }
                else if (ccommand.ticks == 4 || ccommand.ticks >= 10)//if (ccommand.ticks > 4 && ccommand.ticks < 10)
                {
                    ccommand = { com: 0, ticks: 0, id: ccommand.id,lastticks:0, frameforcommand: 0, lastframeforcommand:0};
                    //var someticks = ccommand.ticks;
                    //ccommand.lastframeforcommand = FrameForCommand;
                    //ccommand.lastticks = someticks;
                    storage.SetGlobal("player_command_", ccommand);
                    console.PrintError("RESETTING COMMS");
                    resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                    //the command was gibberish so you reset the frame counter for "listen to global variable switch and commands set from the script OnBoardComputer"
                    hasReceivedCommand = 2;

                    FrameForCommand = 0;
                    startFrameSwtc = 0;
                    mainOptionsSelector = -1;
                }

            }
            else if (ccommand.ticks == 3) {
                ccommand = { com: 0, ticks: 0, id: ccommand.id,lastticks:0, frameforcommand: 0, lastframeforcommand:0 };
                //var someticks = ccommand.ticks;
                //ccommand.lastframeforcommand = FrameForCommand;
                //ccommand.lastticks = someticks;
                storage.SetGlobal("player_command_", ccommand);
                console.PrintError("RETRIEVE");

                hasReceivedCommand = 1;
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                FrameForCommand = 0;
                startFrameSwtc = 3;
                mainOptionsSelector = 3;
            }
            else if (ccommand.ticks > 4 && ccommand.ticks < 10) {
                ccommand = { com: 0, ticks: 0, id: ccommand.id, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0 };
                //var someticks = ccommand.ticks;
                //ccommand.lastframeforcommand = FrameForCommand;
                //ccommand.lastticks = someticks;
                storage.SetGlobal("player_command_", ccommand);

                console.PrintError("accessing the onboard computer");

                //the command was gibberish so you reset the frame counter for "listen to global variable switch and commands set from the script OnBoardComputer"
                hasReceivedCommand = 2;
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                FrameForCommand = 0;
                startFrameSwtc = 0;
                mainOptionsSelector = -1;
            }
            else if (ccommand.ticks == 4 || ccommand.ticks >= 10)//if (ccommand.ticks > 4 && ccommand.ticks < 10)
            {
                ccommand = { com: 0, ticks: 0, id: ccommand.id, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0 };
                //var someticks = ccommand.ticks;
                //ccommand.lastframeforcommand = FrameForCommand;
                //ccommand.lastticks = someticks;
                storage.SetGlobal("player_command_", ccommand);
                storage.SetGlobal("player_command_", ccommand); // + ccommand.id
                console.PrintError("RESETTING COMMS");
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                //the command was gibberish so you reset the frame counter for "listen to global variable switch and commands set from the script OnBoardComputer"
                hasReceivedCommand = 2;

                FrameForCommand = 0;
                startFrameSwtc = 0;
                mainOptionsSelector = -1;
            }

        }
        FrameForCommand++;
    }
    else if (startFrameSwtc == 3) {

        // || FrameForCommand > maxFrameForCommandOption2 * 0.75 || FrameForCommand > maxFrameForCommandOption3 * 0.75 || FrameForCommand > maxFrameForCommandOption4 * 0.75
        if (FrameForCommand > maxFrameForCommandOption3 * 0.75) {
            if (storage.IsSetGlobal("player_command_")) {
                ccommand = storage.GetGlobal("player_command_");
                if (ccommand.ticks != ccommand.lastticks) {
                    console.PrintError("resetted coms because this script captured a tick");
                    FrameForCommand = Math.floor(FrameForCommand * 0.5);
                    ///ccommand = storage.GetGlobal("player_command_");
                    ccommand = { com: ccommand.com, ticks: ccommand.ticks, id: ccommand.id, lastticks: 0, frameforcommand: ccommand.frameforcommand, lastframeforcommand: ccommand.lastframeforcommand };
                    //var someticks = ccommand.ticks;
                    //ccommand.lastframeforcommand = FrameForCommand;
                    //ccommand.lastticks = someticks;
                    storage.SetGlobal("player_command_", ccommand);
                }
            }
        }

        if (FrameForCommand >= maxFrameForCommandOption3) {
            ccommand = storage.GetGlobal("player_command_");

            if (ccommand.ticks == 1) // open the market hub for the player.
            {
                ccommand = { com: 0, ticks: 0, id: ccommand.id, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0 };
                //var someticks = ccommand.ticks;
                //ccommand.lastframeforcommand = FrameForCommand;
                //ccommand.lastticks = someticks;
                storage.SetGlobal("player_command_", ccommand); // + ccommand.id
                console.PrintError("MARKET");
                hasReceivedCommand = 1;
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                FrameForCommand = 0;
                startFrameSwtc = 0;
                mainOptionsSelector = 1;
                var name = game.GetShipOwner(ccommand.id);
                player.StartDialogue(name, "OnBoardComputer");
            }
            else if (ccommand.ticks == 2) {


                /*ccommand = { com: 0, ticks: 0, id: ccommand.id };
                storage.SetGlobal("player_command_", ccommand); // + ccommand.id
                console.PrintError("COMBAT");



                hasReceivedCommand = 1;
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                FrameForCommand = 0;
                startFrameSwtc = 0;
                mainOptionsSelector = 2;*/





                ccommand = { com: 0, ticks: 0, id: ccommand.id, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0 };
                //var someticks = ccommand.ticks;
                //ccommand.lastframeforcommand = FrameForCommand;
                //ccommand.lastticks = someticks;
                storage.SetGlobal("player_command_", ccommand); // + ccommand.id
                console.PrintError("COMBAT");
                hasReceivedCommand = 1;
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                FrameForCommand = 0;
                startFrameSwtc = 0;
                mainOptionsSelector = 2;







                var totalDronesQuant = 0;
                var droneAICombat = "SC_Drone_Combat_AI_";
                var selectedDroneAI = "";
                var selectedDroneType = "combat";
                var selectedDroneXML = "";
                var tempDroneCombatArray = [];
                var droneCounter = 0;
                var arrayOfFormationPosNIder = [];
                //var currentDroneCounter = storage.GetGlobal("data" + ccommand.id);
                //var arrayOfDroneData = SC_AI_Drone_Manager_Utilities.getIDSOfDrones(ccommand.id);

                var currentDroneCounter = SC_AI_Drone_Manager_Utilities.resetGlobalArray(ccommand.id);
                //var array = { pos: arrayOfFormationPosNId[i].pos, id: arrayOfFormationPosNId[i].id, formation: arrayOfFormationPosNId[i].formation };

                /*if (currentDroneCounter.droneCounter > 0 && currentDroneCounter.droneCounter < 5) {
                    topic.AddChoice(1, "Release Drone");
                    topic.AddChoice(2, "Recover Drone");
                    topic.AddChoice(11, "Go back to the OnBoard Computer");
                    //topic.AddChoice(801, "Recall Drone");
                }
                else if (currentDroneCounter.droneCounter == 0) {
                    topic.AddChoice(1, "Release Drone");
                    topic.AddChoice(11, "Go back to the OnBoard Computer");
                }
                else if (currentDroneCounter.droneCounter == 5) {
                    //topic.AddPhrase("You cannot release other Drones! The On-Board Computer doesn't have enough capacity to manage more Drones than what are currently released.");
                    topic.AddChoice(2, "Recover Drone");
                    topic.AddChoice(11, "Go back to the OnBoard Computer");
                }*/











                var playerContainerID = items.GetGameObjectContainerId(ccommand.id);
                var itemsAndCargo = items.GetItemsAndCargo(playerContainerID);

                var tempDroneCombatArray = [];
                var quantityOfDrones = 0;

                if (itemsAndCargo.length > 0) {
                    for (var i = 0; i < itemsAndCargo.length; i++) {
                        if (itemsAndCargo[i].xml_id.substring(0, 12) == "drone_combat") {
                            //mystring = mystring.replace(/\/r/g, '');
                            //var chosenDrone = itemsAndCargo[i].xml_id.replace('_', '');

                            var tempXMLID = itemsAndCargo[i].xml_id;

                            tempXMLID = tempXMLID.replace(/_/g, " ");

                            //topic.AddChoice(800 + totalDronesQuant, " Select " + tempXMLID); //+ itemsAndCargo[i].xml_id //totalDronesQuant
                            tempDroneCombatArray.push(itemsAndCargo[i].xml_id);
                            quantityOfDrones++;
                        }
                    }

                    /*for (var i = 0; i < itemsAndCargo.length; i++) {
                        if (itemsAndCargo[i].xml_id == selectedDroneXML) {
                            quantityOfDrones = itemsAndCargo[i].quantity;
                            break;
                        }
                    }*/

                    if (quantityOfDrones > 0) {
                        //topic.AddPhrase("Choose a quantity to release");
                        //morse code 2 lasers, 1 that shoots for 1 frame and another for 2 frames, with modified visuals, and i could have void expanse "speaking" back in morse code.



                        //TO REVIEW SOON
                        var quantityOfDronesToRelease = 1;
                        var currentDroneCounter = storage.GetGlobal("data" + ccommand.id);
                        var droneCounter = currentDroneCounter.droneCounter;

                        if (quantityOfDronesToRelease <= 5 - droneCounter) {

                            //topic.AddPhrase("1 drone selected! Choose its position in the formation."); // now choose its position in the formation.
                            //topic.AddChoice(830, "Choose a Position");
                            var currentDroneCounter = storage.GetGlobal("data" + ccommand.id);
                            var arrayOfFormationPosNId = currentDroneCounter.arrayOfPos;

                            if (arrayOfFormationPosNId != null) {
                                console.PrintError("arrayOfFormationPosNId != null " + arrayOfFormationPosNId.length);
                            }

                            if (arrayOfFormationPosNId[0].pos == 0) {
                                //topic.AddChoice(1000, " Random Position");
                            }

                            if (arrayOfFormationPosNId[1].pos == 0) {
                                //topic.AddChoice(1001, " Front Right");

                            }

                            if (arrayOfFormationPosNId[2].pos == 0) {
                                //topic.AddChoice(1002, " Front Left");

                            }

                            if (arrayOfFormationPosNId[3].pos == 0) {
                                //topic.AddChoice(1003, " Back Right");
                            }

                            if (arrayOfFormationPosNId[4].pos == 0) {
                                //topic.AddChoice(1004, " Back Left");

                            }
                            if (arrayOfFormationPosNId[5].pos == 0) {
                                //topic.AddChoice(1005, " Middle Back");
                            }
                        }
                        else {
                            //topic.AddPhrase("You cannot release that many drones. Please make another selection.");
                        }

                        /*if (quantityOfDrones <= 1) {
                            topic.AddChoice(820, "1");
                        }
                        else if (quantityOfDrones <= 2) {
                            topic.AddChoice(820, "1");
                            topic.AddChoice(821, "2");
                        }
                        else if (quantityOfDrones <= 3) {
                            topic.AddChoice(820, "1");
                            topic.AddChoice(821, "2");
                            topic.AddChoice(822, "3");
                        }
                        else if (quantityOfDrones <= 4) {
                            topic.AddChoice(820, "1");
                            topic.AddChoice(821, "2");
                            topic.AddChoice(822, "3");
                            topic.AddChoice(823, "4");
                        }
                        else if (quantityOfDrones <= 5) {
                            topic.AddChoice(820, "1");
                            topic.AddChoice(821, "2");
                            topic.AddChoice(822, "3");
                            topic.AddChoice(823, "4");
                            topic.AddChoice(824, "5");
                        }*/




                        //var maxQuantity = quantityOfDronesToRelease;

                        //maxQuantity = quantityOfDronesToRelease;

                        selectedDroneXML = tempDroneCombatArray[0];
                        var chosenPosition = 0;

                        var tempDroneXML = selectedDroneXML;
                        //console.PrintError("test00");
                        if (quantityOfDronesToRelease > 0) {
                            //console.PrintError("test0");
                            for (var r = 0; r < 1; r++) //quantityOfDronesToRelease
                            {
                                //console.PrintError("test1");
                                selectedDroneXML = tempDroneXML;

                                /*if (input == 1000) {
                                    chosenPosition = SC_AI_Drone_Manager_Utilities.randomInArray(ccommand.id);
                                }
                                else {
                                    chosenPosition = input - 1000;
                                }*/

                                chosenPosition = SC_AI_Drone_Manager_Utilities.randomInArray(ccommand.id);

                                //chosenPosition = 0;
                                //console.PrintError("RELEASE 2");

                                //console.PrintError("chosenPosition " + chosenPosition);
                                var randomPos = Math.floor(Math.random() * (360 - 0) + 0);

                                //itemsAndCargo[i].xml_id.substring(0, 12) == "drone_combat"

                                //var currentDroneItemStart = selectedDroneXML.substring(0, 12);

                                var weaponPropulsionType = "";
                                var weaponDistanceType = "";
                                var index = -1;

                                if (tempDroneXML.indexOf("ballistic") >= 0) {
                                    weaponPropulsionType = "ballistic";
                                    weaponDistanceType = selectedDroneXML.substring(23, 23 + 2);
                                    selectedDroneAI = droneAICombat + weaponDistanceType + "_" + chosenPosition;
                                    selectedDroneXML = "xml_" + tempDroneXML;

                                    index = SC_AI_Drone_Manager_Utilities.setInitialGlobalArray(weaponDistanceType, chosenPosition);
                                }
                                else if (tempDroneXML.indexOf("missile") >= 0) {
                                    weaponPropulsionType = "missile";
                                    weaponDistanceType = selectedDroneXML.substring(21, 21 + 2);
                                    selectedDroneAI = droneAICombat + weaponDistanceType + "_" + chosenPosition;
                                    selectedDroneXML = "xml_" + tempDroneXML;

                                    index = SC_AI_Drone_Manager_Utilities.setInitialGlobalArray(weaponDistanceType, chosenPosition);
                                }
                                else if (tempDroneXML.indexOf("energy") >= 0) {
                                    weaponPropulsionType = "energy";
                                    weaponDistanceType = selectedDroneXML.substring(20, 20 + 2);
                                    selectedDroneAI = droneAICombat + weaponDistanceType + "_" + chosenPosition;
                                    selectedDroneXML = "xml_" + tempDroneXML;

                                    index = SC_AI_Drone_Manager_Utilities.setInitialGlobalArray(weaponDistanceType, chosenPosition);
                                }

                                var level = ship.GetLevel(ccommand.id);

                                /*var id1 = generator.AddNPCShipToSystem(selectedDroneType, selectedDroneAI, level, selectedDroneXML, sys_id, coords.x + (5 * Math.cos(randomPos * Math.PI / 180)), coords.y + (5 * Math.cos(randomPos * Math.PI / 180)), { ownerPlayerShiid: ccommand.id, class: "drone", someClass: "drone", droneIndex: index, wepPropType: weaponPropulsionType, wepDistType: weaponDistanceType, typeOfDrone: "combat", aDock: 1, sDock: 0, aDef: 0, mDef: 0, sDefLoc: 0, aRep: 0, mRep: 0, minHP: 0.15, formation: chosenPosition, xml: tempDroneXML });
                                //ship.RemoveItemByTypeCount(ccommand.id, "drone_mining", 1);
                                generator.SetNPCAvatarImg(id1, "avatars/unique/drone.png");
                                relations.SetShipFaction(id1, "none");
                                storage.SetGlobal("currentCommand_switch_" + id1, 1);
                                //storage.SetGlobal("currentCommand" + id1, { id: id1, command: 0, formation: chosenPosition, lastCommand: -1, wepPropType: weaponPropulsionType, wepDistType: weaponDistanceType, droneIndex: index, addFriend: 0, isInBaseVicinity: -1, AIControlType: 1, bid: null, speedSwitch: 0 }); //, behavior: behavior
                                storage.SetGlobal("currentCommand" + id1, { id: id1, command: 0, formation: chosenPosition, lastCommand: -1, wepPropType: weaponPropulsionType, wepDistType: weaponDistanceType, droneIndex: index, addFriend: 0, isInBaseVicinity: -1, AIControlType: 1, bid: null, speedSwitch: 0, typeOfDrone: "combat", aDock: 1, sDock: 0, mDock: 0, aDef: 0, mDef: 0, sDef: 0, sDefLoc: 0, aRep: 0, mRep: 0, sRep: 0, minHP: 0.15, xml: tempDroneXML }); //, behavior: behavior
         
                                //TO FIX LATER
                                var currentDroneCounter = storage.GetGlobal("data" + ccommand.id);
                                currentDroneCounter.arrayOfPos[chosenPosition].id = id1;
                                currentDroneCounter.arrayOfPos[chosenPosition].pos = 1;
                                currentDroneCounter.droneCounter++;
                                storage.SetGlobal("data" + ccommand.id, currentDroneCounter);
                                //TO FIX LATER
         
                                SC_AI_Drone_Manager_Utilities.setFinalGlobalDroneArray(id1, index, weaponDistanceType, weaponPropulsionType);
         
                                var PlayerContainerID = items.GetGameObjectContainerId(ccommand.id);
                                ship.RemoveItemByTypeCount(ccommand.id, tempDroneXML, 1);
         
                                storage.SetGlobal("currentCommand_switch_" + id1, 1);*/

                                var sys_id = ship.GetSystemID(ccommand.id);
                                var coords = ship.GetCoordinates(ccommand.id);
                                console.PrintError("selectedDroneType: " + selectedDroneType + " selectedDroneAI: " + selectedDroneAI + " level: " + level + " selectedDroneXML: " + selectedDroneXML);
                                //var id1 = generator.AddNPCShipToSystem(selectedDroneType, selectedDroneAI, level, selectedDroneXML, sys_id, coords.x + (5 * Math.cos(randomPos * Math.PI / 180)), coords.y + (5 * Math.cos(randomPos * Math.PI / 180)), { ownerPlayerShiid: ccommand.id, class: "drone", droneIndex: index, wepPropType: weaponPropulsionType, wepDistType: weaponDistanceType, typeOfDrone: "combat", aDock: 1, sDock: 0, aDef: 0, mDef: 0, sDefLoc: 0, aRep: 0, mRep: 0, minHP: 0.15, formation: chosenPosition, xml: tempDroneXML});
                                var id1 = generator.AddNPCShipToSystem(selectedDroneType, selectedDroneAI, level, selectedDroneXML, sys_id, coords.x + (5 * Math.cos(randomPos * Math.PI / 180)), coords.y + (5 * Math.cos(randomPos * Math.PI / 180)), { ownerPlayerShipid: ccommand.id, class: "drone", someClass: "drone", droneIndex: index, wepPropType: weaponPropulsionType, wepDistType: weaponDistanceType, typeOfDrone: "combat", aDock: 1, mDock: 0, sDock: 0, aDef: 0, mDef: 0, sDef: 0, sDefLoc: 0, aRep: 0, mRep: 0, sRep: 0, minHP: 0.15, formation: chosenPosition, xml: tempDroneXML });


                                //ship.RemoveItemByTypeCount(ccommand.id, "drone_mining", 1);
                                generator.SetNPCAvatarImg(id1, "avatars/unique/drone.png");
                                relations.SetShipFaction(id1, "none");
                                storage.SetGlobal("currentCommand_switch_" + id1, 1);
                                //storage.SetGlobal("currentCommand" + id1, { id: id1, command: 0, formation: chosenPosition, lastCommand: -1, wepPropType: weaponPropulsionType, wepDistType: weaponDistanceType, droneIndex: index, addFriend: 0, baseVicinity: -1, AIControlType: 1   }); //, behavior: behavior
                                storage.SetGlobal("currentCommand" + id1, { id: id1, command: 0, formation: chosenPosition, lastCommand: -1, wepPropType: weaponPropulsionType, wepDistType: weaponDistanceType, droneIndex: index, addFriend: 0, isInBaseVicinity: -1, AIControlType: 1, bid: null, speedSwitch: 0, aDock: 1, mDock: 0, sDock: 0, aDef: 0, mDef: 0, sDef: 0, sDefLoc: 0, aRep: 0, mRep: 0, sRep: 0, minHP: 0.15, xml: tempDroneXML }); //, behavior: behavior

                                //TO FIX LATER
                                var currentDroneCounter = storage.GetGlobal("data" + ccommand.id);
                                currentDroneCounter.arrayOfPos[chosenPosition].id = id1;
                                currentDroneCounter.arrayOfPos[chosenPosition].pos = 1;
                                currentDroneCounter.droneCounter++;
                                storage.SetGlobal("data" + ccommand.id, currentDroneCounter);
                                //TO FIX LATER

                                SC_AI_Drone_Manager_Utilities.setFinalGlobalDroneArray(id1, index, weaponDistanceType, weaponPropulsionType);

                                var PlayerContainerID = items.GetGameObjectContainerId(ccommand.id);
                                ship.RemoveItemByTypeCount(ccommand.id, tempDroneXML, 1);
                                //var PlayerContainerID = items.GetGameObjectContainerId(ccommand.id);
                                //var obj = generator.GetItemByXmlID(tempDroneXML);
                                //items.RemoveItemQuantity(PlayerContainerID, obj.item_id, 1);
                                //var PlayerContainerID = items.GetGameObjectContainerId(ccommand.id);
                                //var obj = generator.GetItemByXmlID(tempDroneXML);
                                //items.RemoveItemQuantity(PlayerContainerID, obj.item_id, 1);

                            }

                            //original
                            //ccommand = { com: 2, ticks: 0, id: ccommand.id };
                            //original


                            ccommand = { com: 0, ticks: 0, id: ccommand.id, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0 };
                            //var someticks = ccommand.ticks;
                            //ccommand.lastframeforcommand = FrameForCommand;
                            //ccommand.lastticks = someticks;
                            storage.SetGlobal("player_command_", ccommand); // + ccommand.id
                            console.PrintError("RESETTING COMMS. SPAWNED DRONE");

                            //the command was gibberish so you reset the frame counter for "listen to global variable switch and commands set from the script OnBoardComputer"
                            hasReceivedCommand = 2;
                            resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                            FrameForCommand = 0;
                            startFrameSwtc = 0;
                            mainOptionsSelector = -1;

                            //topic.AddChoice(lastInput, "Go back!");
                            //topic.AddChoice(9, " Quit the drone manager!");

                            //lastInput = -1;

                        }
                        else {
                            //topic.AddPhrase("There are no Drones in your inventory!");
                            //topic.AddChoice(lastInput, "Go back!");
                            //lastInput = -1;
                            ccommand = { com: 0, ticks: 0, id: ccommand.id, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0 };
                            //var someticks = ccommand.ticks;
                            //ccommand.lastframeforcommand = FrameForCommand;
                            //ccommand.lastticks = someticks;
                            storage.SetGlobal("player_command_", ccommand); // + ccommand.id
                            console.PrintError("RESETTING COMMS. Player has no inventory.");

                            //the command was gibberish so you reset the frame counter for "listen to global variable switch and commands set from the script OnBoardComputer"
                            hasReceivedCommand = 2;
                            resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                            FrameForCommand = 0;
                            startFrameSwtc = 0;
                            mainOptionsSelector = -1;
                        }




                    }
                    else {
                        //topic.AddPhrase("You do not have drones!");
                        console.PrintError("you do not have drones.");
                        ccommand = { com: 0, ticks: 0, id: ccommand.id, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0 };
                        //var someticks = ccommand.ticks;
                        //ccommand.lastframeforcommand = FrameForCommand;
                        //ccommand.lastticks = someticks;
                        storage.SetGlobal("player_command_", ccommand); // + ccommand.id
                        console.PrintError("RESETTING COMMS. Player has no inventory.");

                        //the command was gibberish so you reset the frame counter for "listen to global variable switch and commands set from the script OnBoardComputer"
                        hasReceivedCommand = 2;
                        resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                        FrameForCommand = 0;
                        startFrameSwtc = 0;
                        mainOptionsSelector = -1;
                    }


                }
                else {
                    ccommand = { com: 0, ticks: 0, id: ccommand.id, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0 };
                    //var someticks = ccommand.ticks;
                    //ccommand.lastframeforcommand = FrameForCommand;
                    //ccommand.lastticks = someticks;
                    storage.SetGlobal("player_command_", ccommand); // + ccommand.id
                    console.PrintError("RESETTING COMMS. Player has no inventory.");

                    //the command was gibberish so you reset the frame counter for "listen to global variable switch and commands set from the script OnBoardComputer"
                    hasReceivedCommand = 2;
                    resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                    FrameForCommand = 0;
                    startFrameSwtc = 0;
                    mainOptionsSelector = -1;
                }
            }

            else if (ccommand.ticks == 3) {
                ccommand = { com: 0, ticks: 0, id: ccommand.id, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0 };
                //var someticks = ccommand.ticks;
                //ccommand.lastframeforcommand = FrameForCommand;
                //ccommand.lastticks = someticks;
                storage.SetGlobal("player_command_", ccommand); // + ccommand.id
                console.PrintError("REPAIR");

                hasReceivedCommand = 1;
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                FrameForCommand = 0;
                startFrameSwtc = 0;
                mainOptionsSelector = 3;
            }
            else if (ccommand.ticks == 4) {
                ccommand = { com: 0, ticks: 0, id: ccommand.id, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0 };
                //var someticks = ccommand.ticks;
                //ccommand.lastframeforcommand = FrameForCommand;
                //ccommand.lastticks = someticks;
                storage.SetGlobal("player_command_", ccommand); // + ccommand.id
                console.PrintError("MINING");

                hasReceivedCommand = 1;
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                FrameForCommand = 0;
                startFrameSwtc = 0;
                mainOptionsSelector = 4;
            }

            else //if (ccommand.ticks > 4)
            {
                ccommand = { com: 0, ticks: 0, id: ccommand.id, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0 };
                //var someticks = ccommand.ticks;
                //ccommand.lastframeforcommand = FrameForCommand;
                //ccommand.lastticks = someticks;
                storage.SetGlobal("player_command_", ccommand); // + ccommand.id
                console.PrintError("RESETTING COMMS");

                //the command was gibberish so you reset the frame counter for "listen to global variable switch and commands set from the script OnBoardComputer"
                hasReceivedCommand = 2;
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                FrameForCommand = 0;
                startFrameSwtc = 0;
                mainOptionsSelector = -1;

            }
        }
        FrameForCommand++;
    }
    _frameCounter++;


    /*ccommand = storage.GetGlobal("player_command_");
    //var someticks = ccommand.ticks;
    //ccommand.lastframeforcommand = FrameForCommand;
    //ccommand.lastticks = someticks;
    storage.SetGlobal("player_command_", ccommand);*/

    lastFramePlayerid = player_id;
}

var someticks = 0;


var someInitialLoadingCounter = 0;
var someInitialLoadingSwtc = 0;
var someInitialLoadingCounterMAX = 40;

var shipid;

function Decision(args)
{
    shipid = SHIP_ID;

    if (someInitialLoadingSwtc == 0)
    {
        if (someInitialLoadingCounter >= someInitialLoadingCounterMAX) {
            someTimer = timer.SetTimer(0, "PlayerIsNotInTheSameSystemKeepNpcBrainAlive", { what_to_say: "hello" }, 0);
            someInitialLoadingSwtc = 1;
            someInitialLoadingCounter = 0;
        }

        someInitialLoadingCounter++;
    }



    if (someTimer == null) {

        timer.ClearTimer(someTimer);
        someTimer = timer.SetTimer(0, "PlayerIsNotInTheSameSystemKeepNpcBrainAlive", { what_to_say: "hello" }, 0);
        console.PrintError("THE TIMER IS NULL. PLEASE FIX THIS NINEKORN. YOU CAN DO THIS.");
        
        //someTimer = timer.SetTimer(1, "PlayerIsNotInTheSameSystemKeepNpcBrainAlive", { what_to_say: "hello" }, 0);
        // later change the 1 for a variable based on a global setup for server reloads otherwise everything restarting at 1 will be a huge load depending on the situation and i've thought of
        // unloading everything from the server in some instances, before i would do a debug_reinit for server updates, instead of having to close the server and reloading it back again.
    }
    else {

    }
}





/*else //if (ccommand.ticks > 4)
{
    ccommand = { com: 0, ticks: 0, id: ccommand.id };
    storage.SetGlobal("player_command_", ccommand); // + ccommand.id
    console.PrintError("RESETTING COMMS");
    resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
    //the command was gibberish so you reset the frame counter for "listen to global variable switch and commands set from the script OnBoardComputer"
    hasReceivedCommand = 2;

    FrameForCommand = 0;
    startFrameSwtc = 0;
    mainOptionsSelector = -1;
}*/




/*var arrayOfDrones = [];
var arrayOfStringPos = [];
var listOfPlayers = [];
var arrayOfLivingDrones = null;
var playerListOfFriendliesers = [];

arrayOfStringPos.push(frontR);
arrayOfStringPos.push(frontL);
arrayOfStringPos.push(backR);
arrayOfStringPos.push(backL);
arrayOfStringPos.push(middleB);

var currentDroneCounter = storage.GetGlobal("data" + ccommand.id);

if (currentDroneCounter == null || currentDroneCounter.droneCounter == null || currentDroneCounter.init != 1 || currentDroneCounter.arrayOfPos[0].pos == null || currentDroneCounter.arrayOfPos[0].id == null) //|| isNaN(currentDroneCounter)
{
    //resetGlobalArray(player_id);
    storage.Remove("data" + ccommand.id);
    var arrayOfFormations = [];

    for (var i = 0; i < 6; i++) {
        var test = { pos: 0, id: -1, formation: -1 };
        arrayOfFormations.push(test);
    }
    //console.PrintError("init");

    var currentDroneCounter3 = { init: 1, droneCounter: 0, arrayOfPos: arrayOfFormations };

    storage.SetGlobal("data" + ccommand.id, currentDroneCounter3);
}

















var playerContainerID = items.GetGameObjectContainerId(ccommand.id);
var itemsAndCargo = items.GetItemsAndCargo(playerContainerID);

var totalDronesQuant = 0;

for (var i = 0; i < itemsAndCargo.length; i++)
{
    if (itemsAndCargo[i].xml_id.substring(0, 12) == "drone_combat")
    {
        //mystring = mystring.replace(/\/r/g, '');
        //var chosenDrone = itemsAndCargo[i].xml_id.replace('_', '');

        //var tempXMLID = itemsAndCargo[i].xml_id;

        //tempXMLID = tempXMLID.replace(/_/g, " ");
        //topic.AddChoice(800 + totalDronesQuant, " Select " + tempXMLID); //+ itemsAndCargo[i].xml_id // totalDronesQuant
        tempDroneCombatArray.push(itemsAndCargo[i].xml_id);
        totalDronesQuant++;
    }
}



var isPlayerShipDocked = ship.IsDocked(ccommand.id);

if (!isPlayerShipDocked)
{
    var coords = ship.GetCoordinates(ccommand.id);
    var sys_id = ship.GetSystemID(ccommand.id);
    var level = ship.GetLevel(ccommand.id);
    var droneAICombat = "SCDroneCombatAI";
    var selectedDroneType = "combat";

    //var selectedDroneType = "";
    var selectedDroneAI = "";
    var selectedDroneXML = "";
    var weaponPropulsionType = "";
    var weaponDistanceType = "";
    var index = 0;

    var currentDroneCounter = SC_AI_Drone_Manager_Utilities.resetGlobalArray(ccommand.id);

    for (var i = 0; i < tempDroneCombatArray.length; i++)
    {
        var randomPos = Math.floor(Math.random() * (360 - 0) + 0);
        var chosenPosition = SC_AI_Drone_Manager_Utilities.randomInArray(ccommand.id);

        var tempDroneXML = tempDroneCombatArray[i];

        if (tempDroneCombatArray[i].indexOf("ballistic") >= 0)
        {
            weaponPropulsionType = "ballistic";
            weaponDistanceType = tempDroneCombatArray[i].substring(23, 23 + 2);
            selectedDroneAI = droneAICombat + weaponDistanceType + chosenPosition;
            selectedDroneXML = "xml_" + tempDroneCombatArray[i];

            index = SC_AI_Drone_Manager_Utilities.setInitialGlobalArray(weaponDistanceType, chosenPosition);
        }
        else if (tempDroneCombatArray[i].indexOf("missile") >= 0) {
            weaponPropulsionType = "missile";
            weaponDistanceType = tempDroneCombatArray[i].substring(21, 21 + 2);
            selectedDroneAI = droneAICombat + weaponDistanceType + chosenPosition;
            selectedDroneXML = "xml_" + tempDroneCombatArray[i];

            index = SC_AI_Drone_Manager_Utilities.setInitialGlobalArray(weaponDistanceType, chosenPosition);
        }
        else if (tempDroneCombatArray[i].indexOf("energy") >= 0) {
            weaponPropulsionType = "energy";
            weaponDistanceType = tempDroneCombatArray[i].substring(20, 20 + 2);
            selectedDroneAI = droneAICombat + weaponDistanceType + chosenPosition;
            selectedDroneXML = "xml_" + tempDroneCombatArray[i];

            index = SC_AI_Drone_Manager_Utilities.setInitialGlobalArray(weaponDistanceType, chosenPosition);
        }

        var id1 = generator.AddNPCShipToSystem(selectedDroneType, selectedDroneAI, level, selectedDroneXML, sys_id, coords.x + (5 * Math.cos(randomPos * Math.PI / 180)), coords.y + (5 * Math.cos(randomPos * Math.PI / 180)), { ownerPlayerShiid: ccommand.id, class: "drone", droneIndex: index, wepPropType: weaponPropulsionType, wepDistType: weaponDistanceType, typeOfDrone: "combat", aDock: 1, sDock: 0, aDef: 0, mDef: 0, sDefLoc: 0, aRep: 0, mRep: 0, minHP: 0.15, formation: chosenPosition, xml: selectedDroneXML });
        //ship.RemoveItemByTypeCount(player_id, "drone_mining", 1);

        generator.SetNPCAvatarImg(id1, "avatars/unique/drone.png");
        relations.SetShipFaction(id1, "none");
        storage.SetGlobal("currentCommand_switch_" + id1, 1);
        storage.SetGlobal("currentCommand" + id1, { id: id1, command: 0, formation: chosenPosition, lastCommand: -1, wepPropType: weaponPropulsionType, wepDistType: weaponDistanceType, droneIndex: index, addFriend: 0, baseVicinity: -1, AIControlType: 1 }); //, behavior: behavior

        //TO FIX LATER
        var currentDroneCounter = storage.GetGlobal("data" + ccommand.id);
        currentDroneCounter.arrayOfPos[chosenPosition].id = id1;
        currentDroneCounter.arrayOfPos[chosenPosition].pos = 1;
        currentDroneCounter.droneCounter++;
        storage.SetGlobal("data" + ccommand.id, currentDroneCounter);
        //TO FIX LATER

        SC_AI_Drone_Manager_Utilities.setFinalGlobalDroneArray(id1, index, weaponDistanceType, weaponPropulsionType,chosenPosition);

        var PlayerContainerID = items.GetGameObjectContainerId(ccommand.id);
        ship.RemoveItemByTypeCount(ccommand.id, tempDroneXML, 1);

    }
}*/











//=====================================================================//
//========================================================//
//=====================================================================//
/*if(init_start_variables == 0)	
{
    //droneIndex = npc.GetTag(SHIP_ID, "droneIndex");

    if(init_start_variables_swtch == -1)
    {
        if(args.id == SHIP_ID)
        {
            playerbasecommsindex = storage.GetGlobal("playerbasecommsindex" + SHIP_ID);

            if (!storage.IsSetGlobal("playerbasecommsindex" + SHIP_ID))
            { 
                indexOfStationDroneManager = 0;
                storage.SetGlobal("playerbasecommsindex" + SHIP_ID, indexOfStationDroneManager);
                //storage.SetGlobal("playerbasecommsswtch" + chosenBase.id, indexOfStationDroneManager);
            }
            else
            {
                indexOfStationDroneManager = storage.GetGlobal("playerbasecommsindex" + SHIP_ID);
                indexOfStationDroneManager++;
                storage.SetGlobal("playerbasecommsindex" + SHIP_ID, indexOfStationDroneManager);
                //storage.SetGlobal("playerbasecommsswtch" + chosenBase.id, indexOfStationDroneManager);
            }
        }
        init_start_variables_swtch = 1;
    }


    if(init_start_variables_swtch == 1)
    {
        playerbasecommsswtch = storage.GetGlobal("playerbasecommsswtch" + SHIP_ID);
    	
        if (!storage.IsSetGlobal("playerbasecommsswtch" + SHIP_ID))
        { 
            //{ ownerPlayerccommand.id: player_id, class: "drone", droneIndex: index, wepPropType: weaponPropulsionType, wepDistType: weaponDistanceType,typeOfDrone:"combat", aDock: 1, sDock: 0, aDef: 0, mDef: 0, sDefLoc: 0, aRep: 0, mRep: 0, minHP: 0.15,formation:chosenPosition,xml:tempDroneXML 
            storage.SetGlobal("playerbasecommsswtch" + SHIP_ID, {id: SHIP_ID, class: "playerbasecomms", com: 1,lastcom:-1});
        }
        else
        {
            var ccommand = storage.GetGlobal("playerbasecommsswtch" + SHIP_ID);

            if(ccommand.com ==  1)
            {
                if (npc.CountObjectives(ccommand.id) > 0) // isSelfActive == true => compute Objective
                {
                    currentObjective = npc.GetCurrentObjective(ccommand.id);

                    AICoreComputeObjective(currentObjective); //, ccommand.addFriend

                    if (ccommand.com != ccommand.lastcom) 
                    {
                        ccommand.com = 2;
                        npc.StopFollowingRoute(ccommand.id);
                        npc.NextObjective(ccommand.id);
                        npc.CleanObjectives(ccommand.id);
                        //npc.InstantStop(CToD.id);
                        npc.Stop(ccommand.id);

                        //CToD = storage.GetGlobal("currentCommand" + CToD.id);
                        ccommand.lastcom = ccommand.com;
                        storage.SetGlobal("playerbasecommsswtch" + ccommand.id, ccommand);
                    }
                }	
                else
                {
                    if (ccommand.com == 2)
                    {
                        var sys_id = npc.GetCurrentSystemID(ccommand.id);

                        npc.AddObjective(ccommand.id, "listen_to_player_comms",
                        {
                            nid: ccommand.id,
                            sid: sys_id,
                            com: ccommand.com,
                            lastcom: ccommand.lastcom,

                            //id: player_id,					
                            //bid: ccommand.baseVicinity,
                            //pName: playerName,
                            //command: ccommand.command,
                            //formation: ccommand.formation,
                            //speedSwitch: 0,
                            //maxPlayerForwardSpeed: null,
                            //maxNPCForwardSpeed: null,
                            //maxNPCStrafeSpeed: null,
                            //wepPropType: ccommand.wepPropType,
                            //wepDistType: ccommand.wepDistType,
                            //droneIndex: droneIndex,
                            //aDock: ccommand.aDock,
                            //mDock: ccommand.sDock,
                            //aDef: ccommand.aDef,
                            //mDef: ccommand.sDef,
                            //aRep: ccommand.aRep,
                            //mRep: ccommand.sRep,
                            //minHP: ccommand.minHP,
                            //sDefLoc: ccommand.sDefLoc
                        });
                    }
                    else if(ccommand.com == 0)
                    {
                	
                    }
                }
            }
        }
    }

	
    init_start_variables = 1;
}*/

/*if(start_swtch == 1)
{
    console.PrintError(_frameCounter);
}*/
//=====================================================================//
//========================================================//
//=====================================================================//

function AICoreComputeObjective(currentObjective) //addFriendSwitch
{
    if (currentObjective == null) {
        return;
    }

    switch (currentObjective.name) {
        case "listen_to_player_comms":
            {
                //console.PrintError("" + _frameCounter);

                var ccommand = storage.GetGlobal("player_command");

                if (ccommand != null) {
                    if (!storage.IsSetGlobal("player_command")) {

                    }
                    else {
                        var ccommand = storage.GetGlobal("player_command");

                        if (ccommand.com == 1) {
                            console.PrintError("PLAYER BASE COMMS COMMAND RECEIVED");

                        }
                    }
                }


                break;
            }
    }
}










/*
if (startFrameSwtc == 0)
{
 
    /*else if (mainOptionsSelector == 2)
    {

        /*if (resetCounterForVoiceToKeyStrokeToVoidExpanseDevice >= maxFrameForWaiting)
        {

            maxFrameForWaiting = 0;
        }
        resetCounterForVoiceToKeyStrokeToVoidExpanseDevice++;
    }
    else if (mainOptionsSelector == 3)
    {

    }


    if (mainOptionsSelector == 0)
    {
        if (storage.IsSetGlobal("player_command_"))
        {
            //console.PrintError("isSEtGlobal");
            ccommand = storage.GetGlobal("player_command_");

            //console.PrintError("no comms" + ccommand.com);

            //if (lastCommand != ccommand.com)
            //{
            if (ccommand.com == 1)
            {
                resetCounterForVoiceToKeyStrokeToVoidExpanseDevice = 0;
                console.PrintError("comms" + ccommand.com);

                //has received command.
                //hasReceivedCommand = 1;
               //FrameForCommand = maxFrameForCommand;
                startFrameSwtc = 1;
                lastCommand = ccommand.com;
            }
            //}
            //else
            //{

            //}
        }
    }
}*/





			/*
			if(playerbasecommsswtch == 1||arrayOfLastDroneCommand.length < droneIndex|| arrayOfLastDroneCommand[0] == null)
			{
				CToD = storage.GetGlobal("playerbasecommscommand" + SHIP_ID);
				if(arrayOfLastDroneCommand.length < droneIndex|| arrayOfLastDroneCommand[0] == null)
				{
					arrayOfLastDroneCommand.push(CToD);
				}
				else
				{	
					arrayOfLastDroneCommand[droneIndex] = CToD;
				}
			}
			else if(arrayOfLastDroneCommand[droneIndex] == null )
			{
				CToD = storage.GetGlobal("playerbasecommscommand" + SHIP_ID);
				arrayOfLastDroneCommand[droneIndex] = CToD;
			}

			currentCToD = arrayOfLastDroneCommand[droneIndex];
		
			if (currentCToD != null)
			{
				if (currentCToD.id != null) 
				{
					if (currentCToD.id == SHIP_ID) 
					{
						if (npc.CountObjectives(currentCToD.id) > 0) // isSelfActive == true => compute Objective
						{

						}	
					}
				}
			}*/
	/*if (!storage.IsSetGlobal("StationDroneManagerIndex" + ))
		{ 
			indexOfStationDroneManager = 0;
			storage.SetGlobal("StationDroneManagerIndex", indexOfStationDroneManager);
			storage.SetGlobal("StationDroneManagerIndex_" + chosenBase.id, indexOfStationDroneManager);
		}
		else
		{
		    indexOfStationDroneManager = storage.GetGlobal("StationDroneManagerIndex");
            indexOfStationDroneManager++;
            storage.SetGlobal("StationDroneManagerIndex", indexOfStationDroneManager);
			storage.SetGlobal("StationDroneManagerIndex_" + chosenBase.id, indexOfStationDroneManager);
		}*/

  //=====================================================================//
    //============================GALAXY MARKET============================//
    //=====================================================================//
	/*if (_initiliazeOnce)
    {
        _systemsList = generator.GetAllSystems();

        for (var index = 0; index < _systemsList.length; index++)
        {
            var systemId = _systemsList[index];
            var systemBases = game.GetSystemBases(systemId);

            if (systemBases.length <= 0)
            {
                _systemsList.splice(index, 1);
                index--;
            }
        }
        _initiliazeOnce = false;
    }

    if (_frameCounter > 2)
    {
        if (_systemsList.length > 0)
        {
            //systemBasesVirtualLocker = game.GetSystemBases(systemId);

            for (var index = 0; index < _systemsList.length; index++)
            {
                var _systemBasesTotal = game.GetSystemBases(_systemsList[index]);

                for (var j = 0; j < _systemBasesTotal.length; j++)
                {
                    //console.PrintError(_systemBasesTotal[i] + " item generation completed");

                    var base = generator.GetBaseByID(_systemBasesTotal[j]);
                    var faction = relations.GetBaseFaction(_systemBasesTotal[j]);

                    // get all items sorted by types
                    var types = game.GetItemTypes();
                    var allItems = {};
                    for (var i = 0; i < types.length; i++) {
                        allItems[types[i]] = game.GetItems(base.tech_level, types[i], faction);
                    }

                    // get all civilian items
                    var civilianItems = game.GetCivilianItems();
                    for (var i = 0; i < civilianItems.length; i++) {
                        station.AddItem(_systemBasesTotal[j], civilianItems[i]);
                    }

                    // get all item types marked with "spawnSeparately" tag 
                    // - this is required to ensure that the item will be spawned without any randomization
                    var spawnRequiredItems = game.GetSpawnSeparatelyItems(base.tech_level, faction);
                    for (var i = 0; i < spawnRequiredItems.length; i++) {
                        station.AddItem(_systemBasesTotal[j], spawnRequiredItems[i]);
                    }

                    // generate quantities and types for base
                    var dict = {};
                    var distribution = station.GenerateStockDistribution(_systemBasesTotal[j]);
                    for (var type in distribution)
                    {
                        // please note: type can be hull/engine/consumable/etc
                        var itemsOfType = allItems[type];
                        if (itemsOfType.length != 0)
                        {
                            var amount = distribution[type];

                            //now we have type and amount - it's time to generate
                            for (var i = 0; i < amount; i++) {
                                // take random itemType
                                var itemType = utils.SelectRandom(itemsOfType);

                                if (dict[itemType] != null) {
                                    dict[itemType]++;
                                }
                                else {
                                    dict[itemType] = 1;
                                }
                            }
                        }                   
                    }

                    for (var itemType in dict)
                    {
                        var quantity = dict[itemType];
                        station.AddItem(_systemBasesTotal[j], itemType, quantity);
                    }
                    //console.PrintError(_systemBasesTotal[j] + " item generation completed");
                }
                _systemsList.splice(index, 1);
                index--;
                break;
            }
        }
        else
        {
            //console.PrintError("total item generation completed");
            generator.RemoveShip(SHIP_ID);
        }
        _frameCounter = 0;
    }*/
    //=====================================================================//
    //============================GALAXY MARKET============================//
    //=====================================================================//
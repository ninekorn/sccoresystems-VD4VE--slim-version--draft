/*
	Script can contain following functions:
		OnUpdateCache - called when device is equipped or initialized, or some values are changed
		OnStart - called when device is activated (for all)
		OnFrame - called every frame (only for "per frame" mode)
		OnFinished - called when device effect should be applied (only for "per frame" and "on complete")
*/
include(pickByChance.js);
using(ship);
using(console);
using(game);
using(generator);
using(relations);
using(player);
//using(actions);
using(storage);
using(visual);
using(npc);

function OnAIInited()
{
	
}

function OnInit()
{
}

function OnUpdateCache(args)
{

}

function OnEquipped(args)
{

}

var droneReleaseCounter = 0;
var numberOfDrones = "numberOfDrones";
//var numberOfDronesReleased = ;
var numberOfDronesInCargo = 5;
var listOfDrones = [];
//var reReleaser = 0;

var ccommand;

var ccommandtwo;

function OnStart(args)
{
	var playerID = args.ship_id;
    //var name = game.GetShipOwner(playerID);

    console.PrintError("frame");




    //npc.Fire(SHIP_ID, 1);
    //npc.Fire(SHIP_ID, 2);





    var coords = ship.GetCoordinates(playerID);
    visual.DeviceActivateEffectOnTarget(playerID, args.device_id, "emp_nova_heavy_visual_effect", 3, coords.x+3, coords.y);



    //npc.Fire(SHIP_ID, 0);
    //npc.Fire(SHIP_ID, 1);

    if (storage.IsSetGlobal("player_command_")) {

        //player_id = npc.GetTag(SHIP_ID, "ownerPlayerShipId");
        console.PrintError("playerID: " + playerID);
        ccommand = storage.GetGlobal("player_command_");
        ccommand.ticks = ccommand.ticks + 1;
        ccommand = { com: 1, ticks: ccommand.ticks, id: playerID, lastticks: ccommand.ticks,  frameforcommand: ccommand.frameforcommand, lastframeforcommand: ccommand.lastframeforcommand};
        storage.SetGlobal("player_command_", ccommand);
        console.PrintError("ticks: " + ccommand.ticks);
    }
    else
    {
        var ccommand = { com: 1, ticks: 0, id: playerID, lastticks: 0, frameforcommand: 0, lastframeforcommand: 0 };

        //ccommand.ticks = ccommand.ticks + 1;
        //ccommand = { com: 1, ticks: ccommand.ticks, id: playerID };
        //storage.SetGlobal("player_command_", ccommand);
        //console.PrintError("ticks: " + ccommand.ticks);

        storage.SetGlobal("player_command_", ccommand); // + ccommand.pid
    }



    /*
    if (storage.IsSetGlobal("player_command_two")) {

        //player_id = npc.GetTag(SHIP_ID, "ownerPlayerShipId");
        console.PrintError("playerID: " + playerID);
        ccommandtwo = storage.GetGlobal("player_command_two");
        ccommandtwo.ticks = ccommandtwo.ticks + 1;
        ccommandtwo = { com: 1, ticks: ccommandtwo.ticks, id: playerID, lastticks: ccommandtwo.ticks, frameforcommand: ccommandtwo.frameforcommand, lastframeforcommand: ccommandtwo.lastframeforcommand };
        storage.SetGlobal("player_command_two", ccommandtwo);
        console.PrintError("ticks: " + ccommandtwo.ticks);
    }
    else {
        var ccommandtwo = { com: 1, ticks: 0, id: playerID, lastticks: ccommandtwo.ticks, frameforcommand: 0, lastframeforcommand: 0 };

        //ccommandtwo.ticks = ccommandtwo.ticks + 1;
        //ccommandtwo = { com: 1, ticks: ccommandtwo.ticks, id: playerID };
        //storage.SetGlobal("player_command_", ccommandtwo);
        //console.PrintError("ticks: " + ccommandtwo.ticks);

        storage.SetGlobal("player_command_two", ccommandtwo); // + ccommandtwo.pid
    }
    */














    /*if (storage.IsSetGlobal("player_command_receiver")) {

        //player_id = npc.GetTag(SHIP_ID, "ownerPlayerShipId");

        ccommand = storage.GetGlobal("player_command_receiver");

        if (ccommand.com == 2)
        {

        }
        //ccommand.ticks = ccommand.ticks + 1;
        ccommand = { com: 1, ticks: ccommand.ticks, pid: playerID };
        storage.SetGlobal("player_command_receiver", ccommand);
        //console.PrintError("ticks: " + ccommand.ticks);
    }*/





	//var coords = ship.GetCoordinates(playerID);
	//var sys_id = ship.GetSystemID(playerID);

	//console.Print(name);

    //player.StartDialogue(name,"OnBoardComputer"); //mmh. the drone manager... where the hell do i spawn him again... do i spawn him in every system or just in one system?... do i spawn him in every stations of every systems?
}

function OnFrame(args)
{

}

function OnFinished(args)
{

}

function OnCancel(args)
{

}







































/*
 Script can contain following functions:
 OnUpdateCache - called when device is equipped or initialized, or some values are changed
 OnStart - called when device is activated (for all)
 OnFrame - called every frame (only for "per frame" mode)
 OnFinished - called when device effect should be applied (only for "per frame" and "on complete")
 

using(ship);
using(console);
using(generator);
using(game);
using(visual);
using(actions);



function OnUpdateCache(args)
{
	var inf = generator.GetSystemByID(args.sys_id);
    //var id = generator.AddContainer(1, 0, 0, "Container01", "DropList01");
	//var ship_id = player.GetShipOfPlayer(args);
	//console.Print(is_player);
	console.Print(inf);

}







function OnStart(args)
{
	//var ship_id = player.GetShipOfPlayer(ship);
	//var name = game.GetShipOwner(args);
	//var is_player = game.IsShipPlayerControlled(args);
	//var coords = args.GetCoordinates(args);

	var inf = generator.GetSystemByID(args.sys_id);
    //var id = generator.AddContainer(1, 0, 0, "Container01", "DropList01");
	//var ship_id = player.GetShipOfPlayer(args);
	//console.Print(is_player);
	console.Print(inf);
	//console.Print(coords);
	//console.Print(ship_id);



}

function OnFrame(args)
{
	var inf = generator.GetSystemByID(args.sys_id);
    //var id = generator.AddContainer(1, 0, 0, "Container01", "DropList01");
	//var ship_id = player.GetShipOfPlayer(args);
	//console.Print(is_player);
	console.Print(inf);
}

function OnFinished(args)
{

}

function OnCancel(args)
{

}*/
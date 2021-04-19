using(npc);
using(console);
using(player);
using(ship);
using(game);
using(generator);
using(storage);
using(station);
using(items);
using(timer);
using(server);

include(SC_AI_Drone_Combat_cc_Core_5.js);
include(SC_AI_Drone_Manager_Utilities.js);

function OnAIInited() {

    var npcDecisionsPerSeconds = 19;
    npc.SetDecisionsPerSecond(SHIP_ID, npcDecisionsPerSeconds);
    var player_id = npc.GetTag(SHIP_ID, "ownerPlayerShipId");
    storage.SetGlobal("npcDecisionsPerSeconds" + player_id, npcDecisionsPerSeconds);

    //var player_id = player.GetShipOfPlayer(args.name);
    /*if (!storage.IsSetGlobal("npcDecisionsPerSeconds" + player_id))
    {

    }*/

 

    npc.SetBehavior(SHIP_ID, "avoid_asteroids", false);
    npc.SetBehavior(SHIP_ID, "avoid_ships", false);
    npc.SetBehavior(SHIP_ID, "avoid_bases", false);
    npc.SetBehavior(SHIP_ID, "avoid_debris", false);
    npc.StopEvasion(SHIP_ID);

    //NpcLib.SetupDefaultWeaponUsage(SHIP_ID);
    npc.SetWeaponUsage(SHIP_ID, "Ray", 0, 7, 0.001);
    npc.SetWeaponUsage(SHIP_ID, "BulletOrMissile", 2, 7, 0.001);
    npc.SetWeaponUsage(SHIP_ID, "DirectHit", 2, 7, 0.001);
}

var player_id;
var initStartingStuff;
var droneIndex;
var playerName;


var signs_GoSwitchSwitch = 0;

var goSignMainFrameCounter = 0;
var goSignMainFrameCounterMax = 5;

var stopSignMainFrameCounter = 0;
var stopSignMainFrameCounterMax = 1200; //1200 == 60 seconds for a stable thread and thats why i will use the timer class soon, to have a stable frame for some stuff in this mod. when approx setDecisionsPerSeconds is at 20 == 1200 frames for a 60 seconds wait time.


var DockingPointMainFrameCounter = 0;
var DockingPointMainFrameCounterMax = 0;



var baseID;

var mainFrameCounter = 0;

function Decision(args)
{
	player_id = npc.GetTag(SHIP_ID, "ownerPlayerShipId");






            /*if (storage.IsSetGlobal("station_tiles" + baseID))
        {
            var stationTiles = storage.GetGlobal("station_tiles" + baseID);

            for (var i = 0; i < stationTiles.visualTiles.length;i++)
            {
                if (stationTiles.visualTiles[i] == 2)
                {
                    stationTiles.visualTiles[i] = 1;
                }
            }
            storage.SetGlobal("station_tiles" + baseID, stationTiles);
        }*/


    if (mainFrameCounter > 10) {
        var sys_id = npc.GetCurrentSystemID(SHIP_ID);

        if (storage.IsSetGlobal("crates_" + sys_id)) {
            var crates = storage.GetGlobal("crates_" + sys_id);

            if (crates.length > 0) {
                for (var i = 0; i < crates.length; i++) {
                    //generator.RemoveSpecialObject(crates[i].id);
                    game.DeleteGameObject(crates[i].id);
                }
            }
            storage.SetGlobal("crates_" + sys_id, crates);
        }
        mainFrameCounter = 0;
    }


    if (goSignMainFrameCounter >= goSignMainFrameCounterMax) {
        var sys_id = npc.GetCurrentSystemID(SHIP_ID);

        if (storage.IsSetGlobal("crates_Go" + sys_id)) {
            var crates = storage.GetGlobal("crates_Go" + sys_id);

            if (crates.length > 0) {
                for (var i = 0; i < crates.length; i++) {
                    //generator.RemoveSpecialObject(crates[i].id);
                    game.DeleteGameObject(crates[i].id);
                }
                goSignMainFrameCounter = 0;
            }
            storage.SetGlobal("crates_Go" + sys_id, crates);
        }

    }
    goSignMainFrameCounter++;

    if (stopSignMainFrameCounter > stopSignMainFrameCounterMax) {

        var sys_id = npc.GetCurrentSystemID(SHIP_ID);

        if (storage.IsSetGlobal("crates_Stop" + sys_id)) {
            var crates = storage.GetGlobal("crates_Stop" + sys_id);

            if (crates.length > 0) {
                baseID = crates[0].staID;
                for (var i = 0; i < crates.length; i++) {
                    //generator.RemoveSpecialObject(crates[i].id);
                    game.DeleteGameObject(crates[i].id);
                }
                stopSignMainFrameCounter = 0;
            }
            storage.SetGlobal("crates_Stop" + sys_id, crates);


        }
        /*if (storage.IsSetGlobal("crates_Stop" + sys_id))
        {
            var crates = storage.GetGlobal("crates_Stop" + sys_id);

            if (crates.length > 0)
            {
                baseID = crates[0].staID;
                for (var i = 0; i < crates.length; i++)
                {
                    generator.RemoveSpecialObject(crates[i].id);
                }
            }
            storage.SetGlobal("crates_Stop" + sys_id, crates);

            //if (storage.IsSetGlobal("station_tiles" + baseID))
            //{
            //    var stationTiles = storage.GetGlobal("station_tiles" + baseID);
            //
            //    for (var i = 0; i < stationTiles.visualTiles.length;i++)
            //    {
            //        if (stationTiles.visualTiles[i] == 2)
            //        {
            //            stationTiles.visualTiles[i] = 1;
            //        }
            //    }
            //    storage.SetGlobal("station_tiles" + baseID, stationTiles);
            //}
        }*/


    }



    if (DockingPointMainFrameCounter > stopSignMainFrameCounterMax + DockingPointMainFrameCounterMax) {
        var sys_id = npc.GetCurrentSystemID(SHIP_ID);


       


        if (storage.IsSetGlobal("signs_station_docking_points" + sys_id)) {
            var crates = storage.GetGlobal("signs_station_docking_points" + sys_id);

            if (crates.length > 0) {
                baseID = crates[0].staID;
                for (var i = 0; i < crates.length; i++) {
                    //generator.RemoveSpecialObject(crates[i].id);
                    game.DeleteGameObject(crates[i].id);
                }
                DockingPointMainFrameCounter = 0;
            }
            storage.SetGlobal("signs_station_docking_points" + sys_id, crates);


            /*if (storage.IsSetGlobal("crates_Stop" + sys_id))
            {
                var crates = storage.GetGlobal("crates_Stop" + sys_id);

                if (crates.length > 0) {
                    baseID = crates[0].staID;
                    for (var i = 0; i < crates.length; i++) {
                        generator.RemoveSpecialObject(crates[i].id);
                    }
                }
                storage.SetGlobal("crates_Stop" + sys_id, crates);

                if (storage.IsSetGlobal("station_tiles" + baseID))
                {
                    var stationTiles = storage.GetGlobal("station_tiles" + baseID);

                    for (var i = 0; i < stationTiles.visualTiles.length; i++) {
                        if (stationTiles.visualTiles[i] == 2) {
                            stationTiles.visualTiles[i] = 1;
                        }
                    }
                    storage.SetGlobal("station_tiles" + baseID, stationTiles);
                }

                if (storage.IsSetGlobal("signs_Go" + sys_id)) {
                    var crates = storage.GetGlobal("signs_Go" + sys_id);

                    if (crates.length > 0) {
                        for (var i = 0; i < crates.length; i++) {
                            generator.RemoveSpecialObject(crates[i]);
                        }
                    }
                    storage.SetGlobal("signs_Go" + sys_id, crates);
                }
            }*/

            /*if (storage.IsSetGlobal("station_tiles" + baseID))
            {
                var stationTiles = storage.GetGlobal("station_tiles" + baseID);

                for (var i = 0; i < stationTiles.visualTiles.length; i++)
                {
                    if (stationTiles.visualTiles[i] == 2) {
                        stationTiles.visualTiles[i] = 1;
                    }
                }
                storage.SetGlobal("station_tiles" + baseID, stationTiles);
            }*/
        }


    }










    DockingPointMainFrameCounter++;
    stopSignMainFrameCounter++;
    mainFrameCounter++;









    if (generator.ShipExists(player_id))
    {
        playerName = game.GetShipOwner(player_id);

        if (!server.IsPlayerOnline(playerName))
        {
            //generator.RemoveShip(SHIP_ID);
        }
        else
        {
            droneIndex = parseInt(npc.GetTag(SHIP_ID, "droneIndex"));	
            SC_AI_Drone_Combat_cc_Core_5.AICoreInit(SHIP_ID, droneIndex, playerName, player_id, args);
            return;
        }
    }
    else
    {
        if (!server.IsPlayerOnline(playerName))
        {
            //generator.RemoveShip(SHIP_ID);
        }
        else
        {
            //player id changed but player is still online??? 
        }
    }




}

function OnDied(args)
{
    droneIndex = npc.GetTag(SHIP_ID, "droneIndex");
    currentObjective = npc.GetCurrentObjective(SHIP_ID);

    SC_AI_Drone_Manager_Utilities.setRemovedFromGlobalDroneArray(currentObjective.droneIndex, currentObjective.wepDistType, currentObjective.wepPropType,currentObjective.formation);
}
function OnTakeDamage(args) {
    //NpcLib.StandardOnTakeDamage(SHIP_ID, args);
}

function OnFrame(args) {

}

/*function OnTakeDamage(args)
{
    NpcLib.StandardOnTakeDamage(SHIP_ID, args);
}*/

function OnUpdateCache(args) {

}

/*function OnCalculateShipParameters(args)
{
    console.Print("OnCalculateShipParameters");
}

function OnCalculateInventoryCache(args) {
    console.Print("OnCalculateInventoryCache");
}

function OnCalculateSkillsCache(args) {
    console.Print("OnCalculateSkillsCache");
}

function OnCalculateBuffsCache(args) {
    console.Print("OnCalculateBuffsCache");
}
*/

/*function OnDied(args) {

}

function OnFrame(args)
{

}

function OnFinished(args) {

}

function OnCancel(args) {

}

function OnTakeDamage(args) {

}*/
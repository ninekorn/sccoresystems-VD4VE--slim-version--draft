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

include(SC_AI_Drone_Manager_Utilities.js);
include(SC_AI_Drone_Repair_Core_2.js);

function OnAIInited() {
    npc.SetDecisionsPerSecond(SHIP_ID, 5);

    npc.SetBehavior(SHIP_ID, "avoid_asteroids", true);
    npc.SetBehavior(SHIP_ID, "avoid_ships", true);
    npc.SetBehavior(SHIP_ID, "avoid_bases", true);
    npc.SetBehavior(SHIP_ID, "avoid_debris", true);

    //NpcLib.SetupDefaultWeaponUsage(SHIP_ID);
    npc.SetWeaponUsage(SHIP_ID, "Ray", 0, 7, 0.001);
    npc.SetWeaponUsage(SHIP_ID, "BulletOrMissile", 2, 7, 0.001);
    npc.SetWeaponUsage(SHIP_ID, "DirectHit", 2, 7, 0.001);
}

var player_id;
var initStartingStuff;
var droneIndex;
var playerName;

function Decision(args)
{
	player_id = npc.GetTag(SHIP_ID, "ownerPlayerShipId");

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
            SC_AI_Drone_Repair_Core_2.AICoreInit(SHIP_ID, droneIndex, playerName, player_id, args);
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
    SC_AI_Drone_Manager_Utilities.setRemovedFromGlobalDroneArrayRepair(currentObjective.droneIndex, currentObjective.wepDistType, currentObjective.wepPropType,currentObjective.formation);
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
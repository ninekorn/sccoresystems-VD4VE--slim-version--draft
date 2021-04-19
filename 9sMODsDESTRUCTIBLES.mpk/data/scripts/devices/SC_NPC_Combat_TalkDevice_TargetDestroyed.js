/*
 Script can contain following functions:
 OnUpdateCache - called when device is equipped or initialized, or some values are changed
 OnStart - called when device is activated (for all)
 OnFrame - called every frame (only for "per frame" mode)
 OnFinished - called when device effect should be applied (only for "per frame" and "on complete")
 */

using(ship);
using(console);
using(game);

function OnUpdateCache(args)
{
    //no updates required
}

function OnStart(args)
{
	//args.ship_id, 
	//args.slot_id
	//args.custom_parameters.required_energy
	//args.cooldown

    //ship.DeviceDeactivate(args.ship_id, args.slot_id, true);
    //game.SendNotificationError(game.GetShipOwner(args.ship_id), $n0009, $n0010, args.custom_parameters.required_fraclasite); // Repair failed: Not enough ore in cargo to use in repairs. Required %amount% fraclasite.

   //ship.AddBuff(args.ship_id, "consumables_structure_repair_value", args.custom_parameters.repair_value, args.custom_parameters.repair_time);
   //default cooldown - 60 seconds
    //args.cooldown = 60;

    //game.ShipPlaySound(args.ship_id, "SC_NPC/SC_NPC_Combat_TalkDevice_TargetDestroyed.ogg", 0, false);
	//game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + args.target_id, "special/mining_start.ogg", 0, false);
	//game.ShipPlaySound(args.ship_id, "SC_NPC/SC_NPC_TalkDevice/SC_NPC_Combat_TalkDevice_TargetDestroyed.ogg", 0, false);
}

function OnFrame(args)
{
		game.ShipPlaySound(args.ship_id, "SC_NPC/SC_NPC_Combat_TalkDevice_TargetDestroyed.ogg", 0, false);
}

function OnFinished(args)
{
	
}

function OnCancel(args)
{
}
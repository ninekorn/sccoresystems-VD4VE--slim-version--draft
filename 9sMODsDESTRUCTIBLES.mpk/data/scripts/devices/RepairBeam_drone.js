/*
 Script can contain following functions:
 OnUpdateCache - called when device is equipped or initialized, or some values are changed
 OnStart - called when device is activated (for all)
 OnFrame - called every frame (only for "per frame" mode)
 OnFinished - called when device effect should be applied (only for "per frame" and "on complete")
 */

using(ship);
using(console);
using(generator);
using(game);
using(visual);
using(actions);
using(npc);

function OnUpdateCache(args)
{
    //console.Print("Healing beam: duration is " + args.duration);
    //console.Print("Healing beam: range now is " + args.range);
    //console.Print("Healing beam: heal per second now is " + args.custom_parameters.heal_per_second_percents);
    //console.Print("Healing beam: energy per second now is " + args.custom_parameters.energy_per_second);
}

function OnStart(args)
{
    console.Print("repair module started");

    //calculate max_length property value based on speed and asteroid's difficulty

    var required_energy = args.custom_parameters.energy_per_second;
    var current_energy = ship.GetCurrentValue(args.ship_id, "energy");


    if (generator.ShipExists(args.target_id))
    {
        var current_structure = ship.GetCurrentValue(args.target_id, "structure");
        var max_structure = ship.GetFinalCacheValue(args.target_id, "structure");

        //checking if repair is needed at all
        if (current_structure >= max_structure) {
            //ship.DeviceDeactivate(args.ship_id, args.slot_id, true);
            //game.SendNotificationError(game.GetShipOwner(args.ship_id), $n0015, $n0016); // Repair is not required: Structure is at maximum level.
            return;
        }

        /*//check if player has required energy
        if (current_energy < required_energy)
        {
            //ship.DeviceDeactivate(args.ship_id, args.slot_id, true);
            //game.SendNotificationError(game.GetShipOwner(args.ship_id), $n0017, $n0018); // Repair failed: Not enough energy.
            return;
        }*/

        args.duration = args.custom_parameters.duration;
        args.cooldown = args.duration;

        visual.DeviceActivateEffectOnObject(args.ship_id, args.device_id, "healing_visual_effect_drone", args.duration, args.target_id);

        //game.ShipPlaySound(args.ship_id, "healing_start_" + args.slot_id + "_" + args.target_id, "special/mining_start.ogg", 0, false);
        //game.ShipPlaySound(args.ship_id, "healing_process_" + args.slot_id + "_" + args.target_id, "special/grappler_process.ogg", 0.1, true);

        //game.ShipPlaySound(args.ship_id, "healing_start_" + args.target_id, "special/mining_start.ogg", 0, false);
        //game.ShipPlaySound(args.ship_id, "healing_process_" + args.target_id, "special/mining_process.ogg", 0.1, false);


        game.ShipPlaySound(args.ship_id, "healing_start_" , "special/mining_start.ogg", 0, false);
        game.ShipPlaySound(args.ship_id, "healing_process_", "special/mining_process.ogg", 0.1, false);

    }


}

function OnFrame(args)
{
    var energy = args.custom_parameters.energy_per_second * args.seconds_multiplier;
    var current_energy = ship.GetCurrentValue(args.ship_id, "energy");
	var current_structure = ship.GetCurrentValue(args.target_id, "structure");
	var max_structure = ship.GetFinalCacheValue(args.target_id, "structure");

	if (current_structure >= max_structure) 
	{
		current_structure = max_structure;
		game.ShipStopSound(args.ship_id, "healing_process_");
		npc.DeviceDeactivate(args.ship_id, "drone_device_repair_beam");
		visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "healing_visual_effect_drone");
		return;
	}



	if(game.IsShipPlayerControlled(args.target_id) || ship.IsNpc(args.target_id))
	{
		if (generator.ShipExists(args.target_id))
		{
			
			ship.SetCurrentValue(args.ship_id, "energy", current_energy - energy);
			current_structure += args.seconds_multiplier * max_structure * (args.custom_parameters.heal_per_second_percents / 100.0);
			/*if (current_structure >= max_structure) 
			{
				current_structure = max_structure;
				visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "healing_visual_effect_drone");
				game.ShipStopSound(args.ship_id, "healing_process_");
				npc.DeviceDeactivate(args.ship_id, "drone_device_repair_beam");
			}*/
			ship.SetCurrentValue(args.target_id, "structure", current_structure);
		}
	}
	else if(!game.IsShipPlayerControlled(args.target_id) && !ship.IsNpc(args.target_id))
	{
		var current_structure = ship.GetCurrentValue(args.target_id, "structure");
		var max_structure = ship.GetFinalCacheValue(args.target_id, "structure");
		ship.SetCurrentValue(args.ship_id, "energy", current_energy - energy);
		current_structure += args.seconds_multiplier * max_structure * (args.custom_parameters.heal_per_second_percents / 100.0);
		/*if (current_structure >= max_structure) 
		{
			current_structure = max_structure;
			//ship.DeviceDeactivate(args.ship_id, args.slot_id, true);
			//ship.DeviceDeactivate(args.ship_id, args.slot_id, true);
			visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "healing_visual_effect_drone");
			game.ShipStopSound(args.ship_id, "healing_process_");
			npc.DeviceDeactivate(args.ship_id, "drone_device_repair_beam");
		}*/
		ship.SetCurrentValue(args.target_id, "structure", current_structure);
	}








    /*if (generator.ShipExists(args.target_id) && ship.IsNpc(args.target_id) || generator.ShipExists(args.target_id) && !ship.IsNpc(args.target_id))
    {
        var current_structure = ship.GetCurrentValue(args.target_id, "structure");
        var max_structure = ship.GetFinalCacheValue(args.target_id, "structure");
        ship.SetCurrentValue(args.ship_id, "energy", current_energy - energy);
        current_structure += args.seconds_multiplier * max_structure * (args.custom_parameters.heal_per_second_percents / 100.0);
        if (current_structure >= max_structure) 
		{
            console.PrintError("testttt");
            current_structure = max_structure;
            //ship.DeviceDeactivate(args.ship_id, args.slot_id, true);
			//ship.DeviceDeactivate(args.ship_id, args.slot_id, true);
			visual.DeviceDeactivateEffect(args.ship_id, "drone_device_repair_beam", "healing_visual_effect_drone");
			npc.DeviceDeactivate(args.ship_id, "drone_device_repair_beam");
        }
        ship.SetCurrentValue(args.target_id, "structure", current_structure);
    }*/








   /* if (current_energy > energy)
    {
        ship.SetCurrentValue(args.ship_id, "energy", current_energy - energy);
				
        current_structure += args.seconds_multiplier * max_structure * (args.custom_parameters.heal_per_second_percents / 100.0);

        if (current_structure >= max_structure)
        {
            // healed
            current_structure = max_structure;
            //ship.DeviceDeactivate(args.ship_id, args.slot_id, true);
        }

        ship.SetCurrentValue(args.target_id, "structure", current_structure);
    }
    else
    {
        //ship.DeviceDeactivate(args.ship_id, args.slot_id, true);
        //game.SendNotificationError(game.GetShipOwner(args.ship_id), $n0019, $n0020); // Healing beam discharged: Energry depleted.
    }*/
}

/*function OnFinished(args)
{
    visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "healing_visual_effect_drone");

    game.ShipStopSound(args.ship_id, "healing_process_" + args.slot_id + "_" + args.target_id);
    game.ShipPlaySound(args.ship_id, "healing_complete_" + args.slot_id + "_" + args.target_id, "special/mining_complete.ogg", 0, false);
}

function OnCancel(args)
{
    visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "mining_visual_effect");

    game.ShipStopSound(args.ship_id, "healing_process_" + args.slot_id + "_" + args.target_id);
}*/

function OnFinished(args)
{
    visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "healing_visual_effect_drone");

    if (generator.ShipExists(args.target_id))
    {
        //game.ShipStopSound(args.ship_id, "healing_process_" + args.target_id);
        //game.ShipPlaySound(args.ship_id, "healing_complete_" + args.target_id, "special/mining_complete.ogg", 0, false);

        game.ShipStopSound(args.ship_id, "healing_process_" );
        game.ShipPlaySound(args.ship_id, "healing_complete_", "special/mining_complete.ogg", 0, false);

    }



}

function OnCancel(args)
{
    visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "healing_visual_effect_drone");

    if (generator.ShipExists(args.target_id))
    {
        //game.ShipStopSound(args.ship_id, "healing_process_" + args.target_id);

        game.ShipStopSound(args.ship_id, "healing_process_");
    }


}
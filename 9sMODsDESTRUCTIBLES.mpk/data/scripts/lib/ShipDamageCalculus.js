using(ship);
using(visual);
using(storage);

include(SC_Utilities.js);
include(SC_Salvage_Object_Storage.js);

var ShipDamageCalculus = {
    //----------------------------------------------
    // supported args:
    //
    // ignore_shield: bool
    // ignore_armor: bool
    //
    // returns:
    // {shield_damage, structure_damage, total_damage, armor_absorbed}
    //
    DamageShip: function(ship_id, damage_value, damage_multiplier, args)
    {
		// please note that damage_value is already premultiplied on damage_multiplier,
		// and damage_multiplier is used only for armor calculation
		
        // if (ship.IsNpc(ship_id))
        // {
			// console.Print("Hit: shipId=" + ship_id  + " damage_value: " + damage_value + "; damage_multiplier: "+ damage_multiplier);
        // }

        var res = {
            shield_damage: 0,
            structure_damage: 0,
            armor_absorbed: 0
        };

        var currentShip =
            {
                structure: ship.GetCurrentValue(ship_id, "structure"),
                max_structure: ship.GetFinalCacheValue(ship_id, "structure"),
                shield: ship.GetCurrentValue(ship_id, "shield"),
                shield_deflection: ship.GetFinalCacheValue(ship_id, "shield_deflection"),
                shield_reestablishment: ship.GetFinalCacheValue(ship_id, "shield_reestablishment"),
				// that's correct, as armor absorb damage per frame
                armor: ship.GetFinalCacheValue(ship_id, "armor") * damage_multiplier
            };


        //-------------------------------------
        // S H I E L D

        //if shield is ready, calculate shield
        if (args.ignore_shield != true && ship.IsShieldUp(ship_id))
        {
            var shield_damage = (currentShip.shield_deflection / 100.0) * damage_value;
            if (shield_damage > currentShip.shield)
            {
                shield_damage = currentShip.shield;
            }

            damage_value -= shield_damage;

            //store shield damage
            res.shield_damage = shield_damage;

            var shield = currentShip.shield - shield_damage;
            //shield down
            if (shield <= 0)
            {
                shield = 0;
                ship.SetShieldUp(ship_id, false);
            }

            //apply shield changes
            ship.SetCurrentValue(ship_id, "shield", shield);

            visual.ShieldHitEffect(ship_id, 0, args.collision_point_x, args.collision_point_y, args.shield_effect, false);
        }

        //console.PrintError(" x: " + args.collision_point_x + " y: " + args.collision_point_y);


        //---------------------------------------
        // A R M O R

        //calculate armor resistance
        if (currentShip.armor > 0 && args.ignore_armor != true)
        {
            res.armor_absorbed = Math.min(currentShip.armor, damage_value);
            damage_value -= currentShip.armor;
        }
        if (damage_value < 0)
        {
            damage_value = 0;
        } //to prevent healing

        res.structure_damage = damage_value;
        ship.SetCurrentValueDelta(ship_id, "structure", -damage_value);

        // if (ship.IsNpc(ship_id))
        // {
        // console.PrintError("Hit result: shipId=" + ship_id  
        // + "; shield damage: " + res.shield_damage
        // + "; structure_damage: " + res.structure_damage 
        // + "; armor_absorbed: " + res.armor_absorbed);
        // }



        //9sMODs - i think that stuff is mine but i didn't remember putting that there. maybe that's why salvaging wasn't working in other backups where i didn't have this script? nine... i'll check later as it might just be a second way to make sure
        //that the npc ship is destroyed.
        var scnpcdata = storage.GetGlobal("destroyed_" + ship_id);

        if (scnpcdata == null) {
            scnpcdata = { swtchdestroyed: 1 };
            storage.SetGlobal("destroyed_" + ship_id, scnpcdata);
        }
        //9sMODs - i think that stuff is mine but i didn't remember putting that there. maybe that's why salvaging wasn't working in other backups where i didn't have this script? nine...

        return res;
    }

};






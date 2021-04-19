//------------------------------------------------------------
// Name: Calculate ship script
//
// Desc: this script calculates ship's parameters, based on it's
//		 equipped items, passive skills and buffs.
//

include(IMCache.js);
include(ShipUpdate.js);

using(generator);
using(relations);
using(spawn);
using(actions);
using(ship);
using(console);
using(game);

function OnInit()
{
    actions.Bind("OnCalculateInventoryCache", "CalculateInventoryCache");
    actions.Bind("OnCalculateSkillsCache", "CalculateSkillsCache");
    actions.Bind("OnCalculateBuffsCache", "CalculateBuffsCache");
    actions.Bind("OnCalculateShipParameters", "CalculateShipParameters");
    actions.Bind("OnMaximizeShipValues", "MaximizeShipValues");
    actions.Bind("OnUpdateShipParameters", "UpdateShipParams");
    actions.Bind("OnCalculateNpcLeveledCache", "CalculateNpcLeveledCache");
    actions.Bind("OnShipWarps", "OnShipWarpsHandler");
}

// Name: CalculateNpcLeveledCache
// Desc: calculates npc cache, based on current npc level
function CalculateNpcLeveledCache(args)
{
    IMCache.Clear();

    var level = args.level;
    var maxlevel = 100;
    var lerp = 1 - (maxlevel - level) / maxlevel;

    //hull & booster


    if (ship.GetTag(args.ship_id, "someClass") != null)
    {
        IMCache.AddValue("structure_value", 400 + lerp * 2000);
        IMCache.AddValue("armor_value", 0 + lerp * 25);
        if (ship.GetTag(args.ship_id, "someClass") == "drone")
        {
            /*//engine
            IMCache.AddValue("speed_max_value", 400 + lerp * 200);
            IMCache.AddValue("acceleration_value", 175 + lerp * 200);
            IMCache.AddValue("maneuvering_value", 500 + lerp * 300);
            IMCache.AddValue("speed_reverse_special", 0.1);
            IMCache.AddValue("speed_strafe_special", 2.25);
            IMCache.AddValue("deceleration_special", 1);*/

            //GOOD STRAFE SPEED - WAY TOO FAST FORWARD
            /*IMCache.AddValue("speed_max_value", 500 + lerp * 200); // 
            IMCache.AddValue("acceleration_value", 450 + lerp * 200); //325 is still too high // 225 is good but the push is too big still... no continuous "engine trail"
            IMCache.AddValue("maneuvering_value", 500 + lerp * 300);
            IMCache.AddValue("speed_reverse_special", 0.1);
            IMCache.AddValue("speed_strafe_special", 2.25);
            IMCache.AddValue("deceleration_special", 1);*/


            /*//engine
            IMCache.AddValue("speed_max_value", 400 + lerp * 200);
            IMCache.AddValue("acceleration_value", 175 + lerp * 200);
            IMCache.AddValue("maneuvering_value", 500 + lerp * 300);
            IMCache.AddValue("speed_reverse_special", 0.1);
            IMCache.AddValue("speed_strafe_special", 2.25);
            IMCache.AddValue("deceleration_special", 1);*/


            //console.PrintError("drone");
            //engine
            IMCache.AddValue("speed_max_value", 175 + lerp * 100); //500//200//135
            IMCache.AddValue("acceleration_value", 250 + lerp * 100); //200//200//135
            IMCache.AddValue("maneuvering_value", 800 + lerp * 100); //500//300//750/1000 is a bit much for 550xml speed and npc.Stop is making the drone completely stop.
            IMCache.AddValue("speed_reverse_special", 3.5); //1.75
            IMCache.AddValue("speed_strafe_special", 7.5 + lerp * 150);  //1.75
            IMCache.AddValue("deceleration_special", 12.5 + lerp * 150);  //1.75
        }
    }


    //ship.GetTag(args.ship_id, "someTag") == "stationREPAIRLOW" ||
    //ship.GetTag(args.ship_id, "someTag") == "stationREPAIRHIGH"

    else if (ship.GetTag(args.ship_id, "someTag") == "stationEXT" ||
        ship.GetTag(args.ship_id, "someTag") == "stationINT")
    {
        IMCache.AddValue("structure_value", 99999 + lerp * 2000);
        IMCache.AddValue("armor_value", 0 + lerp * 25);
        //generator
        IMCache.AddValue("energy_production_generator_value", 99999);
        IMCache.AddValue("energy_capacity_value", 99999);

        //shields
        IMCache.AddValue("shield_charge_value", -1000);
        IMCache.AddValue("shield_regeneration_value", -1000);
        IMCache.AddValue("shield_deflection_value", 0);
        IMCache.AddValue("shield_reestablishment_value", -1000);
        IMCache.AddValue("shield_regeneration_delay_value", -1000);
        IMCache.AddValue("energy_consumption_shield_value", -1000);

        //engine
        //IMCache.AddValue("maneuvering_value", 400 + lerp * 300);
    }
    else {
        IMCache.AddValue("structure_value", 400 + lerp * 2000);
        IMCache.AddValue("armor_value", 0 + lerp * 25);
        //engine
        IMCache.AddValue("speed_max_value", 500 + lerp * 200);
        IMCache.AddValue("acceleration_value", 200 + lerp * 200);
        IMCache.AddValue("maneuvering_value", 500 + lerp * 300);
        IMCache.AddValue("speed_reverse_special", 0.5);
        IMCache.AddValue("speed_strafe_special", 0.8);
        IMCache.AddValue("deceleration_special", 1);
    }





    //generator
    IMCache.AddValue("energy_production_generator_value", 25 + lerp * 100);
    IMCache.AddValue("energy_capacity_value", 250 + lerp * 1000);

    //shields
    IMCache.AddValue("shield_charge_value", 250 + lerp * 500);
    IMCache.AddValue("shield_regeneration_value", 10 + lerp * 65);
    IMCache.AddValue("shield_deflection_value", 80);
    IMCache.AddValue("shield_reestablishment_value", 100);
    IMCache.AddValue("shield_regeneration_delay_value", 1 + 2 * (1 - lerp));
    IMCache.AddValue("energy_consumption_shield_value", 5 + lerp);

    //targeting
    IMCache.AddValue("radar_targeting_range_value", 20 + lerp * 10); //20-30
    IMCache.AddValue("radar_targeting_speed_value", 400 + lerp * 850); //400-1250
    IMCache.AddValue("radar_targeting_defense", 800 + lerp * 1200);

    // set constants
    //IMCache.AddValue("weapon_tracking_speed_value", 1);
    //IMCache.AddValue("weapon_tracking_angle_value", 1);

    IMCache.AddValue("mining_amount_value", 1);
    IMCache.AddValue("mining_range_value", 1);
    IMCache.AddValue("mining_speed_value", 1);

    //set resulting value	
    IMCache.ApplyCacheFunction(args.ship_id,
        function(ship_id, param_name, param_value)
        {
            ship.SetEquipmentCacheValue(ship_id, param_name, param_value);
        });
}

// Name: CalculateShipParameters
// Desc: resulting calculation, combines cache
function CalculateShipParameters(args)
{
    var shipId = args.ship_id;
    var allEffects = ship.GetAllEffects();
    var defaults = ship.GetDefaultEffects();
    var invCache = ship.GetEquipmentCache(shipId);
    var skillCache = ship.GetSkillsCache(shipId);
    var buffsCache = ship.GetBuffsCache(shipId);

    IMCache.Clear();

    var isPlayer = !ship.IsNpc(shipId);

    for (var i = 0; i < allEffects.length; i++)
    {
        var effect = allEffects[i];
        switch (effect.type)
        {
            case "Calculate":
                ValueWithPercentParameterCalculation(effect.id, invCache, skillCache, buffsCache, defaults);
                break;

            case "Special":
                SimpleParameterCalculation(effect.id, effect.id + "_special", invCache, skillCache, buffsCache, defaults);
                break;

            case "Instance":
                InstanceParameterCalculation(effect.id, invCache, skillCache, buffsCache, defaults);
                break;

            case "Ability":
                if (isPlayer)
                {
                    SimpleParameterCalculation(effect.id, effect.id + "_ability", invCache, skillCache, buffsCache, defaults);
                }
                break;

            default:
                console.Print("Unknown 'calculate' effect: id=" + effect.id + " calculation=" + effect.calculation);
                continue;
        }
    }

    // set constants
    // IMCache.AddValue("weapon_tracking_speed", 1);
    // IMCache.AddValue("weapon_tracking_angle", 1);

    //set resulting value
    IMCache.ApplyCacheFunction(shipId,
        function(ship_id, param_name, param_value)
        {
            ship.SetFinalCacheValue(ship_id, param_name, param_value);
            //console.Print(shipId+" ship: FCACHE{ "+param_name+"; "+param_value + "}");
        });

    //console.Print("CalculateShipParameters succeeded");	
}

function SimpleParameterCalculation(param_name, key, invCache, skillCache, buffsCache, defaults)
{
    var v = defaults[key];

    if (v != undefined)
	{
        IMCache.AddValue(param_name, v);
    }
    else
    {
        IMCache.AddValue(param_name, 0);
    }

    v = invCache[key];
    if (v != undefined)
    {
        IMCache.AddValue(param_name, v);
    }

    v = skillCache[key];
    if (skillCache[key] != undefined)
    {
        IMCache.AddValue(param_name, v);
    }

    v = buffsCache[key];
    if (v != undefined)
    {
        IMCache.AddValue(param_name, v);
    }
}

function InstanceParameterCalculation(param_name, invCache, skillCache, buffsCache, defaults)
{
	/*if(param_name.indexOf("level") >= 0)
	{
		console.PrintError("*****TESTESTESTEESTETETETEES00**********");
	}*/
    SimpleParameterCalculation(param_name + "_instancevalue", param_name + "_instancevalue", invCache, skillCache, buffsCache, defaults);
    SimpleParameterCalculation(param_name + "_instancepercent", param_name + "_instancepercent", invCache, skillCache, buffsCache, defaults);
}

function ValueWithPercentParameterCalculation(param_name, invCache, skillCache, buffsCache, defaults)
{
    // implements one of default logics of calculation:
    // resulting parameter = sum of this parameter values * sum of this parameters percent bonuses

	/*if(param_name.indexOf("level") >= 0)
	{
		console.PrintError("*****TESTESTESTEESTETETETEES01**********");
	}*/

    var key_value = param_name + "_value";
    var key_percent = param_name + "_percent";

    // values
    var v = defaults[key_value];
    if (v != undefined)
    {
        IMCache.AddValue(param_name, v);
    }
    else
    {
        IMCache.AddValue(param_name, 0);
    }

    v = invCache[key_value];
    if (v != undefined)
    {
        IMCache.AddValue(param_name, v);
    }

    v = skillCache[key_value];
    if (v != undefined)
    {
        IMCache.AddValue(param_name, v);
    }

    v = buffsCache[key_value];
    if (v != undefined)
    {
        IMCache.AddValue(param_name, v);
    }

    // percents
    v = invCache[key_percent];
    if (v != undefined)
    {
        IMCache.MulValue(param_name, (100 + v) / 100.0);
    }

    v = skillCache[key_percent];
    if (v != undefined)
    {
        IMCache.MulValue(param_name, (100 + v) / 100.0);
    }

    v = buffsCache[key_percent];
    if (v != undefined)
    {
        IMCache.MulValue(param_name, (100 + v) / 100.0);
    }
}

// Name:
// Desc:
function MaximizeShipValues(args)
{	
    // if user added some of his own values, he must implement here it's maximizing,
    // something like this:
    //
    // var val1 = ship.GetFinalCacheValue( args.ship_id, "base_charisma" );
    // var val2 = ship.GetFinalCacheValue( args.ship_id, "base_charm" );
    // ship.SetCurrentValue( args.ship_id, "charisma", val1 + val2);
    //

    //console.Print("...maximizing values");
}

// Name: UpdateShipParams
// Desc:
function UpdateShipParams(args)
{
    var isNpc = ship.IsNpc(args.ship_id);





	/*if(!isNpc)
	{	
		var someLevel = ship.GetCurrentValue(args.ship_id, "level");

		if(someLevel!= null)
		{
			console.PrintError("***************");
			console.PrintError(someLevel);
		}
	}*/
	



    var timeMultiplier = args.seconds_multiplier; //this is because this function isn't called every frame, in args is accumulated value
    var ship_id = args.ship_id;
    var energy_base = ship.GetCurrentValue(ship_id, "energy");
    var energy_increase = ship.GetFinalCacheValue(ship_id, "energy_production_generator") * timeMultiplier;
    var energy_free_increase = ship.GetFinalCacheValue(ship_id, "consumables_energy_recharge") * timeMultiplier;
    var energy_max = ship.GetFinalCacheValue(ship_id, "energy_capacity");
    var energy = energy_base + energy_increase + energy_free_increase;

    // fuel
    /*if (!isNpc)
    {
        var fuel = ship.GetCurrentValue(ship_id, "fuel");
        if (fuel <= 0)
        {
            energy = energy_base + energy_free_increase; // NO energy_increase, as there are no more fuel for generator
        }

        // engine
        if (fuel > 0 && ship.IsEngineOn(ship_id))
        {
            var engine_consumption_per_frame = ship.GetFinalCacheValue(ship_id, "energy_consumption_engine") * timeMultiplier;
            energy -= engine_consumption_per_frame;
        }

        // radar
        var radar_consumption_per_frame = ship.GetFinalCacheValue(ship_id, "energy_consumption_radar") * timeMultiplier;
        energy -= radar_consumption_per_frame;
    }*/

    // devices
    // ! important !
    // devices' energy consuming (and all other stuff about devices) is calculated in
    // device's script
    energy -= ship.GetFinalCacheValue(ship_id, "energy_consumption_generic") * timeMultiplier;

    // grappling
    if (!isNpc)
    {
        var grappled_targets_count = ship.GetGrappledCount(ship_id);
        if (grappled_targets_count > 0)
        {
            var grappler_consumption_per_frame = ship.GetFinalCacheValue(ship_id, "energy_consumption_grappler") * timeMultiplier;
            energy -= grappled_targets_count * grappler_consumption_per_frame;
            if (energy <= 0)
            {
                energy = 0;
                ship.ReleaseGrappler(ship_id);
            }
        }
    }

    // try to regenerate shields
    var shield_max = ship.GetFinalCacheValue(ship_id, "shield_charge");
    if (shield_max <= 0) //if there's no shield
    {
        // Shield up
        if (ship.IsShieldUp(ship_id))
        {
            ship.SetShieldUp(ship_id, false);
        }
    }
    else
    {
        var regen_delay = ship.GetFinalCacheValue(ship_id, "shield_regeneration_delay");
        var shield_regenerated = 0;

        if (ship.GetTimeSinceLastHit(ship_id) > regen_delay)
        {
            //call library function
            var result = ParamUpdater.CalculateShield({
                energy: energy,
                energy_max: energy_max,
                shield_max: shield_max,
                shield_current: ship.GetCurrentValue(ship_id, "shield"),
                shield_regeneration_per_frame: ship.GetFinalCacheValue(ship_id, "shield_regeneration") * timeMultiplier,
                shield_regeneration_per_frame_cost: ship.GetFinalCacheValue(ship_id, "energy_consumption_shield") * timeMultiplier
            });

            shield_regenerated = result.shield_regenerated;
            energy -= result.energy_spent;
        }

        //active effects contribution
        shield_regenerated += ship.GetFinalCacheValue(ship_id, "consumables_shield_restore") * timeMultiplier;
        if (shield_regenerated > 0)
        {
            ship.SetCurrentValueDelta(ship_id, "shield", shield_regenerated);
        }

        // Shield up
        if (!ship.IsShieldUp(ship_id))
        {
            var curshield = ship.GetCurrentValue(ship_id, "shield");
            var reestablishment = ship.GetFinalCacheValue(ship_id, "shield_reestablishment");

            if (curshield >= reestablishment)
            {
                ship.SetShieldUp(ship_id, true);
            }
        }
    }

    // try to regenerate weapons
    if (!isNpc)
    {
        var energy_spent_of_weapons = ship.RegenerateWeapons(ship_id, energy, timeMultiplier);
        energy -= energy_spent_of_weapons;
    }

    // set energy itself
    if (!isNpc)
    {
        var energy_wasted = 0;

        if (energy > energy_max)
        {
            energy_wasted = energy - energy_max;
            energy = energy_max;
        }

        var coef = (energy_increase - energy_wasted) / energy_increase;
        if (coef > 0)
        {
            var fuel_consumption = ship.GetFinalCacheValue(ship_id, "fuel_consumption_generator") / 60.0 * timeMultiplier * coef;

            if (fuel_consumption > 0.0001)
            {
                ship.SetCurrentValueDelta(ship_id, "fuel", -fuel_consumption);
            }
        }
    }

    ship.SetCurrentValue(ship_id, "energy", energy);

    // if you want to get know how much energy was consumpted on this frame by player(s) ship(s) uncomment this. Warning - it creates unneeded extra load!
    /*if (!isNpc)
     {
     console.Print("Current energy production-consumption delta per second: " 
     + ((energy - energy_base) / (timeMultiplier))
     + " current energy_production per second: " + ((energy_increase + energy_free_increase) / timeMultiplier));
     }*/

    // OTHER REGENERATIONS

    if (!ship.IsCurrentValueMaximum(ship_id, "structure"))
    {
        var structure_regenerated = ship.GetFinalCacheValue(ship_id, "hull_regeneration") * timeMultiplier;
        structure_regenerated += ship.GetFinalCacheValue(ship_id, "consumables_structure_repair") * timeMultiplier;

        if (structure_regenerated > 0)
        {
            ship.SetCurrentValueDelta(ship_id, "structure", structure_regenerated);
        }
    }
}

// Should be called every time ship warps
function OnShipWarpsHandler(args)
{
    var ship_id = args.ship_id;
    var energy_base = ship.GetCurrentValue(ship_id, "energy");
    var warp_cost = ship.GetFinalCacheValue(ship_id, "energy_consumption_warp");

    if (warp_cost > energy_base)
    {
        return { allow: false };
    }
    else
    {
        //spend energy
        ship.SetCurrentValueDelta(ship_id, "energy", -warp_cost);
        return { allow: true };
    }
}

// Name: CalculateInventoryCache
// Desc: calculate inventory items base parameters
function CalculateInventoryCache(args)
{
    var items = ship.GetEquipment(args.ship_id);
    IMCache.Clear();

    //no active or percentage effects here
    for (var i = 0; i < items.length; i++)
    {
        var effects = game.GetItemEffects(game.GetItemSID(items[i]));

        for (var j = 0; j < effects.length; j++)
        {
            var increase = effects[j].effect_base;
            IMCache.AddValue(effects[j].effect_type, increase);
        }
    }

    //set resulting value
    ship.ClearEquipmentCache(args.ship_id);

    IMCache.ApplyCacheFunction(args.ship_id,
        function(ship_id, param_name, param_value)
        {
            ship.SetEquipmentCacheValue(ship_id, param_name, param_value);
        });

    //console.Print("CalculateInventoryCache succeeded");	
}

// Name: CalculateSkillsCache
// Desc: calculate passive skills influence
function CalculateSkillsCache(args)
{
    if (!game.IsShipPlayerControlled(args.ship_id))
    {
        return;
    }

    var skills = ship.GetSkills(args.ship_id);
    var defaults = ship.GetDefaultAbilities();
    IMCache.Clear();

    //possible percentage and other active effects
    for (var i = 0; i < skills.length; i++)
    {
        var effects = game.GetSkillEffects(skills[i].id);
        var level = skills[i].level;

        for (var j = 0; j < effects.length; j++)
        {
            var value = effects[j].effect_base + level * effects[j].effect_per_level;
            IMCache.AddValue(effects[j].effect_type, value);
        }
    }

    var keys = Object.keys(defaults);
    //console.PrintError("Update skills cache. Keys count: " + keys.length);
    for (var i = 0; i < keys.length; i++)
    {
        var effect_type = keys[i];
        var value = defaults[effect_type];
        IMCache.AddValue(effect_type, value);
    }

    //set resulting value
    ship.ClearSkillsCache(args.ship_id);

    IMCache.ApplyCacheFunction(args.ship_id,
        function(ship_id, param_name, param_value)
        {
            ship.SetSkillsCacheValue(ship_id, param_name, param_value);
        });

    //console.Print("CalculateSkillsCache succeeded");	
}

// Name: CalculateBuffsCache
// Desc: Calculates active effects cache.
function CalculateBuffsCache(args)
{
    var effects = ship.GetBuffs(args.ship_id);

    IMCache.Clear();

    //possible percentage and other active effects
    for (var i = 0; i < effects.length; i++)
    {
        var effect = effects[i];
        IMCache.AddValue(effect.effect_type, effect.effect_base);
    }

    //set resulting value
    ship.ClearBuffsCache(args.ship_id);

    IMCache.ApplyCacheFunction(args.ship_id,
        function(ship_id, param_name, param_value)
        {
            ship.SetBuffsCacheValue(ship_id, param_name, param_value);
        });

    //console.Print("CalculateBuffsCache succeeded");	
}
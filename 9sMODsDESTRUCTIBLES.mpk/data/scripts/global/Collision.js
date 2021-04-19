//-----------------------------------------------
// Name: Collision
// base collision detection system
//
/*
 Desc:
 
 Object type can be one of the following:
 "none" - error
 "ship", "asteroid", "base", "jumpgate", "projectile", "crate", "debris"
 */

include(ShipDamageCalculus.js);

using(generator);
using(relations);
using(spawn);
using(actions);
using(ship);
using(game);
using(console);
using(visual);
using(npc);
using(config);

var isPVP;
var DefaultCollisionDamage = 300;
var dmg_mult_min;
var dmg_mult_max_delta_min;

function OnInit()
{
    isPVP = config.GetSetting("pvp");

    var coef = game.GetNpcDamageLeveledCoefficientMin();
    dmg_mult_min = coef.min;
    dmg_mult_max_delta_min = coef.max - coef.min;

    //create custom events
    actions.Create("OnProjectileHitShip");
    actions.Create("OnDebrisCollided");

    //bindings
    actions.Bind("OnCollision", "OnCollisionHandler");
    actions.Bind("OnHit", "OnHitHandler");
    actions.Bind("OnRayHitProjectile", "OnRayHitProjectileHandler");

    actions.Bind("OnProjectileHitShip", "OnProjectileHitShipHandler");
    actions.Bind("OnShipDestroyed", "OnShipDestroyedHandler");
}

// Function called on collisions, excluding projectile collisions!
// args:
// object_type_1
// object_type_2
// object_id_1
// object_id_2
// vector_x
// vector_y
// impulse
// collision_point_x
// collision_point_y
function OnCollisionHandler(args)
{
    //console.Print("collided " + args.object_type_1 + " " + args.object_type_2);
    var ship1_is_npc = args.object_type_1 != "ship" || ship.IsNpc(args.object_id_1);
    var ship2_is_npc = args.object_type_2 != "ship" || ship.IsNpc(args.object_id_2);

    // A S T E R O I D S
    if (args.object_type_1 == "ship" && args.object_type_2 == "asteroid")
    {
        OnShipHitSomething(args.object_id_1, DefaultCollisionDamage, !ship1_is_npc, args);
    }
    else if (args.object_type_2 == "ship" && args.object_type_1 == "asteroid")
    {
        OnShipHitSomething(args.object_id_2, DefaultCollisionDamage, !ship2_is_npc, args);
    }

    // D E B R I S
    if (args.object_type_1 == "ship" && args.object_type_2 == "debris")
    {
        OnShipHitSomething(args.object_id_1, DefaultCollisionDamage, !ship1_is_npc, args);
    }
    else if (args.object_type_2 == "ship" && args.object_type_1 == "debris")
    {
        OnShipHitSomething(args.object_id_2, DefaultCollisionDamage, !ship2_is_npc, args);
    }

    if (args.object_type_2 == "debris")
    {
        DebrisCollided(args.object_id_2, args.object_type_1, args.object_id_1, args);
    }
    else if (args.object_type_1 == "debris")
    {
        DebrisCollided(args.object_id_1, args.object_type_2, args.object_id_2, args);
    }

    if (args.object_type_1 == "ship" && args.object_type_2 == "jumpgate")
    {
        OnShipHitSomething(args.object_id_1, DefaultCollisionDamage, false, args);
    }
    else if (args.object_type_2 == "ship" && args.object_type_1 == "jumpgate")
    {
        OnShipHitSomething(args.object_id_2, DefaultCollisionDamage, false, args);
    }

    if (args.object_type_1 == "ship"
        && (args.object_type_2 == "base"
            || args.object_type_2 == "special_object"))
    {
        OnShipHitSomething(args.object_id_1, DefaultCollisionDamage, !ship1_is_npc, args);
    }
    else if (args.object_type_2 == "ship"
        && (args.object_type_1 == "base"
            || args.object_type_1 == "special_object"))
    {
        OnShipHitSomething(args.object_id_2, DefaultCollisionDamage, !ship2_is_npc, args);
    }

    if (args.object_type_1 == "ship" && args.object_type_2 == "crate")
    {
        OnShipHitSomething(args.object_id_1, DefaultCollisionDamage, !ship1_is_npc, args);
    }
    else if (args.object_type_2 == "ship" && args.object_type_1 == "crate")
    {
        OnShipHitSomething(args.object_id_2, DefaultCollisionDamage, !ship1_is_npc, args);
    }

    if (args.object_type_1 == "ship" && args.object_type_2 == "ship")
    {
		var ship1 = args.object_id_1;
		var ship2 = args.object_id_2;
		var is_do_damage_structure = !ship1_is_npc || !ship2_is_npc;

		if(npc.GetTag(ship1, "ownerPlayerShipId") != null && npc.GetTag(ship1, "ownerPlayerShipId") == ship2||
		   npc.GetTag(ship2, "ownerPlayerShipId") != null && npc.GetTag(ship2, "ownerPlayerShipId") == ship1 ||
		   npc.GetTag(ship2, "ownerPlayerShipId") != null && npc.GetTag(ship1, "ownerPlayerShipId") != null &&
		   npc.GetTag(ship2, "ownerPlayerShipId") == npc.GetTag(ship1, "ownerPlayerShipId"))
		{
			//console.PrintError("owner collision");
			OnShipHitSomething(ship2,0, false, args,1);
			OnShipHitSomething(ship1, 0, false, args,1);
			return;
		}
		else
		{
			// colliding NPC's do zero damage to their structure
					
			var ship_1_was_alive = ship.GetCurrentValue(ship1, "structure") > 0;
			var ship_2_was_alive = ship.GetCurrentValue(ship2, "structure") > 0;

			var damage_mult_from_ship1 = 1 + ship.GetFinalCacheValue(ship1, "collision_damage_target");
			var damage_mult_from_ship2 = 1 + ship.GetFinalCacheValue(ship2, "collision_damage_target");

			// console.Print("Here is collision between ship1=" + ship1 + " and ship2=" + ship2
			// + ": damage_mult_from_ship1=" + damage_mult_from_ship1 + " damage_mult_from_ship2=" + damage_mult_from_ship2);

			ship.UpdateLastHitShipID(ship1, ship2);
			ship.UpdateLastHitShipID(ship2, ship1);

			OnShipHitSomething(ship2, DefaultCollisionDamage * damage_mult_from_ship1, is_do_damage_structure, args,0);
			OnShipHitSomething(ship1, DefaultCollisionDamage * damage_mult_from_ship2, is_do_damage_structure, args,0);
		}        
    }
}


// Function called on projectile collisions
// args:
// object_id
// object_type
// projectile_type (bullet, ray)
// caster_id
// weapon_type_id
// projectile_id
// vector_x
// vector_y
// collision_point_x
// collision_point_y
function OnHitHandler(args)
{
    //console.Print("OnHit! " + args.object_type + " with " + args.weapon_type_id + " " + args.projectile_id+", "+args.projectile_type);

    if (args.object_type == "ship")
    {
        actions.Start("OnProjectileHitShip", args);
    }
}

// args:
// projectile_id
// projectile_caster_id
// projectile_weapon_type_id
// ray_projectile_id
// ray_caster_id
// ray_weapon_type_id
// system_id
function OnRayHitProjectileHandler(args)
{
    //get damage from ray_projectile_id
    var multiplier = game.GetSecondsMultiplier();
    var projectile = game.GetProjectileDescription(args.projectile_weapon_id, args.projectile_index);
    var damage_value = projectile.damage * multiplier;

    game.DamageProjectile(args.system_id, args.projectile_object_id, damage_value);
}

//---------------------------------------------------
// calculates damage
//
//---------------------------------------------------
var frags = {};

function OnProjectileHitShipHandler(args)
{
    var shipId = args.object_id;
    var caster_is_npc = ship.IsNpc(args.caster_id);

    if (isPVP == "0") // if pvp turned off and both are players
    {
        if (!caster_is_npc && !ship.IsNpc(shipId))
        {
            return;
        }
    }

	var damage_multiplayer = args.multiplier;
    var projectile = game.GetProjectileDescription(args.weapon_id, args.projectile_index);
    var damage_value = projectile.damage * damage_multiplayer;
    var shield_effect = 1;

    if (projectile.damage_type == "Explosion")
    {
        damage_value *= args.hit_coef;
    }
    else if (args.distance > args.projectile_distance)
    {
        if (args.distance > args.projectile_max_distance)
        {
            damage_value = 0;
        }
        else
        {
            // damage-distance fade out
            damage_value *= 1 - (args.distance - args.projectile_distance) / (args.projectile_max_distance - args.projectile_distance);
        }
    }

    // calculate damage and stuff
    if (args.projectile_type == "Ray")
    {
        shield_effect = 0.2;
    }

    if (caster_is_npc)
    {
        // reduce damage by NPC according to its level
        var dmg_mult = GetDamageMultiplierForNpc(args.caster_id);
        damage_value *= dmg_mult;
        // console.Print("DMG multiplier for NPC is: " + dmg_mult);
    }

    // call library function
    var res = ShipDamageCalculus.DamageShip(shipId, damage_value, damage_multiplayer,
        {
            collision_point_x: args.collision_point_x,
            collision_point_y: args.collision_point_y,
            shield_effect: shield_effect
        });

    if (!caster_is_npc 
		&& (res.shield_damage + res.structure_damage) >= 5000)
    {
        game.GiveAchievement(args.caster_id, "deal_5000_damage");
    }

    ship.SetDamageByShip(shipId, res.structure_damage, args.caster_id);

    if (projectile.damage_type == "Explosion")
    {
        // apply force
        // var ship_coord = ship.GetCoordinates(shipId);
        var force = { x: -args.vector_x * args.impulse * args.hit_coef, y: -args.vector_y * args.impulse * args.hit_coef };
        game.ApplyForceToObject(shipId, force.x, force.y);
        // console.Print("Explosion hit: hit_coef=" + args.hit_coef + " normal: " + args.vector_x + ";" + args.vector_y + " impulse=" + args.impulse);
    }
}

function GetDamageMultiplierForNpc(npc_id)
{
    var level = ship.GetLevel(npc_id);
    return dmg_mult_min + dmg_mult_max_delta_min * (level / 100.0);
}

//---------------------------------------------------------
// Standart processor of someone killed someone
//
//---------------------------------------------------------
function OnShipDestroyedHandler(args)
{
    if (game.IsShipPlayerControlled(args.ship_id))
    {
        actions.InvokeTrigger("onPlayerDie",
            {
                ship_id: args.ship_id,
                name: game.GetShipOwner(args.ship_id)
            });
    }
    else
    {
        actions.InvokeTrigger("onNpcDie",
            {
                killer_ship_id: args.caster_id,
                ship_id: args.ship_id,
                npc_id: game.GetShipOwner(args.ship_id)
            });
    }

	var ship1 = args.caster_id;
	var ship2 = args.ship_id;

	if(npc.GetTag(ship1, "ownerPlayerShipId") != null)
	{
		var playerName = game.GetShipOwner(npc.GetTag(ship1, "ownerPlayerShipId"));
		
		var level = ship.GetLevel(args.ship_id);
		var exp_earned = 150 + Math.pow(level, 1.3);
		ship.SetCurrentValueDelta(npc.GetTag(args.caster_id, "ownerPlayerShipId"), "experience", exp_earned);	
		//game.SendNotification(playerName, "Server notification","Experience earned: " + exp_earned );
	}
	else
	{
		var level = ship.GetLevel(args.ship_id);
		var exp_earned = 150 + Math.pow(level, 1.3);

		if (args.caster_id > 0 && game.IsShipPlayerControlled(args.caster_id))
		{
			ship.SetCurrentValueDelta(args.caster_id, "experience", exp_earned);
		}
	}
}



 

// called when debris is collided with something
function DebrisCollided(debris_id, object_type, object_id, args)
{
    actions.Start("OnDebrisCollided",
        {
            debris_id: debris_id,
            object_type: object_type,
            object_id: object_id,
            collision_point_x: args.collision_point_x,
            collision_point_y: args.collision_point_y,
            vector_x: args.vector_x,
            vector_y: args.vector_y
        }
    );
}

// standart collision damage processor
function OnShipHitSomething(ship_id, damage, is_do_damage_structure, args,isOwner)
{
	if(isOwner == 0 || isOwner == null)
	{
		var impulse = args.impulse;
		var damage_mult = 1 + ship.GetFinalCacheValue(ship_id, "collision_damage_self");
		//console.Print("Collision damage (self) multiplier for ship_id=" + ship_id + " is " + damage_mult);

		damage_mult *= impulse * game.GetSecondsMultiplier();

		if (damage_mult < 0)
		{
			damage_mult = 0;
		}

		damage *= damage_mult;

		if (ship.IsShieldUp(ship_id))
		{
			// deplete kinetics shield
			var shield = ship.GetCurrentValue(ship_id, "shield");

			//shield down
			if (shield > damage)
			{
				shield -= damage;
				damage = 0;
			}
			else
			{
				damage -= shield;
				shield = 0;
				ship.SetShieldUp(ship_id, false);
			}

			// send shield hit effect
			ship.SetCurrentValue(ship_id, "shield", shield);
			visual.ShieldHitEffect(ship_id, 0, args.collision_point_x, args.collision_point_y, args.impulse, true);
		}
		if (is_do_damage_structure && damage > 0)
		{
			ship.SetCurrentValueDelta(ship_id, "structure", -damage);
			visual.HullHitEffect(ship_id, 0, args.collision_point_x, args.collision_point_y, args.impulse, true);
		}
	}   
	else
	{
		/*if(ship.GetCurrentValue(ship_id, "shield") > 0)
		{
			visual.ShieldHitEffect(ship_id, 0, args.collision_point_x, args.collision_point_y, args.impulse, true);
		}
		else
		{
			visual.HullHitEffect(ship_id, 0, args.collision_point_x, args.collision_point_y, args.impulse, true);
		}*/		
	}
}
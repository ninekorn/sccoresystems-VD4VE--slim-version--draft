/*
 =========================================================================
 Some core functionality for essential quests and game mechanics.
 ! DO NOT MODIFY UNLESS YOU HAVE VERY GOOD REASONS TO DO SO !
 =========================================================================
 */

include(LegacyCompatibility.js);

using(actions);
using(console);
using(generator);
using(ship);
using(npc);
using(relations);
using(game);
using(timer);
using(storage);

var max_merchants_ships_in_normal_systems = 3;
var max_merchants_ships_in_capital_systems = 5;

function OnInit()
{
    /*LegacyCompatibility.RemoveOldVersionTimer("merchants_spawn_timer");

    timer.AddOrUpdate("merchants_spawn_timer", 90, "OnMerchantSpawn", {}, 0); //infinite*/
}

/*
 ==================================================
 Timer-based function
 ==================================================
 */
function OnMerchantSpawn(args)
{
    /*var systems = generator.GetPopulatedSystems();
    for (var i = 0; i < systems.length; i++)
    {
        var faction = generator.GetSystemFaction(systems[i]);

        // spawn merchants only in friendly systems
        switch (faction)
        {
            case "none":
            case "order":
            case "freedom":
            case "fanatics":
                // allowed faction
                break;
            default:
                // other factions are not allowed
                continue;
        }

        SpawnMerchant(systems[i], faction);
    }*/
}

/*
 ===================================================
 Spawn Merchant
 ===================================================
 */
function SpawnMerchant(system_id, faction, jumpgates)
{
    /*var inf = generator.GetSystemByID(system_id);

    var jumpgates = game.GetSystemJumpgates(system_id);

    //pick random jumpgate
    var jumpgate_id = utils.SelectRandom(jumpgates);
    var from_system_id = game.GetJumpgateDestinationSystem(jumpgate_id);
    var from_system_faction = generator.GetSystemFaction(from_system_id);
    if (from_system_faction == "aliens"
        || from_system_faction == "pirates")
    {
        return;
    }

    var isCapital = relations.IsCapital(system_id);
    var max_ships = isCapital
                        ? max_merchants_ships_in_capital_systems
                        : max_merchants_ships_in_normal_systems;

    var currentShipsCount = ship.CountSystemNPCShipsWithBehavior(system_id, "Trader");
    if (currentShipsCount >= max_ships)
    {
        // limit ships count
        return;
    }

    var level = inf.danger_level + MathExt.RandRange(0, 5);
    level = utils.Clamp(level, 1, 100);

    var npc_type_id;

    if (inf.tech_level < 30
        && inf.danger_level < 30)
    {
        if (MathExt.RandRange(0, 2) == 0)
        {
            npc_type_id = "special_human_tradership_endeavor";
        }
        else
        {
            npc_type_id = "special_human_tradership";
        }
    }
    else
    {
        switch (MathExt.RandRange(0, 6))
        {
            case 0:
                npc_type_id = "special_human_tradership_endeavor";
                break;

            case 1:
            case 2:
                npc_type_id = "special_human_tradership";
                break;

            case 3:
            case 4:
            case 5:
            default:
                // most probably you will see Asteria in the high tech/high danger level systems
                npc_type_id = "special_human_tradership_asteria";
                break;
        }
    }

    var coords = game.GetObjectCoordinates(system_id, jumpgate_id);

    // spawn merchant
    var id = generator.AddNPCShipToSystem($i0001, "Trader", level, npc_type_id, system_id, coords.x, coords.y, { class: "merchant", meta: "human" }); // Merchant

    console.Print("Trader spawned: shipID=" + id + " npc_type_id=" + npc_type_id + " systemID=" + system_id);

    relations.SetShipFaction(id, faction);
    ship.SetShipAsArrivedFromJumpgate(id, jumpgate_id);*/
}
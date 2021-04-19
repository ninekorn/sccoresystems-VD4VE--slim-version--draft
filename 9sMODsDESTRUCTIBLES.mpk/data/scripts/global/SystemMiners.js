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

var max_miners_ships_in_normal_systems = 3;
var max_miners_ships_in_capital_systems = 6;

function OnInit()
{
    /*LegacyCompatibility.RemoveOldVersionTimer("miners_spawn_timer");

    timer.AddOrUpdate("miners_spawn_timer", 60, "OnMinerSpawn", {}, 0); //infinite*/
}

/*
 ==================================================
 Timer-based function
 ==================================================
 */
function OnMinerSpawn(args)
{
    /*var systems = generator.GetPopulatedSystems();
    for (var i = 0; i < systems.length; i++)
    {
        var faction = generator.GetSystemFaction(systems[i]);

        switch (faction)
        {
            case "none":
            case "order":
            case "freedom":
            case "fanatics":
            case "pirates":
                // allowed faction
                break;
            default:
                // other factions are not allowed
                continue;
        }

        SpawnMiner(systems[i], faction);
    }*/
}

/*
 ===================================================
 Spawn Miner
 ===================================================
 */
function SpawnMiner(system_id, faction, jumpgates)
{
    /*var allStations = generator.GetStationsAtSystem(system_id);
    if (allStations.length == 0)
    {
        return;
    }

    var isCapital = relations.IsCapital(system_id);
    var max_ships = isCapital
                        ? max_miners_ships_in_capital_systems
                        : max_miners_ships_in_normal_systems;
    var currentShipsCount = ship.CountSystemNPCShipsWithBehavior(system_id, "TechMiner");
    if (currentShipsCount >= max_ships)
    {
        // limit ships count
        return;
    }

    var inf = generator.GetSystemByID(system_id);
    var level = inf.danger_level + MathExt.RandRange(0, 5);
    level = utils.Clamp(level, 1, 100);
    var npc_type_id = "special_human_techship";

    // spawn miners ships
    for (var i = currentShipsCount; i < max_ships; i++)
    {
        var station_id = utils.SelectRandom(allStations); // spawn miner
        var id = generator.AddNPCShipToSystem($i0001, "TechMiner", level, npc_type_id, system_id, 1000, 1000, { class: "miner", meta: "human" }); // Miner

        console.Print("Miner spawned: shipID=" + id + " npc_type_id=" + npc_type_id + " systemID=" + system_id);

        relations.SetShipFaction(id, faction);
        generator.DockShipToBase(id, station_id);
        npc.LeaveBase(id);

        switch (faction == "none")
        {
            case "pirates":
            case "fanatics":
                // spawn not more than a one miners ship per time in these factions systems
                return;

            default:
                continue;
        }
    }*/
}
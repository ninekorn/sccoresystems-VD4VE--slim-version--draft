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

include(NpcLib.js);

var max_patrol_ships_in_normal_systems = 3;
var max_patrol_ships_in_capital_systems = 5;

function OnInit()
{
    /*// npc on-the-fly adder
    actions.Bind("OnPlayerEntersSystem", "OnPlayerEntersSystemHandler");
    actions.Bind("OnShipAttacked", "OnShipAttackedHandler");

    LegacyCompatibility.RemoveOldVersionTimer("patrol_spawn_timer");
    timer.AddOrUpdate("patrol_spawn_timer", 90, "OnPatrolSpawn", {}, 0); //infinite*/
}

function OnPatrolSpawn(args)
{
    /*var systems = generator.GetPopulatedSystems();
    for (var i = 0; i < systems.length; i++)
    {
        var system_id = systems[i];
        SpawnPatrol(system_id);
    }*/
}

function SpawnPatrol(system_id)
{
    /*var factionName = generator.GetSystemFaction(system_id);
    switch (factionName)
    {
        case "none":
        case "pirates":
        case "aliens":
            // not allowed faction
            return;

        default:
            // allowed faction
            break;
    }

    var allStations = generator.GetStationsAtSystem(system_id);
    if (allStations.length == 0)
    {
        return;
    }

    var isCapital = relations.IsCapital(system_id);
    var max_ships = isCapital
        ? max_patrol_ships_in_capital_systems
        : max_patrol_ships_in_normal_systems;

    var currentShipsCount = ship.CountSystemNPCShipsWithBehavior(system_id, "Patrol");
    if (currentShipsCount >= max_ships)
    {
        // limit patrols count
        return;
    }

    var npc_type_id = "patrol_" + factionName + "_01";

    // select outfit
    var outfit = "";
    switch (factionName)
    {
        case "order":
            outfit = "military";
            break;

        case "fanatics":
            outfit = "casual";
            break;

        default:
            break;
    }

    var systemInfo = generator.GetSystemByID(system_id);

    // spawn patrol ships
    for (var i = currentShipsCount; i < max_ships; i++)
    {
        var station_id = utils.SelectRandom(allStations);
        var level = systemInfo.danger_level + 10;
        level = utils.Clamp(level, 1, 100);
        var id = generator.AddNPCShipToSystem($i0001, "Patrol", level, npc_type_id, system_id, 1000, 1000, { class: "Patrol", meta: "human", outfit: outfit }); // Patrol
        relations.SetShipFaction(id, factionName);
        generator.DockShipToBase(id, station_id);
        npc.LeaveBase(id);
        console.Print("Spawned patrol shipID=" + id + " at systemID=" + system_id + " faction " + factionName);
    }*/
}

function OnPlayerEntersSystemHandler(args)
{
    /*var system_id = args.system_id;
    if (storage.IsSet("_system_spawning", "patrol_spawn_initial_system" + system_id))
    {
        return;
    }

    SpawnPatrol(system_id);
    storage.Set("_system_spawning", "patrol_spawn_initial_system" + system_id, 1);*/
}

function OnShipAttackedHandler(args)
{
    /*var victim_ship_id = args.ship_id;
    var target_ship_id = args.caster_id;
    var max_distance = 300;

    NpcLib.LockPatrolAndTurretsOnTarget(victim_ship_id, target_ship_id, max_distance);*/
}
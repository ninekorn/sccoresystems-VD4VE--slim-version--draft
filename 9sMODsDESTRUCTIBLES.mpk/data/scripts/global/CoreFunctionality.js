/*
 =========================================================================
 Some core functionality for essential quests and game mechanics.
 ! DO NOT MODIFY UNLESS YOU HAVE A VERY GOOD REASONS TO DO SO !
 =========================================================================
 */

include(SystemsPresets.js);
include(pickByChance.js);
include(LegacyCompatibility.js);

using(actions);
using(console);
using(generator);
using(ship);
using(npc);
using(player);
using(relations);
using(game);
using(topics);
using(timer);
using(storage);
using(spawn);

var tn = "_table_system_generation";
var isSubscribedOnEveryFrameUpdate = false;

function OnInit()
{


    /*var someStartingBase = storage.GetGlobal("startingsystem");
    spawn.SetCoordinates(someStartingBase.coords.x, someStartingBase.coords.y);
    generator.DockShipToBase(player_id, someStartingBase.base_id);*/


    actions.Create("OnShipAttacked"); // it's called by NpcLib.js

    // main quests handlers and jumpgate restrictor
    actions.Bind("OnShipTryToEnterJumpgate", "OnShipTryToEnterJumpgateHandler");
    actions.Bind("OnShipDestroyed", "OnShipDestroyedHandler");
    actions.Bind("OnShipDestroyed", "OnCWaveEmitterDestroyedHandler");
    actions.Bind("OnPlayerEntersGame", "OnPlayerEntersGameHandler");

    // npc on-the-fly adder
    actions.Bind("OnPlayerEntersSystem", "OnPlayerEntersSystemHandler");

    // star system routine
    actions.Bind("OnStarSystemUpdate", "StarSystemUpdate");

    LegacyCompatibility.RemoveOldVersionTimer("guard_spawn_timer");
    LegacyCompatibility.RemoveOldVersionTimer("merchant_spawn_timer");
    LegacyCompatibility.RemoveOldVersionTimer("miner_spawn_timer");

    timer.AddOrUpdate("guard_spawn_timer", 60, "OnGuardSpawn", {}, 0); // infinite
}

function OnPlayerEntersGameHandler(args)
{
    /*var player_id = player.GetShipOfPlayer("ninekorn");
    //var someStartingBase = { sys_id: startsysdata.id, baseID: base_id, sys_name: startsysdata.sys_name, coords: bas_coord };
    var someStartingBase = storage.GetGlobal("startingsystem");
    spawn.SetCoordinates(someStartingBase.coords.x, someStartingBase.coords.y - 12.5);
    generator.DockShipToBase(args.ship_id, someStartingBase.base_id);
    //var someStartingBase = { sys_id: startsysdata.id, baseID: base_id, sys_name: startsysdata.sys_name, coords: bas_coord };
    */
    game.SendGalaxyUpdate(args.ship_id);
    storage.Set("_system_common", args.ship_id + "_completed_tutorial", 1);


    var player_id = player.GetShipOfPlayer(args.name);


    //var someStartingBase = { sys_id: startsysdata.id, baseID: base_id, sys_name: startsysdata.sys_name, coords: bas_coord };


    //var someStartingBase = storage.GetGlobal("startingsystem");
    //spawn.SetCoordinates(someStartingBase.coords.x, someStartingBase.coords.y);
    //generator.DockShipToBase(player_id, someStartingBase.base_id);








    if (!storage.IsSetGlobal("Global_Player_Coms" + args.name))
    {
        var sys_id = ship.GetSystemID(player_id);//npc.GetCurrentSystemID(player_id);
        var systemBases = game.GetSystemBases(sys_id);

        var bsinf = generator.GetBaseByID(systemBases[0]);

        console.PrintError(systemBases[0]);
        var coords = game.GetObjectCoordinates(sys_id, systemBases[0]);

        //console.PrintError(bsinf.coord.x);

        var id;
        id = generator.AddNPCShipToSystem("Comms Manager ", "sc_station_comms_systems", 5, "special_no_ship", sys_id, coords.x, coords.y, { class: "CommsManager", greeting: "terminal", unique_id: "CommsManager", player_id: player_id, ownerPlayerShipId: player_id, baseID: systemBases[0]});
        relations.SetShipFaction(id, "none");
        generator.DockShipToBase(id, systemBases[0]);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputerDialog.png");
        npc.SetHiddenInStation(id, true);
        storage.SetGlobal("Global_Player_Coms" + args.name, id);
    }



    if (!storage.IsSetGlobal("GlobalIndex_Player_" + args.name))
    {
        if (storage.IsSetGlobal("GlobalIndex_Player"))
        {
            var someGlobalIndex = storage.GetGlobal("GlobalIndex_Player");
            someGlobalIndex++;
            storage.SetGlobal("GlobalIndex_Player_" + args.name, someGlobalIndex);
            storage.SetGlobal("GlobalIndex_Player", someGlobalIndex);
        }
        else
        {
            


            var someGlobalIndex = 0;
            storage.SetGlobal("GlobalIndex_Player_" + args.name, someGlobalIndex);
            storage.SetGlobal("GlobalIndex_Player", someGlobalIndex);
        }
    }



    //storage.GetGlobal("data" + player_id);
    //var name = game.GetShipOwner(player_id);



    var arrayOfFormationPosNId = [];

    if (!storage.IsSetGlobal("data" + player_id))
    {
        for (var i = 0; i < 6; i++) //arrayOfFormationPosNId.length // 0 for random and 1 to 5 for the formations
        {
            arrayOfFormationPosNId[i] = { pos: 0, id: -1 };
        }

        var currentDroneCounter = { droneCounter: 0, arrayOfPos: arrayOfFormationPosNId };

        storage.SetGlobal("data" + player_id, currentDroneCounter);


        /*if (storage.IsSetGlobal("data")) {
            //var someGlobalIndex = storage.GetGlobal("data");
            //someGlobalIndex++;
            //storage.SetGlobal("data" + player_id, someGlobalIndex);
            //storage.SetGlobal("data", someGlobalIndex);
        }
        else {
            //var someGlobalIndex = 0;
            //storage.SetGlobal("data" + player_id, someGlobalIndex);
            storage.SetGlobal("data", someGlobalIndex);
        }*/
        /*//var someData = storage.GetGlobal("data" + player_id);
        //var arrayOfPos = someData.arrayOfPos;
        var tempPosArray = [];

        for (var i = 1; i < arrayOfPos.length; i++) {
            if (arrayOfPos[i].pos == 0) {
                tempPosArray.push(i);
            }
        }

        var miner = 0;
        var maxer = tempPosArray.length; //- 1

        var randomNumber = Math.floor(Math.random() * (maxer - miner) + miner); //Math.floor(

        var result = tempPosArray[randomNumber];
        arrayOfPos[result].pos = 1;

        someData.arrayOfPos = arrayOfPos;
        storage.SetGlobal("data" + player_id, someData);*/
    }





 







    timer.SetTimer(20, "OnTimerWorked", { name: args.name, sys_id: args.system_id });
}

function OnTimerWorked(args)
{
    var playerName = args.name;

    timer.ClearTimer(args.timer_id);

    if (!player.HasPlayer(playerName))
	{
		return;
	}
	
    if (topics.PlayerHasQuest(playerName, "startquest"))
    {
        return;
    }
		
    // add start quest
    topics.QuestStart(playerName, "startquest", $q0001); // Talk to Joe
    topics.QuestAddLog(playerName, "startquest", $q0002); // I've bought an old civilian shuttle and finally left my home planet. However, it won't take me far. Now I should head to the nearest space station and look for One-Eyed Joe. I've heard he's helping beginners figure out how things work. He should be inside the station on one of the decks.
    topics.QuestAddMark(playerName, "startquest", args.sys_id);

    //find station
    var bases = game.GetSystemBases(args.sys_id);
    topics.QuestAddLocalMarkObject(playerName, "startquest", args.sys_id, bases[0]);
}


/*
 ===================================================================
 args:
 ship_id
 current_system_id
 destination_system_id
 jumpgate_id
 
 MUST RETURN:
 bool: allow
 string: message
 if these values are not returned, result of this function will be ignored
 ===================================================================
 */
function OnShipTryToEnterJumpgateHandler(args)
{
    // DISABLE SPAWN SYSTEM JUMPGATES UNTIL TUTORIAL COMPLETED
    if (args.current_system_id == spawn.GetSystemID()
        && args.destination_system_id != args.current_system_id)
    {
        if (!storage.IsSet("_system_common", args.ship_id + "_completed_tutorial"))
        {
            return {
                allow: false,
                message: $s0001 // First I need to speak to Joe at the station.
            };
        }
    }

    // DISABLE EXODUS JUMPGATE
    var exodus = storage.GetGlobal("exodus");
    if (exodus != null)
    {
        if (exodus.jumpgate_id == args.jumpgate_id)
        {
            if (typeof (exodus.allowed_to_leave) != "undefined")
            {
                //TODO
                game.SendNotification(args.player_name, $n0001, $n0002); // (debug) You've won the game!
                return;
            }
            else
            {
                return {
                    allow: false,
                    message: $s0002 // This jumpgate doesn't seem to be working.
                };
            }
        }
    }

    // DO NOT LET PLAYER INSIDE LAST SYSTEM UNTIL QUEST TELLS SO
    var capital = relations.GetFactionCapital("aliens");
    if (args.destination_system_id == capital)
    {
        var flag = storage.IsSet("_system_common", args.ship_id + "_ready_to_enter_final");
        if (!flag)
        {
            return {
                allow: false,
                message: $s0003 // This system appears to be closed.
            };
        }
    }

    // DO NOT LET ENTERING XENGATARN SYSTEMS WITHOUT DEVICE
    var systemInfo = generator.GetSystemByID(args.destination_system_id);
    if (systemInfo.faction == "aliens")
    {
        if (!ship.HasItemOfTypeEquipped(args.ship_id, "quest_device_xengatarn_travel"))
        {
            if (ship.HasItemOfType(args.ship_id, "quest_device_xengatarn_travel"))
            {
                return {
                    allow: false,
                    message: $s0004 // Alien's technology interference doesn't allow your ship to plot the course through this jumpgate. Equip the Barrich jumpgate amplifier device to avoid it.
                };
            }

            return {
                allow: false,
                message: $s0005 // Alien's technology interference doesn't allow your ship to plot the course through this jumpgate.
            };
        }
    }
}


/*
 =================================================================================
 args:
 
 ship_id
 
 FOR HIVES
 =================================================================================
 */

function OnShipDestroyedHandler(args)
{
    var ship_id = args.ship_id;
    var sys_id = ship.GetSystemID(ship_id);
    var sys_inf = generator.GetSystemByID(sys_id);

    if (sys_inf.faction != "aliens")
    {
        return;
    }

    if (ship.IsNpc(ship_id)
        && npc.GetTag(ship_id, "class") == "alien_hive")
    {
        // count hives left in a system		
        var ships = game.GetSystemShipsOfClass(sys_id, "alien_hive");
        var hivesLeft = ships.length;

        console.Print("OnShipDestroyedHandler: hives left: " + hivesLeft);
        if (hivesLeft == 0)
        {
            // system liberated!! hooray!
            relations.SetSystemFaction(sys_id, "none");

            actions.InvokeTrigger("onSystemLiberated", { system_id: sys_id });

            var systems = generator.GetLinkedSystems(sys_id);

            for (var i = 0; i < systems.length; i++)
            {
                var inf = generator.GetSystemByID(systems[i]);
                if (inf.faction == "aliens")
                {
                    generator.SystemOpen(systems[i]);
                }
            }
        }

        actions.InvokeTrigger("hiveDestroyedBy" + args.caster_id, {});
    }
}


/*
 ==============================================================================
 If alien mindcontrol-device was destroyed (not possible in multiplayer)
 ===============================================================================
 */
function OnCWaveEmitterDestroyedHandler(args)
{
    if (ship.IsNpc(args.ship_id) && npc.HasTag(args.ship_id, "cwaveemitter"))
    {
        actions.InvokeTrigger("onCWaveEmitterDestroyed", {});
    }
}


/*
 ===============================================================================
 When player enters system, which wasn't entered before by any player,
 fills system with NPCs
 ===============================================================================
 */
function OnPlayerEntersSystemHandler(args)
{
    actions.InvokeTrigger("onPlayerEntersSystem", args);
    var systems_to_init;
    if (storage.IsSet(tn, "systems_to_init"))
    {
        systems_to_init = storage.Get(tn, "systems_to_init");
    }
    else
    {
        systems_to_init = [];
    }

    //see if npc were already generated for this one
    if (!storage.IsSet(tn, "system_" + args.system_id))
    {
        console.Print("Generating NPCs for star system ID=" + args.system_id + " queued");

        storage.Set(tn, "system_" + args.system_id, { step: 0 }); //set flag, that in this system already was somebody
        systems_to_init.push(args.system_id);
        storage.Set(tn, "systems_to_init", systems_to_init);
        SetSubscriptionOnEveryFrame();
    }
    else if (systems_to_init.length > 0)
    {
        SetSubscriptionOnEveryFrame();
    }
}

function SetSubscriptionOnEveryFrame()
{
    if (isSubscribedOnEveryFrameUpdate != true)
    {
        isSubscribedOnEveryFrameUpdate = false;
    }

    var systems_to_init = storage.Get(tn, "systems_to_init");
    if (systems_to_init == null
        || systems_to_init.length == 0)
    {
        if (isSubscribedOnEveryFrameUpdate)
        {
            // unsubscribe
            actions.Unbind("OnEnterFrame", "EveryFrame");
            isSubscribedOnEveryFrameUpdate = false;
            // console.PrintError("unsubscribe on EveryFrame event");
        }
        return;
    }

    if (!isSubscribedOnEveryFrameUpdate)
    {
        // subscribe
        actions.Bind("OnEnterFrame", "EveryFrame");
        isSubscribedOnEveryFrameUpdate = true;
        // console.PrintError("subscribe on EveryFrame event");
    }
}

// every frame spawns NPC (executed only if subscribed on frame event)
function EveryFrame(args)
{
    var systems_to_init = storage.Get(tn, "systems_to_init");
    if (systems_to_init == null
        || systems_to_init.length == 0)
    {
        SetSubscriptionOnEveryFrame();
        return;
    }

    var somethingChanged = false;

    for (var i = 0; i < systems_to_init.length; i++)
    {
        var sys_id = systems_to_init[i];
        if (sys_id == 0)
        {
            // impossible
            continue;
        }

        var presetId = generator.GetSystemTag(sys_id, "preset");
        var sys_step = storage.Get(tn, "system_" + sys_id).step;

        //find preset by name
        var currentPreset = SystemsPresets[presetId];
		if (currentPreset == null)
		{
			// this system preset is handled by another script
			console.Print("Generating NPCs for star system ID=" + sys_id + " will be handled by another script");
			systems_to_init.splice(i, 1);
			i--;
			somethingChanged = true;
			continue;
		}

        //prepare arguments
        var args = {
            sys_id: sys_id,
            system_info: generator.GetSystemByID(sys_id),
            step: sys_step,
            safe_zones_centers_x: storage.GetGlobal("system_" + sys_id + "_safe_zones_centers_x"),
            safe_zones_centers_y: storage.GetGlobal("system_" + sys_id + "_safe_zones_centers_y"),
            safe_zones_radiuses: storage.GetGlobal("system_" + sys_id + "_safe_zones_radiuses")
        };

        var retval = currentPreset.NpcGenerationStep(args);
        storage.Set(tn, "system_" + sys_id, { step: retval });

        // console.PrintError("NPC gen for sysId=" + sys_id + " step=" + args.step + " returned: " + retval);

        if (retval == -1) //it means npc generation process is over
        {
            console.Print("Generating NPCs for star system ID=" + sys_id + " completed");
            systems_to_init.splice(i, 1);
            i--;
            somethingChanged = true;
        }
    }

    if (somethingChanged)
    {
        storage.Set(tn, "systems_to_init", systems_to_init);
        SetSubscriptionOnEveryFrame();
    }
}

function StarSystemUpdate(args)
{


    /*
    var player_id = player.GetShipOfPlayer("ninekorn");
    //var someStartingBase = { sys_id: startsysdata.id, baseID: base_id, sys_name: startsysdata.sys_name, coords: bas_coord };
    var someStartingBase = storage.GetGlobal("startingsystem");
    spawn.SetCoordinates(someStartingBase.coords.x, someStartingBase.coords.y);
    generator.DockShipToBase(player_id, someStartingBase.base_id);*/





    if (!args.populated)
    {
        return;
    }

    var jag_ids = game.GetSystemJumpgates(args.system_id);
    var inf = generator.GetSystemByID(args.system_id);

    // special cases
    //--------------------------------------------------------------

    var cpt = relations.GetFactionCapital("aliens");
    if (cpt == args.system_id)
    {
        //update final system every 15 seconds
        game.SetSecondsBetweenStarSystemUpdates(args.system_id, 15);
        AlienCapitalPopulation(args, inf, jag_ids);
        return;
    }

    var exodus = storage.GetGlobal("exodus");
    if (exodus != null && exodus.sys_id == args.system_id)
    {
        //update exodus system every 15 seconds
        game.SetSecondsBetweenStarSystemUpdates(args.system_id, 15);
        ExodusSystemPopulation(args, inf, jag_ids, exodus);
        return;
    }
    //--------------------------------------------------------------


    // update ordinary systems every 50 seconds
    game.SetSecondsBetweenStarSystemUpdates(args.system_id, 50);

    // PIRATES population control
    if (inf.faction != "aliens") //everywhere except aliens systems
    {
        RestorePiratesPopulation(args, inf, jag_ids);
    }
    else
    {
        RestoreAliensPopulation(args, inf, jag_ids);
    }

    // HEADHUNTERS after players - in every system
    if (inf.faction == "pirates")
    {
        // pirate system - 3 simultaneous headhunters max
        SpawnPirateHeadhunter(args, inf, jag_ids, 3);
    }
    else if (generator.GetSystemTag(args.system_id, "preset") == "StartingSystem")
    {
        // nothing in starting system
    }
    else if (relations.IsCapital(args.system_id))
    {
        // nothing in capital systems (except pirates (look above))
    }
    else if (inf.faction != "aliens")
    {
        // usual non-aliens system - 1 simultaneous headhunter max
        SpawnPirateHeadhunter(args, inf, jag_ids, 1);
    }

    // try to spawn alien headhunter (it's spawn only in jumpgates neighbor to aliens systems)
    SpawnAlienHeadhunterIfPossible(args, inf, jag_ids, 3);
}


/*
 ==========================================================
 Special function for controlling population in alien capital.
 
 Two divergent cases:
 1. for order
 2. for fanatics
 freedom last system is another one (nearby, "Exodus")
 
 ==========================================================
 */
function AlienCapitalPopulation(args, inf, jag_ids)
{
    var preset = storage.Get("_system_singleplayer", "last_system_preset");
    //preset
    //{name, [params]}


    //--------------------------------------------
    // O R D E R
    //
    /*if (preset.name == "order")
    {
        //order routine
        var jag_id = preset.jumpgate;
        var coords = game.GetObjectCoordinates(args.system_id, jag_id);

        //spawn order soldiers
        var orderCount = generator.CountNpcInSystemByTags(args.system_id, { class: "order.soldier" });
        if (orderCount <= 10)
        {
            for (var i = 0; i < 3; i++)
            {
                var ships = ["generic_pirates_banshee_mk2_01", "generic_pirates_avalanche_mk2_01", "generic_pirates_fang_01"];
                var picked = utils.SelectRandom(ships);

                //create headhunter ship, targeted on random hive
                var id = generator.AddNPCShipToSystem($i0001, "FinalSystemSoldier", 5, picked, args.system_id, coords.x, coords.y, { class: "order.soldier", meta: "human", sex: "male" }); // Order soldier
                relations.SetShipFaction(id, "order");
                ship.SetShipAsArrivedFromJumpgate(id, jag_id);
            }
        }

        //spawn xengatarn soldiers
        var xenCount = generator.CountNpcInSystemByTags(args.system_id, { class: "alien.soldier" });
        if (xenCount <= 10)
        {
            var hives = game.GetSystemShipsOfClass(args.system_id, "alien_hive");

            for (var i = 0; i < 3; i++)
            {
                var ships = ["generic_xengatarn_pest_brown", "generic_xengatarn_pest_red", "generic_xengatarn_violator"];
                var picked = utils.SelectRandom(ships);
                var hive = utils.SelectRandom(hives);
                var coord = game.GetObjectCoordinates(args.system_id, hive);

                //create alien ship, targeted on random soldier
                var id = generator.AddNPCShipToSystem($i0002, "FinalSystemSoldier", 5, picked, args.system_id, coord.x + 9, coord.y, { class: "alien.soldier" }); // Xengatarn
                relations.SetShipFaction(id, "aliens");
            }
        }
    }
    //--------------------------------------------------------
    // F A N A T I C S
    //
    //
    else if (preset.name == "fanatics")
    {
        //fanatics routine
    }*/
}

function ExodusSystemPopulation(args, inf, jag_ids, exodus)
{
    /*if (!exodus.exodus_active)
    {
        return;
    }

    // if active is true
    var civShips = generator.CountNpcInSystemByTags(args.system_id, { class: "freedom.civilian.ship" });

    for (var i = civShips; i < 3; i++)
    {
        var coords = game.GetObjectCoordinates(args.system_id, jag_ids[0]);

        //create civilian ship
        var id = generator.AddNPCShipToSystem($i0003, "ExodusCivilianVessel", 5, "special_human_tradership", args.system_id, coords.x, coords.y, { class: "freedom.civilian.ship" }); // Civilian vessel
        relations.SetShipFaction(id, "freedom");
        ship.SetShipAsArrivedFromJumpgate(id, jag_ids[0]);
    }*/
}

function OnGuardSpawn(args)
{
    /*var guards_respawn = storage.Get("_system_spawning", "trasure_guards_respawn");
    if (guards_respawn == null || guards_respawn.length == 0)
    {
        return;
    }

    var systems = generator.GetPopulatedSystems();

    for (var i = 0; i < guards_respawn.length; i++)
    {
        var system = guards_respawn[i].system;
        var crate = guards_respawn[i].crate;

        if (systems.indexOf(system) >= 0)
        {
            var inf = generator.GetSystemByID(system);
            var jag_ids = game.GetSystemJumpgates(system);
            var jag_id = utils.SelectRandom(jag_ids);
            var from_system_id = game.GetJumpgateDestinationSystem(jag_id);
            var from_system_faction = generator.GetSystemFaction(from_system_id);
            if (from_system_faction != "none"
                && from_system_faction != "pirates")
            {
                continue;
            }

            var coords = game.GetObjectCoordinates(system, jag_id);

            // add mighty guardian
            var possibleNpcs = generator.GetNpcsForLevelByTags(["treasure_guard"], inf.danger_level);
            var l = pickByChance(possibleNpcs);
            var random_type = possibleNpcs[l].xml_type;

            var id = generator.AddNPCShipToSystem($i0004, "TreasureGuard", inf.danger_level, random_type, system, coords.x + MathExt.RandInt() % 3, coords.y + MathExt.RandInt() % 3, { class: "pirate", crate: crate }); // Pirate
            relations.SetShipFaction(id, "pirates");
            ship.SetShipAsArrivedFromJumpgate(id, jag_id);

            //remove from tasks
            guards_respawn.splice(i, 1);
            i--;
        }
    }

    storage.Set("_system_spawning", "trasure_guards_respawn", guards_respawn);*/
}

/*
 ====================================================================
 Check whether any pirates left in system, and add new ones if necessary 
 ====================================================================
 */
function RestorePiratesPopulation(args, inf, jag_ids)
{
    /*var piratesCount = generator.CountNpcInSystemByTags(args.system_id, { class: "pirate" });

    if (piratesCount < 30) //no less then 30 pirates in every system
    {
        //spawn a group of pirates and send them to available point

        //pick random jumpgate
        var jag_id = utils.SelectRandom(jag_ids);
        var from_system_id = game.GetJumpgateDestinationSystem(jag_id);
        var from_system_faction = generator.GetSystemFaction(from_system_id);
        if (from_system_faction != "none"
            && from_system_faction != "pirates")
        {
            return;
        }

        var coords = game.GetObjectCoordinates(args.system_id, jag_id);

        //find available free point
        var stickCoords = { x: MathExt.RandRange(-350, 350), y: MathExt.RandRange(-350, 350) };
        //TODO: availability check	

        var possibleNpcs = generator.GetNpcsForLevelByTags(["pirate"], inf.danger_level);
        var l = pickByChance(possibleNpcs);
        var random_type = possibleNpcs[l].xml_type;

        var id = generator.AddNPCShipToSystem($i0006, "StickToPointAggr", inf.danger_level, random_type, args.system_id, coords.x, coords.y, { class: "pirate", stick_coords_x: stickCoords.x, stick_coords_y: stickCoords.y }); // Pirate
        relations.SetShipFaction(id, "pirates");
        ship.SetShipAsArrivedFromJumpgate(id, jag_id);

        // console.PrintError("Added npc " + random_type + " to coordinate " + stickCoords.x + ";" + stickCoords.y);
    }*/
}

function RestoreAliensPopulation(args, inf, jag_ids)
{
    /*var aliensCount = generator.CountNpcInSystemByTags(args.system_id, { class: "alien" });

    if (aliensCount < 30) //no less then 30 aliens in every alien system
    {
        //spawn a group of aliens and send them to available point

        //pick random jumpgate
        var jag_id = utils.SelectRandom(jag_ids);
        var coords = game.GetObjectCoordinates(args.system_id, jag_id);
        var from_system_id = game.GetJumpgateDestinationSystem(jag_id);
        var from_system_faction = generator.GetSystemFaction(from_system_id);
        if (from_system_faction != "aliens")
        {
            return;
        }

        //find available free point
        var stickCoords = { x: MathExt.RandRange(-350, 350), y: MathExt.RandRange(-350, 350) };
        //TODO: availability check	

        var possibleNpcs = generator.GetNpcsForLevelByTags(["alien"], inf.danger_level);
        var l = pickByChance(possibleNpcs);
        var random_type = possibleNpcs[l].xml_type;

        var id = generator.AddNPCShipToSystem($i0007, "StickToPointAggr", inf.danger_level, random_type, args.system_id, coords.x, coords.y, { class: "alien", sex: "male", stick_coords_x: stickCoords.x, stick_coords_y: stickCoords.y }); // Xengatarn
        relations.SetShipFaction(id, "aliens");
        ship.SetShipAsArrivedFromJumpgate(id, jag_id);

        //console.PrintError("Added npc " + random_type + " to coordinate " + stickCoords.x + ";" + stickCoords.y);
    }*/
}

/*
 ====================================================================
 Spawns headhunter, who is chasing after player
 ====================================================================
 */
function SpawnPirateHeadhunter(args, inf, jag_ids, max_number)
{
    /*if (MathExt.RandRange(0, 3) != 0)
    {
        // no headhunter now, 33% probability
        return;
    }

    //pick random jumpgate
    var jag_id = utils.SelectRandom(jag_ids);
    var from_system_id = game.GetJumpgateDestinationSystem(jag_id);
    var from_system_faction = generator.GetSystemFaction(from_system_id);
    if (from_system_faction != "none"
        && from_system_faction != "pirates")
    {
        return;
    }

    var coords = game.GetObjectCoordinates(args.system_id, jag_id);

    var possibleNpcs = generator.GetNpcsForLevelByTags(["pirate"], inf.danger_level);
    var l = pickByChance(possibleNpcs);
    var random_type = possibleNpcs[l].xml_type;

    //pick target
    var playerShips = game.GetSystemPlayersShips(args.system_id);
    var playerShip = utils.SelectRandom(playerShips);

    var hhCount = generator.CountNpcInSystemByTags(args.system_id, { class: "pirate", target: playerShip });
    if (hhCount >= max_number)
    {
        return;
    }

    var id = generator.AddNPCShipToSystem($i0008, "HeadHunter", inf.danger_level, random_type, args.system_id, coords.x, coords.y, { class: "pirate", target: playerShip }); // Pirate Headhunter
    relations.SetShipFaction(id, "pirates");
    ship.SetShipAsArrivedFromJumpgate(id, jag_id);

    console.Print("Added headhunter " + random_type + " after " + playerShip + " from system of " + from_system_faction);*/
}

/*
 ====================================================================
 Spawns alien headhunter, who is chasing after player (it's spawn only in jumpgates neighbor to aliens systems)
 ====================================================================
 */
function SpawnAlienHeadhunterIfPossible(args, inf, jag_ids, max_number)
{
    /*if (MathExt.RandRange(0, 3) != 0)
    {
        // no headhunter now, 33% probability
        return;
    }

    // pick random jumpgate
    var jag_id = utils.SelectRandom(jag_ids);
    var from_system_id = game.GetJumpgateDestinationSystem(jag_id);
    var from_system_faction = generator.GetSystemFaction(from_system_id);
    if (from_system_faction != "aliens")
    {
        // spawn only in jumpgates neighbor to aliens systems
        return;
    }

    var coords = game.GetObjectCoordinates(args.system_id, jag_id);

    var possibleNpcs = generator.GetNpcsForLevelByTags(["alien"], inf.danger_level);
    var l = pickByChance(possibleNpcs);
    var random_type = possibleNpcs[l].xml_type;

    //pick target
    var playerShips = game.GetSystemPlayersShips(args.system_id);
    var playerShip = utils.SelectRandom(playerShips);

    var hhCount = generator.CountNpcInSystemByTags(args.system_id, { class: "alien", target: playerShip });
    if (hhCount >= max_number)
    {
        return;
    }

    var id = generator.AddNPCShipToSystem($i0009, "HeadHunter", inf.danger_level, random_type, args.system_id, coords.x, coords.y, { class: "pirate", target: playerShip }); // Xengatarn Seeker
    relations.SetShipFaction(id, "aliens");
    ship.SetShipAsArrivedFromJumpgate(id, jag_id);

    console.Print("Added alien headhunter " + random_type + " after " + playerShip + " from system of " + from_system_faction);*/
}
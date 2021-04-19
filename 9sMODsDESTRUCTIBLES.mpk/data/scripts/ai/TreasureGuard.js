/*
 ================================================================
 TreasureGuard
 
 Desc:
 pirate in waiting sticks to point on which he was born and wait there for
 enemies.
 
 ================================================================
 */

using(console);
using(relations);
using(npc);
using(ship);
using(storage);
using(actions);
using(generator);
using(visual);
using(game);

include(NpcLib.js);
include(SC_Utilities.js);
include(SC_Salvage_Object_Storage.js);


function OnAIInited()
{
    //set update interval
    npc.SetDecisionsPerSecond(SHIP_ID, 2);

    NpcLib.AvoidAll(SHIP_ID);
    NpcLib.EvasionOn(SHIP_ID);

    NpcLib.SetupDefaultWeaponUsage(SHIP_ID);

    //battle decisions setup
    //npc.SetBehavior(SHIP_ID, "run_away_on_hp", 10.0);
}


//-----------------------------------------------------
// Name: Decision
// Desc: classic behavior of decision - look into scope, try to find target,
//		 and if none, patrol, else attack
//-----------------------------------------------------
/* args:
 scope_ships:
 disposition
 id
 is_npc
 name
 distance
 */
function Decision(args)
{
    var stickradius = 20;
    var crateId = ship.GetTag(SHIP_ID, "crate");

    //stick to point
    if (!npc.IsStickToObject(SHIP_ID))
    {
        npc.StickToObject(SHIP_ID, crateId, stickradius);
    }

    //parameters
    var lockdist = 25;
    var getawaydist = 26;

    if (!npc.IsLocked(SHIP_ID))
    {
        //if there's no locked target, search for a target within { lockdist }
        var scopeObjects = args.scope_ships;

        if (npc.GetDistanceToStickObject(SHIP_ID) > stickradius)
        {
            lockdist = 12;
        }

        for (var i = 0; i < args.scope_ships.length; i++)
        {
            var otherShip = args.scope_ships[i];

            //take closest enemy
            if (otherShip.distance < lockdist
                && otherShip.distance < lockdist
                //&& !otherShip.is_npc
                && otherShip.disposition < 0)
            {
                npc.LockOnTarget(SHIP_ID, otherShip.id, getawaydist);
                break;
            }
        }
    }
}


function OnTakeDamage(args)
{
    NpcLib.StandardOnTakeDamage(SHIP_ID, args);
}

function OnDied(args)
{
    //add special info to respawn table
    var system = ship.GetSystemID(SHIP_ID);
    var crateId = ship.GetTag(SHIP_ID, "crate");

    var guards_respawn = storage.Get("_system_spawning", "trasure_guards_respawn");
    if (guards_respawn == null)
    {
        guards_respawn = [];
    }
    guards_respawn.push({ system: system, crate: crateId });
    storage.Set("_system_spawning", "trasure_guards_respawn", guards_respawn);
}



function OnDied(args) {
    //if (ship.GetCurrentValue(SHIP_ID, "structure") <= 0 || !generator.ShipExists(SHIP_ID)) // the ship doesn't exist anymore //!generator.ShipExists(SHIP_ID) || 
    //{

        var value = ship.GetTag(SHIP_ID, "xmlnpcship");

        var sys_idNPC = ship.GetSystemID(SHIP_ID);
        var coords = game.GetObjectCoordinates(sys_idNPC, SHIP_ID);

        var angle = ship.GetRotation(SHIP_ID);
        var radToDeg = angle * (180.0 / Math.PI);

        //var rotatedCoordRefuel = SC_Utilities.RotatePoint(tempCoordRefuel, coords, radToDeg);

        var randomYieldToStartOff = Math.floor(Math.random() * (50 - 15) + 15); //var randomPos = Math.floor(Math.random() * (2 - 0) + 0);  between 0 and 1     

        var randomPartsSalvageable = 0;

        if (value == "generic_pirates_shuttle_01" || value == "generic_pirates_shuttle_02") {
            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipShuttleParts.length + 1));
        }
        else if (value == "generic_pirates_heretic_01" || value == "generic_pirates_heretic_02") {

            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipHereticParts.length + 1));

        }
        else if (value == "generic_pirates_fang_01" || value == "generic_pirates_fang_02" || value == "generic_pirates_fang_03") {

            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipFangParts.length + 1));
        }
        else if (value == "generic_pirates_endeavor_01" || value == "generic_pirates_endeavor_02" || value == "special_human_tradership_endeavor") {

            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipEndeavorParts.length + 1));
        }
        else if (value == "generic_pirates_orca_01" || value == "generic_pirates_orca_02" || value == "generic_pirates_orca_03") {

            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipOrcaParts.length + 1));
        }
        else if (value == "special_human_tradership_asteria") {
            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipAsteriaParts.length + 1));
        }
        else if (value == "generic_pirates_aurora_01" || value == "generic_pirates_aurora_02" || value == "generic_pirates_aurora_03") {
            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipAuroraParts.length + 1));
        }
        else if (value == "generic_pirates_avalanche_01" || value == "generic_pirates_avalanche_02" || value == "generic_pirates_avalanche_mk2_01" || value == "generic_pirates_avalanche_mk2_02" || value == "generic_pirates_avalanche_mk2_03") {
            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipAvalancheParts.length + 1));
        }
        else if (value == "generic_pirates_banshee_01" || value == "generic_pirates_banshee_02" || value == "generic_pirates_banshee_03") {
            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipBansheeParts.length + 1));

        }
        else if (value == "generic_pirates_banshee_mk2_01" || value == "generic_pirates_banshee_mk2_02") {
            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipBansheeMk2Parts.length + 1));

        }
        else if (value == "generic_pirates_boomerang_01" || value == "generic_pirates_boomerang_02") {
            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipBoomerangParts.length + 1));
        }
        else if (value == "generic_pirates_crab_01" || value == "generic_pirates_crab_02" || value == "generic_pirates_crab_03") {
            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipCrabParts.length + 1));
        }
        else if (value == "generic_pirates_death_bringer_01" || value == "generic_pirates_death_bringer_02") {
            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipDeathBringerParts.length + 1));
        }
        else if (value == "special_human_tradership") {
            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipFreighterParts.length + 1));
        }
        else if (value == "generic_guard_hammer_01" || value == "generic_guard_hammer_02" || value == "generic_pirates_hammer_01") {
            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipHammerParts.length + 1));
        }
        else if (value == "generic_pirates_stingray_01" || value == "generic_pirates_stingray_02") {
            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipStingrayParts.length + 1));
        }
        else if (value == "generic_pirates_stingray_mk2_01" || value == "generic_pirates_stingray_mk2_02") {
            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipStingrayMK2Parts.length + 1));
        }
        else if (value == "generic_pirates_stryker_01" || value == "generic_pirates_stryker_02") {
            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipStrykerParts.length + 1));
        }
        else if (value == "generic_pirates_wasp") {
            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipWaspParts.length + 1));
        }
        else if (value == "generic_pirates_yamato_01" || value == "generic_pirates_yamato_02") {

            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipYamatoParts.length + 1));
        }

        //console.PrintError("1: " + randomPartsSalvageable);
        if (randomPartsSalvageable == 0) {

        }
        else {
            //spawnOnRandom(sys_idNPC, coords, randomYieldToStartOff, randomPartsSalvageable);

            var arrayOfSwtchTest =
                [
                    0,
                    1,
                    2
                ];

            for (var i = 0; i < randomPartsSalvageable; i++) {
                var random = Math.floor(Math.random() * (arrayOfSwtchTest.length));

                //console.PrintError("random xml: " + random + " arrayOfSwtchTest: " + arrayOfSwtchTest[random] + " iterator: " + i);

                if (value == "generic_pirates_shuttle_01" || value == "generic_pirates_shuttle_02") {
                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipShuttleParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "generic_pirates_heretic_01" || value == "generic_pirates_heretic_02") {
                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipHereticParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "generic_pirates_fang_01" || value == "generic_pirates_fang_02" || value == "generic_pirates_fang_03") {
                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipFangParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "generic_pirates_endeavor_01" || value == "generic_pirates_endeavor_01" || value == "special_human_tradership_endeavor") {

                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipEndeavorParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "generic_pirates_orca_01" || value == "generic_pirates_orca_02" || value == "generic_pirates_orca_03") {

                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipOrcaParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "special_human_tradership_asteria") {
                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipAsteriaParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "generic_pirates_aurora_01" || value == "generic_pirates_aurora_02" || value == "generic_pirates_aurora_03") {

                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipAuroraParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "generic_pirates_avalanche_01" || value == "generic_pirates_avalanche_02" || value == "generic_pirates_avalanche_mk2_01" || value == "generic_pirates_avalanche_mk2_02" || value == "generic_pirates_avalanche_mk2_03") {

                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipAvalancheParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "generic_pirates_banshee_01" || value == "generic_pirates_banshee_02" || value == "generic_pirates_banshee_03") {

                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipBansheeParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "generic_pirates_banshee_mk2_01" || value == "generic_pirates_banshee_mk2_02") {
                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipBansheeMk2Parts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "generic_pirates_boomerang_01" || value == "generic_pirates_boomerang_02") {
                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipBoomerangParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "generic_pirates_crab_01" || value == "generic_pirates_crab_02" || value == "generic_pirates_crab_03") {
                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipCrabParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "generic_pirates_death_bringer_01" || value == "generic_pirates_death_bringer_02") {
                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipDeathBringerParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "special_human_tradership") {
                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipFreighterParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "generic_guard_hammer_01" || value == "generic_guard_hammer_02" || value == "generic_pirates_hammer_01") {
                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipHammerParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "generic_pirates_stingray_01" || value == "generic_pirates_stingray_02") {

                    arrayOfSwtchTest =
                        [
                            0,
                            1
                        ];

                    random = Math.floor(Math.random() * (arrayOfSwtchTest.length));

                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipStingrayParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "generic_pirates_stingray_mk2_01" || value == "generic_pirates_stingray_mk2_02") {
                    arrayOfSwtchTest =
                        [
                            0,
                            1
                        ];

                    random = Math.floor(Math.random() * (arrayOfSwtchTest.length));
                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipStingrayMK2Parts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "generic_pirates_stryker_01" || value == "generic_pirates_stryker_02") {
                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipStrykerParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }
                else if (value == "generic_pirates_wasp") {
                    arrayOfSwtchTest =
                        [
                            0,
                            1,
                            2,
                            3
                        ];

                    random = Math.floor(Math.random() * (arrayOfSwtchTest.length));
                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipWaspParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }

                else if (value == "generic_pirates_yamato_01" || value == "generic_pirates_yamato_02") {
                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipYamatoParts[arrayOfSwtchTest[random]], { index: index });
                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }

                arrayOfSwtchTest.splice(random, 1);
            }
        }

        //SC_Salvage_Object_Storage.systemSpaceObjects.length;

        var scnpcdata = storage.GetGlobal("destroyed_" + SHIP_ID);

        if (scnpcdata == null) {
            scnpcdata = { swtchdestroyed: 1 };
            storage.SetGlobal("destroyed_" + SHIP_ID, scnpcdata);
        }
    //}
}


var arrayOfDestroyedShipShuttleParts =
    [
        "SCdestroyedShuttleParts01-0",
        "SCdestroyedShuttleParts01-1",
        "SCdestroyedShuttleParts01-2"
    ];
var arrayOfDestroyedShipHereticParts =
    [
        "SCdestroyedHereticParts01-0",
        "SCdestroyedHereticParts01-1",
        "SCdestroyedHereticParts01-2"
    ];
var arrayOfDestroyedShipFangParts =
    [
        "SCdestroyedFangParts01-0",
        "SCdestroyedFangParts01-1",
        "SCdestroyedFangParts01-2"
    ];
var arrayOfDestroyedShipEndeavorParts =
    [
        "SCdestroyedEndeavorParts01-0",
        "SCdestroyedEndeavorParts01-1",
        "SCdestroyedEndeavorParts01-2"
    ];
var arrayOfDestroyedShipOrcaParts =
    [
        "SCdestroyedOrcaParts01-0",
        "SCdestroyedOrcaParts01-1",
        "SCdestroyedOrcaParts01-2"
    ];
var arrayOfDestroyedShipScarabParts =
    [
        "SCdestroyedScarabParts01-0",
        "SCdestroyedScarabParts01-1",
        "SCdestroyedScarabParts01-2"
    ];
var arrayOfDestroyedShipBoomerangParts =
    [
        "SCdestroyedBoomerangParts01-0",
        "SCdestroyedBoomerangParts01-1",
        "SCdestroyedBoomerangParts01-2"
    ];
var arrayOfDestroyedShipBansheeParts =
    [
        "SCdestroyedBansheeParts01-0",
        "SCdestroyedBansheeParts01-1",
        "SCdestroyedBansheeParts01-2"
    ];

var arrayOfDestroyedShipBansheeMk2Parts =
    [
        "SCdestroyedBansheeMk2Parts01-0",
        "SCdestroyedBansheeMk2Parts01-1",
        "SCdestroyedBansheeMk2Parts01-2"
    ];

var arrayOfDestroyedShipAsteriaParts =
    [
        "SCdestroyedAsteriaParts01-0",
        "SCdestroyedAsteriaParts01-1",
        "SCdestroyedAsteriaParts01-2",
    ];

var arrayOfDestroyedShipAuroraParts =
    [
        "SCdestroyedAuroraParts01-0",
        "SCdestroyedAuroraParts01-1",
        "SCdestroyedAuroraParts01-2"
    ];

var arrayOfDestroyedShipAvalancheParts =
    [
        "SCdestroyedAvalancheParts01-0",
        "SCdestroyedAvalancheParts01-1",
        "SCdestroyedAvalancheParts01-2"
    ];

var arrayOfDestroyedShipAvalancheMk2Parts =
    [
        "SCdestroyedAvalancheMk2Parts01-0",
        "SCdestroyedAvalancheMk2Parts01-1",
        "SCdestroyedAvalancheMk2Parts01-2"
    ];


var arrayOfDestroyedShipCrabParts =
    [
        "SCdestroyedCrabParts01-0",
        "SCdestroyedCrabParts01-1",
        "SCdestroyedCrabParts01-2"
    ];

var arrayOfDestroyedShipDeathBringerParts =
    [
        "SCdestroyedDeathBringerParts01-0",
        "SCdestroyedDeathBringerParts01-1",
        "SCdestroyedDeathBringerParts01-2"
    ];

var arrayOfDestroyedShipFreighterParts =
    [
        "SCdestroyedFreighterParts01-0",
        "SCdestroyedFreighterParts01-1",
        "SCdestroyedFreighterParts01-2"
    ];

var arrayOfDestroyedShipHammerParts =
    [
        "SCdestroyedHammerParts01-0",
        "SCdestroyedHammerParts01-1",
        "SCdestroyedHammerParts01-2"
    ];


var arrayOfDestroyedShipStingrayParts =
    [
        "SCdestroyedStingrayParts01-0",
        "SCdestroyedStingrayParts01-1"
    ];

var arrayOfDestroyedShipStingrayMK2Parts =
    [
        "SCdestroyedStingrayMk2Parts01-0",
        "SCdestroyedStingrayMk2Parts01-1",
        "SCdestroyedStingrayMk2Parts01-2",
    ];


var arrayOfDestroyedShipStrykerParts =
    [
        "SCdestroyedStrykerParts01-0",
        "SCdestroyedStrykerParts01-1",
        "SCdestroyedStrykerParts01-2"
    ];

var arrayOfDestroyedShipWaspParts =
    [
        "SCdestroyedWaspParts01-0",
        "SCdestroyedWaspParts01-1",
        "SCdestroyedWaspParts01-2",
        "SCdestroyedWaspParts01-3",
    ];

var arrayOfDestroyedShipYamatoParts =
    [
        "SCdestroyedYamatoParts01-0",
        "SCdestroyedYamatoParts01-1",
        "SCdestroyedYamatoParts01-2",
    ];




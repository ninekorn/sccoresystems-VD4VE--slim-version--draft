/*
 ============================================================
 Turret
 
 Desc: ehm... turret
 ============================================================
 */

using(console);
using(relations);
using(npc);
using(ship);
using(generator);
using(game);
using(storage);

include(NpcLib.js);
include(SC_Utilities.js);
include(SC_Salvage_Object_Storage.js);

function OnAIInited()
{
    // set update interval
    npc.SetDecisionsPerSecond(SHIP_ID, 2);

    npc.SetupWeaponUsage(SHIP_ID, { type: "Bullet", min_dist: 0, max_dist: 25, delay: 2, min_duration: 3 });
    npc.SetupWeaponUsage(SHIP_ID, { type: "Missile", min_dist: 0, max_dist: 25, delay: 2, min_duration: 5 });
    npc.SetupWeaponUsage(SHIP_ID, { type: "Ray", min_dist: 0, max_dist: 25, delay: 2, min_duration: 2 });
    npc.SetupWeaponUsage(SHIP_ID, { type: "DirectHit", min_dist: 0, max_dist: 25, delay: 2, min_duration: 3 });

    // battle decisions setup
    npc.SetBehavior(SHIP_ID, "do_not_maneuvre", true);
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
    //parameters
    var lockdist = 25;
    var getawaydist = 26;

    if (!npc.IsLocked(SHIP_ID))
    {
        //if there's no locked target, search for a target within { lockdist }
        var scopeObjects = args.scope_ships;

        for (var i = 0; i < args.scope_ships.length; i++)
        {
            var otherShip = args.scope_ships[i];

            //take closest enemy
            if (otherShip.distance < lockdist
                && otherShip.distance < lockdist
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
    NpcLib.OnTakeDamageNoMemory(SHIP_ID, args);
}

function OnDied(args)
{
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
    
        if (value == "special_human_turret" || value == "special_human_turret_mk2" || value == "special_human_turret_mk3")
        {
            randomPartsSalvageable = Math.floor(Math.random() * (arrayOfDestroyedShipTurretParts.length + 1));//arrayOfDestroyedShipTurretParts.length; //Math.floor(Math.random() * ((arrayOfDestroyedShipOrcaParts.length) - 0) + 0);
        }

        //console.PrintError("parts: " + randomPartsSalvageable);

        if (randomPartsSalvageable == 0)
        {

        }
        else
        {
            //spawnOnRandom(sys_idNPC, coords, randomYieldToStartOff, randomPartsSalvageable);

            /*var arrayOfDestroyedShipTurretParts =
            [
                "SCdestroyedTurretPlatformParts01-0",
                "SCdestroyedTurretPlatformParts01-1"
            ];*/

            var arrayOfSwtchTest =
                [
                    0,
                    1
                ];

            for (var i = 0; i < randomPartsSalvageable; i++)
            {
                var random = Math.floor(Math.random() * ((arrayOfSwtchTest.length) - 0) + 0);

                //console.PrintError(arrayOfSwtchTest.length);
                //console.PrintError("random xml: " + random + " arrayOfSwtchTest: " + arrayOfSwtchTest[random] + " iterator: " + i);
                
                if (value == "special_human_turret" || value == "special_human_turret_mk2" || value == "special_human_turret_mk3")
                {        
                    var index = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();

                    var id0 = generator.AddSpaceObject(sys_idNPC, coords.x, coords.y, arrayOfDestroyedShipTurretParts[arrayOfSwtchTest[random]], { index: index });

                    SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_idNPC, id0, randomYieldToStartOff);
                }

                arrayOfSwtchTest.splice(random, 1);
            }
        }



        //SC_Salvage_Object_Storage.systemSpaceObjects.length;

        var scnpcdata = storage.GetGlobal("destroyed_" + SHIP_ID);

        if (scnpcdata == null)
        {
            scnpcdata = { swtchdestroyed: 1 };
            storage.SetGlobal("destroyed_" + SHIP_ID, scnpcdata);
        }
    //}
}


var arrayOfDestroyedShipTurretParts =
    [
        "SCdestroyedTurretPlatformParts01-0",
        "SCdestroyedTurretPlatformParts01-1"
    ];

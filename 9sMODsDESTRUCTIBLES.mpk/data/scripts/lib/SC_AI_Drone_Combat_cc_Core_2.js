using(console);
using(npc);

include(SC_AI_Drone_Combat_cc_Rout_For_2.js);
include(SC_AI_Drone_Combat_cc_Rout_For_FWP_2.js);
include(SC_AI_Drone_Combat_Sta_aRep_2.js);
include(SC_Utilities.js);
include(sc_imcache.js);



var currentIndex = -1;
var initOnce = 1;
var CToD = null;

//var arrayOfDronePathfindMainSwitch= [];
var arrayOfDroneAutoDocked = []; // 
var arrayOfDroneAutoRepair = []; // 
var arrayOfDroneAutoDefense = []; // 
var arrayOfDroneAutoSwitches = []; // 

var arrayOfDroneManualDocked = []; // 
var arrayOfDroneManualRepair = []; // 
var arrayOfDroneManualDefense = []; // 

var arrayOfDroneFormation = [];
var arrayOfFriendlies = [];
var lockdist = 50;




var SC_AI_Drone_Combat_cc_Core_2 =
{
    AICoreInit: function (SHIP_ID, droneIndex, playerName, player_id, frameCounterArrayResetVisualTiles)
    {
        //FIX THE MAIN "currentCommand" to NOT get checked every frame MAYBE... instead create another global variable for that drone that has only 1 variable that is a 0 for no and 1 for yes... next to fetch the BIG
        //Command array, and 0 to NOT fetch the big Command Array. There is not a lot of things in this new global variables instead of the big one.

        CToD = storage.GetGlobal("currentCommand" + SHIP_ID); // to be changed with a switch instead... the switch purpose is that it will only contain 0 and 1 instead of ALL the data of the command
        //and the drones wont have to check the global command containing all of the data every frame. it will only check a global variable every frame.





        if (CToD != null) {
            if (CToD.id != null) {
                if (CToD.id === SHIP_ID) {
                    if (npc.CountObjectives(CToD.id) > 0) // isSelfActive == true => compute Objective
                    {




                        currentObjective = npc.GetCurrentObjective(CToD.id);

                        AICoreComputeObjective(currentObjective, CToD.addFriend);



                        if (CToD.isInBaseVicinity == 1 && currentObjective.bidSwtch != 1 && CToD.command == 0)
                        {
                            console.PrintError("setting command for base vicinity");
                            var currentCom = storage.GetGlobal("currentCommand" + CToD.id );
                            currentCom.command = 1;
                            storage.SetGlobal("currentCommand" + CToD.id , currentCom);
                        }



                        //console.PrintError("CToD.command: " + CToD.command);

                        if (CToD.lastCommand != CToD.command) {
                            /*if (CToD.isInBaseVicinity == 1)
                            {
                                console.PrintError("is in base vicinity. updating current objective.");
                                if (npc.IsStickToPoint(CToD.id) || npc.IsStickToObject(CToD.id)) {
                                    npc.Unstick(CToD.id);
                                }

                                if (npc.IsLocked(CToD.id) || npc.IsFriendlyLocked(CToD.id)) {
                                    npc.Unlock(CToD.id);
                                }

                                npc.StopFollowingRoute(CToD.id);
                                npc.NextObjective(CToD.id);
                                npc.CleanObjectives(CToD.id);
                                //npc.InstantStop(CToD.id);
                                npc.Stop(CToD.id);
                            }
                            else
                            {

                            }*/

                            console.PrintError("new command : " + CToD.id + " command " + CToD.command);

                            if (npc.IsStickToPoint(CToD.id) || npc.IsStickToObject(CToD.id)) {
                                npc.Unstick(CToD.id);
                            }

                            if (npc.IsLocked(CToD.id) || npc.IsFriendlyLocked(CToD.id)) {
                                npc.Unlock(CToD.id);
                            }

                            npc.StopFollowingRoute(CToD.id);
                            npc.NextObjective(CToD.id);
                            npc.CleanObjectives(CToD.id);
                            //npc.InstantStop(CToD.id);
                            npc.InstantStop(CToD.id);

                            //CToD = storage.GetGlobal("currentCommand" + CToD.id);
                            CToD.lastCommand = CToD.command;
                            storage.SetGlobal("currentCommand" + CToD.id, CToD);
                        }
                        else {
                          
                        }
                    }
                    else
                    {
                        if (CToD.command == 0 || CToD.command == 1)
                        {
                            //aDock: 1, aDef: 0, sDef: 0, aRep: 0, sRep: 0
                            var sys_id = npc.GetCurrentSystemID(CToD.id);


            

                            var swtchisInBaseVicinity = 0;

                            if (isNaN(CToD.isInBaseVicinity) || CToD.isInBaseVicinity == undefined || isNaN(CToD.bid) || CToD.bid == null || CToD.isInBaseVicinity != 1)
                            {
                                if (isNaN(CToD.isInBaseVicinity) || CToD.isInBaseVicinity == undefined)
                                {
                                    swtchisInBaseVicinity = -1;
                                    console.PrintError("00CToD.isInBaseVicinity: " + swtchisInBaseVicinity);
                                 
                                }
                                else if (CToD.isInBaseVicinity != 1)
                                {
                                    swtchisInBaseVicinity = CToD.isInBaseVicinity;
                                    console.PrintError("01CToD.isInBaseVicinity: " + swtchisInBaseVicinity);
                                 
                                }
                                else if (CToD.isInBaseVicinity == 1)
                                {
                                
                                    swtchisInBaseVicinity = CToD.isInBaseVicinity;
                                    console.PrintError("01CToD.isInBaseVicinity: " + swtchisInBaseVicinity);

                                }




                                if (isNaN(CToD.bid)) {
                                    CToD.bid = null;
                                }
                                if (CToD.bid == null) {
                                    CToD.bid = null;
                                }
                            }

                            console.PrintError("===CToD.isInBaseVicinity=== : " + CToD.isInBaseVicinity);
                            if (CToD.isInBaseVicinity == 1)
                            {

                                var originalPlayerShipSpeed = ship.GetFinalCacheValue(CToD.id, "speed_max");
                                var originalNpcShipFORWARDSpeed = ship.GetFinalCacheValue(CToD.id, "speed_max");
                                var originalNpcShipSTRAFESpeed = ship.GetFinalCacheValue(CToD.id, "speed_strafe");




                                //npc.CapSpeed(CToD.id, originalNpcShipFORWARDSpeed * 0.15);


                                var speedType = 1;
                                //1 for station viscinity and 0 for !

                                //sc_imcache.ClearNSetNpcLeveledCache(CToD.id, speedType, player_id);

                                swtchisInBaseVicinity = CToD.isInBaseVicinity;
                                console.PrintError("swtchisInBaseVicinity: " + swtchisInBaseVicinity);
                            }


                            /*aDock: CToD.aDock,
                            mDock: CToD.sDock,
                            aDef: CToD.aDef,
                            mDef: CToD.sDef,
                            aRep: CToD.aRep,
                            mRep: CToD.sRep,
                            minHP: CToD.minHP,
                            sDefLoc: CToD.sDefLoc,
                            frameCounterArrayResetVisualTiles: frameCounterArrayResetVisualTiles,*/







                            //also change it in the reset speed script
                            npc.AddObjective(CToD.id, "get_in_formation",
                                {

                                    nid: CToD.id,
                                    pid: player_id,
                                    sid: sys_id,
                                    bid: CToD.bid,
                                    bidSwtch: swtchisInBaseVicinity,
                                    pName: playerName,
                                    command: CToD.command,
                                    formation: CToD.formation,
                                    speedSwitch: 0,
                                    maxPlayerForwardSpeed: null,
                                    maxNPCForwardSpeed: null,
                                    maxNPCStrafeSpeed: null,
                                    wepPropType: CToD.wepPropType,
                                    wepDistType: CToD.wepDistType,
                                    droneIndex: droneIndex,


                                });
                        }
                        /*else if (CToD.command == 1) //drone is in the station viscinity
                        {
                            var sys_id = npc.GetCurrentSystemID(CToD.id);

                            npc.AddObjective(CToD.id, "station_vicinity",
                                {
                                    nid: CToD.id,
                                    pid: player_id,
                                    sid: sys_id,
                                    bid: CToD.bid,
                                    bidSwtch: CToD.isInBaseVicinity,
                                    pName: playerName,
                                    command: CToD.command,
                                    formation: CToD.formation,
                                    speedSwitch: 0,
                                    maxPlayerForwardSpeed: null,
                                    maxNPCForwardSpeed: null,
                                    maxNPCStrafeSpeed: null,
                                    wepPropType: CToD.wepPropType,
                                    wepDistType: CToD.wepDistType,
                                    droneIndex: droneIndex,
                                    aDock: CToD.aDock,
                                    mDock: CToD.sDock,
                                    aDef: CToD.aDef,
                                    mDef: CToD.sDef,
                                    aRep: CToD.aRep,
                                    mRep: CToD.sRep,
                                    minHP: CToD.minHP,
                                    sDefLoc: CToD.sDefLoc
                                });
                        }*/
                    }
                }
                else {
                    return;
                }
            }
            else {
                return;
            }
        }
        else {
            return;
        }
    }
};

function AICoreComputeObjective(currentObjective, addFriendSwitch) {
    if (currentObjective == null) {
        return;
    }

    switch (currentObjective.name) {
        case "get_in_formation":
            {
                SC_AI_Drone_Combat_cc_Rout_For_2.AIRoutineInit(currentObjective, addFriendSwitch);
                break;
            }
        case "station_vicinity":
            {            
                if (currentObjective.formation == 1) {
                    globIndex = storage.GetGlobal("maxDroneIndex0"); //formation 1
                }
                else if (currentObjective.formation == 2) {
                    globIndex = storage.GetGlobal("maxDroneIndex1"); //formation 2
                }
                else if (currentObjective.formation == 3) {
                    globIndex = storage.GetGlobal("maxDroneIndex2"); //formation 3
                }
                else if (currentObjective.formation == 4) {
                    globIndex = storage.GetGlobal("maxDroneIndex3"); //formation 4
                }
                else if (currentObjective.formation == 5) {
                    globIndex = storage.GetGlobal("maxDroneIndex4"); //formation 5
                }

                if (arrayOfDroneAutoSwitches.length < globIndex + 1)
                {
                    for (var i = 0; i < (globIndex + 1) - arrayOfDroneAutoSwitches.length; i++)
                    {
                        arrayOfDroneAutoSwitches.push({ counter: 0 });
                    }
                }
                else
                {              
                    if (arrayOfDroneAutoSwitches[currentObjective.droneIndex].counter == 0)
                    {                  
                        var aRepStatus = SC_AI_Drone_Combat_Sta_aRep_2.AIaRep(currentObjective, globIndex);

                        if (aRepStatus == 0) //not enough money or no need to repair or auto Rep is OFF
                        {
                     


                            //currently setting the drone to return to the player in formation.
                            var currentCom = storage.GetGlobal("currentCommand" + currentObjective.nid);
                            currentCom.command = 0;
                            storage.SetGlobal("currentCommand" + currentObjective.nid, currentCom);

                            npc.LeaveBase(currentObjective.nid); 

                            console.PrintError("ship wasn't damaged but was sent to the station anyway to test the pathfind. To finish later.");

                            //move on to auto task 2 
                        }
                        else if (aRepStatus == 1)
                        {
                            console.PrintError("test1");


                            //auto repair task is done. move on to auto task 2 
                        }
                        else if (aRepStatus == 2)
                        {
                            console.PrintError("test2");


                            //keep on working on it... the drone is still working on the aRep Task
                        }
                        else {
                            console.PrintError("test3");


                        }
                    }
                    else if (arrayOfDroneAutoSwitches[currentObjective.droneIndex].counter == 0)
                    {
                        console.PrintError("auto repair success");



                    }
                }
                break;
            }















        case "player_has_docked":
            {
                var isPlayerDocked = ship.IsDocked(currentObjective.pid);
                //var isNpcDocked = npc.IsOnBase(currentObjective.nid);
                var isNpcDocked = ship.IsDocked(currentObjective.nid);

                if (isPlayerDocked)
                {
                    if (isNpcDocked)
                    {
                        //console.PrintError("player has docked 001");
                        return;
                    }
                    else {

                        //console.PrintError("player has docked 002");
                        /*var current_base = ship.GetCurrentBase(currentObjective.pid);

                        if (npc.IsAtBaseEntry(currentObjective.nid, current_base))
                        {
                            console.PrintError("player has docked 000");
                            npc.EnterBase(currentObjective.nid, current_base);
                        }
                        else
                        {
                            var sys_idPlayer = ship.GetSystemID(currentObjective.pid);
                            var theBase = storage.GetGlobal("system_" + sys_idPlayer + "_base_" + current_base + "_xmlStationType");

                            //theBase


                            //console.PrintError("player has docked 001");
                            //npc.FollowRouteToBase(currentObjective.nid, current_base);
                        }*/
                    }
                }
                else
                {
                    if (isNpcDocked)
                    {
                        npc.LeaveBase(currentObjective.nid);
                    }
                    else
                    {
                        var currentCom = storage.GetGlobal("currentCommand" + currentObjective.nid);
                        currentCom.command = 0;
                        storage.SetGlobal("currentCommand" + currentObjective.nid, currentCom);

                        npc.NextObjective(currentObjective.nid);
                        npc.CleanObjectives(currentObjective.nid);
                    }
                }
                break;
            }

        case "player_has_jumped":
            {
                var jumpGateToTravel = ship.EnteringJumpgate(currentObjective.pid);

                if (jumpGateToTravel != 0) {
                    npc.TravelThroughJumpgate(currentObjective.nid, jumpGateToTravel);
                }
                else {
                    var sys_idPlayer = ship.GetSystemID(currentObjective.pid);
                    var sys_idNPC = npc.GetCurrentSystemID(currentObjective.nid);

                    if (sys_idPlayer != sys_idNPC) {
                        //npc is NOT in the same system. For the moment, do not do anything... anyway the NPC AI will be deactivated.
                        return;
                    }
                    else {
                        var currentCom = storage.GetGlobal("currentCommand" + currentObjective.nid);
                        currentCom.command = 0;
                        storage.SetGlobal("currentCommand" + currentObjective.nid, currentCom);

                        npc.NextObjective(currentObjective.nid);
                        npc.CleanObjectives(currentObjective.nid);
                    }
                }
                break;
            }
    }
}













































/*var arrayOfShips = npc.GetShipsInScope(currentObjective.nid);

if (arrayOfShips != null && arrayOfShips.length > 0)
{
    var shipNDist = [];
    for (var i = 0; i < arrayOfShips.length; i++)
    {
        var otherShip = arrayOfShips[i];
        if (generator.ShipExists(otherShip) && ship.GetCurrentValue(otherShip, "structure") > 0) {
            //var playName = ceData.pName.toLowerCase();
            var shipName = game.GetShipOwner(otherShip).toLowerCase();

            if (!SC_Utilities.contains(arrayOfFriendlies, shipName)) // && shipName != ceData.objt.pName.toLowerCase()
            {
                var otherShipCoord = ship.GetCoordinates(otherShip);
                var distPlayerToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.pData.pCoord); // playerCoordinates... But not really since its base vicinity
                var distNpcToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.nData.nCoord); // npc coordinates... But not really since its base vicinity

                if (distPlayerToEnemy <= lockdist && distNpcToEnemy <= lockdist)   //&& distPlayerToDrone <= lockdist// && distToEnemy <= distPlayerToDrone
                {
                    ship.SetDamageByShip(otherShip, 1, ceData.objt.nid);
                    ship.SetDamageByShip(ceData.objt.nid, 1, otherShip);

                    var someData = { id: otherShip, dist: distNpcToEnemy, eToD: 0 };
                    shipNDist.push(someData);
                }
            }
        }
    }

    if (shipNDist != null)
    {
        if (shipNDist.length > 0)
        {
            shipNDist.sort(function (a, b)
            {
                var adist = a.dist;
                var bdist = b.dist;
                var acdam = a.damage;
                var bcdam = b.damage;

                if (adist < bdist) {
                    return 1;
                }
                else if (adist > bdist) {
                    return -1;
                }
                else {
                    return 0;
                }
            });
        }
    }
}
else
{
    //start priorities
}*/


















/*if (currentObjective.aRep == 1)
{
    //auto dock and repair when in range of base.

    if (arrayOfDroneAutoRepair.length < globIndex + 1)
    {
        for (var i = 0; i < (globIndex + 1) - arrayOfDroneAutoRepair.length; i++)
        {
            arrayOfDroneAutoRepair.push({ counter: 0 });
        }
    }
    else
    {
        if (arrayOfDroneAutoRepair[currentObjective.droneIndex].counter == 0)
        {
            var currentHP = ship.GetCurrentValue(currentObjective.nid, "structure");
            var maxHP = ship.GetFinalCacheValue(currentObjective.nid, "structure");

            if (currentHP < maxHP)
            {
                var current_base = currentObjective.bid;

                if (npc.IsAtBaseEntry(currentObjective.nid, current_base))
                {
                    npc.EnterBase(currentObjective.nid, current_base);
                    arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 1;
                }
                else
                {
                    npc.FollowRouteToBase(currentObjective.nid, current_base);
                }
            }
        }
    }



    /*if (arrayOfDroneAutoRepair.length < globIndex + 1)
    {
        for (var i = 0; i < (globIndex + 1) - arrayOfDroneAutoRepair.length; i++)
        {
            arrayOfDroneAutoRepair.push({ counter: 0 });
        }
    }
    else
    {
        //even if auto repair is at ON, do NOT send the drone back to repair if the player doesnt have money.
        if (arrayOfDroneAutoRepair[currentObjective.droneIndex].counter == 0) {
            var currentHP = ship.GetCurrentValue(currentObjective.nid, "structure");
            var maxHP = ship.GetFinalCacheValue(currentObjective.nid, "structure");

            if (currentHP < maxHP) {
                var current_base = currentObjective.bid;

                if (npc.IsAtBaseEntry(currentObjective.nid, current_base)) {
                    npc.EnterBase(currentObjective.nid, current_base);
                    arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 1;
                }
                else {
                    npc.FollowRouteToBase(currentObjective.nid, current_base);
                }
            }
        }
        else if (arrayOfDroneAutoRepair[currentObjective.droneIndex].counter == 1) {
            //if drone is docked and has repaired, it can undock and it will return to other duties or anything of the sort.
            var currentHP = ship.GetCurrentValue(currentObjective.nid, "structure");
            var maxHP = ship.GetFinalCacheValue(currentObjective.nid, "structure");

            if (currentHP < maxHP) {
                //repair ship. check if player has money... if no money then not able to repair.

                if (arrayOfDroneAutoDefense.length < globIndex + 1) {
                    for (var i = 0; i < (globIndex + 1) - arrayOfDroneAutoDefense.length; i++) {
                        arrayOfDroneAutoDefense.push({ counter: 0 });
                    }
                }
                else {
                    if (arrayOfDroneAutoDefense[currentObjective.droneIndex].counter == 0) {
                        var isPlayerDocked = ship.IsDocked(currentObjective.pid);
                        var isNpcDocked = ship.IsDocked(currentObjective.nid);

                        if (isPlayerDocked) {
                            //stay in base.
                            arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 2;
                        }
                        else {
                            if (isNpcDocked) {
                                //undock then go in formation.
                                arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 2;
                            }
                            else {
                                //go in formation.
                                arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 2;
                            }
                        }
                    }
                    else {
                        if (isNpcDocked) {
                            //undock then go defend base
                            arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 2;
                        }
                        else {
                            //go defend base
                            arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 2;
                        }
                    }
                }
            }
            else
            {
                if (arrayOfDroneAutoDefense.length < globIndex + 1)
                {
                    for (var i = 0; i < (globIndex + 1) - arrayOfDroneAutoDefense.length; i++)
                    {
                        arrayOfDroneAutoDefense.push({ counter: 0 });
                    }
                }
                else
                {
                    if (arrayOfDroneAutoDefense[currentObjective.droneIndex].counter == 0) {
                        var isPlayerDocked = ship.IsDocked(currentObjective.pid);
                        var isNpcDocked = ship.IsDocked(currentObjective.nid);

                        if (isPlayerDocked) {
                            //stay in base.
                            arrayOfDroneAutoDefense[currentObjective.droneIndex].counter = 2;
                        }
                        else {
                            if (isNpcDocked) {
                                //undock then go in formation.
                                arrayOfDroneAutoDefense[currentObjective.droneIndex].counter = 2;
                            }
                            else {
                                //go in formation.
                                arrayOfDroneAutoDefense[currentObjective.droneIndex].counter = 2;
                            }
                        }
                    }
                    else {
                        if (isNpcDocked) {
                            //undock then go defend base
                            arrayOfDroneAutoDefense[currentObjective.droneIndex].counter = 2;
                        }
                        else {
                            //go defend base
                            arrayOfDroneAutoDefense[currentObjective.droneIndex].counter = 2;
                        }
                    }
                }
            }
        }
        else if (arrayOfDroneAutoRepair[currentObjective.droneIndex].counter == 2)
        {

        }
    }
}*/












































                //activate pathfind to only go on tiles that are accessible... to start off.

/*if (currentObjective.formation == 1) {
    globIndex = storage.GetGlobal("maxDroneIndex0");
}
else if (currentObjective.formation == 2)
{
    globIndex = storage.GetGlobal("maxDroneIndex1");
}
else if (currentObjective.formation == 3)
{
    globIndex = storage.GetGlobal("maxDroneIndex2");
}
else if (currentObjective.formation == 4)
{
    globIndex = storage.GetGlobal("maxDroneIndex3");
}
else if (currentObjective.formation == 5)
{
    globIndex = storage.GetGlobal("maxDroneIndex4");
}


if (arrayOfDronePathfindData.length < globIndex + 1)
{
    for (var i = 0; i < (globIndex + 1) - arrayOfDronePathfindData.length; i++)
    {
        arrayOfDronePathfindData.push({ counter: 0 });
    }
}
else
{
    if (arrayOfDronePathfindData[currentObjective.droneIndex].counter == 0)
    {
        //var currentFormationWaypoint = SC_AI_Drone_Combat_cc_Rout_For_FWP_2.npcGWFP(currentObjective);
        //var distToWaypoint = SC_Utilities.GetDistance(currentFormationWaypoint, nData.nCoord);
        //arrayOfDronePathfindData[currentObjective.nid].counter = 1;

        var current_base = currentObjective.bid;

        if (npc.IsAtBaseEntry(currentObjective.nid, current_base))
        {
            //console.PrintError("player has docked 000");
            npc.EnterBase(currentObjective.nid, current_base);
        }
        else
        {
            //var sys_idPlayer = ship.GetSystemID(currentObjective.pid);
            //var theBase = storage.GetGlobal("system_" + sys_idPlayer + "_base_" + current_base + "_xmlStationType");

            //console.PrintError("player has docked 001");
            npc.FollowRouteToBase(currentObjective.nid, current_base);
        }
    }
    else
    {

    }
}*/


















/*if (currentObjective.droneIndex > globIndex)
{
    //console.PrintError("adding drone index");
    for (var i = 0; i < currentObjective.droneIndex - globIndex; i++)
    {

        globIndex++;
    }

    storage.SetGlobal("maxDroneIndex0", globIndex);
}
else if (currentObjective.droneIndex <= globIndex)
{
    //ok.
}*/
















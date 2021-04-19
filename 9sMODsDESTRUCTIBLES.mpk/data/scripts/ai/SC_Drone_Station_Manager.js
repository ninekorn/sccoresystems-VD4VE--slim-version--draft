using(npc);
using(console);
using(player);
using(ship);
using(game);
using(generator);
using(storage);
using(station);
using(items);
using(timer);

var player_id;
var playerName;
var sys_id;
var sys_idPlayer;

include(SC_Utilities.js);

var mainFrameWaitForGameLoadingInit = 0;
var mainFrameWaitForGameLoadingInitMax = 30; // 30 to 60 for 10 or 20 seconds for 3 decisions per second of waiting before starting.
var initStarterVariables = -1;

var npcDecisionsPerSeconds = 1; //1 minimum

var arrayOfRooftopVisualSwtc = 0;
var arrayOfRooftopVisualCounter = 0;
var arrayOfRooftopVisualCounterMAX = 10;

// 2 frame behind the spawning time so that the object isn't deleted for the end of the remaining frame where it is spawned at counter 0, then 1 full frame at counter 1 for the
// engine to gobble up the fact it now has a new container in it's memory, than we delete it if the player is in range... let me check if i did that right.

var arrayOfRooftopVisual = [];

function OnAIInited()
{
    //SHIP_ID = SHIP_ID;
    //npc.SetDecisionsPerSecond(SHIP_ID, 3);
    npcDecisionsPerSeconds = 19;
    npc.SetDecisionsPerSecond(SHIP_ID, npcDecisionsPerSeconds);

    //player_id = npc.GetTag(SHIP_ID, "ownerPlayerShipId");

    storage.SetGlobal("npcDecisionsPerSeconds" + SHIP_ID, npcDecisionsPerSeconds);

    npc.SetBehavior(SHIP_ID, "avoid_asteroids", false);
    npc.SetBehavior(SHIP_ID, "avoid_ships", false);
    npc.SetBehavior(SHIP_ID, "avoid_bases", false);
    npc.SetBehavior(SHIP_ID, "avoid_debris", false);

    //NpcLib.SetupDefaultWeaponUsage(SHIP_ID);
    npc.SetWeaponUsage(SHIP_ID, "Ray", 0, 7, 0.001);
    npc.SetWeaponUsage(SHIP_ID, "BulletOrMissile", 2, 7, 0.001);
    npc.SetWeaponUsage(SHIP_ID, "DirectHit", 2, 7, 0.001);
}

var currentIndex = -1;
var mainTotalIndexOfStationDroneManager = -1;

var droneToGiveCommands = [];
var droneInSystem = [];


var newDrones = [];

var sdmi;
var droneNDistPerStation = [];
var LastdroneNDistPerStation = [];


var current_base;


var maxFrameCounterForDroneViscOrNotResetCheck = 150;

var viscinityStationDistIn = 750; //750 // 700
var viscinityStationDistMiddle = 725; //700 //650
var viscinityStationDistOut = 700; //700 //650

//MINING 750/700
//BUSINESS 500/450
//OUTPOST 700/650
//SCIENCE 800/750
//MILITARY 900/850



var sys_id;

var droneMaxFrameBeforeStationChecksIt = 10;
var stationManagerCoords = null;

var initLoadingTimeWaitCounter = 0;
var initLoadingTimeWaitCounterMAX = 12;//40 original
var initLoadingTimeWaitCounterMAXOriginal = 12;//40 original
var initLoadingTimeWaitCounterSwtc = 0;

var counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBase = 0;
var counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAX = 3;
var counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAXOriginal = 3;
var counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAXREDUCEFORCE = 0;

var playerInRangeInverseForcePlayerImpedenceSwtch = -1;
var distMaxForplayerInRangeInverseForcePlayerImpedence = 22; //23 great except the ship is destroyed in a crash at start of game.

var counterForResetRoof = 0;
var counterForResetRoofMAX = 3;
var counterForResetRoofMAXOriginal = 3;
var playerInRange = -1;
var distMaxForRoofVisual = 15; //15 original

var maxFrameForCommandOption1 = 0;


function Decision(args) {

    if (storage.IsSetGlobal("npcDecisionsPerSeconds" + SHIP_ID)) {

        var npcDecisionsPerSeconds = storage.GetGlobal("npcDecisionsPerSeconds" + SHIP_ID);

    

        if (npcDecisionsPerSeconds == 1)
        {
            initLoadingTimeWaitCounterMAX = initLoadingTimeWaitCounterMAXOriginal * npcDecisionsPerSeconds;
            counterForResetRoofMAX = 0;//counterForResetRoofMAX * npcDecisionsPerSeconds;
            counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAX = 0;//counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAX * npcDecisionsPerSeconds;
            counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAXOriginal = 0;//counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAXOriginal * npcDecisionsPerSeconds;

            //maxFrameForCommandOption1 = 30;//COMPUTER
            //maxFrameForCommandOption2 = 60;//RELEASE
            //maxFrameForCommandOption3 = 60;//COMBAT/MINING/REPAIR
            //maxFrameForCommandOption4 = 60;
        }
        else //if (npcDecisionsPerSeconds == 19) 
        {
            initLoadingTimeWaitCounterMAX = initLoadingTimeWaitCounterMAXOriginal * npcDecisionsPerSeconds;
            counterForResetRoofMAX = counterForResetRoofMAXOriginal * npcDecisionsPerSeconds;
            /*
            counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAX = counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAX * npcDecisionsPerSeconds;
            counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAXOriginal = counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAXOriginal * npcDecisionsPerSeconds;
            */




            //maxFrameForCommandOption1 = 30 * npcDecisionsPerSeconds;//COMPUTER
            //maxFrameForCommandOption2 = 60 * npcDecisionsPerSeconds;//RELEASE
            //maxFrameForCommandOption3 = 60 * npcDecisionsPerSeconds;//COMBAT/MINING/REPAIR
            //maxFrameForCommandOption4 = 60 * npcDecisionsPerSeconds;

        }

        //storage.SetGlobal("npcDecisionsPerSeconds" + player_id, npcDecisionsPerSeconds);
    }

    ////console.PrintError("station manager is alive");

    if (initStarterVariables == -1 || sys_id == null || stationManagerCoords == null || stationManagerCoords.x == null || stationManagerCoords.y == null || current_base == null) {
        //console.PrintError("resetted variables");
        sys_id = npc.GetCurrentSystemID(SHIP_ID);
        stationManagerCoords = npc.GetCurrentCoordinates(SHIP_ID);
        current_base = ship.GetCurrentBase(SHIP_ID);
        initStarterVariables = 1;
    }

    if (initLoadingTimeWaitCounterSwtc == 0) {

        //console.PrintError("c:" + initLoadingTimeWaitCounter);
        if (initLoadingTimeWaitCounter >= initLoadingTimeWaitCounterMAX) {

            //console.PrintError("loading time is over. rooftop visual enabled.");
            initLoadingTimeWaitCounterSwtc = 1;
            initLoadingTimeWaitCounter = 0;
        }

        initLoadingTimeWaitCounter++;
    }




    if (mainFrameWaitForGameLoadingInit >= mainFrameWaitForGameLoadingInitMax) {
     
        //var scopeObjects = game.GetSystemShips(sys_id);






        //var pirate_ships = game.GetSystemShipsOfClass(sys_id, "pirate");
        //var aliensoldier_ships = game.GetSystemShipsOfClass(sys_id, "order.soldier");
        //var ordersoldier_ships = game.GetSystemShipsOfClass(sys_id, "alien.soldier");
        //var fanaticssoldier_ships = game.GetSystemShipsOfClass(sys_id, "fanatics.soldier");
        //var merchant_ships = game.GetSystemShipsOfClass(sys_id, "merchant");
        //var patrol_ships = game.GetSystemShipsOfClass(sys_id, "Patrol");
        //var miner_ships = game.GetSystemShipsOfClass(sys_id, "miner");
        //var alienhive_ships = game.GetSystemShipsOfClass(sys_id, "alien_hive");*/



        //var scopeObjects = npc.GetShipsInScope(SHIP_ID);
        //////console.PrintError("test");
        //var scopeObjects = npc.GetShipsInScope(SHIP_ID);//args.scope_ships;
        //////console.PrintError("id: " + scopeObjects[0] + " length " + scopeObjects.length);



        //SALVAGEABLE OBJECTS
        //SALVAGEABLE OBJECTS
        //SALVAGEABLE OBJECTS
        var systemSalvageableObjects = storage.GetGlobal("systemid_" + sys_id + "_salvage");

        if (systemSalvageableObjects != null) {
            if (systemSalvageableObjects.length > 0) {
                ////console.PrintError("salvageable length" + systemSalvageableObjects.length);

                for (var s = 0; s < systemSalvageableObjects.length; s++) {
                    if (Math.abs(systemSalvageableObjects[s].GameTime - timer.GetGameTime()) > 300) {
                        generator.RemoveSpaceObject(systemSalvageableObjects[s].id);
                    }
                }
            }
        }
        //SALVAGEABLE OBJECTS
        //SALVAGEABLE OBJECTS
        //SALVAGEABLE OBJECTS







        //LOGIC FOR THE STATION DRONE MANAGER TO FIND DRONES IN THE STATION VISCINITY.
        //LOGIC FOR THE STATION DRONE MANAGER TO FIND DRONES IN THE STATION VISCINITY.
        //LOGIC FOR THE STATION DRONE MANAGER TO FIND DRONES IN THE STATION VISCINITY.
        droneInSystem = [];
        //droneShips = [];
        mainTotalIndexOfStationDroneManager = storage.GetGlobal("StationDroneManagerIndex");
        sdmi = parseInt(npc.GetTag(SHIP_ID, "stationDroneIndex"));

        if (droneNDistPerStation.length < mainTotalIndexOfStationDroneManager) // if the main array of manager drones doesnt contain the index of this station
        {
            var diff = mainTotalIndexOfStationDroneManager + 1 - droneNDistPerStation.length;
            for (var ia = 0; ia < diff; ia++) {
                var droneData = { npcid: null, dist: 0, droneResetCounterCheck: 0, droneResetCounterCheckInit: 1, droneResetCounterCheckMAX: droneMaxFrameBeforeStationChecksIt, isInBaseVicinity: -1, isInBaseVicinityLast: -1 };
                var someData = [];
                someData.push(droneData);
                droneNDistPerStation.push(null);
            }
        }
        else {

            var droneShips = game.GetSystemShipsOfClass(sys_id, "drone");

            if (droneShips.length > 0) {
                if (droneNDistPerStation[sdmi] != null) {
                    if (droneNDistPerStation[sdmi].data != null) {
                        var indexArray = [];
                        var tempArray = [];
                        for (var j = 0; j < droneNDistPerStation[sdmi].data.length; j++) {

                            if (droneNDistPerStation[sdmi].data[j].npcid != null) {
                                tempArray.push(droneNDistPerStation[sdmi].data[j].npcid);
                                indexArray.push(j);
                            }
                        }

                        ////console.PrintError("tempArray: " + tempArray.length + " id "); // tempArray.push(droneNDistPerStation[sdmi].data[0].npcid)

                        if (tempArray.length > 0) {

                            for (var j = 0; j < droneShips.length; j++) {

                                ////console.PrintError("tempArray: " + tempArray.length + " id ");
                                var someContains = SC_Utilities.containsNReturnIndex(tempArray, droneShips[j]);

                                if (someContains.result == true) {
                                    ////console.PrintError(" index0 " + someContains.index + " index1 " + indexArray[someContains.index] + " shipid " + droneShips[j]);
                                    ////console.PrintError("contains shipid " + droneShips[j]);
                                    if (droneNDistPerStation[sdmi].data != null) {
                                        ////console.PrintError("!null");
                                        if (droneNDistPerStation[sdmi].data[indexArray[someContains.index]].droneResetCounterCheck > droneNDistPerStation[sdmi].data[indexArray[someContains.index]].droneResetCounterCheckMAX) {

                                            ////console.PrintError("Max frame " + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].droneResetCounterCheckMAX + " reached for drone id " + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid + ". checking drone id " + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid + " distance to this manager station id " + SHIP_ID + " for system " + sys_id + " for the current base id " + current_base + ".");



                                            var currentDroneCoords = npc.GetCurrentCoordinates(droneShips[j]);
                                            var droneDist = SC_Utilities.npcCheckDistance(stationManagerCoords, currentDroneCoords);



                                            //droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid
                                            if (droneDist < viscinityStationDistOut) {

                                                if (droneNDistPerStation[sdmi].data[indexArray[someContains.index]].isInBaseVicinity == -1) {
                                                    var globDroneData = storage.GetGlobal("currentCommand" + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid);
                                                    ////console.PrintError("npcid0:" + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid + " globDroneData.command " + globDroneData.command);
                                                    ////console.PrintError("is in base viscinity " + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid);
                                                    if (globDroneData.command != 1) {
                                                        globDroneData.command = 1;
                                                        globDroneData.bid = current_base;
                                                        //globDroneData.bidSwtch = 1;
                                                        globDroneData.isInBaseVicinity = 1;
                                                        storage.SetGlobal("currentCommand" + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid, globDroneData);
                                                        droneNDistPerStation[sdmi].data[indexArray[someContains.index]].isInBaseVicinity = 1;
                                                        droneNDistPerStation[sdmi].data[indexArray[someContains.index]].dist = droneDist;
                                                        //console.PrintError("changed drone to INSIDE station viscinity" + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid);

                                                    }

                                                    //droneNDistPerStation[sdmi].data[indexArray[someContains.index]].isInBaseVicinity = 1;
                                                }
                                                else if (droneNDistPerStation[sdmi].data[indexArray[someContains.index]].isInBaseVicinity == 1) {

                                                    var globDroneData = storage.GetGlobal("currentCommand" + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid);
                                                    ////console.PrintError("npcid0:" + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid + " globDroneData.command " + globDroneData.command);
                                                    ////console.PrintError("is in base viscinity " + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid);
                                                    if (globDroneData.command != 1) {
                                                        globDroneData.command = 1;
                                                        globDroneData.bid = current_base;
                                                        //globDroneData.bidSwtch = 1;
                                                        globDroneData.isInBaseVicinity = 1;
                                                        storage.SetGlobal("currentCommand" + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid, globDroneData);
                                                        droneNDistPerStation[sdmi].data[indexArray[someContains.index]].isInBaseVicinity = 1;
                                                        droneNDistPerStation[sdmi].data[indexArray[someContains.index]].dist = droneDist;
                                                        //console.PrintError("changed drone to INSIDE station viscinity" + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid);

                                                    }

                                                    ////console.PrintError("is in base viscinity. checking again later. " + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid);
                                                }
                                                else if (droneNDistPerStation[sdmi].data[indexArray[someContains.index]].isInBaseVicinity == 2) {
                                                    //do nothing
                                                    //the drone has just arrived in station viscinity. check global variable command
                                                }

                                                ////console.PrintError("< viscinityStationDistIn");
                                            }
                                            else {

                                                if (droneDist >= viscinityStationDistIn) {

                                                    if (droneNDistPerStation[sdmi].data[indexArray[someContains.index]].isInBaseVicinity == 1) {

                                                        /*//console.PrintError("is NOT in base viscinity " + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid);
                                                        var globDroneData = storage.GetGlobal("currentCommand" + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid);
                                                        ////console.PrintError("npcid1:" + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid + " globDroneData.command " + globDroneData.command);

                                                        if (globDroneData.command == 1) {
                                                            globDroneData.command = 0;
                                                            globDroneData.bid = null;
                                                            globDroneData.isInBaseVicinity = -1;
                                                            storage.SetGlobal("currentCommand" + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid, globDroneData);
                                                            droneNDistPerStation[sdmi].data[indexArray[someContains.index]].isInBaseVicinity = -1;
                                                            droneNDistPerStation[sdmi].data[indexArray[someContains.index]].dist = droneDist;
                                                            //console.PrintError("changed drone to outside station viscinity" + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid);
                                                        }*/

                                                    }
                                                }
                                                else {
                                                    ////console.PrintError("is NOT in base viscinity " + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid);
                                                    var globDroneData = storage.GetGlobal("currentCommand" + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid);
                                                    ////console.PrintError("npcid1:" + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid + " globDroneData.command " + globDroneData.command);

                                                    if (globDroneData.command == 0) {
                                                        globDroneData.command = 1;
                                                        globDroneData.bid = current_base;
                                                        globDroneData.isInBaseVicinity = 1;
                                                        storage.SetGlobal("currentCommand" + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid, globDroneData);
                                                        droneNDistPerStation[sdmi].data[indexArray[someContains.index]].isInBaseVicinity = -1;
                                                        droneNDistPerStation[sdmi].data[indexArray[someContains.index]].dist = droneDist;
                                                        //console.PrintError("changed drone to outside station viscinity" + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid);
                                                    }
                                                }
                                                //////console.PrintError("> viscinityStationDistIn");
                                                //if (droneNDistPerStation[sdmi].data[indexArray[someContains.index]].isInBaseVicinity == 2) {
                                                //                              
                                                //    //do nothing
                                                //    //the drone has just arrived in station viscinity. check global variable command
                                                //}
                                            }
                                            droneNDistPerStation[sdmi].data[indexArray[someContains.index]].droneResetCounterCheck = 0;
                                        }

                                        ////console.PrintError("counter " + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].droneResetCounterCheck + " id " + droneNDistPerStation[sdmi].data[indexArray[someContains.index]].npcid);


                                        droneNDistPerStation[sdmi].data[indexArray[someContains.index]].droneResetCounterCheck++;
                                        droneNDistPerStation[sdmi].data[indexArray[someContains.index]].isInBaseVicinityLast = droneNDistPerStation[sdmi].data[indexArray[someContains.index]].isInBaseVicinity;
                                    }
                                    else {
                                        ////console.PrintError("null");
                                    }

                                }
                                else {
                                    ////console.PrintError("!contains shipid " + droneShips[j]);
                                    //var stationManagerCoords = npc.GetCurrentCoordinates(SHIP_ID);

                                    var currentDroneCoords = npc.GetCurrentCoordinates(droneShips[j]);

                                    var droneDist = SC_Utilities.npcCheckDistance(stationManagerCoords, currentDroneCoords);

                                    ////console.PrintError("!contains " + " shipid " + droneShips[j]);

                                    var droneData = { npcid: droneShips[j], dist: droneDist, droneResetCounterCheck: 0, droneResetCounterCheckInit: 1, droneResetCounterCheckMAX: droneMaxFrameBeforeStationChecksIt, isInBaseVicinity: 1, isInBaseVicinityLast: 1 };
                                    droneNDistPerStation[sdmi].data.push(droneData);
                                }
                            }
                        }
                        else {
                            ////console.PrintError("tempArray == 0");
                            for (var j = 0; j < droneShips.length; j++) {
                                //var stationManagerCoords = npc.GetCurrentCoordinates(SHIP_ID);

                                var currentDroneCoords = npc.GetCurrentCoordinates(droneShips[j]);

                                var droneDist = SC_Utilities.npcCheckDistance(stationManagerCoords, currentDroneCoords);


                                if (droneDist < viscinityStationDistOut) {
                                    var droneData = { npcid: droneShips[j], dist: droneDist, droneResetCounterCheck: 0, droneResetCounterCheckInit: 1, droneResetCounterCheckMAX: droneMaxFrameBeforeStationChecksIt, isInBaseVicinity: 1, isInBaseVicinityLast: 1 };
                                    droneNDistPerStation[sdmi].data.push(droneData);
                                }
                                else {

                                }
                            }
                        }

                    }
                    else {
                        droneNDistPerStation[sdmi].data = [];
                        for (var j = 0; j < droneShips.length; j++) {
                            //var stationManagerCoords = npc.GetCurrentCoordinates(SHIP_ID);

                            var currentDroneCoords = npc.GetCurrentCoordinates(droneShips[j]);

                            var droneDist = SC_Utilities.npcCheckDistance(stationManagerCoords, currentDroneCoords);

                            //{ npcid: null, dist: 0, droneResetCounterCheck: 0, droneResetCounterCheckInit: 1, droneResetCounterCheckMAX: droneMaxFrameBeforeStationChecksIt, isInBaseVicinity: -1, isInBaseVicinityLast: -1 }
                            var droneData = { npcid: droneShips[j], dist: droneDist, droneResetCounterCheck: 0, droneResetCounterCheckInit: 1, droneResetCounterCheckMAX: droneMaxFrameBeforeStationChecksIt, isInBaseVicinity: -1, isInBaseVicinityLast: -1 };
                            //var someData = [];
                            //someData.push(droneData);
                            droneNDistPerStation[sdmi].data.push(droneData);
                        }
                    }
                }
                else {
                    ////console.PrintError("init null sdmi");
                    var droneData = { npcid: null, dist: 0, droneResetCounterCheck: 0, droneResetCounterCheckInit: 1, droneResetCounterCheckMAX: droneMaxFrameBeforeStationChecksIt, isInBaseVicinity: -1, isInBaseVicinityLast: -1 };
                    //var someData = [];
                    //someData.push(droneData);
                    droneNDistPerStation[sdmi] = [];

                    droneNDistPerStation[sdmi].push({ data: droneData });
                }
            }

        }
    }
    mainFrameWaitForGameLoadingInit++;
    //var sys_idtwo = npc.GetCurrentSystemID(droneNDistPerStation[sdmi].npcid);
    //LOGIC FOR THE STATION DRONE MANAGER TO FIND DRONES IN THE STATION VISCINITY.
    //LOGIC FOR THE STATION DRONE MANAGER TO FIND DRONES IN THE STATION VISCINITY.
    //LOGIC FOR THE STATION DRONE MANAGER TO FIND DRONES IN THE STATION VISCINITY.













    /*if (arrayOfRooftopVisualCounter > arrayOfRooftopVisualCounterMAX) {

        if (arrayOfRooftopVisualSwtc == 1) {
            //remove everything except the 2 last jumpgates
            for (var i = 0; i < arrayOfRooftopVisual.length - 2; i++) {
                game.DeleteGameObject(arrayOfRooftopVisual[i]);
            }
            arrayOfRooftopVisualSwtc = 0;
        }
        else {
            for (var i = 0; i < arrayOfRooftopVisual.length; i++) {
                game.DeleteGameObject(arrayOfRooftopVisual[i]);
            }
        }
        arrayOfRooftopVisualCounter = 0;
    }
    arrayOfRooftopVisualCounter++;*/

    //FOR ACTIVATING THE LEFT AND RIGHT ENTRANCE ROOFTOP VISUAL CRATE OBJECT BECAUSE THE INSTANCE OPTION/SETTING IS AVAILABLE IN THE CRATES XML FILES.
    //FOR ACTIVATING THE LEFT AND RIGHT ENTRANCE ROOFTOP VISUAL CRATE OBJECT BECAUSE THE INSTANCE OPTION/SETTING IS AVAILABLE IN THE CRATES XML FILES.
    //FOR ACTIVATING THE LEFT AND RIGHT ENTRANCE ROOFTOP VISUAL CRATE OBJECT BECAUSE THE INSTANCE OPTION/SETTING IS AVAILABLE IN THE CRATES XML FILES.
    if (arrayOfRooftopVisualSwtc == 1)
    {
        if (arrayOfRooftopVisualCounter >= arrayOfRooftopVisualCounterMAX)
        {
            if (arrayOfRooftopVisual.length > 0)
            {
                for (var r = 0; r < arrayOfRooftopVisual.length; r++)
                {
                    if (arrayOfRooftopVisual[i] != null)
                    {
                        var infSys = generator.GetSystemByID(sys_id);
                        if (infSys.name == "player base")
                        {
                            var baseInteriorGlobalVariable = storage.GetGlobal("stationINT" + sys_id);

                            if (baseInteriorGlobalVariable == null) {

                                //console.PrintError("1null");
                            }
                            else
                            {
                                var playerShipCoords = ship.GetCoordinates(player_id);
                                var stationDataINT = storage.GetGlobal("stationINT" + sys_id);
                                var getSomeIndex = stationDataINT.xml_id.substring(11, stationDataINT.xml_id.length); //outpost_01_0
                                var parsedAngle = parseInt(getSomeIndex);

                                var tempCoordINTMIDDLEX = stationDataINT.x;
                                var tempCoordINTMIDDLEY = stationDataINT.y - 12.5;
                                var tempCoordINTMIDDLE = { x: tempCoordINTMIDDLEX, y: tempCoordINTMIDDLEY };

                                //var tempCoordINTRIGHTX = stationDataINT.x + 15.5;
                                //var tempCoordINTRIGHTY = stationDataINT.y - 12.5;
                                //var tempCoordINTRIGHT = { x: tempCoordINTRIGHTX, y: tempCoordINTRIGHTY };

                                var rotatedCoordINTMIDDLE = SC_Utilities.RotatePoint(tempCoordINTMIDDLE, stationDataINT, parsedAngle);
                                //var rotatedCoordINTRIGHT = SC_Utilities.RotatePoint(tempCoordINTRIGHT, stationDataINT, parsedAngle);

                                var degToRadTWO = ((parsedAngle + 180) * Math.PI / 180);

                                var distanceFromPlayerToRoofEntranceMIDDLE = SC_Utilities.GetDistance(rotatedCoordINTMIDDLE, playerShipCoords);

                                if (distanceFromPlayerToRoofEntranceMIDDLE >= distMaxForRoofVisual)
                                {

                                }
                                else
                                {
                                    game.DeleteGameObject(arrayOfRooftopVisual[i]);
                                }
                            }
                        }               
                    }
                }
                arrayOfRooftopVisualCounter = 0;
                arrayOfRooftopVisualSwtc = 0;
            }
            else
            {
                arrayOfRooftopVisualCounter = 0;
                arrayOfRooftopVisualSwtc = 0;
            }

        }
        arrayOfRooftopVisualCounter++;
    }

    if (initLoadingTimeWaitCounterSwtc == 1)
    {
        if (player_id != null) {
            if (counterForResetRoof >= counterForResetRoofMAX && arrayOfRooftopVisualSwtc == 0)
            {
                if (playerInRange == -1) {
                    var listOfCrates = npc.GetSystemCrateContainersByDistanceToNPC(SHIP_ID, 20);

                    if (listOfCrates[0] != null) {

                        for (var i = 0; i < listOfCrates.length; i++) {
                            //var someclass = npc.GetTag(SHIP_ID, "class");

                            //if (someclass == "outpost_01_roof") {

                            //}
                            var baseInteriorGlobalVariable = storage.GetGlobal("stationINT" + sys_id);

                            if (baseInteriorGlobalVariable == null) {

                                //console.PrintError("0null");
                            }
                            else
                            {
                                if (baseInteriorGlobalVariable.name == "player base")
                                {
                                    //console.PrintError("!null");
                                    var playerShipCoords = ship.GetCoordinates(player_id);
                                    var stationDataINT = storage.GetGlobal("stationINT" + sys_id);
                                    var getSomeIndex = stationDataINT.xml_id.substring(11, stationDataINT.xml_id.length); //outpost_01_0
                                    var parsedAngle = parseInt(getSomeIndex);

                                    var tempCoordINTMIDDLEX = stationDataINT.x;
                                    var tempCoordINTMIDDLEY = stationDataINT.y - 12.5;
                                    var tempCoordINTMIDDLE = { x: tempCoordINTMIDDLEX, y: tempCoordINTMIDDLEY };

                                    /*var tempCoordINTRIGHTX = stationDataINT.x + 15.5;
                                    var tempCoordINTRIGHTY = stationDataINT.y - 12.5;
                                    var tempCoordINTRIGHT = { x: tempCoordINTRIGHTX, y: tempCoordINTRIGHTY };
                                    */
                                    var rotatedCoordINTMIDDLE = SC_Utilities.RotatePoint(tempCoordINTMIDDLE, stationDataINT, parsedAngle);
                                    //var rotatedCoordINTRIGHT = SC_Utilities.RotatePoint(tempCoordINTRIGHT, stationDataINT, parsedAngle);

                                    var degToRadTWO = ((parsedAngle + 180) * Math.PI / 180);

                                    var distanceFromPlayerToRoofEntranceMIDDLE = SC_Utilities.GetDistance(rotatedCoordINTMIDDLE, playerShipCoords);
                                    //console.PrintError("0dist: " + distanceFromPlayerToRoofEntranceMIDDLE);
                                    if (distanceFromPlayerToRoofEntranceMIDDLE < distMaxForRoofVisual) {
                                        counterForResetRoof = 0;
                                        //console.PrintError("0visual rooftop removed.");

                                        if (listOfCrates[i]!= null) {

                                            var coords = game.GetObjectCoordinates(sys_id, listOfCrates[i]);

                                            if (coords != null) {
                                                if (coords.x !== null && coords.y !== null) {

                                                    var isObjectADrone = npc.GetTag(listOfCrates[i], "someClass");

                                                    if (isObjectADrone != null) {
                                                        if (isObjectADrone != "drone") {
                                                            game.DeleteGameObject(listOfCrates[i]);
                                                        }
                                                        else {
                                                            console.PrintError("object is drone. not deleting object id " + listOfCrates[i] + "...");
                                                        }
                                                    }
                                                    else {
                                                        game.DeleteGameObject(listOfCrates[i]);
                                                        playerInRange = -1;
                                                    }
                                                }
                                                else {
                                                    console.PrintError("null coords.x or coords.y");
                                                }
                                            }
                                            else {
                                                console.PrintError("null coords");
                                            }
                                        }
                                        else {
                                            console.PrintError("0 null listOfCrates[i]");
                                        }
                                        //testing
                                        //var containerId0 = items.GetInstancedContainerId(listOfCrates[0], player_id);

                                        //var containerId1 = game.GetGameObjectContainerId(listOfCrates[0]);

                                        ////console.PrintError("id0: " + containerId0 + " id:1 " + containerId1);
                                        //if (generateItems) {
                                        //var difficultyLevel = InternalCalculateDifficultyLevel(objId);
                                        //var itemsListId = "droplist_special_ergrek_terminal_0" + difficultyLevel;
                                        //items.GenerateItemsByItemslist(containerId, itemsListId);
                                        //}

                                        //items.OpenContainer(player_id, listOfCrates[0] );
                                        playerInRange = 1;
                                    }
                                }
                            }
                        }
                    }
                    else {
                        playerInRange = 1;
                    }
                }
                else if (playerInRange == 1) {

                    var systems = generator.GetAllSystems();
    
                    for (var j = 0; j < systems.length; j++) {
                        var inf = generator.GetSystemByID(systems[j]);
                        if (systems[j] == sys_id && inf.name == "player base") {
    
                            var baseInteriorGlobalVariable = storage.GetGlobal("stationINT" + sys_id);
    
                            if (baseInteriorGlobalVariable == null) {
    
                                //console.PrintError("1null");
                            }
                            else {
    
                                var playerShipCoords = ship.GetCoordinates(player_id);
                                var stationDataINT = storage.GetGlobal("stationINT" + sys_id);
                                var getSomeIndex = stationDataINT.xml_id.substring(11, stationDataINT.xml_id.length); //outpost_01_0
                                var parsedAngle = parseInt(getSomeIndex);
    
                                var tempCoordINTMIDDLEX = stationDataINT.x;
                                var tempCoordINTMIDDLEY = stationDataINT.y - 12.5;
                                var tempCoordINTMIDDLE = { x: tempCoordINTMIDDLEX, y: tempCoordINTMIDDLEY };
    
                                //var tempCoordINTRIGHTX = stationDataINT.x + 15.5;
                                //var tempCoordINTRIGHTY = stationDataINT.y - 12.5;
                                //var tempCoordINTRIGHT = { x: tempCoordINTRIGHTX, y: tempCoordINTRIGHTY };
                                
                                var rotatedCoordINTMIDDLE = SC_Utilities.RotatePoint(tempCoordINTMIDDLE, stationDataINT, parsedAngle);
                                //var rotatedCoordINTRIGHT = SC_Utilities.RotatePoint(tempCoordINTRIGHT, stationDataINT, parsedAngle);
    
                                var degToRadTWO = ((parsedAngle + 180) * Math.PI / 180);
    
    
                                var distanceFromPlayerToRoofEntranceMIDDLE = SC_Utilities.GetDistance(rotatedCoordINTMIDDLE, playerShipCoords);
    
                                if (distanceFromPlayerToRoofEntranceMIDDLE >= distMaxForRoofVisual) {
    
    
                                    var item_list = "droplist_empty";
                                    //console.PrintError("1visual rooftop added.");
                                    var containerID = generator.AddContainer(sys_id, baseInteriorGlobalVariable.x, baseInteriorGlobalVariable.y, "outpost_01_roof", item_list, { itemlist: item_list, class: "outpost_01_roof" });
                                    arrayOfRooftopVisual.push(containerID);
                                    arrayOfRooftopVisualSwtc = 1;
                                    counterForResetRoof = 0;
                                    playerInRange = -1;
                                }
                                else if (distanceFromPlayerToRoofEntranceMIDDLE < distMaxForRoofVisual) {
                                    counterForResetRoof = 0;
                                    //console.PrintError("1visual rooftop removed.");
                                    //game.DeleteGameObject(listOfCrates[i]);

                                    var listOfCrates = npc.GetSystemCrateContainersByDistanceToNPC(SHIP_ID, 20);

                                    if (listOfCrates[0] != null) {

                                        for (var i = 0; i < listOfCrates.length; i++) {

                                            if (listOfCrates[i] != null) {

                                                if (listOfCrates[i] != null) {

                                                    var coords = game.GetObjectCoordinates(sys_id, listOfCrates[i]);

                                                    if (coords != null) {
                                                        if (coords.x !== null && coords.y !== null) {

                                                            var isObjectADrone = npc.GetTag(listOfCrates[i], "someClass");

                                                            if (isObjectADrone != null) {
                                                                if (isObjectADrone != "drone") {
                                                                    game.DeleteGameObject(listOfCrates[i]);
                                                                }
                                                                else {
                                                                    console.PrintError("object is drone. not deleting object id " + listOfCrates[i] + "...");
                                                                }
                                                            }
                                                            else {
                                                                game.DeleteGameObject(listOfCrates[i]);
                                                                playerInRange = -1;
                                                            }
                                                        }
                                                        else {
                                                            console.PrintError("null coords.x or coords.y");
                                                        }
                                                    }
                                                    else {
                                                        console.PrintError("null coords");
                                                    }
                                                }
                                                else {
                                                    console.PrintError("1 null listOfCrates[i]");
                                                }
                                            }
                                        }
                                    }
                                }
                            }
    
                        }
                        else {
                            continue;
                        }
                    }
                }
            }
        }
        else {
            var ships = game.GetSystemShips(sys_id);

            if (ships != null) {
                for (var i = 0; i < ships.length; i++) {
                    if (ships[i] != null) {
                        var playerSysID = ship.GetSystemID(ships[i]);

                        if (playerSysID == sys_id) {
                            //var isPlayer = !ship.IsNpc(ships[i]);
                            var isPlayer = game.IsShipPlayerControlled(ships[i]);
                            if (isPlayer) {
                                player_id = ships[i];

                            }
                        }
                    }
                }
            }
        }
    }
    counterForResetRoof++;
    //FOR ACTIVATING THE LEFT AND RIGHT ENTRANCE ROOFTOP VISUAL CRATE OBJECT BECAUSE THE INSTANCE OPTION/SETTING IS AVAILABLE IN THE CRATES XML FILES.
    //FOR ACTIVATING THE LEFT AND RIGHT ENTRANCE ROOFTOP VISUAL CRATE OBJECT BECAUSE THE INSTANCE OPTION/SETTING IS AVAILABLE IN THE CRATES XML FILES.
    //FOR ACTIVATING THE LEFT AND RIGHT ENTRANCE ROOFTOP VISUAL CRATE OBJECT BECAUSE THE INSTANCE OPTION/SETTING IS AVAILABLE IN THE CRATES XML FILES.










    //FOR ACTIVATING THE LEFT AND RIGHT ENTRANCE INVERSE FORCE TO STOP THE PLAYER FROM LEAVING THE BASE WHEN THEY ARE INSIDE AND TRYING TO REDUCE/OR NOT TO THE COLLIDERS TOTAL NUMBER.
    //FOR ACTIVATING THE LEFT AND RIGHT ENTRANCE INVERSE FORCE TO STOP THE PLAYER FROM LEAVING THE BASE WHEN THEY ARE INSIDE AND TRYING TO REDUCE/OR NOT TO THE COLLIDERS TOTAL NUMBER.
    //FOR ACTIVATING THE LEFT AND RIGHT ENTRANCE INVERSE FORCE TO STOP THE PLAYER FROM LEAVING THE BASE WHEN THEY ARE INSIDE AND TRYING TO REDUCE/OR NOT TO THE COLLIDERS TOTAL NUMBER.
    if (player_id != null) {
        if (counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBase >= counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAX) {
            if (playerInRangeInverseForcePlayerImpedenceSwtch == -1) {
                var baseInteriorGlobalVariable = storage.GetGlobal("stationINT" + sys_id);

                var inf = generator.GetSystemByID(sys_id);

                var sys_id_player = ship.GetSystemID(player_id);
                var infplayersys = generator.GetSystemByID(sys_id_player);

                if (inf.name == "player base" && infplayersys.name == "player base") {

                    if (baseInteriorGlobalVariable == null) {

                        //console.PrintError("0null");
                    }
                    else {
                        counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAX = counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAXOriginal;

                        ////console.PrintError("0adding force");
                        var playerShipCoords = ship.GetCoordinates(player_id);
                        //var stationDataINT = storage.GetGlobal("stationINT" + sys_id);
                        var getSomeIndex = baseInteriorGlobalVariable.xml_id.substring(11, baseInteriorGlobalVariable.xml_id.length); //outpost_01_0
                        var parsedAngle = parseInt(getSomeIndex);


                        var tempCoordINTMIDDLEX = baseInteriorGlobalVariable.x;
                        var tempCoordINTMIDDLEY = baseInteriorGlobalVariable.y - 12.5;
                        var tempCoordINTMIDDLE = { x: tempCoordINTMIDDLEX, y: tempCoordINTMIDDLEY };

                        /*var tempCoordINTRIGHTX = baseInteriorGlobalVariable.x + 15.5;
                        var tempCoordINTRIGHTY = baseInteriorGlobalVariable.y - 12.5;
                        var tempCoordINTRIGHT = { x: tempCoordINTRIGHTX, y: tempCoordINTRIGHTY };
                        */
                        var rotatedCoordINTMIDDLE = SC_Utilities.RotatePoint(tempCoordINTMIDDLE, baseInteriorGlobalVariable, parsedAngle);
                        //var rotatedCoordINTRIGHT = SC_Utilities.RotatePoint(tempCoordINTRIGHT, baseInteriorGlobalVariable, parsedAngle);

                        var degToRadTWO = ((parsedAngle + 180) * Math.PI / 180);

                        var distancePlayerToForcedStopPivot = SC_Utilities.GetDistance(rotatedCoordINTMIDDLE, playerShipCoords);
                        var directionPlayerToImpedencePivotPointX = rotatedCoordINTMIDDLE.x - playerShipCoords.x;
                        var directionPlayerToImpedencePivotPointY = rotatedCoordINTMIDDLE.y - playerShipCoords.y;
                        var directionPlayerToImpedencePivotPoint = { x: directionPlayerToImpedencePivotPointX, y: directionPlayerToImpedencePivotPointY };
                        var distanceTo = Math.sqrt(directionPlayerToImpedencePivotPoint.x * directionPlayerToImpedencePivotPoint.x + directionPlayerToImpedencePivotPoint.y * directionPlayerToImpedencePivotPoint.y);
                        directionPlayerToImpedencePivotPoint.x /= distanceTo;
                        directionPlayerToImpedencePivotPoint.y /= distanceTo;

                        if (distancePlayerToForcedStopPivot >= distMaxForplayerInRangeInverseForcePlayerImpedence) {

                            //TO KEEP AND MAKE THIS NEGATIVE PART BETTER LATER WITH THE DOT PRODUCT
                            /*var coordsPlayer = ship.GetCoordinates(player_id);
                            var angler = ship.GetRotation(player_id);
                            var radToDegrer = angler * (180.0 / Math.PI);
                            var npcPointX00 = (1 * Math.cos(radToDegrer * Math.PI / 180)) + coordsPlayer.x;
                            var npcPointY00 = (1 * Math.sin(radToDegrer * Math.PI / 180)) + coordsPlayer.y;
                            var dirX00 = npcPointX00 - coordsPlayer.x;
                            var dirY00 = npcPointY00 - coordsPlayer.y;
                            var forwardNPC = { x: dirX00, y: dirY00 };
                            var veloNPC = ship.GetVelocity(player_id);
                            var speedNPC = Math.sqrt(veloNPC.x * veloNPC.x + veloNPC.y * veloNPC.y);
                            veloNPC.x /= speedNPC;
                            veloNPC.y /= speedNPC;*/
                            //TO KEEP AND MAKE THIS NEGATIVE PART BETTER LATER WITH THE DOT PRODUCT

                            if (isNaN(directionPlayerToImpedencePivotPoint.x) || directionPlayerToImpedencePivotPoint.x === 0 || isNaN(directionPlayerToImpedencePivotPoint.y) || directionPlayerToImpedencePivotPoint.y === 0) {

                                //console.PrintError("direction is null?");
                            }

                            //TO READD
                            //game.ApplyForceToObject(player_id, directionPlayerToImpedencePivotPoint.x * 5, directionPlayerToImpedencePivotPoint.y * 5);
                             //TO READD

                            counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBase = 0;
                            //game.DeleteGameObject(listOfCrates[i]);
                            //playerInRangeInverseForcePlayerImpedenceSwtch = 1;
                        }
                        else {
                            //counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAX = 3;

                            //game.ApplyForceToObject(player_id, -directionPlayerToImpedencePivotPoint.x * 3, -directionPlayerToImpedencePivotPoint.y * 3);

                        }
                    }
                }
            }

            /*if (playerInRangeInverseForcePlayerImpedenceSwtch == 1) {
                var systems = generator.GetAllSystems();

                for (var j = 0; j < systems.length; j++) {
                    var inf = generator.GetSystemByID(systems[j]);
                    if (systems[j] == sys_id && inf.name == "player base") {

                        var baseInteriorGlobalVariable = storage.GetGlobal("stationINT" + sys_id);

                        if (baseInteriorGlobalVariable == null) {

                            //console.PrintError("1null");
                        }
                        else {
                            counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAX = counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAXOriginal;
                            //console.PrintError("1adding force");
                            var playerShipCoords = ship.GetCoordinates(player_id);
                            //var baseInteriorGlobalVariable = storage.GetGlobal("stationINT" + sys_id);
                            var getSomeIndex = baseInteriorGlobalVariable.xml_id.substring(11, baseInteriorGlobalVariable.xml_id.length); //outpost_01_0
                            var parsedAngle = parseInt(getSomeIndex);


                            var tempCoordINTMIDDLEX = baseInteriorGlobalVariable.x;
                            var tempCoordINTMIDDLEY = baseInteriorGlobalVariable.y - 12.5;
                            var tempCoordINTMIDDLE = { x: tempCoordINTMIDDLEX, y: tempCoordINTMIDDLEY };

                            //var tempCoordINTRIGHTX = baseInteriorGlobalVariable.x + 15.5;
                            //var tempCoordINTRIGHTY = baseInteriorGlobalVariable.y - 12.5;
                            //var tempCoordINTRIGHT = { x: tempCoordINTRIGHTX, y: tempCoordINTRIGHTY };
                            
                            var rotatedCoordINTMIDDLE = SC_Utilities.RotatePoint(tempCoordINTMIDDLE, baseInteriorGlobalVariable, parsedAngle);
                            //var rotatedCoordINTRIGHT = SC_Utilities.RotatePoint(tempCoordINTRIGHT, baseInteriorGlobalVariable, parsedAngle);

                            var degToRadTWO = ((parsedAngle + 180) * Math.PI / 180);


                            var distancePlayerToForcedStopPivot = SC_Utilities.GetDistance(rotatedCoordINTMIDDLE, playerShipCoords);

                            var directionPlayerToImpedencePivotPointX = rotatedCoordINTMIDDLE.x - playerShipCoords.x;
                            var directionPlayerToImpedencePivotPointY = rotatedCoordINTMIDDLE.y - playerShipCoords.y;
                            var directionPlayerToImpedencePivotPoint = { x: directionPlayerToImpedencePivotPointX, y: directionPlayerToImpedencePivotPointY };

                 

                            if (distancePlayerToForcedStopPivot >= distMaxForplayerInRangeInverseForcePlayerImpedence) {



                                //TO KEEP AND MAKE THIS NEGATIVE PART BETTER LATER WITH THE DOT PRODUCT
                                /*var coordsPlayer = ship.GetCoordinates(player_id);
                                var angler = ship.GetRotation(player_id);
                                var radToDegrer = angler * (180.0 / Math.PI);
                                var npcPointX00 = (1 * Math.cos(radToDegrer * Math.PI / 180)) + coordsPlayer.x;
                                var npcPointY00 = (1 * Math.sin(radToDegrer * Math.PI / 180)) + coordsPlayer.y;
                                var dirX00 = npcPointX00 - coordsPlayer.x;
                                var dirY00 = npcPointY00 - coordsPlayer.y;
                                var forwardNPC = { x: dirX00, y: dirY00 };
                                var veloNPC = ship.GetVelocity(player_id);
                                var speedNPC = Math.sqrt(veloNPC.x * veloNPC.x + veloNPC.y * veloNPC.y);
                                veloNPC.x /= speedNPC;
                                veloNPC.y /= speedNPC;
                                //TO KEEP AND MAKE THIS NEGATIVE PART BETTER LATER WITH THE DOT PRODUCT

                                game.ApplyForceToObject(player_id, directionPlayerToImpedencePivotPoint.x * 5, directionPlayerToImpedencePivotPoint.y * 5);




                                /*var rotationDotForward = SC_Utilities.Dot(forwardNPC.x, forwardNPC.y, veloNPC.x, veloNPC.y);

                                if (!isNaN(rotationDotForward) || rotationDotForward != 0) //player is not moving
                                {
                                    if (!isNaN(lastFramerotationDOTVELO)) //player is not moving
                                    {
                                        if (rotationDotForward < lastFramerotationDOTVELO) {
                                            npc.InstantStop(cData.objt.nid);
                                        }
                                    }
                                    else {

                                        if (!isNaN(rotationDOTVELO)) //player is not moving
                                        {
                                            if (rotationDotForward < rotationDOTVELO) {
                                                npc.InstantStop(cData.objt.nid);
                                            }
                                        }
                                    }
                                }
                                
                                //game.ApplyForceToObject(153, 10, 0);
                                //game.ApplyTorqueToObject(153, 10)

                                //generator.AddContainer(sys_id, baseInteriorGlobalVariable.x, baseInteriorGlobalVariable.y, "outpost_01_roof", item_list, { itemlist: item_list, class: "outpost_01_roof" });
                                counterForResetRoof = 0;
                                playerInRangeInverseForcePlayerImpedenceSwtch = -1;
                            }
                            else {
                                counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAX = 3;

                                game.ApplyForceToObject(player_id, -directionPlayerToImpedencePivotPoint.x * 3, -directionPlayerToImpedencePivotPoint.y * 3);
                                counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBaseMAXREDUCEFORCE++;
                            }
                        }

                    }
                    else {
                        continue;
                    }
                }
            }*/







        }
    }
    else {
        var ships = game.GetSystemShips(sys_id);

        if (ships != null) {
            for (var i = 0; i < ships.length; i++) {
                if (ships[i] != null) {
                    var playerSysID = ship.GetSystemID(ships[i]);

                    if (playerSysID == sys_id) {
                        //var isPlayer = !ship.IsNpc(ships[i]);
                        var isPlayer = game.IsShipPlayerControlled(ships[i]);
                        if (isPlayer) {
                            player_id = ships[i];

                        }
                    }
                }
            }
        }
    }

    counterForResetInverseForceToImpedePlayerFromLeavingTooFarOutOfTheBase++;
    //FOR ACTIVATING THE LEFT AND RIGHT ENTRANCE INVERSE FORCE TO STOP THE PLAYER FROM LEAVING THE BASE WHEN THEY ARE INSIDE AND TRYING TO REDUCE/OR NOT TO THE COLLIDERS TOTAL NUMBER.
    //FOR ACTIVATING THE LEFT AND RIGHT ENTRANCE INVERSE FORCE TO STOP THE PLAYER FROM LEAVING THE BASE WHEN THEY ARE INSIDE AND TRYING TO REDUCE/OR NOT TO THE COLLIDERS TOTAL NUMBER.
    //FOR ACTIVATING THE LEFT AND RIGHT ENTRANCE INVERSE FORCE TO STOP THE PLAYER FROM LEAVING THE BASE WHEN THEY ARE INSIDE AND TRYING TO REDUCE/OR NOT TO THE COLLIDERS TOTAL NUMBER.





}




function ProcessObjectives(nextCommandToDispatch, args) {
    /*currentObjective = npc.GetCurrentObjective(nextCommandToDispatch.id);

    if (currentObjective == null) {
        return;
    }

    switch (currentObjective.name) {
        case "get_in_formation":
            {
                formationCore(nextCommandToDispatch, args);
                break;
            }
    }*/
}

function Dot(aX, aY, bX, bY) {
    return (aX * bX) + (aY * bY);
}

function finder(cmp, arr, getter) {
    var val = getter(arr[0]);
    for (var i = 1; i < arr.length; i++) {
        val = cmp(val, getter(arr[i]));
    }
    return val;
}

function GetDistance(x1, y1, x2, y2) {
    return Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
}

function contains(a, obj) {
    for (var i = 0; i < a.length; i++) {
        if (a[i] === obj) {
            return true;
        }
    }
    return false;
}

function alphaOnly(a) {
    var b = '';
    for (var i = 0; i < a.length; i++) {
        if (a[i] >= 'A' && a[i] <= 'z') b += a[i];
    }
    return b;
}

function numberOnly(a) {
    var b = '';
    for (var i = 0; i < a.length; i++) {
        if (a[i] >= '0' && a[i] <= '9') b += a[i];
    }
    return b;
}




function OnUpdateCache(args) {

}





/*function OnTakeDamage(args)
{
    NpcLib.StandardOnTakeDamage(SHIP_ID, args);
}*/


/*function OnCalculateShipParameters(args)
{
    console.Print("OnCalculateShipParameters");
}


function OnCalculateInventoryCache(args) {
    console.Print("OnCalculateInventoryCache");
}

function OnCalculateSkillsCache(args) {
    console.Print("OnCalculateSkillsCache");
}

function OnCalculateBuffsCache(args) {
    console.Print("OnCalculateBuffsCache");
}
*/



/*function OnDied(args) {

}

function OnFrame(args) {

}

function OnFinished(args) {

}

function OnCancel(args) {

}

function OnTakeDamage(args) {

}*/


/*
  var constant = 0;
  LABEL1: do {
    x++;
    if (x < 100) {
        npc.RotLeft(SHIP_ID);
    }
    else {
        break;
    }

} while (constant);*/
using(console);
using(npc);

include(SC_AI_Drone_Combat_cc_Rout_For_5.js);
include(SC_AI_Drone_Combat_cc_Rout_For_FWP_5.js);
include(SC_Utilities.js);

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

var SC_AI_Drone_Combat_Sta_aDock_5 =
{
    AIaDock: function (SHIP_ID, droneIndex, playerName, player_id)
    {
             /*//npc.InstantStop(currentObjective.nid);

             //add option menu in a new TOPICS to repair drones when they are docked... //sRep - repair switch
             //add option menu in a new TOPICS to AUTO send drones to dock and repair when they are near the base // aRep
             //add option to auto dock or not from outside the base from the onboard computer... same thing but from inside the base on the station terminal. //aDock
             //choose a random point near the turrets and station entrance. Then patrol between all turrets? maybe. // //add option to auto defend base or to follow player. // aDef mixed with sDef
             //choose a specific point near the turrets and station entrance. // sDef
             //when players leaves base radius, make drone follow route to player instead of stickToObject. // already there - its currentObjective.bid but bid has to be set to null when leaving base radius.
             //make a menu to choose the HP number that the repair drone is in AUTO mode to go repair...

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

             //list of priorities...
             //if current HP is lower than the minimum allowed HP set on Drone.
             //make a menu to select the minimum hp on drones...

             var current_base = ship.GetCurrentBase(currentObjective.bid);

             if (currentObjective.bid != -1 || currentObjective.bid != null)
             {
                 var stationManagerCoords = npc.GetCurrentCoordinates(currentObjective.bid);
                 //var currentDroneCoords = npc.GetCurrentCoordinates(currentObjective.nid);
                 var playerCoords = ship.GetCoordinates(currentObjective.pid);

                 var playerDist = SC_Utilities.npcCheckDistance(stationManagerCoords, playerCoords);

                 if (playerDist > 500) // DO NOT RESET THE CURRENT COMMAND OF THE PLAYER... MAYBE THE PLAYER WANTS TO KEEP DEFEND THE BASE WHILE GOING MINING. UNLESS all autos/manuals are OFF.
                 {
                     //reset objective to make the auto settings back to 1 or back to the settings they were before entering the base vicinity.
                     //send command to go back in formation..
                     //if drone still in station Vicinity - use follow route to waypoint.
                     //if drone NOT in station vicinity anymore - change back the Global Command to 0 - //advise "station drone manager" that the drone has left the premises.
                 }
                 else
                 {
                     if (arrayOfDroneAutoSwitches.length < globIndex + 1)
                     {
                         for (var i = 0; i < (globIndex + 1) - arrayOfDroneAutoSwitches.length; i++)
                         {
                             arrayOfDroneAutoSwitches.push({ counter: 0 });
                         }
                     }
                     else {
                         if (arrayOfDroneAutoSwitches[currentObjective.droneIndex].counter == 0) {
                             if (currentObjective.aRep == 1) {
                                 if (arrayOfDroneAutoRepair.length < globIndex + 1) {
                                     for (var i = 0; i < (globIndex + 1) - arrayOfDroneAutoRepair.length; i++) {
                                         arrayOfDroneAutoRepair.push({ counter: 0 });
                                     }
                                 }
                                 else {
                                     if (arrayOfDroneAutoRepair[currentObjective.droneIndex].counter == 0) {
                                         var currentHP = ship.GetCurrentValue(currentObjective.nid, "structure");
                                         var maxHP = ship.GetFinalCacheValue(currentObjective.nid, "structure");

                                         if (currentHP < maxHP * currentObjective.minHP) {
                                             var current_base = currentObjective.bid;

                                             if (npc.IsAtBaseEntry(currentObjective.nid, current_base)) {
                                                 npc.EnterBase(currentObjective.nid, current_base);
                                                 arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 1;
                                             }
                                             else {
                                                 npc.FollowRouteToBase(currentObjective.nid, current_base); // calling it every frame... not good. but for the moment lets leave it like that.
                                             }
                                         }
                                         else {
                                             arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 2;
                                         }
                                     }
                                     else if (arrayOfDroneAutoRepair[currentObjective.droneIndex].counter == 1) {
                                         var currentHP = ship.GetCurrentValue(currentObjective.nid, "structure");
                                         var maxHP = ship.GetFinalCacheValue(currentObjective.nid, "structure");

                                         if (currentHP < maxHP * currentObjective.minHP) {
                                             var shipId = currentObjective.nid;
                                             var playerName = game.GetShipOwner(shipId);

                                             var current_structure = ship.GetCurrentValue(currentObjective.nid, "structure");
                                             var max_structure = ship.GetFinalCacheValue(currentObjective.nid, "structure");

                                             var structure_diff = (max_structure - current_structure);

                                             var money = player.GetMoney(playerName);
                                             var level = ship.GetLevel(npc_id);
                                             var repairMoneyCost = level * 0.05 * structure_diff; //missing the damage of THIS npc.

                                             //var moneyDiff = money - finalResult;

                                             if (repairMoneyCost > money) {
                                                 var moneyDiff = repairMoneyCost - money;
                                                 var percentOfRepairPossible = moneyDiff / repairMoneyCost;
                                                 var structureRepairPossibility = structure_diff * percentOfRepairPossible;

                                                 var repairMoneyCostAdjusted = level * 0.05 * structureRepairPossibility;

                                                 player.RemoveMoney(playerName, repairMoneyCostAdjusted);
                                             }
                                             else {
                                                 player.RemoveMoney(playerName, repairMoneyCost);
                                             }
                                         }
                                         else {
                                             arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 2;
                                         }
                                     }
                                     else if (arrayOfDroneAutoRepair[currentObjective.droneIndex].counter == 2) {
                                         arrayOfDroneAutoSwitches[currentObjective.droneIndex].counter = 1;
                                     }
                                 }
                             }
                             else if (currentObjective.aRep == 0)
                             {
                                 if (currentObjective.sRep == 1)
                                 {
                                     if (arrayOfDroneManualRepair.length < globIndex + 1)
                                     {
                                         for (var i = 0; i < (globIndex + 1) - arrayOfDroneManualRepair.length; i++)
                                         {
                                             arrayOfDroneManualRepair.push({ counter: 0 });
                                         }
                                     }
                                     else
                                     {
                                         if (arrayOfDroneManualRepair[currentObjective.droneIndex].counter == 0)
                                         {
                                             var currentHP = ship.GetCurrentValue(currentObjective.nid, "structure");
                                             var maxHP = ship.GetFinalCacheValue(currentObjective.nid, "structure");

                                             //NOPE... missing the money check before sending a drone to be repaired... if no money, send notification that the player doesnt even have enough money anyway so the
                                             //drone WONT go in the base to repair, instead it will stay in formation
                                             if (currentHP < maxHP * currentObjective.minHP)
                                             {
                                                 var current_base = currentObjective.bid;

                                                 if (npc.IsAtBaseEntry(currentObjective.nid, current_base)) {
                                                     npc.EnterBase(currentObjective.nid, current_base);
                                                     arrayOfDroneManualRepair[currentObjective.droneIndex].counter = 1;
                                                 }
                                                 else {
                                                     npc.FollowRouteToBase(currentObjective.nid, current_base); // calling it every frame... not good. but for the moment lets leave it like that.
                                                 }
                                             }
                                             else
                                             {
                                                 arrayOfDroneManualRepair[currentObjective.droneIndex].counter = 2;
                                             }
                                         }
                                         else if (arrayOfDroneManualRepair[currentObjective.droneIndex].counter == 1)
                                         {
                                             var currentHP = ship.GetCurrentValue(currentObjective.nid, "structure");
                                             var maxHP = ship.GetFinalCacheValue(currentObjective.nid, "structure");

                                             if (currentHP < maxHP * currentObjective.minHP)
                                             {
                                                 var shipId = currentObjective.nid;
                                                 var playerName = game.GetShipOwner(shipId);

                                                 var current_structure = ship.GetCurrentValue(currentObjective.nid, "structure");
                                                 var max_structure = ship.GetFinalCacheValue(currentObjective.nid, "structure");

                                                 var structure_diff = (max_structure - current_structure);

                                                 var money = player.GetMoney(playerName);
                                                 var level = ship.GetLevel(npc_id);
                                                 var repairMoneyCost = level * 0.05 * structure_diff; //missing the damage of THIS npc.

                                                 //var moneyDiff = money - finalResult;

                                                 if (repairMoneyCost > money)
                                                 {
                                                     var moneyDiff = repairMoneyCost - money;
                                                     var percentOfRepairPossible = moneyDiff / repairMoneyCost;
                                                     var structureRepairPossibility = structure_diff * percentOfRepairPossible;

                                                     var repairMoneyCostAdjusted = level * 0.05 * structureRepairPossibility;

                                                     player.RemoveMoney(playerName, repairMoneyCostAdjusted);
                                                 }
                                                 else
                                                 {
                                                     player.RemoveMoney(playerName, repairMoneyCost);
                                                 }
                                             }
                                             else
                                             {
                                                 arrayOfDroneManualRepair[currentObjective.droneIndex].counter = 2;
                                             }
                                         }
                                         else if (arrayOfDroneManualRepair[currentObjective.droneIndex].counter == 2) {
                                             arrayOfDroneAutoSwitches[currentObjective.droneIndex].counter = 1;
                                         }
                                     }
                                 }
                                 else if (currentObjective.sRep == 0) {
                                     arrayOfDroneAutoSwitches[currentObjective.droneIndex].counter = 1;
                                 }
                             }
                         }
                         else if (arrayOfDroneAutoSwitches[currentObjective.droneIndex].counter == 1) {
                             if (currentObjective.aDef == 1) {
                                 if (arrayOfDroneAutoDefense.length < globIndex + 1) {
                                     for (var i = 0; i < (globIndex + 1) - arrayOfDroneAutoDefense.length; i++) {
                                         arrayOfDroneAutoDefense.push({ counter: 0 });
                                     }
                                 }
                                 else {
                                     if (arrayOfDroneAutoDefense[currentObjective.droneIndex].counter == 0) {
                                         //var isPlayerDocked = ship.IsDocked(currentObjective.pid);
                                         var isNpcDocked = ship.IsDocked(currentObjective.nid);

                                         if (isNpcDocked) {
                                             npc.LeaveBase(currentObjective.nid);
                                             //undock then go defend base if player is still in vicinity.
                                             arrayOfDroneAutoDefense[currentObjective.droneIndex].counter = 1;
                                         }
                                         else {
                                             //go defend base
                                             arrayOfDroneAutoDefense[currentObjective.droneIndex].counter = 1;
                                         }
                                     }
                                     else if (arrayOfDroneAutoDefense[currentObjective.droneIndex].counter == 1) {
                                         //go to the "random" points chosen to automatically defend base. // to be changed later for something a bit less random. for example, check where the closest
                                         //enemies are and calculate it's directions and if one is heading towards base, go to the turret where it would be closest to when it would reach the base...
                                         //also check of course the distance of the enemies.
                                         var selectedTurretPositionIndex = currentObjective.sDefLoc;
                                         var stationData = storage.GetGlobal("station_tiles" + currentObjective.bid);

                                         if (stationData != null) {
                                             if (stationData.xml_id.indexOf("outpost") >= 0) {
                                                 var turretX;
                                                 var turretY;

                                                 if (selectedTurretPositionIndex == 0) {
                                                     turretX = stationData.coord.x + 30;
                                                     turretY = stationData.coord.y - 5;
                                                 }
                                                 else if (selectedTurretPositionIndex == 1) {
                                                     turretX = stationData.coords.x + 30;
                                                     turretY = stationData.coords.y - 18;
                                                 }
                                                 else if (selectedTurretPositionIndex == 2) {
                                                     turretX = stationData.coords.x - 30;
                                                     turretY = stationData.coords.y - 18;
                                                 }
                                                 else if (selectedTurretPositionIndex == 3) {
                                                     turretX = stationData.coords.x - 30;
                                                     turretY = stationData.coords.y - 5;
                                                 }

                                                 var stationPosX = stationData.coord.x;
                                                 var stationPosY = stationData.coord.y;
                                                 var stationDefensePos = { x: stationPosX, y: stationPosY };
                                                 var currentDefenseZonePos = SC_Utilities.RotatePoint(stationDefensePos, stationData.coord, stationData.rot);

                                                 //patrol around coord.

                                                 arrayOfDroneAutoDefense[currentObjective.droneIndex].counter = 2;
                                             }
                                             else if (stationData.xml_id.indexOf("science") >= 0) {
                                                 arrayOfDroneAutoDefense[currentObjective.droneIndex].counter = 2;
                                             }
                                             else if (stationData.xml_id.indexOf("military") >= 0) {
                                                 arrayOfDroneAutoDefense[currentObjective.droneIndex].counter = 2;

                                             }
                                             else if (stationData.xml_id.indexOf("mining") >= 0) {
                                                 arrayOfDroneAutoDefense[currentObjective.droneIndex].counter = 2;
                                             }
                                             else if (stationData.xml_id.indexOf("business") >= 0) {
                                                 arrayOfDroneAutoDefense[currentObjective.droneIndex].counter = 2;

                                             }
                                         }
                                         console.PrintError("auto defend base");
                                     }
                                 }
                             }
                             else
                             {
                                 if (currentObjective.sDef == 1)
                                 {
                                     if (arrayOfDroneManualDefense.length < globIndex + 1)
                                     {
                                         for (var i = 0; i < (globIndex + 1) - arrayOfDroneManualDefense.length; i++)
                                         {
                                             arrayOfDroneManualDefense.push({ counter: 0 });
                                         }
                                     }
                                     else
                                     {
                                         if (arrayOfDroneManualDefense[currentObjective.droneIndex].counter == 0)
                                         {
                                             //var isPlayerDocked = ship.IsDocked(currentObjective.pid);
                                             var isNpcDocked = ship.IsDocked(currentObjective.nid);

                                             if (isNpcDocked)
                                             {
                                                 npc.LeaveBase(currentObjective.nid);
                                                 //undock then go defend base if player is still in vicinity.
                                                 arrayOfDroneManualDefense[currentObjective.droneIndex].counter = 1;
                                             }
                                             else
                                             {
                                                 //go defend base
                                                 arrayOfDroneManualDefense[currentObjective.droneIndex].counter = 1;
                                             }
                                         }
                                         else if (arrayOfDroneManualDefense[currentObjective.droneIndex].counter == 1)
                                         {
                                             //go to the any "random" points chosen to automatically defend base. // to be changed later for something a bit less random. for example, check where the closest
                                             //enemies are and calculate it's directions and if one is heading towards base, go to the turret where it would be closest to when it would reach the base...
                                             //also check of course the distance of the enemies.
                                             //selectedTurretPosition = currentObjective.sDefLoc;

                                             //4 turrets per base max? for the moment yes... Science and Military and mining will have 4 turrets. in order of strength from high to low Military/science/mining
                                             var selectedTurretPositionIndex = currentObjective.sDefLoc;
                                             var stationData = storage.GetGlobal("station_tiles" + currentObjective.bid);

                                             if (stationData != null)
                                             {
                                                 if (stationData.xml_id.indexOf("outpost") >= 0)
                                                 {
                                                     var turretX;
                                                     var turretY;

                                                     if (selectedTurretPositionIndex == 0) {
                                                         turretX = stationData.coord.x + 30;
                                                         turretY = stationData.coord.y - 5;
                                                     }
                                                     else if (selectedTurretPositionIndex == 1) {
                                                         turretX = stationData.coords.x + 30;
                                                         turretY = stationData.coords.y - 18;
                                                     }
                                                     else if (selectedTurretPositionIndex == 2) {
                                                         turretX = stationData.coords.x - 30;
                                                         turretY = stationData.coords.y - 18;
                                                     }
                                                     else if (selectedTurretPositionIndex == 3) {
                                                         turretX = stationData.coords.x - 30;
                                                         turretY = stationData.coords.y - 5;
                                                     }

                                                     var stationPosX = stationData.coord.x;
                                                     var stationPosY = stationData.coord.y;
                                                     var stationDefensePos = { x: stationPosX, y: stationPosY };
                                                     var currentDefenseZonePos = SC_Utilities.RotatePoint(stationDefensePos, stationData.coord, stationData.rot);

                                                     //patrol around coord.

                                                     arrayOfDroneManualDefense[currentObjective.droneIndex].counter = 2;
                                                 }
                                                 else if (stationData.xml_id.indexOf("science") >= 0) {
                                                     arrayOfDroneManualDefense[currentObjective.droneIndex].counter = 2;
                                                 }
                                                 else if (stationData.xml_id.indexOf("military") >= 0) {
                                                     arrayOfDroneManualDefense[currentObjective.droneIndex].counter = 2;
                                                 }
                                                 else if (stationData.xml_id.indexOf("mining") >= 0) {
                                                     arrayOfDroneManualDefense[currentObjective.droneIndex].counter = 2;
                                                 }
                                                 else if (stationData.xml_id.indexOf("business") >= 0) {
                                                     arrayOfDroneManualDefense[currentObjective.droneIndex].counter = 2;
                                                 }
                                             }
                                             console.PrintError("Manual defend base");
                                         }
                                         else if (arrayOfDroneManualDefense[currentObjective.droneIndex].counter == 2) {
                                             //proceed to patrol base for defense.
                                         }
                                     }
                                 }
                                 else {
                                     arrayOfDroneAutoSwitches[currentObjective.droneIndex].counter = 2;
                                 }
                             }
                         }
                         else if (arrayOfDroneAutoSwitches[currentObjective.droneIndex].counter == 2)
                         {
                             //auto dock feature... //dock right away when arriving to base.
                             //or manual dock to manually send the drone to dock.
                         }
                     }
                 }
             }*/
                //pfff lol... i thought I had only done the turrets building on the outpost station.... well I did half of them on the science and military and mining.... Maybe only 2 turrets on mining and science
                //but 4 on the military. // SO only the military station is left...

    }
};
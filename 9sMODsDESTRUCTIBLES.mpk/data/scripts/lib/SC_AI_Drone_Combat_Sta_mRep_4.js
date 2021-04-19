using(console);
using(npc);

include(SC_Utilities.js);

//var arrayOfDronePathfindMainSwitch= [];
var arrayOfDroneAutoRepair = []; 
var arrayOfDroneAutoSwitches = []; 

var arrayOfDroneManualRepair = []; 

var lockdist = 50;

var SC_AI_Drone_Combat_Sta_aRep_4 =
{
    AIaRep: function (SHIP_ID, droneIndex, playerName, player_id)
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

        var current_base = ship.GetCurrentBase(currentObjective.bid);

        if (currentObjective.bid != -1 || currentObjective.bid != null) {
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
            else {
                if (arrayOfDroneAutoSwitches.length < globIndex + 1) {
                    for (var i = 0; i < (globIndex + 1) - arrayOfDroneAutoSwitches.length; i++) {
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
                        else if (currentObjective.aRep == 0) {
                            if (currentObjective.sRep == 1) {
                                if (arrayOfDroneManualRepair.length < globIndex + 1) {
                                    for (var i = 0; i < (globIndex + 1) - arrayOfDroneManualRepair.length; i++) {
                                        arrayOfDroneManualRepair.push({ counter: 0 });
                                    }
                                }
                                else {
                                    if (arrayOfDroneManualRepair[currentObjective.droneIndex].counter == 0) {
                                        var currentHP = ship.GetCurrentValue(currentObjective.nid, "structure");
                                        var maxHP = ship.GetFinalCacheValue(currentObjective.nid, "structure");

                                        //NOPE... missing the money check before sending a drone to be repaired... if no money, send notification that the player doesnt even have enough money anyway so the
                                        //drone WONT go in the base to repair, instead it will stay in formation
                                        if (currentHP < maxHP * currentObjective.minHP) {
                                            var current_base = currentObjective.bid;

                                            if (npc.IsAtBaseEntry(currentObjective.nid, current_base)) {
                                                npc.EnterBase(currentObjective.nid, current_base);
                                                arrayOfDroneManualRepair[currentObjective.droneIndex].counter = 1;
                                            }
                                            else {
                                                npc.FollowRouteToBase(currentObjective.nid, current_base); // calling it every frame... not good. but for the moment lets leave it like that.
                                            }
                                        }
                                        else {
                                            arrayOfDroneManualRepair[currentObjective.droneIndex].counter = 2;
                                        }
                                    }
                                    else if (arrayOfDroneManualRepair[currentObjective.droneIndex].counter == 1) {
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
                }
            }
        }
    }
};
using(console);
using(npc);
using(player);
using(ship);

include(SC_Utilities.js);
include(SC_AI_Drone_Combat_cc_Rout_For_SFWPP_1.js);


var arrayOfDroneAutoRepair = []; 

var cData = [];
var cDat = null;
var hpNPC = 0;
var nData = null;



var SC_AI_Drone_Combat_Sta_aRep_1 =
{   
    AIaRep: function (currentObjective, globIndex)
    {
        if (currentObjective.aRep == 1)
        {
            if (arrayOfDroneAutoRepair.length < globIndex + 1)
            {
                for (var i = 0; i < (globIndex + 1) - arrayOfDroneAutoRepair.length; i++)
                {
                    arrayOfDroneAutoRepair.push({ counter: 0 });
                    addingToArray(currentObjective, globIndex);
                }
            }
            else
            {
                if (arrayOfDroneAutoRepair[currentObjective.droneIndex].counter == 0)
                {
                    if (!ship.IsDocked(currentObjective.nid))
                    {
                        hpNPC = ship.GetCurrentValue(currentObjective.nid, "structure");
                        //console.PrintError(hpNPC);
                        if (hpNPC > 0) {
                            nData = SC_AI_Drone_Get_nData.npcGetSelfNPCData(currentObjective, 1);
                        }
                        else {
                            return;
                        }
                        var currentHP = ship.GetCurrentValue(currentObjective.nid, "structure");
                        var maxHP = ship.GetFinalCacheValue(currentObjective.nid, "structure");

                        if (currentHP < maxHP) // check if need repairing first. // * currentObjective.minHP
                        {
                            //var playerName = game.GetShipOwner(shipId);

                            var current_structure = ship.GetCurrentValue(currentObjective.nid, "structure");
                            var max_structure = ship.GetFinalCacheValue(currentObjective.nid, "structure");

                            var structureMin = 1;//(max_structure - current_structure);

                            var money = player.GetMoney(currentObjective.pName);
                            var level = ship.GetLevel(currentObjective.nid);
                            var repairMoneyCostMin = level * 0.05 * structureMin;

                            if (money > repairMoneyCostMin)
                            {
                                var current_base = ship.GetCurrentBase(currentObjective.bid);


                                //console.PrintError("****************");
                                //console.PrintError("bid" + currentObjective.bid);
                                //console.PrintError("****************");
                                //console.PrintError("bid" + current_base);
                                //console.PrintError("****************");


                                //var current_baseNPC = currentObjective.bid;
                                var currentWaypoint = npc.GetCurrentCoordinates(currentObjective.bid);
                                var distToWaypoint = SC_Utilities.GetDistance(currentWaypoint, nData.nCoord);
                                //npc.FollowRouteToBase(25334, 2535);

                                //console.PrintError(distToWaypoint);
                                if (npc.IsAtBaseEntry(currentObjective.nid, current_base) || Math.round(currentWaypoint.x) == Math.round(nData.nCoord.x) && Math.round(currentWaypoint.y) == Math.round(nData.nCoord.y) || distToWaypoint < 3)
                                {
                                    console.PrintError("****************");
                                    console.PrintError("bid" + currentObjective.bid);
                                    console.PrintError("****************");
                                    console.PrintError("bid" + current_base);
                                    console.PrintError("****************");

                                    //npc.EnterBase(currentObjective.nid, current_base);
                                    generator.DockShipToBase(currentObjective.nid, current_base);
                                    arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 1;
                                    return 2;
                                }
                                else { //npc.FollowRouteToBase(currentObjective.nid, current_base); // calling it every frame... not good. but for the moment lets leave it like that.
                                    //to replace with pathfinding.

                                    if (nData != null)
                                    {
                                        if (cData[currentObjective.droneIndex] == null) {
                                            //console.PrintError("reset: " + currentObjective.droneIndex + " id: " + currentObjective.nid);
                                            //Server reset or something...gotta reset all main arrays. gotta re-size all the main arrays to the current droneIndex in the server.
                                            cData = [];
                                            addingToArray(currentObjective, globIndex);
                                            return;
                                        }

                                        cData[currentObjective.droneIndex].nData = nData;
                                        cData[currentObjective.droneIndex].pData = pData;
                                        cData[currentObjective.droneIndex].objt = currentObjective;

                                        //SC_AI_Drone_Combat_rc_Rout_For_FWP_1.npcGWFP(currentObjective, 5);

                                        cData[currentObjective.droneIndex].cFWP = currentWaypoint;
                                        cData[currentObjective.droneIndex].cFWPD = distToWaypoint;

                                        cData[currentObjective.droneIndex] = SC_AI_Drone_Combat_cc_Rout_For_SFWPP_1.npcPathfindToWaypoint(cData[currentObjective.droneIndex], current_base);

                                        /*if (cData[currentObjective.droneIndex].mSwtc == -2)
                                        {
                                            npc.EnterBase(currentObjective.nid, current_base);
                                            arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 1;
                                            return 2;
                                        }*/
                                    }
                                }
                                return 2;
                            }
                            else {
                                //not enough money to repair
                                arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 0;
                                return 0;
                            }
                        }
                        else {
                            arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 0; // no need for repairing
                            return 0;
                        }
                    }
                    else
                    {
                        var currentHP = ship.GetCurrentValue(currentObjective.nid, "structure");
                        var maxHP = ship.GetFinalCacheValue(currentObjective.nid, "structure");

                        if (currentHP < maxHP) // check if need repairing first. // * currentObjective.minHP
                        {
                            //var playerName = game.GetShipOwner(shipId);

                            var current_structure = ship.GetCurrentValue(currentObjective.nid, "structure");
                            var max_structure = ship.GetFinalCacheValue(currentObjective.nid, "structure");

                            var structureMin = 1;//(max_structure - current_structure);

                            var money = player.GetMoney(currentObjective.pName);
                            var level = ship.GetLevel(currentObjective.nid);
                            var repairMoneyCostMin = level * 0.05 * structureMin;

                            if (money > repairMoneyCostMin)
                            {
                                arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 1; // no need for repairing
                                return 2;
                            }
                            else
                            {
                                arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 0; // no need for repairing
                                return 0;
                            }
                        }
                        arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 1;
                        return 0;
                    }
                }
                else if (arrayOfDroneAutoRepair[currentObjective.droneIndex].counter == 1)
                {
                    console.PrintError("counter: " + arrayOfDroneAutoRepair[currentObjective.droneIndex].counter  + " SC_AI_Drone_Combat_Sta_aRep_1.js");

                    var currentHP = ship.GetCurrentValue(currentObjective.nid, "structure");
                    var maxHP = ship.GetFinalCacheValue(currentObjective.nid, "structure");

                    if (currentHP < maxHP) // * currentObjective.minHP
                    {
                        var shipId = currentObjective.nid;
                        var playerName = game.GetShipOwner(currentObjective.pid);

                        var current_structure = ship.GetCurrentValue(currentObjective.nid, "structure");
                        var max_structure = ship.GetFinalCacheValue(currentObjective.nid, "structure");

                        var structure_diff = (max_structure - current_structure);

                        var money = player.GetMoney(currentObjective.pName);
                        var level = ship.GetLevel(currentObjective.nid);
                        var repairMoneyCost = level * 0.05 * structure_diff; //missing the damage of THIS npc.

                        //var moneyDiff = money - finalResult;

                        if (repairMoneyCost > money)
                        {
                            var moneyDiff = repairMoneyCost - money;
                            var percentOfRepairPossible = moneyDiff / repairMoneyCost;
                            var structureRepairPossibility = structure_diff * percentOfRepairPossible;

                            var repairMoneyCostAdjusted = level * 0.05 * structureRepairPossibility;

                            if (structureRepairPossibility >= max_structure)
                            {
                                structureRepairPossibility = max_structure;
                            }
                            ship.SetCurrentValue(currentObjective.nid, "structure", structureRepairPossibility);
                       
                            player.RemoveMoney(currentObjective.pName, repairMoneyCostAdjusted);
                            arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 2;
                            return 2;
                        }
                        else
                        {
                            var structureTotal = current_structure + structure_diff;
                            if (structureTotal >= max_structure)
                            {
                                structureTotal = max_structure;
                            }
                            ship.SetCurrentValue(currentObjective.nid, "structure", structureTotal);
                            player.RemoveMoney(currentObjective.pName, repairMoneyCost);
                            arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 2;
                            return 2;
                        }
                    }
                    else //ship is full HP
                    {
                        //console.PrintError("002");

                        arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 0;
                        return 0;
                    }
                }
                else if (arrayOfDroneAutoRepair[currentObjective.droneIndex].counter == 2)
                {

                    //npc.LeaveBase(currentObjective.nid); 
                    arrayOfDroneAutoRepair[currentObjective.droneIndex].counter = 0;
                    return 1;
                }
            }
        }
        else if (currentObjective.aRep == 0)
        {
            return 0;
        }
    }
};

function addingToArray(currentObjective) //globIndex
{
    for (var i = 0; i < (currentObjective.droneIndex + 1) - globIndex; i++) {

        cDat =
            {
                nData: null,
                pData: null,
                pLData: null,
                objt: currentObjective,
                cFWP: { x: null, y: null },
                cLFWP: { x: null, y: null },
                cFWPD: 0,

                fSwtc: 1,
                mSwtc: 1,
                lsP: { x: null, y: null },
                ltP: { x: null, y: null },
                liP: { x: null, y: null },
                gliP: { x: null, y: null },

                dog: { openSet: null, closedSet: null, currentCommand: null, grid: null, path: null, node: null, iot: null, nodeOffset: null },

                log: [],
                pfc: -2,
                stopS: 1,
                stopSC: 0,
                stopSCM: 5,
                stationTiles: null,
                stRot: 0,
                stCoord: { x: null, y: null },
                xtra: null,
                sSwtc: 1,
                iop: -1

            };

        cData.push(cDat);
    }
}
using(console);
using(player);
using(npc);
using(ship);

include(SC_Utilities.js);

var lockdist = 15;
var unwalkableTilesDistance = 5;
var SC_AI_Drone_Combat_cc_Rout_Com_FndTarg_3 =
{
    npcFindTarget: function (ceData)
    {


        //sending drone to the station immediately  when it is damaged.
        //sending drone to the station immediately  when it is damaged.
        /*if (ceData.ts == -1)
        {
            ceData.lLSwtc = 0;
        }
        else
        {
            if (generator.ShipExists(ceData.ts))
            {
                if (ship.GetCurrentValue(ceData.ts, "structure") <= 0)
                {
                    ceData.ts = -1;
                    ceData.lLSwtc = 0;
                }
                else
                {
                    ceData.lLSwtc = 1;
                }
            }
            else
            {
                if (npc.IsLocked(ceData.objt.nid) || npc.IsFriendlyLocked(ceData.objt.nid))
                {
                    npc.Unlock(ceData.objt.nid);
                }
                ceData.ts = -1;
                ceData.lLSwtc = 0;
            }
         }*/
         //sending drone to the station immediately  when it is damaged.
         //sending drone to the station immediately  when it is damaged.









        if (ceData.ts == -1) {
            ceData.hasUnwalkableTiles = -1;
            ceData.hasUnwalkableTilesArray = [];
            var cDroneHP = ship.GetCurrentValue(ceData.objt.nid, "structure");
            ceData.dLStrc = cDroneHP;
            ////console.PrintError("searching for Target");
            ceData.ts = -1;

            var arrayOfShips = null;
            arrayOfShips = npc.GetShipsInScope(ceData.objt.nid);
            //var arrayOfShips = ship.GetShipsInScope(ceData.objt.nid);
            //var arrayOfShips = args.scope_ships;

            var shipNDist = [];
            ////console.PrintError(arrayOfFriendlies.length);
            if (arrayOfFriendlies.length > 0) {
                if (arrayOfShips.length > 0) {
                    for (var i = 0; i < arrayOfShips.length; i++) {
                        var otherShip = arrayOfShips[i];
                        if (generator.ShipExists(otherShip) && ship.GetCurrentValue(otherShip, "structure") > 0) {
                            //var playName = ceData.pName.toLowerCase();
                            var shipName = game.GetShipOwner(otherShip).toLowerCase();
                            //globIndex = parseInt(npc.GetTag(SHIP_ID, "droneIndex"));

                            var otherShipCoord = ship.GetCoordinates(otherShip);
                            var distPlayerToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.pData.pCoord);
                            var distNpcToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.nData.nCoord);


                            var someTag = npc.GetTag(otherShip, "class");




                            if (distNpcToEnemy <= unwalkableTilesDistance)//distPlayerToEnemy <= unwalkableTilesDistance && 
                            { 



                                for (var x = -1; x <= 1; x++) {
                                    for (var y = -1; y <= 1; y++) {
                                        //console.PrintError("turret");

                                        var xpos = Math.round(otherShipCoord.x + x);
                                        var ypos = Math.round(otherShipCoord.y + y);
                                        var newCoordsUnwalkables = { x: xpos, y: ypos };



                                        //var newCoordsUnwalkables = otherShipCoord;
                                        //newCoordsUnwalkables.x += x;
                                        //newCoordsUnwalkables.y += y;


                                        //console.PrintError("turret");
                                        ceData.hasUnwalkableTiles = 1;
                                        ceData.hasUnwalkableTilesArray.push({ id: otherShip, coords: newCoordsUnwalkables, class: someTag });
                                    }
                                }
                            }



                            if (!SC_Utilities.contains(ceData.arrF, shipName)) // && shipName != ceData.objt.pName.toLowerCase()
                            {
                                
                                if (distPlayerToEnemy <= lockdist && distNpcToEnemy <= lockdist)   //&& distPlayerToDrone <= lockdist// && distToEnemy <= distPlayerToDrone
                                {
                                  


                                    ship.SetDamageByShip(otherShip, 1, ceData.objt.nid);
                                    ship.SetDamageByShip(ceData.objt.nid, 1, otherShip);


                                    //var cdamage = ship.SetDamageByShip(otherShip.id, 1, player_id);          
                                    //var gdamage = ship.GetDamageByShip(ceData.objt.pid, otherShip);

                                    //var dist = ship.GetDistanceToObj(ceData.objt.nid, otherShip);
                                    var someData = { id: otherShip, dist: distNpcToEnemy, eToD: 0 };
                                    shipNDist.push(someData);
                                }
                            }
                            else {


                            }
                        }
                    }
                }
                else {
                    //npc.Unstick(ceData.objt.nid);
                    //npc.Unlock(ceData.objt.nid);
                    //formationSwitches[ceData.objt.droneIndex] = 1;
                    npc.Unstick(ceData.objt.nid);
                    ceData.ts = -1;
                    //offsetForRange[ceData.objt.droneIndex].x = 0;
                    //offsetForRange[ceData.objt.droneIndex].y = 0;
                    //npc.StopEvasion(ceData.objt.nid);
                }
            }
            else {
                //npc.Unstick(ceData.objt.nid);
                //npc.Unlock(ceData.objt.nid);
                //formationSwitches[ceData.objt.droneIndex] = 1;
                npc.Unstick(ceData.objt.nid);
                ceData.ts = -1;
                //offsetForRange[ceData.objt.droneIndex].x = 0;
                //offsetForRange[ceData.objt.droneIndex].y = 0;
                //npc.StopEvasion(ceData.objt.nid);
            }

            if (shipNDist != null) {
                if (shipNDist.length > 0) {
                    shipNDist.sort(function (a, b) {
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

                    if (shipNDist[0].id == ceData.objt.pid) 
                    {
                        console.PrintError("target is player. reset.");
                        shipNDist = null;

                        //npc.Unstick(ceData.objt.nid);
                        //npc.Unlock(ceData.objt.nid);

                        npc.Unstick(ceData.objt.nid);
                        ceData.lLSwtc = 0;
                        ceData.ts = -1;
                    }
                    else {
                        if (shipNDist[0].dist < lockdist) {
                            //npc.Unstick(ceData.objt.nid);
                            //npc.Unlock(ceData.objt.nid);
                            /*npc.Stop(ceData.objt.nid);
                            npc.Unstick(ceData.objt.nid);
                            //console.PrintError("target is not player. no reset.");
                            ceData.lLSwtc = 1;
                            ceData.ts = shipNDist[0].id;
                            ceData.eneL = shipNDist;*/
                            //console.PrintError("target is not player. do i attack or not.");
                            npc.Unstick(ceData.objt.nid);
                            ceData.lLSwtc = 1;
                            ceData.ts = shipNDist[0].id;
                            ceData.eneL = shipNDist;
                        }
                        else {
                            npc.Unstick(ceData.objt.nid);
                            ceData.lLSwtc = 0;
                            ceData.ts = -1;
                        }
                    }




                    //npc.LockOnTarget(ceData.objt.nid, shipNDist[0].id, 20);
                    //var cdamage = ship.SetDamageByShip(ceData.objt.nid, 1, shipNDist[0].id);
                    //npc.StartEvasion(ceData.objt.nid);
                    ////console.PrintError("test0000");
                }
                else {
                    //npc.Unstick(ceData.objt.nid);
                    //npc.Unlock(ceData.objt.nid);
                    npc.Unstick(ceData.objt.nid);
                    ceData.lLSwtc = 0;
                    //formationSwitches[ceData.objt.droneIndex] = 1;
                    ceData.ts = -1;
                    //offsetForRange[ceData.objt.droneIndex].x = 0;
                    //offsetForRange[ceData.objt.droneIndex].y = 0;
                    //npc.StopEvasion(ceData.objt.nid);
                }
            }
            else {

                //npc.Unstick(ceData.objt.nid);
                //npc.Unlock(ceData.objt.nid);
                //formationSwitches[ceData.objt.droneIndex] = 1;
                npc.Unstick(ceData.objt.nid);
                ceData.lLSwtc = 0;
                ceData.ts = -1;
                //offsetForRange[ceData.objt.droneIndex].x = 0;
                //offsetForRange[ceData.objt.droneIndex].y = 0;
                //npc.StopEvasion(ceData.objt.nid);     
            }
        }
        else {

            if (npc.IsLocked(ceData.objt.nid)) {
                //console.PrintError("npc is locked");
                var lockedTarget = npc.GetLockTarget(ceData.objt.nid);

                var otherShipCoord = ship.GetCoordinates(lockedTarget);


                if (otherShipCoord != null) {

                    if (otherShipCoord.x != null && otherShipCoord.y != null) {


                        if (ceData.nData.nCoord.x != null && ceData.nData.nCoord.y != null) {

                            var distNpcToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.nData.nCoord);
                            ceData.eneL = distNpcToEnemy;
                        }
                        else {
                            npc.Unstick(ceData.objt.nid);
                            ceData.eneL = lockdist + 1;
                            ceData.lLSwtc = 0;
                            ceData.ts = -1;
                        }
                    }
                    else {
                        npc.Unstick(ceData.objt.nid);
                        ceData.eneL = lockdist + 1;
                        ceData.lLSwtc = 0;
                        ceData.ts = -1;
                    }
                }
            }
            else
            {

            }
        }
        return ceData;
    }
};

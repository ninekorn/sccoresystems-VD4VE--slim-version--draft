using(console);
using(npc);
using(ship);

include(SC_Utilities.js);
var maxIncrFar = 6;

var SC_AI_Drone_Combat_cc_Rout_Com_Att_Tec0_5 =
{
    npcAttackTec0: function (ceData, otherShipCoord, distDroneToEnemy)
    {
        if (ceData.tec == 0) // drone current command is to lock on target and use incorporated Combat API;
        {
            if (npc.IsStickToPoint(ceData.objt.nid)) {
                npc.Unstick(ceData.objt.nid);
            }
            /*if (npc.IsLocked(ceData.objt.nid) || npc.IsFriendlyLocked(ceData.objt.nid)) {
                npc.Unlock(ceData.objt.nid);
            }*/

            if (distDroneToEnemy <= 8)
            {               
                var dirDroneToEnemyX = (otherShipCoord.x - ceData.nData.nCoord.x) / distDroneToEnemy;
                var dirDroneToEnemyY = (otherShipCoord.y - ceData.nData.nCoord.y) / distDroneToEnemy;
                var dirDroneToEnemy = { x: dirDroneToEnemyX, y: dirDroneToEnemyY };
                var alignedTowardsEnemy0 = SC_Utilities.Dot(ceData.nData.nForward.x, ceData.nData.nForward.y, dirDroneToEnemy.x, dirDroneToEnemy.y);

                var veloNPC = ship.GetVelocity(ceData.objt.nid);
                var speedNPC = Math.sqrt(veloNPC.x * veloNPC.x + veloNPC.y * veloNPC.y);
                veloNPC.x /= speedNPC;
                veloNPC.y /= speedNPC;

                var alignedTowardsEnemy1 = SC_Utilities.Dot(ceData.nData.nVelo.x, ceData.nData.nVelo.y, dirDroneToEnemy.x, dirDroneToEnemy.y);
                var alignedTowardsEnemy2 = SC_Utilities.Dot(ceData.nData.nVelo.x, ceData.nData.nVelo.y, ceData.eData.eVelo.x, ceData.eData.eVelo.y);


                if (alignedTowardsEnemy0 >= 0.95 || alignedTowardsEnemy1 >= 0.75) // || alignedTowardsEnemy2 >= 0 && alignedTowardsEnemy0 >= 0.95
                {
                    if (!npc.IsLocked(ceData.objt.nid)) {
                        npc.LockOnTarget(ceData.objt.nid, ceData.ts, 20);
                    }

                    if (distDroneToEnemy <= 4)
                    {
                        if (alignedTowardsEnemy0 >= 0.97 && speedNPC > 0.0001 || alignedTowardsEnemy1 >= 0.97 && speedNPC > 0.0001)
                        {
                            if (ceData.stopS == 0)
                            {
                                npc.InstantStop(ceData.objt.nid);
                                ceData.stopS = 1;
                            }
                        }
                    }
                }
                else
                {
                    var dirDroneToEnemyX = (otherShipCoord.x - ceData.nData.nCoord.x) / distDroneToEnemy;
                    var dirDroneToEnemyY = (otherShipCoord.y - ceData.nData.nCoord.y) / distDroneToEnemy;
                    var dirDroneToEnemy = { x: dirDroneToEnemyX, y: dirDroneToEnemyY };

                    var newWaypointX = ceData.nData.nCoord.x + (dirDroneToEnemy.x * maxIncrFar);
                    var newWaypointY = ceData.nData.nCoord.y + (dirDroneToEnemy.y * maxIncrFar);
                    var newWaypoint = { x: newWaypointX, y: newWaypointY };

                    /*if ()
                    {

                    }
                    //npc.StickToPoint(ceData.objt.nid, newWaypoint.x, newWaypoint.y, 0);
                    */
                    if (alignedTowardsEnemy0 >= 0.97) //|| alignedTowardsEnemy1 >= 0.75 && alignedTowardsEnemy0 >= 0.95 || alignedTowardsEnemy2 >= -0.25 && alignedTowardsEnemy0 >= 0.97
                    {
                        if (!npc.IsLocked(ceData.objt.nid)) {
                            npc.LockOnTarget(ceData.objt.nid, ceData.ts, 20);
                        }

                        if (distDroneToEnemy <= 4) {
                            if (alignedTowardsEnemy0 >= 0.97 && speedNPC > 0.0001 || alignedTowardsEnemy1 >= 0.97 && speedNPC > 0.0001) {
                                if (ceData.stopS == 0) {
                                    npc.InstantStop(ceData.objt.nid);
                                    ceData.stopS = 1;
                                }
                            }
                        }
                    }
                    else
                    {
                        if (ceData.tec0 > 10)
                        {
                            if (distDroneToEnemy <= 4)
                            {
                                if (alignedTowardsEnemy0 >= 0.97 && speedNPC > 0.0001 || alignedTowardsEnemy1 >= 0.97 && speedNPC > 0.0001)
                                {
                                    if (ceData.stopS == 0)
                                    {
                                        npc.InstantStop(ceData.objt.nid);
                                        ceData.tec0 = 0;
                                        ceData.stopS = 1;
                                    }
                                }

                            }
                           
                            ceData.tec = 2;
                            ceData.hasStP = 1;
                            ceData.hasStPC = 0;
                            ceData.hasLo = 0;
                            ceData.hasLoC = 0;
                            ceData.hasLoF = 0;
                            ceData.hasLoFC = 0;
                            ceData.evadC = 0;
                            ceData.stopS = 0;
                            ceData.tec3 = 0;
                            ceData.tec3C = 0;
                            ceData.tec2 = 0;
                            ceData.tec2C = 0;
                        }
                        ceData.tec0++;
                    }




                    /*//console.PrintError("here00");
                    if (npc.IsStickToPoint(ceData.objt.nid)) {
                        npc.Unstick(ceData.objt.nid);
                    }
                    if (npc.IsLocked(ceData.objt.nid) || npc.IsFriendlyLocked(ceData.objt.nid)) {
                        npc.Unlock(ceData.objt.nid);
                    }

                    ceData.tec = 2;
                    ceData.hasStP = 1;
                    ceData.hasStPC = 0;
                    ceData.hasLo = 0;
                    ceData.hasLoC = 0;
                    ceData.hasLoF = 0;
                    ceData.hasLoFC = 0;
                    ceData.evadC = 0;
                    ceData.stopS = 0;
                    ceData.tec3 = 0;
                    ceData.tec3C = 0;
                    ceData.tec2 = 0;
                    ceData.tec2C = 0;*/
                }




















                /*if (alignedTowardsEnemy0 < 0)
                {
                    if (npc.IsStickToPoint(ceData.objt.nid)) {
                        npc.Unstick(ceData.objt.nid);
                    }
                    if (npc.IsLocked(ceData.objt.nid) || npc.IsFriendlyLocked(ceData.objt.nid)) {
                        npc.Unlock(ceData.objt.nid);
                    }

                    ceData.tec = 2;
                    ceData.hasStP = 1;
                    ceData.hasStPC = 0;
                    ceData.hasLo = 0;
                    ceData.hasLoC = 0;
                    ceData.hasLoF = 0;
                    ceData.hasLoFC = 0;
                    ceData.evadC = 0;
                    ceData.stopS = 0;
                    ceData.tec3 = 0;
                    ceData.tec3C = 0;
                    ceData.tec2 = 0;
                    ceData.tec2C = 0;
                }
                else
                {
                    if (!npc.IsLocked(ceData.objt.nid))
                    {
                        npc.LockOnTarget(ceData.objt.nid, ceData.ts, 20);
                    }

                    if (distDroneToEnemy <= 4)
                    {
                        if (alignedTowardsEnemy >= 0.97 && speedNPC > 0.0001) {
                            if (ceData.stopS == 0) {
                                npc.InstantStop(ceData.objt.nid);
                                ceData.stopS = 1;
                            }
                        }
                    }

                }*/
            }
            else
            {
                if (npc.IsStickToPoint(ceData.objt.nid)) {
                    npc.Unstick(ceData.objt.nid);
                }
                if (npc.IsLocked(ceData.objt.nid) || npc.IsFriendlyLocked(ceData.objt.nid)) {
                    npc.Unlock(ceData.objt.nid);
                }
                ceData.tec = 2;
                ceData.hasStP = 1;
                ceData.hasStPC = 0;
                ceData.hasLo = 0;
                ceData.hasLoC = 0;
                ceData.hasLoF = 0;
                ceData.hasLoFC = 0;
                ceData.evadC = 0;
                ceData.stopS = 0;
                ceData.tec3 = 0;
                ceData.tec3C = 0;
                ceData.tec2 = 0;
                ceData.tec2C = 0;
            }
        }
        return ceData;
    }
};

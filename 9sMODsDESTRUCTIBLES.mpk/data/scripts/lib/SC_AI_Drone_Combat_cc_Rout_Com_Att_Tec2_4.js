using(console);
using(npc);

include(SC_Utilities.js);

var randomPos = 1;
var maxIncrFar = 10;
var maxIncr = 4;

var isLeftOrRight = 0;
var newPointer = { x: null, y: null };
var pointInFrontOfVeloEnemy;
var dist0 = 0;
var dist1 = 0;
var pointVeloBack;
var pointVeloFront;
var modDist = 0;

var SC_AI_Drone_Combat_cc_Rout_Com_Att_Tec2_4 =
{
    npcAttackTec2: function (ceData, otherShipCoord, distDroneToEnemy) {

        ceData.hasStP = 1;
        if (ceData.tec2 >= 2)
        {
            if (npc.IsLocked(ceData.objt.nid)) {
                npc.Unlock(ceData.objt.nid);
            }
            ceData.tec2 = 0;
        }
        var dirDroneToEnemyX = (otherShipCoord.x - ceData.nData.nCoord.x) / distDroneToEnemy;
        var dirDroneToEnemyY = (otherShipCoord.y - ceData.nData.nCoord.y) / distDroneToEnemy;
        var dirDroneToEnemy = { x: dirDroneToEnemyX, y: dirDroneToEnemyY };

        var alignedTowardsEnemy0 = SC_Utilities.Dot(ceData.nData.nForward.x, ceData.nData.nForward.y, dirDroneToEnemy.x, dirDroneToEnemy.y);
        var alignedTowardsEnemy1 = SC_Utilities.Dot(ceData.nData.nVelo.x, ceData.nData.nVelo.y, dirDroneToEnemy.x, dirDroneToEnemy.y);
        var alignedTowardsEnemy2 = SC_Utilities.Dot(ceData.nData.nVelo.x, ceData.nData.nVelo.y, ceData.eData.eVelo.x, ceData.eData.eVelo.y);
        var enemyAlignedTowardsNPC = SC_Utilities.Dot(ceData.eData.eVelo.x, ceData.eData.eVelo.y, -dirDroneToEnemy.x, -dirDroneToEnemy.y);

        if (ceData.stopS == 0 && ceData.tec2C > 6 && alignedTowardsEnemy1 < 0.90 && alignedTowardsEnemy2 < 0 && distDroneToEnemy > ceData.npcLD)
        {

            //console.PrintError("test0");
            npc.InstantStop(ceData.objt.nid);
            ceData.tec2C = 0;
            ceData.stopS = 1;   
        }
        else
        {
            pointInFrontOfVeloEnemy = { x: ceData.eData.eCoord.x + ceData.eData.eVelo.x, y: ceData.eData.eCoord.y + ceData.eData.eVelo.y };

            isLeftOrRight = SC_Utilities.EWDIST(ceData.eData.eCoord, pointInFrontOfVeloEnemy, ceData.nData.nCoord);

            if (isLeftOrRight > 0)//is RIGHT
            {
                var veloEnemyRight = { x: ceData.eData.eVelo.y, y: -ceData.eData.eVelo.x };

                randomPos = (Math.random() * (3 - 1) + 0);
                pointVeloFront = {
                    x: (otherShipCoord.x + (veloEnemyRight.x * randomPos)) + (ceData.eData.eVelo.x * (1)),
                    y: (otherShipCoord.y + (veloEnemyRight.y * randomPos)) + (ceData.eData.eVelo.y * (1))
                };

                newPointer.x = pointVeloFront.x;
                newPointer.y = pointVeloFront.y;
            }
            else //is LEFT
            {
                randomPos = (Math.random() * (3 - 1) + 0);

                var veloEnemyLeft = { x: -ceData.eData.eVelo.y, y: ceData.eData.eVelo.x };

                pointVeloFront = {
                    x: (otherShipCoord.x + (veloEnemyLeft.x * randomPos)) + (ceData.eData.eVelo.x * (1)),
                    y: (otherShipCoord.y + (veloEnemyLeft.y * randomPos)) + (ceData.eData.eVelo.y * (1))
                };

                newPointer.x = pointVeloFront.x;
                newPointer.y = pointVeloFront.y;
            }


            if (distDroneToEnemy > ceData.npcLD && distDroneToEnemy > 4 && alignedTowardsEnemy0 < 0.85) {
                //console.PrintError("test1");
                if (npc.IsLocked(ceData.objt.nid) || npc.IsFriendlyLocked(ceData.objt.nid)) {
                    npc.Unlock(ceData.objt.nid);
                }
                npc.StickToPoint(ceData.objt.nid, newPointer.x + (ceData.nData.nForward.x * distDroneToEnemy * 4), newPointer.y + (ceData.nData.nForward.y * distDroneToEnemy * 4), 0);

            }
            else if (alignedTowardsEnemy0 < 0.95)
            {
                //console.PrintError("test2");
                modDist = distDroneToEnemy;

                if (distDroneToEnemy < 6)
                {
                    modDist = 6;
                }

                if (npc.IsLocked(ceData.objt.nid) || npc.IsFriendlyLocked(ceData.objt.nid)) {
                    npc.Unlock(ceData.objt.nid);
                }

                var dirDroneToEnemyX = (otherShipCoord.x - ceData.nData.nCoord.x) / distDroneToEnemy;
                var dirDroneToEnemyY = (otherShipCoord.y - ceData.nData.nCoord.y) / distDroneToEnemy;
                var dirDroneToEnemy = { x: dirDroneToEnemyX, y: dirDroneToEnemyY };

                var newWaypointX = ceData.nData.nCoord.x + (dirDroneToEnemy.x * modDist);
                var newWaypointY = ceData.nData.nCoord.y + (dirDroneToEnemy.y * modDist);
                var newWaypoint = { x: newWaypointX, y: newWaypointY };

                npc.StickToPoint(ceData.objt.nid, newWaypoint.x, newWaypoint.y, 0);
            }
            else if (distDroneToEnemy > 4)
            {
                modDist = distDroneToEnemy;

                if (distDroneToEnemy < 6) {
                    modDist = 6;
                }

                if (npc.IsLocked(ceData.objt.nid) || npc.IsFriendlyLocked(ceData.objt.nid))
                {
                    npc.Unlock(ceData.objt.nid);
                }

                var dirDroneToEnemyX = (otherShipCoord.x - ceData.nData.nCoord.x) / distDroneToEnemy;
                var dirDroneToEnemyY = (otherShipCoord.y - ceData.nData.nCoord.y) / distDroneToEnemy;
                var dirDroneToEnemy = { x: dirDroneToEnemyX, y: dirDroneToEnemyY };

                var newWaypointX = ceData.nData.nCoord.x + (dirDroneToEnemy.x * modDist);
                var newWaypointY = ceData.nData.nCoord.y + (dirDroneToEnemy.y * modDist);
                var newWaypoint = { x: newWaypointX, y: newWaypointY };

                npc.StickToPoint(ceData.objt.nid, newWaypoint.x, newWaypoint.y, 0);
            }
            else if (alignedTowardsEnemy0 < 0.99 && distDroneToEnemy<= 5)
            {
                if (ceData.tec2 == 0) {
                    if (!npc.IsLocked(ceData.objt.nid)) {
                        npc.LockOnTarget(ceData.objt.nid, ceData.ts, 0);
                    }

                }
                else {
                    /*if (npc.IsLocked(ceData.objt.nid)) {
                        npc.Unlock(ceData.objt.nid, ceData.ts, 0);
                    }*/


                    if (npc.IsStickToPoint(ceData.objt.nid)) {
                        npc.Unstick(ceData.objt.nid);
                    }

                    if (!npc.IsFriendlyLocked(ceData.objt.nid)) {
                        npc.FriendlyLockOnTarget(ceData.objt.nid, ceData.ts);
                    }
                    npc.Fire(ceData.objt.nid, 0);
                }

                if (enemyAlignedTowardsNPC >= 0.90 && ceData.stopS == 0 && distDroneToEnemy <= 5) {
                    npc.GoBackward(ceData.objt.nid);
                }
                else if (distDroneToEnemy <= ceData.npcLD && distDroneToEnemy <= 5 && ceData.stopS == 0) {
                    //console.PrintError("test4");
                    npc.InstantStop(ceData.objt.nid);
                    ceData.stopS = 1;
                    ceData.tec2C = 0;
                }
                else {
                    //console.PrintError("test5");
                    npc.InstantStop(ceData.objt.nid);
                    ceData.stopS = 1;
                    ceData.tec2C = 0;
                }
                ceData.tec2++;
            }
            if (alignedTowardsEnemy1 > 0.75 && distDroneToEnemy <= ceData.npcLD && distDroneToEnemy <= 5 && ceData.stopS == 0 || alignedTowardsEnemy1 > 0.75 && distDroneToEnemy < ceData.npcLD && distDroneToEnemy <= 5 && ceData.tec2C >= 0 ||
                distDroneToEnemy <= ceData.npcLD && distDroneToEnemy <= 5)
            {

                if (ceData.tec2 == 0) {
                    if (!npc.IsLocked(ceData.objt.nid)) {
                        npc.LockOnTarget(ceData.objt.nid, ceData.ts, 0);
                    }

                }
                else {
                    /*if (npc.IsLocked(ceData.objt.nid)) {
                        npc.Unlock(ceData.objt.nid, ceData.ts, 0);
                    }*/


                    if (npc.IsStickToPoint(ceData.objt.nid)) {
                        npc.Unstick(ceData.objt.nid);
                    }

                    if (!npc.IsFriendlyLocked(ceData.objt.nid)) {
                        npc.FriendlyLockOnTarget(ceData.objt.nid, ceData.ts);
                    }
                    npc.Fire(ceData.objt.nid, 0);
                }

                if (enemyAlignedTowardsNPC >= 0.90 && ceData.stopS == 0 && distDroneToEnemy <= 5) {
                    npc.GoBackward(ceData.objt.nid);
                }
                else if (distDroneToEnemy <= ceData.npcLD && distDroneToEnemy <= 5 && ceData.stopS == 0) {
                    //console.PrintError("test4");
                    npc.InstantStop(ceData.objt.nid);
                    ceData.stopS = 1;
                    ceData.tec2C = 0;
                }
                else {
                    //console.PrintError("test5");
                    npc.InstantStop(ceData.objt.nid);
                    ceData.stopS = 1;
                    ceData.tec2C = 0;
                }
                ceData.tec2++;
            }
        }
        ceData.tec2C++;
        return ceData;
    }
};
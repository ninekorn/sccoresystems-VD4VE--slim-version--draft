using(console);
using(npc);

include(SC_Utilities.js);

var SC_AI_Drone_Combat_cc_Rout_Com_Att_Tec1_2 =
{
    npcAttackTec1: function (ceData, otherShipCoord, distDroneToEnemy)
    {
        if (distDroneToEnemy <= 8)
        {
            
            var dirDroneToEnemyX = (otherShipCoord.x - ceData.nData.nCoord.x) / distDroneToEnemy;
            var dirDroneToEnemyY = (otherShipCoord.y - ceData.nData.nCoord.y) / distDroneToEnemy;
            var dirDroneToEnemy = { x: dirDroneToEnemyX, y: dirDroneToEnemyY };
            var alignedTowardsEnemy0 = SC_Utilities.Dot(ceData.nData.nForward.x, ceData.nData.nForward.y, dirDroneToEnemy.x, dirDroneToEnemy.y);

            var alignedTowardsEnemy1 = SC_Utilities.Dot(ceData.nData.nVelo.x, ceData.nData.nVelo.y, dirDroneToEnemy.x, dirDroneToEnemy.y);
            var alignedTowardsEnemy2 = SC_Utilities.Dot(ceData.nData.nVelo.x, ceData.nData.nVelo.y, ceData.eData.eVelo.x, ceData.eData.eVelo.y);


            if (alignedTowardsEnemy0 >= 0.95 || alignedTowardsEnemy1 >= 0.75 && alignedTowardsEnemy0 >= 0.95 || alignedTowardsEnemy2 >= 0 && alignedTowardsEnemy0 >= 0.95)
            {
                if (distDroneToEnemy <= 4)
                {
                    if (alignedTowardsEnemy0 >= 0.97 && speedNPC > 0.0001 || alignedTowardsEnemy1 >= 0.97 && speedNPC > 0.0001) {
                        if (ceData.stopS == 0)
                        {
                            npc.InstantStop(ceData.objt.nid);
                            ceData.stopS = 1;
                        }
                    }
                }
                if (npc.IsStickToPoint(ceData.objt.nid))
                {
                    npc.Unstick(ceData.objt.nid);
                }
                //if (npc.IsLocked(ceData.objt.nid))
                //{
                //    npc.Unlock(ceData.objt.nid);
                //}
                if (!npc.IsFriendlyLocked(ceData.objt.nid))
                {
                    npc.FriendlyLockOnTarget(ceData.objt.nid, ceData.ts);
                }
                npc.Fire(ceData.objt.nid, 0);
            }
            else
            {
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
        else
        {
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
        
        return ceData;
    }
};

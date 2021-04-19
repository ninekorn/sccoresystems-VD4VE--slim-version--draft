using(console);
using(npc);
using(ship);
using(visual);

include(SC_Utilities.js);
include(SC_AI_Drone_Get_eData.js);

var lockdist = 20;
var randomPos = -1;

var SC_AI_Drone_Repair_Rout_Com_Att_5 =
{
    npcCombatRoutine: function (ceData) {

		var max_structure = ship.GetFinalCacheValue(ceData.ts, "structure");
        var current_structure = ship.GetCurrentValue(ceData.ts, "structure");

        if (generator.ShipExists(ceData.ts) && current_structure < max_structure && current_structure > 0)
        {
            var otherShipCoord = ship.GetCoordinates(ceData.ts);

            var distPlayerToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.pData.pCoord);

            if (distPlayerToEnemy <= 25)
            {
                var distDroneToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.nData.nCoord);
                ceData.eData = SC_AI_Drone_Get_eData.npcGetEnemyData(ceData.ts, 1);

				if (distDroneToEnemy < 6)
                {
                    npc.StickToPoint(ceData.objt.nid, ceData.eData.eCoord.x, ceData.eData.eCoord.y, 0);
                    				
					if(ceData.hasNA == 0)
					{
						npc.DeviceActivateOnObject(ceData.objt.nid, "drone_device_repair_beam", ceData.ts);
						ceData.hasNA = 1;
					}
					else if(ceData.hasNA  == 1)
					{
						if(ceData.hasNAC >= 30)
						{
							ceData.hasNAC = 0;
							ceData.hasNA = 0;
						}
						ceData.hasNAC++;
					}
                }
                else 
				{
                    npc.StickToPoint(ceData.objt.nid, ceData.eData.eCoord.x, ceData.eData.eCoord.y, 0);
                }
			}
			else
			{
				npc.Unstick(ceData.objt.nid);
				npc.Unlock(ceData.objt.nid);
				ceData.lLSwtc = 0;
				ceData.ts = -1;
				ceData.hasNA = 0;
				ceData.mReset = 0;
			}
		}
		else
		{
			if(ceData.hasNA == 1)
			{
				if(current_structure >= max_structure)
				{
					npc.DeviceDeactivate(ceData.objt.nid, "drone_device_repair_beam");
					//visual.DeviceDeactivateEffect(ceData.objt.nid, "drone_device_repair_beam", "healing_visual_effect");
				}
			}

			npc.Unstick(ceData.objt.nid);
            npc.Unlock(ceData.objt.nid);
			ceData.lLSwtc = 0;
            ceData.ts = -1;
			ceData.hasNA = 0;
			ceData.mReset = 0;
		}
        return ceData;
    }
};
using(console);

include(SC_AI_Drone_Repair_Rout_For_SFWP_3.js);
include(SC_AI_Drone_Repair_Rout_Com_FndTarg_3.js);
include(SC_AI_Drone_Repair_Rout_Com_Att_3.js);
var tempLceData = null;
var dataToReturn = null;

var SC_AI_Drone_Repair_Rout_Com_3 =
{
    AICombatRoutine: function (cData, ceData)
    {  
		if(ceData.lLSwtc == 0)
		{
			cData.cdtSwtc = 1;
			cData.cdSwtc = 1;
			tempLceData = SC_AI_Drone_Repair_Rout_Com_FndTarg_3.npcFindTarget(ceData);
			ceData = tempLceData;
		}
		if(ceData.eneL > 25)
		{
			ceData.lLSwtc = 0;
			ceData.ts = -1;
		}
		if (ceData.ts != -1)
        {
			if (!generator.ShipExists(ceData.ts) || ship.GetCurrentValue(ceData.ts, "structure") <= 0)
			{	
				if(npc.IsFriendlyLocked(ceData.objt.nid))
				{
					npc.Unlock(ceData.objt.nid);
				}
				if(npc.IsLocked(ceData.objt.nid))
				{
                    npc.Unlock(ceData.objt.nid);
				} 
				ceData.lLSwtc = 0;
				ceData.ts = -1;
			}
			else
			{
				tempLceData = SC_AI_Drone_Repair_Rout_Com_Att_3.npcCombatRoutine(ceData);
				ceData = tempLceData;
			}
		}
		else
		{
			cData = SC_AI_Drone_Repair_Rout_For_SFWP_3.npcStayInFormation(cData);			
		}
        dataToReturn = { forData: cData, comData: ceData };
        return dataToReturn;
    }
};

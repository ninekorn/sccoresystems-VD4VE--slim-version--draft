using(console);
using(npc);
using(ship);
using(storage);

include(SC_Utilities.js);

include(SC_AI_Drone_Get_nData.js);
include(SC_AI_Drone_Get_pData.js);

include(SC_AI_Drone_Mining_Rout_For_FWP_4.js);
include(SC_AI_Drone_Mining_Rout_Com_4.js);
include(SC_AI_Drone_Mining_Reset_Speed_4.js);
include(SC_AI_Drone_Mining_Friendlies_4.js);

var nData = null;
var pData = null;
var hpNPC;
var hpPlayer;

var cData = [];
var cDat = null;

var ceData = [];
var ceDat = null;

var lcData = [];
var lcDat = null;

var lceData = [];
var lceDat = null;

var distToWaypoint = 0;
var currentFormationWaypoint = { x: null, y: null };
var initOnce = 1;
var initArrayOfFriendlies = 1;
var arrayOfFriendlies = [];
var dataToReturn = null;
var globIndex;
var resetSwitch = 0;
var someTester = 1;

var SC_AI_Drone_Mining_Rout_For_4 =
{
    AIRoutineInit: function (currentObjective,addFriendSwitch) 
	{
		arrayOfFriendlies = SC_AI_Drone_Mining_Friendlies_4.AIAddFriendlies(currentObjective,addFriendSwitch);

        if (cData.length >= currentObjective.droneIndex && ceData.length >= currentObjective.droneIndex && cData[currentObjective.droneIndex] != null && ceData[currentObjective.droneIndex] != null )
        {
            hpNPC = ship.GetCurrentValue(currentObjective.nid, "structure");

            if (hpNPC > 0)
            {
                nData = SC_AI_Drone_Get_nData.npcGetSelfNPCData(currentObjective, 1);
            }
            else
            {
                return;
            }
            hpPlayer = ship.GetCurrentValue(currentObjective.pid, "structure");

            if (hpPlayer > 0)
            {
                pData = SC_AI_Drone_Get_pData.npcGetPlayerData(currentObjective, 0);
            }
			else
			{
				storage.SetGlobal("currentCommand_switch_" + currentObjective.nid, 1);
				storage.SetGlobal("currentCommand" + currentObjective.nid, { id: currentObjective.nid, command: 0, formation: currentObjective.formation, lastCommand: -1, wepPropType: currentObjective.wepPropType, wepDistType: currentObjective.wepDistType, droneIndex: currentObjective.droneIndex, addFriend: 0, baseVicinity: -1 });											
			}
            //else
            //{
            //  pData = lastArrayOfPlayerData[currentObjective.droneIndex];
            //}

            if (currentObjective.speedSwitch == 1)
            {
                if (nData != null && pData != null)
                {
                    /*if (cData[currentObjective.droneIndex] == null || ceData[currentObjective.droneIndex] == null)
                    {        
                        cData = [];
                        ceData = [];
                        addToArray(currentObjective, globIndex, 0);
                        return;
                    }*/
                    cData[currentObjective.droneIndex].nData = nData;
                    cData[currentObjective.droneIndex].pData = pData;
                    cData[currentObjective.droneIndex].objt = currentObjective;

                    ceData[currentObjective.droneIndex].nData = nData;
                    ceData[currentObjective.droneIndex].pData = pData;
                    ceData[currentObjective.droneIndex].pLData = pData;
                    ceData[currentObjective.droneIndex].objt = currentObjective;
					ceData[currentObjective.droneIndex].arrF = arrayOfFriendlies;

                    
                    currentFormationWaypoint = SC_AI_Drone_Mining_Rout_For_FWP_4.npcGWFP(currentObjective, 5);
                    distToWaypoint = SC_Utilities.GetDistance(currentFormationWaypoint, nData.nCoord);

                    cData[currentObjective.droneIndex].cFWP = currentFormationWaypoint;
                    cData[currentObjective.droneIndex].cFWPD = distToWaypoint;
                    				
					if (distToWaypoint >= 25) 
					{
						cData[currentObjective.droneIndex].lLSwtc = 0;
						cData[currentObjective.droneIndex].mSwtc = 1;
                        npc.Unstick(currentObjective.nid);
                        npc.Unlock(currentObjective.nid);
                        npc.StickToPoint(currentObjective.nid, cData[currentObjective.droneIndex].cFWP.x, cData[currentObjective.droneIndex].cFWP.y, 0);
                    }
                    else {
                        dataToReturn = SC_AI_Drone_Mining_Rout_Com_4.AICombatRoutine(cData[currentObjective.droneIndex], ceData[currentObjective.droneIndex]);
                        cData[currentObjective.droneIndex] = dataToReturn.forData;
                        ceData[currentObjective.droneIndex] = dataToReturn.comData;
                    }               
                }
                else if (nData != null && pData == null)
                {
                    if (ceData[currentObjective.droneIndex].pLData != null)
                    {
                        console.PrintError("check if player is docked? or destroyed");
                    }
                }
            }
            else if (currentObjective.speedSwitch == 0)
            {
                SC_AI_Drone_Mining_Reset_Speed_4.AICombatResetSpeed(currentObjective);
            }
        }
        else
        {
			if(cData.length < currentObjective.droneIndex || cData[0] == null)
			{
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
					mSwtc: 0,
					lsP: { x: null, y: null },
					ltP: { x: null, y: null },
					liP: { x: null, y: null },
					gliP: { x: null, y: null },

					dog: [],
					log: [],
					pfc: -2,
					cdSwtc: 0,
					cdtSwtc: 0,
					loRLDOT: 0,
					stopS: 1,
					stopSC: 0,
					stopSCM: 5,
					noFWP: 0,
					lLSwtc: 0,
					cAstP: { x: null, y: null },
					cAstPD: 0,
					iop: -1
				};
				cData.push(cDat);
			}
			else if(cData[currentObjective.droneIndex] == null)
			{
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
					mSwtc: 0,
					lsP: { x: null, y: null },
					ltP: { x: null, y: null },
					liP: { x: null, y: null },
					gliP: { x: null, y: null },

					dog: [],
					log: [],
					pfc: -2,
					cdSwtc: 0,
					cdtSwtc: 0,
					loRLDOT: 0,
					stopS: 1,
					stopSC: 0,
					stopSCM: 5,
					noFWP: 0,
					lLSwtc: 0,
					cAstP: { x: null, y: null },
					cAstPD: 0,
					iop: -1
				};
				cData[currentObjective.droneIndex] = cDat;
			}
			

			if(ceData.length < currentObjective.droneIndex|| ceData[0] == null)
			{
				ceDat =
				{
					nData: null,
					pData: null,
					eData: null,
					arrF: arrayOfFriendlies,
					objt: currentObjective,
					ts: -1,
					lLSwtc: 0,

					hasNA: 0,
					hasNAC: 0,

					hasStP: 0,
					hasStO: 0,

					mReset: 0,
					evad: 0,
					evadC: 0,

					loRLDOT: 0,
					dLStrc: 0,
					eneL: 0,
					eToP: 0,
					dToE: 0,
					fSwtc: 0,
					eneLSh: 0,
					npcLD: 0,

					stopS: null,
					stopSC: null,
				
				};
				ceData.push(ceDat);
			}
			else if(ceData[currentObjective.droneIndex] == null)
			{
				ceDat =
				{
					nData: null,
					pData: null,
					eData: null,
					arrF: arrayOfFriendlies,
					objt: currentObjective,
					ts: -1,
					lLSwtc: 0,

					hasNA: 0,

					hasStP: 0,
					hasStO: 0,

					tec: 0,
					tecC: 0,
					mReset: 0,
					evad: 0,
					evadC: 0,

					tec2: 0,
					tec2C: 0,
					loRLDOT: 0,
					tec3: 0,
					tec3C: 0,
					dLStrc: 0,
					eneL: 0,
					eToP: 0,
					dToE: 0,
					fSwtc: 0,

					eneLSh: 0,
					npcLD: 0,

					stopS: null,
					stopSC: null,
				};
				ceData[currentObjective.droneIndex] = ceDat;
			}
        }
    }
};


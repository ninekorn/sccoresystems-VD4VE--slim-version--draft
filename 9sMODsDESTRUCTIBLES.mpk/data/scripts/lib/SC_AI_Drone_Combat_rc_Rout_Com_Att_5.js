using(console);
using(npc);
using(ship);

include(SC_Utilities.js);
include(SC_AI_Drone_Get_eData.js);
include(SC_AI_Drone_Combat_rc_Rout_Com_Att_Tec2_5.js);
include(SC_AI_Drone_Combat_rc_Rout_Com_Att_Tec3_5.js);

var lockdist = 20;
var randomPos = -1;

var SC_AI_Drone_Combat_rc_Rout_Com_Att_5 =
{
    npcCombatRoutine: function (ceData) {

        if (generator.ShipExists(ceData.ts) && ship.GetCurrentValue(ceData.ts, "structure") > 0)
        {
            var otherShipCoord = ship.GetCoordinates(ceData.ts);

            var distPlayerToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.pData.pCoord);

            if (distPlayerToEnemy <= 25) // && distPlayerToDrone <= 20
            {
                var distDroneToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.nData.nCoord);

                ceData.eData = SC_AI_Drone_Get_eData.npcGetEnemyData(ceData.ts, 1);

				ceData.evad = 0;

                if (ceData.evad == 0) //ceData.evad != 1
                {
                    if (ceData.mReset == 0)
                    {
                        randomPos = Math.floor(Math.random() * (2 - 0) + 0);                                   
                        if (randomPos== 0)
                        {
                            ceData.tec = 2;
                        }
                        else if (randomPos == 1)
                        {
                            ceData.tec = 3;
                        }
						 ceData.mReset = 1;
                    }

                    if (ceData.mReset == 1)
                    {
                        if (ceData.hasNA == 0)
                        {
                            ceData.hasStP = 0;
                            ceData.hasStPC = 0;

                            ceData.evad = 0;

                            ceData.stopS = 0;
                            ceData.stopSC = 0;

							ceData.tec3 = 0;
							ceData.tec3C = 0;
							ceData.tec2 = 0;
							ceData.tec2C = 0;

                            ceData.hasNA = 1;
                        }

                        if (ceData.hasNA == 1)
                        {
                            if (ceData.tec == 2) {
                                ceData = SC_AI_Drone_Combat_rc_Rout_Com_Att_Tec2_5.npcAttackTec2(ceData, otherShipCoord, distDroneToEnemy);
                            }
                            else if (ceData.tec == 3) {
                                ceData = SC_AI_Drone_Combat_rc_Rout_Com_Att_Tec3_5.npcAttackTec3(ceData, otherShipCoord, distDroneToEnemy);
                            }
                        }

						if (ceData.tecC >= 2)
						{
							var dToEDamage = ship.GetDamageByShip(ceData.ts, ceData.objt.nid);
							var curShield = ship.GetCurrentValue(ceData.ts, "shield");
							//console.PrintError(curShield);
							//ceData.eneLSh
							if (dToEDamage > ceData.dToE || curShield < ceData.eneLSh) {
								//console.PrintError("dmg");
								//doing damage
								//do not change current combat routine
								//do what you are doing.
							}
							else// if (ceData.hasLo == 1 && dToEDamage <= ceData.dToE) 
							{
								//console.PrintError("!dmg");
								//not doing any damage
								//change combat routine
								 ceData.hasStP = 0;
								ceData.hasStPC = 0;

								ceData.evad = 0;

								ceData.stopS = 0;
								ceData.stopSC = 0;

								ceData.tec3 = 0;
								ceData.tec3C = 0;
								ceData.tec2 = 0;
								ceData.tec2C = 0;

								ceData.hasNA = 0;
								ceData.mReset = 0;
							}
                       
							ceData.dToE = dToEDamage;
							ceData.eneLSh = curShield;
							ceData.tecC = 0;
							//ceData.eToP = eToPDamage; // not implemented yet
						}
						ceData.tecC++;
                    }
                }
                else
                {
					ceData.hasNA = 0;
					ceData.mReset = 0;
					ceData.hasStP = 0;
					ceData.hasStPC = 0;

					ceData.evad = 0;

					ceData.stopS = 0;
					ceData.stopSC = 0;

					ceData.tec3 = 0;
					ceData.tec3C = 0;
					ceData.tec2 = 0;
					ceData.tec2C = 0;

                }

                if (ceData.hasStP == 1)
                {
                    ceData.hasStPC++;
                }

                if (ceData.stopS == 1)
                {
                    if (ceData.stopSC >= 0)
                    {
                        ceData.stopS = 0;
                        ceData.stopSC = 0;
                    }
                    ceData.stopSC++;
                }

                ceData.npcLD = distDroneToEnemy;
            }
            else {
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
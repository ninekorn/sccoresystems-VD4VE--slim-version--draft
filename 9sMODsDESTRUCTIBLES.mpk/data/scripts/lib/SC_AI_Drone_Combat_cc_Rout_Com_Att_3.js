using(console);
using(npc);
using(ship);

include(SC_Utilities.js);
include(SC_AI_Drone_Get_eData.js);
include(SC_AI_Drone_Combat_cc_Rout_Com_Evad_3.js);
include(SC_AI_Drone_Combat_cc_Rout_Com_Att_Tec0_3.js);
include(SC_AI_Drone_Combat_cc_Rout_Com_Att_Tec1_3.js);
include(SC_AI_Drone_Combat_cc_Rout_Com_Att_Tec2_3.js);
include(SC_AI_Drone_Combat_cc_Rout_Com_Att_Tec3_3.js);

var lockdist = 20;
var randomPos = -1;
var distPlayerToEnemy = 0;

var SC_AI_Drone_Combat_cc_Rout_Com_Att_3 =
{
    npcCombatRoutine: function (ceData)
    {

        if (generator.ShipExists(ceData.ts) && ship.GetCurrentValue(ceData.ts, "structure") > 0)
        {
            var otherShipCoord = ship.GetCoordinates(ceData.ts);

            /*if (ceData.objt.command == 0)
            {
                distPlayerToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.pData.pCoord);
            }
            else if (ceData.objt.command == 1)
            {
                distPlayerToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.forProDC);
            }   */       


            distPlayerToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.pData.pCoord);





            if (distPlayerToEnemy <= 25) // && distPlayerToDrone <= 20
            {
                var distDroneToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.nData.nCoord);

                ceData.eData = SC_AI_Drone_Get_eData.npcGetEnemyData(ceData.ts, 1);

                ceData = SC_AI_Drone_Combat_cc_Rout_Com_Evad_3.npcIsShouldEvade(ceData, otherShipCoord);

                if (ceData.evad == 0) //ceData.evad != 1
                {
                    //console.PrintError("tec: " + ceData.tec + " mRe: " + ceData.mReset + " na: " + ceData.hasNA);
                    if (ceData.mReset == 0)
                    {
                        randomPos = Math.floor(Math.random() * (2 - 0) + 0);
               
                        ceData.mReset = 1;

                        if (randomPos== 0)
                        {
                            ceData.tec = 2;
                        }
                        else if (randomPos == 1)
                        {
                            ceData.tec = 3;
                        }
                    }
                    else if (ceData.mReset == 1)
                    {
                        if (ceData.hasNA == 0)
                        {
                            ceData.hasNA = 0;
                            ceData.hasNAC = 0;
                            ceData.hasH = 0;
                            ceData.hasHC = 0;
                            ceData.hasStP = 0;
                            ceData.hasStPC = 0;
                            ceData.tecC = 0;

                            ceData.hasNA = 0;
                            ceData.evad = 0;

                            ceData.hasLo = 0;
                            ceData.hasLoC = 0;
                            ceData.hasLoF = 0;
                            ceData.hasLoFC = 0;
                            cData.stopS = 0;
                            cData.stopSC = 0;



                            if (ceData.tec == 0) {
                                ceData.hasLo = 1;
                            }
                            else if (ceData.tec == 1) {

                                ceData.hasLoF = 1;
                            }
                            else if (ceData.tec == 2) {

                                ceData.hasStP = 1;
                            }
                            else if (ceData.tec == 3) {

                                ceData.hasStP = 1;
                            }
                            //else if (ceData.tec == 4) // stick to object. - not implemented yet.
                            //{
                            //  
                            //}
                            ceData.hasNA = 1;
                        }

                        if (ceData.hasNA == 1)
                        {
                            if (ceData.tec == 0)
                            {
                                ceData = SC_AI_Drone_Combat_cc_Rout_Com_Att_Tec0_3.npcAttackTec0(ceData, otherShipCoord, distDroneToEnemy);
                            }
                            else if (ceData.tec == 1) {
                                ceData = SC_AI_Drone_Combat_cc_Rout_Com_Att_Tec1_3.npcAttackTec1(ceData, otherShipCoord, distDroneToEnemy);
                            }
                            else if (ceData.tec == 2) {
                                ceData = SC_AI_Drone_Combat_cc_Rout_Com_Att_Tec2_3.npcAttackTec2(ceData, otherShipCoord, distDroneToEnemy);
                            }
                            else if (ceData.tec == 3) {
                                ceData = SC_AI_Drone_Combat_cc_Rout_Com_Att_Tec3_3.npcAttackTec3(ceData, otherShipCoord, distDroneToEnemy);
                            }
                        }
                    }
                }
                else
                {
                    ceData.hasNA = 0;
                    ceData.hasNAC = 0;
                    ceData.hasH = 0;
                    ceData.hasHC = 0;
                    ceData.hasStP = 0;
                    ceData.hasStPC = 0;
                    ceData.tecC = 0;

                    ceData.evad = 0;

                    ceData.hasLo = 0;
                    ceData.hasLoC = 0;
                    ceData.hasLoF = 0;
                    ceData.hasLoFC = 0;

                    ceData.stopS = 0;
                    ceData.tec3 = 0;
                    ceData.tec3C = 0;
                    ceData.tec2 = 0;
                    ceData.tec2C = 0;

                    cData.stopS = 0;
                    cData.stopSC = 0;
                }


                if (ceData.hasLoF == 1) {
                    ceData.hasLoFC++;
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

                if (ceData.hasNA == 1) {
                    ceData.hasNAC++;
                }

                if (ceData.mReset == 1)
                {
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
                            ceData.hasNA = 0;
                            ceData.hasNAC = 0;
                            ceData.hasH = 0;
                            ceData.hasHC = 0;
                            ceData.hasStP = 0;
                            ceData.hasStPC = 0;

                            ceData.mReset = 0;
                            ceData.evad = 0;

                            ceData.hasLo = 0;
                            ceData.hasLoC = 0;
                            ceData.hasLoF = 0;
                            ceData.hasLoFC = 0;

                            ceData.stopS = 0;
                            ceData.tec3 = 0;
                            ceData.tec3C = 0;
                            ceData.tec2 = 0;
                            ceData.tec2C = 0;

                            cData.stopS = 0;
                            cData.stopSC = 0;
                        }
                       




                        ceData.dToE = dToEDamage;
                        ceData.eneLSh = curShield;
                        ceData.tecC = 0;
                        //ceData.eToP = eToPDamage; // not implemented yet
                    }
                    ceData.tecC++;
                }
                ceData.npcLD = distDroneToEnemy;
            }
            else {
                npc.Unstick(ceData.objt.nid);
                npc.Unlock(ceData.objt.nid);
                ceData.ts = -1;
            }
        }
        else
        {
            npc.Unstick(ceData.objt.nid);
            npc.Unlock(ceData.objt.nid);
            ceData.ts = -1;
        }

        return ceData;
    }
};











/*else
                       {
                           if (ceData.hasLo == 1 && dToEDamage <= ceData.dToE)
                           {
                               console.PrintError("!dmg");
                               //console.PrintError("!dmg");
                               //not doing any damage
                               //change combat routine
                               ceData.hasNA = 0;
                               ceData.hasNAC = 0;
                               ceData.hasH = 0;
                               ceData.hasHC = 0;
                               ceData.hasStP = 0;
                               ceData.hasStPC = 0;


                               ceData.mReset = 0;
                               ceData.evad = 0;

                               ceData.hasLo = 0;
                               ceData.hasLoC = 0;
                               ceData.hasLoF = 0;
                               ceData.hasLoFC = 0;

                               ceData.stopS = 0;
                               ceData.tec3 = 0;
                               ceData.tec3C = 0;
                               ceData.tec2 = 0;
                               ceData.tec2C = 0;
                           }
                       }*/
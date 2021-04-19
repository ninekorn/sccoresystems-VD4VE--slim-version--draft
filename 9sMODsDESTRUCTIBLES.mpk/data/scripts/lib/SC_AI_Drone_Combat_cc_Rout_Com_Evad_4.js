using(console);
using(player);
using(npc);
using(ship);

include(SC_Utilities.js);
include(SC_AI_Drone_Get_eData.js);

var lockdist = 20;
var cDroneHP = 0;
var drone_max_hp = 0;

var SC_AI_Drone_Combat_cc_Rout_Com_Evad_4 =
{
    npcIsShouldEvade: function (ceData, otherShipCoord)
    {
        if (ceData.evadC > 1)
        {
            drone_max_hp = ship.GetFinalCacheValue(ceData.objt.nid, "structure");
            cDroneHP = ship.GetCurrentValue(ceData.objt.nid, "structure");

            if (cDroneHP < ceData.dLStrc || ceData.evad == 1)
            {
                if ((ceData.dLStrc - cDroneHP) > 10 || cDroneHP <= drone_max_hp * 0.1) //drone is getting damage...
                {
                    if (ceData.eneDSwtc == 0) {
                        npc.StartEvasion(ceData.objt.nid);

                        ceData.eneDSwtc = 1;
                        ceData.evad = 1;
                    }
                }
                else if (ceData.evad == 1)
                {
                    if (ceData.eneDSwtc == 1)
                    {
                        if (ceData.eneDC >= 20)
                        {
                            npc.StopEvasion(ceData.objt.nid);
                            ceData.eneDC = 0;
                            ceData.eneDSwtc = 0;
                            ceData.evad = 0;
                        }
                    }
                    //console.PrintError("not enough damage");
                }
                else
                {
                    ceData.eneDC = 0;
                    ceData.eneDSwtc = 0;
                    ceData.evad = 0;
                }
            }
            else {
                //console.PrintError("no damage");
            }
            ceData.evadC = 0;
        }
        if (ceData.eneDSwtc == 1) {
            ceData.eneDC++;
        }
        ceData.evadC++;
        return ceData;       
    }
};

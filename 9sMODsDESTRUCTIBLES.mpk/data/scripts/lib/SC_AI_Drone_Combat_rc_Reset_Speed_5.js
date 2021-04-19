using(console);
using(npc);

var SC_AI_Drone_Combat_rc_Reset_Speed_5 =
{
    AICombatResetSpeed: function (currentObjective)
    {
        var originalPlayerShipSpeed = ship.GetFinalCacheValue(currentObjective.pid, "speed_max");
        var originalNpcShipFORWARDSpeed = ship.GetFinalCacheValue(currentObjective.nid, "speed_max");
        var originalNpcShipSTRAFESpeed = ship.GetFinalCacheValue(currentObjective.nid, "speed_strafe");

        var lastNID = currentObjective.nid;
        var lastPID = currentObjective.pid;
        var lastCOMMAND = currentObjective.command;
        var lastFormation = currentObjective.formation;
        var lastSysNID = currentObjective.sysIDP;
        var lastSysPID = currentObjective.sysIDN;
        var lastArgz = currentObjective.argz;

        var lastWepPropT = currentObjective.wepPropType;
        var lastWepDistT = currentObjective.wepDistType;
        var lastDroneIndex = currentObjective.droneIndex;

        var lastPName = currentObjective.pName;

        var lastSystem = currentObjective.sid;

        var lastBase = currentObjective.bid;


        var lastADef = currentObjective.aDef;
        var lastSDef = currentObjective.sDef;
        var lastADock = currentObjective.aDock;
        var lastSDock = currentObjective.sDock;
        var lastARep = currentObjective.aRep;
        var lastSRep = currentObjective.sRep;
        var lastMinHP = currentObjective.minHP;

        var lastSDefLoc = currentObjective.sDefLoc;


        npc.CleanObjectives(currentObjective.nid);
        npc.NextObjective(currentObjective.nid);

        npc.AddObjective(currentObjective.nid, "get_in_formation",
            {
                argz: lastArgz,
                nid: lastNID,
                pid: lastPID,
                sid: lastSystem,
                bid: lastBase,
                pName: lastPName,
                command: lastCOMMAND,
                formation: lastFormation,
                speedSwitch: 1,
                maxPlayerForwardSpeed: originalPlayerShipSpeed,
                maxNPCForwardSpeed: originalNpcShipFORWARDSpeed,
                maxNPCStrafeSpeed: originalNpcShipSTRAFESpeed,
                wepPropType: lastWepPropT,
                wepDistType: lastWepDistT,
                droneIndex: lastDroneIndex,
                aDock: lastADock,
                mmDock: lastSDock,
                aDef: lastADef,
                mDef: lastSDef,
                aRep: lastARep,
                mRep: lastSRep,
                minHP: lastMinHP,
                sDefLoc: lastSDefLoc
            });
        return;
    }
};

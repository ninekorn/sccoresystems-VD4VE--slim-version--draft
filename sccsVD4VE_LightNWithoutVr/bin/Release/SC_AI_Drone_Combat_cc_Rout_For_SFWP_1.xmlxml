using(console);
using(player);
using(npc);
using(ship);

include(SC_Utilities.js);

var SC_AI_Drone_Combat_cc_Rout_For_SFWP_1 = 
{
    npcStayInFormation: function (cData) //cData.nData, cData.pData, cData.objt, cData.cFWP,cData.cFWPD
    {
        if (cData.mSwtc == 1)
        {
            if (cData.cFWPD > 6 && !npc.IsLocked(cData.objt.nid))
            {      
                var pointInFrontOfWaypointX = cData.cFWP.x + cData.pData.pForward.x;
                var pointInFrontOfWaypointY = cData.cFWP.y + cData.pData.pForward.y;

                var pointInFrontOfWaypoint = { x: pointInFrontOfWaypointX, y: pointInFrontOfWaypointY };

                var dirToPointInFrontOfWaypointX = pointInFrontOfWaypointX - cData.cFWP.x;
                var dirToPointInFrontOfWaypointY = pointInFrontOfWaypointY - cData.cFWP.y;

                var someOtherMAG = Math.sqrt(((cData.nData.nCoord.x - pointInFrontOfWaypoint.x) * (cData.nData.nCoord.x - pointInFrontOfWaypoint.x)) + ((cData.nData.nCoord.y - pointInFrontOfWaypoint.y) * (cData.nData.nCoord.y - pointInFrontOfWaypoint.y)));

                var dirNPCToPointInFrontOfWaypointX = (pointInFrontOfWaypointX - cData.nData.nCoord.x) / someOtherMAG;
                var dirNPCToPointInFrontOfWaypointY = (pointInFrontOfWaypointY - cData.nData.nCoord.y) / someOtherMAG;

                var someOtherRLDOT = SC_Utilities.Dot(-dirToPointInFrontOfWaypointX, -dirToPointInFrontOfWaypointY, -dirNPCToPointInFrontOfWaypointX, -dirNPCToPointInFrontOfWaypointY);

                if (cData.cdSwtc == 1) {
                    if (someOtherRLDOT < cData.loRLDOT) {
                        npc.Stop(cData.objt.nid);
                        //npc.InstantStop(cData.objt.nid);
                    }

                    cData.cdtSwtc = 1;
                    cData.cdSwtc = 0;
                }

                var waypointOffsetX = cData.cFWP.x + (cData.pData.pForward.x * 2);
                var waypointOffsetY = cData.cFWP.y + (cData.pData.pForward.y * 2);
                var waypointOffset = { x: waypointOffsetX, y: waypointOffsetY };
                npc.StickToPoint(cData.objt.nid, waypointOffset.x, waypointOffset.y, 0);
                cData.loRLDOT = someOtherRLDOT;
                cData.fSwtc = 0;
            }

            if (cData.cFWPD <= 6 && !npc.IsLocked(cData.objt.nid) && cData.fSwtc == 0)
            {
                var pointInFrontOfWaypointX = cData.cFWP.x + cData.pData.pForward.x;
                var pointInFrontOfWaypointY = cData.cFWP.y + cData.pData.pForward.y;

                var pointInFrontOfWaypoint = { x: pointInFrontOfWaypointX, y: pointInFrontOfWaypointY };

                var dirToPointInFrontOfWaypointX = pointInFrontOfWaypointX - cData.cFWP.x;
                var dirToPointInFrontOfWaypointY = pointInFrontOfWaypointY - cData.cFWP.y;

                var someOtherMAG = Math.sqrt(((cData.nData.nCoord.x - pointInFrontOfWaypoint.x) * (cData.nData.nCoord.x - pointInFrontOfWaypoint.x)) + ((cData.nData.nCoord.y - pointInFrontOfWaypoint.y) * (cData.nData.nCoord.y - pointInFrontOfWaypoint.y)));

                var dirNPCToPointInFrontOfWaypointX = (pointInFrontOfWaypointX - cData.nData.nCoord.x) / someOtherMAG;
                var dirNPCToPointInFrontOfWaypointY = (pointInFrontOfWaypointY - cData.nData.nCoord.y) / someOtherMAG;

                var someOtherRLDOT = SC_Utilities.Dot(-dirToPointInFrontOfWaypointX, -dirToPointInFrontOfWaypointY, -dirNPCToPointInFrontOfWaypointX, -dirNPCToPointInFrontOfWaypointY);
          
                if (cData.cdtSwtc == 1) {
                    if (someOtherRLDOT < cData.loRLDOT) {
                        //console.PrintError("test001");
                        npc.Stop(cData.objt.nid);
                        //npc.InstantStop(cData.objt.nid);
                    }

                    cData.cdtSwtc = 0;
                    cData.cdSwtc = 1;
                }

                var waypointOffsetX = cData.cFWP.x + (cData.pData.pForward.x * 2);
                var waypointOffsetY = cData.cFWP.y + (cData.pData.pForward.y * 2);
                var waypointOffset = { x: waypointOffsetX, y: waypointOffsetY };
                npc.StickToPoint(cData.objt.nid, waypointOffset.x, waypointOffset.y, 0);
                cData.loRLDOT = someOtherRLDOT;
                if (cData.cFWPD <= 5) {
                    cData.fSwtc = 1;
                }
            }

            if (cData.cFWPD <= 6 && !npc.IsLocked(cData.objt.nid) && cData.fSwtc == 1)
            {
                if (cData.cFWPD > 1)
                {
                    var dirToWaypointX = (cData.cFWP.x - cData.nData.nCoord.x) / cData.cFWPD;
                    var dirToWaypointY = (cData.cFWP.y - cData.nData.nCoord.y) / cData.cFWPD;

                    var newOffsetWaypointPosX = cData.cFWP.x + (dirToWaypointX * 3);
                    var newOffsetWaypointPosY = cData.cFWP.y + (dirToWaypointY * 3);
                    var newOffsetWaypointPos = { x: newOffsetWaypointPosX, y: newOffsetWaypointPosY };
                    npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                }
                else
                {
                    var newOffsetWaypointPosX = cData.cFWP.x + (cData.pData.pForward.x * 3);
                    var newOffsetWaypointPosY = cData.cFWP.y + (cData.pData.pForward.y * 3);

                    npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosX, newOffsetWaypointPosY, 0);
                }
                cData.fSwtc = 1;
                cData.cdtSwtc = 1;
            }
        }

        return cData;
    }
};

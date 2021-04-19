using(console);
using(player);
using(npc);
using(ship);
using(generator);

include(SC_Utilities.js);
include(SC_AI_PathFind_Mining_InitGrid_5.js);
include(SC_AI_PathFind_Mining_5.js);


var gridWorldSize = { xL: 2, xR: 1, yB: 2, yT: 1 }; //{ xL: 5, xR: 4, yB: 5, yT: 4};
var nodeRadius = 1;
var stationTiles = null; // only increment if there is a different player, otherwise, the drone index doesnt matter here. only 1 pathfind allowed per player for the moment.
var pathData;


var SC_AI_Drone_Mining_Rout_For_SFWPP_5 =
{
    npcPathfindToWaypoint: function (cData) //cData.nData, cData.pData, cData.objt, cData.cFWP,cData.cFWPD
    {
        if (cData.mSwtc == 1) {
            npc.InstantStop(cData.objt.nid);
            npc.StopEvasion(cData.objt.nid);

            var player_name = game.GetShipOwner(cData.objt.pid);

            if (storage.IsSetGlobal("GlobalIndex_Player_" + player_name)) {
                cData.iop = storage.GetGlobal("GlobalIndex_Player_" + player_name);
            }

            cData.xtra = [];
            cData.pfc = -1;
            cData.log = [];
            cData.dog = [];


            cData.sSwtc = 1;
            var wayp = { x: Math.round(cData.cFWP.x), y: Math.round(cData.cFWP.y) };
            var iniPos = { x: Math.round(cData.nData.nCoord.x), y: Math.round(cData.nData.nCoord.y) };
            cData.lip = iniPos;
            cData.lsp = iniPos;
            cData.glip = iniPos;
            cData.ltp = wayp;


            var pathData = SC_AI_PathFind_Mining_InitGrid_5.npcBuildGrid(gridWorldSize, nodeRadius, 0, cData, null);

            cData.dog.grid = pathData.grid;
            cData.dog.openSet = pathData.openSet;
            cData.dog.closedSet = pathData.closedSet;

            var someDataOfGrid = { grid: cData.dog.grid };
            cData.log.push(someDataOfGrid);

            cData.mSwtc = 3;
        }
        else if (cData.mSwtc == 3) {
            var pathData = null;
            for (var i = 0; i < 10; i++) {
                pathData = SC_AI_PathFind_Mining_5.npcPathFind(gridWorldSize, nodeRadius, cData);
                cData.pfc = pathData.currentCommand;
                //console.PrintError("wtf");
                if (pathData.openSet != null && pathData.openSet.length > 0) {
                    //cData.dog.grid = pathData.grid;
                    cData.xtra = pathData.extra;
                    cData.dog.node = pathData.node;
                    cData.dog.openSet = pathData.openSet;
                    //cData.dog.closedSet = pathData.closedSet;
                    cData.dog.path = pathData.path;
                    cData.lsp = { x: cData.dog.node.worldPosition.x, y: cData.dog.node.worldPosition.y };

                    if (cData.pfc == 10) {
                        cData.mSwtc = 5;
                        break;
                    }
                }
            }
        }
        else if (cData.mSwtc == 5) {
            if (cData.dog.path != null) {
                if (cData.dog.path.length > 0) {
                    cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
                    cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

                    var distToNode = SC_Utilities.GetDistance(cData.dog.path[0].worldPosition, cData.nData.nCoord);

                    var dirToWaypointX = ((cData.dog.path[0].worldPosition.x) - (cData.nData.nCoord.x)) / distToNode;
                    var dirToWaypointY = ((cData.dog.path[0].worldPosition.y) - (cData.nData.nCoord.y)) / distToNode;

                    var newOffsetWaypointPosX = (cData.dog.path[0].worldPosition.x + (dirToWaypointX * 4.5));
                    var newOffsetWaypointPosY = (cData.dog.path[0].worldPosition.y + (dirToWaypointY * 4.5));

                    var newOffsetWaypointPos = { x: newOffsetWaypointPosX, y: newOffsetWaypointPosY };

                    var rotationDOTRL = SC_Utilities.Dot(dirToWaypointY, -dirToWaypointX, -cData.nData.nForward.x, -cData.nData.nForward.y);
                    var rotationDOTFB = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nForward.x, cData.nData.nForward.y);
                    var rotationDOTVELO = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nVelo.x, cData.nData.nVelo.y);

                    var distToNodeTwo = SC_Utilities.GetDistance(newOffsetWaypointPos, cData.nData.nCoord);

                    if (distToNodeTwo > 0.25) {
                        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);

                        if (rotationDOTFB >= 0.99) {
                            //npc.Unstick(cData.objt.nid);
                            //npc.GoForward(cData.objt.nid);
                            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                        }
                        else {
                            if (rotationDOTVELO < 0.99) {
                                npc.InstantStop(cData.objt.nid);
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                            }
                            else {
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                            }
                        }
                    }
                    else {
                        //npc.Unstick(cData.objt.nid);
                        cData.dog.path.shift();
                        //npc.Stop(cData.objt.nid);
                        //npc.InstantStop(cData.objt.nid);
                        npc.InstantStop(cData.objt.nid);

                    }
                }
                else {
                    if (cData.cFWPD > 4.5) {
                        cData.mSwtc = 1;
                    }
                    else {
                        //console.PrintError("reached waypoint");
                        cData.mSwtc = 9;
                    }
                }
            }
            else {
                if (cData.cFWPD > 4.5) {
                    cData.mSwtc = 1;
                }
                else {
                    //console.PrintError("reached waypoint");
                    cData.mSwtc = 9;
                }
            }
        }
        return cData;
    }
};
using(ship);
using(console);
using(npc);

include(SC_Utilities.js);

var SC_AI_Drone_Combat_cc_Rout_For_FWPB_1 =
{
    
    npcGWFPB: function (currentObjective, offset)
    {
        var stationData = storage.GetGlobal("station_tiles" + currentObjective.bid);

        if (stationData != null)
        {
            if (stationData.xml_id.indexOf("outpost") >= 0)
            {
                npc.InstantStop(currentObjective.nid);

                //add option menu in a new TOPICS to repair drones when they are docked... //sRep - repair switch
                //add option menu in a new TOPICS to AUTO send drones to dock and repair when they are near the base // aRep
                //add option to auto dock or not from outside the base from the onboard computer... same thing but from inside the base on the station terminal. //aDock
                //choose a random point near the turrets and station entrance. Then patrol between all turrets? maybe. // //add option to auto defend base or to follow player. // aDef mixed with sDef
                //choose a specific point near the turrets and station entrance. // sDef
                //when players leaves base radius, make drone follow route to player instead of stickToObject. // already there - its currentObjective.bid but bid has to be set to null when leaving base radius.



                /*var turretTopXLeft = stationData.coord.x - 30;
                var turretTopYLeft = stationData.coord.y - 5;

                var turretBottomXRight = stationData.coords.x + 30;
                var turretBottomYRight = stationData.coords.y - 18;

                var turretTopXRight = stationData.coord.x + 30;
                var turretTopYRight= stationData.coord.y - 5;

                var turretBottomXLeft = stationData.coords.x - 30;
                var turretBottomYLeft = stationData.coords.y - 18;*/




                //var stationPosX = stationData.coord.x;
                //var stationPosY = stationData.coord.y;
                //var currentDefenseZonePos = SC_Utilities.RotatePoint(, stationData.coord);





                formationWaypoint = { x: pointRightX, y: pointRightY };
            }
            else if (stationData.xml_id.indexOf("science") >= 0)
            {
                npc.InstantStop(currentObjective.nid);
            }
            else if (stationData.xml_id.indexOf("military") >= 0)
            {
                npc.InstantStop(currentObjective.nid);
            }
            else if (stationData.xml_id.indexOf("mining") >= 0)
            {
                npc.InstantStop(currentObjective.nid);
            }
            else if (stationData.xml_id.indexOf("business") >= 0)
            {
                npc.InstantStop(currentObjective.nid);
            }
        }





        //currentObjective.bid





















        /*var formationWaypoint = null;
        var angle = ship.GetRotation(currentObjective.pid);
        var radToDeg = angle * (180.0 / Math.PI);
        var coordsPlayer = ship.GetCoordinates(currentObjective.pid);

        var pointFrontX = (1 * Math.cos(radToDeg * Math.PI / 180)) + coordsPlayer.x;
        var pointFrontY = (1 * Math.sin(radToDeg * Math.PI / 180)) + coordsPlayer.y;

        var playerDirX = pointFrontX - coordsPlayer.x;
        var playerDirY = pointFrontY - coordsPlayer.y;

        var backDirPlayer = { x: -playerDirX, y: -playerDirY };
        var rightDirPlayer = { x: playerDirY, y: -playerDirX };
        var leftDirPlayer = { x: -playerDirY, y: playerDirX };

        if (currentObjective.formation == 1) // top Right
        {
            var pointRightX = coordsPlayer.x + (rightDirPlayer.x * offset);
            var pointRightY = coordsPlayer.y + (rightDirPlayer.y * offset);
            formationWaypoint = { x: pointRightX, y: pointRightY };
        }
        else if (currentObjective.formation == 2) // top Left
        {
            var pointRightX = coordsPlayer.x + (leftDirPlayer.x * offset);
            var pointRightY = coordsPlayer.y + (leftDirPlayer.y * offset);
            formationWaypoint = { x: pointRightX, y: pointRightY };
        }
        else if (currentObjective.formation == 3) // bottom Right
        {
            var pointRightX = (coordsPlayer.x + (rightDirPlayer.x * offset + 1) + (backDirPlayer.x * offset + 1));
            var pointRightY = (coordsPlayer.y + (rightDirPlayer.y * offset + 1) + (backDirPlayer.y * offset + 1));
            formationWaypoint = { x: pointRightX, y: pointRightY };
        }
        else if (currentObjective.formation == 4) // bottom Left
        {
            var pointRightX = (coordsPlayer.x + (leftDirPlayer.x * offset + 1) + (backDirPlayer.x * offset + 1));
            var pointRightY = (coordsPlayer.y + (leftDirPlayer.y * offset + 1) + (backDirPlayer.y * offset + 1));
            formationWaypoint = { x: pointRightX, y: pointRightY };
        }
        else if (currentObjective.formation == 5) // middle bottom
        {
            var pointRightX = coordsPlayer.x + (backDirPlayer.x * offset);
            var pointRightY = coordsPlayer.y + (backDirPlayer.y * offset);
            formationWaypoint = { x: pointRightX, y: pointRightY };
        }
        return formationWaypoint;*/
    }
};

using(ship);
using(console);

var SC_AI_Drone_Combat_rc_Rout_For_FWP_4 =
{
    npcGWFP: function (currentObjective,offset) {
        var formationWaypoint = null;
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
            var pointRightX = coordsPlayer.x + (rightDirPlayer.x * offset+1);
            var pointRightY = coordsPlayer.y + (rightDirPlayer.y * offset+1);
            formationWaypoint = { x: pointRightX, y: pointRightY };
        }
        else if (currentObjective.formation == 2) // top Left
        {
            var pointRightX = coordsPlayer.x + (leftDirPlayer.x * offset+1);
            var pointRightY = coordsPlayer.y + (leftDirPlayer.y * offset+1);
            formationWaypoint = { x: pointRightX, y: pointRightY };
        }
        else if (currentObjective.formation == 3) // bottom Right
        {
            var pointRightX = (coordsPlayer.x + (rightDirPlayer.x * offset + 2) + (backDirPlayer.x * offset + 2));
            var pointRightY = (coordsPlayer.y + (rightDirPlayer.y * offset + 2) + (backDirPlayer.y * offset + 2));
            formationWaypoint = { x: pointRightX, y: pointRightY };
        }
        else if (currentObjective.formation == 4) // bottom Left
        {
            var pointRightX = (coordsPlayer.x + (leftDirPlayer.x * offset + 2) + (backDirPlayer.x * offset + 2));
            var pointRightY = (coordsPlayer.y + (leftDirPlayer.y * offset + 2) + (backDirPlayer.y * offset + 2));
            formationWaypoint = { x: pointRightX, y: pointRightY };
        }
        else if (currentObjective.formation == 5) // middle bottom
        {
            var pointRightX = coordsPlayer.x + (backDirPlayer.x * offset+2);
            var pointRightY = coordsPlayer.y + (backDirPlayer.y * offset+2);
            formationWaypoint = { x: pointRightX, y: pointRightY };
        }
        return formationWaypoint;
    }
};

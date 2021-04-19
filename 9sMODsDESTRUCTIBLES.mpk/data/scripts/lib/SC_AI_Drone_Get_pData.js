using(console);
using(ship);

var currentCoordsPlayer;
var anglerOfPlayer;
var radToDegrerOfPlayer;
var pointXOfPlayer;
var pointYOfPlayer;
var dirXOfPlayer;
var dirYOfPlayer;
var forwardOfPlayer;
var veloPlayer;
var speedPlayer;
var pData;

var SC_AI_Drone_Get_pData =
{
    npcGetPlayerData: function (currentObjective,veloSwitch) 
	{
		var current_structure = ship.GetCurrentValue(currentObjective.pid, "structure");

		if(generator.ShipExists(currentObjective.pid) && current_structure > 0)
		{
			currentCoordsPlayer = ship.GetCoordinates(currentObjective.pid);
			anglerOfPlayer = ship.GetRotation(currentObjective.pid);
			radToDegrerOfPlayer = anglerOfPlayer * (180.0 / Math.PI);
			pointXOfPlayer = (1 * Math.cos(radToDegrerOfPlayer * Math.PI / 180)) + currentCoordsPlayer.x;
			pointYOfPlayer = (1 * Math.sin(radToDegrerOfPlayer * Math.PI / 180)) + currentCoordsPlayer.y;
			dirXOfPlayer = pointXOfPlayer - currentCoordsPlayer.x;
			dirYOfPlayer = pointYOfPlayer - currentCoordsPlayer.y;
			forwardOfPlayer = { x: dirXOfPlayer, y: dirYOfPlayer };

			if (veloSwitch == 1)
			{
				veloPlayer = ship.GetVelocity(currentObjective.pid);
				speedPlayer = Math.sqrt(veloPlayer.x * veloPlayer.x + veloPlayer.y * veloPlayer.y);
				veloPlayer.x /= speedPlayer;
				veloPlayer.y /= speedPlayer;
				pData = { pCoord: currentCoordsPlayer, pAngle: radToDegrerOfPlayer, pForward: forwardOfPlayer, pVelo: veloPlayer, pSpeed: speedPlayer };
				return pData;
			}
			else
			{
				pData = { pCoord: currentCoordsPlayer, pAngle: radToDegrerOfPlayer, pForward: forwardOfPlayer, pVelo: null, pSpeed: null  };
				return pData;
			}
		}
		else{
			return null;
		}
    }
};

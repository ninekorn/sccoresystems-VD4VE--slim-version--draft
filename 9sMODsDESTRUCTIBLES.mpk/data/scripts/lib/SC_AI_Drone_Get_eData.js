using(console);
using(ship);

var currentCoordsEnemy;
var anglerOfEnemy;
var radToDegrerOfEnemy;
var pointXOfEnemy;
var pointYOfEnemy;
var dirXOfEnemy;
var dirYOfEnemy;
var forwardOfEnemy;
var veloEnemy;
var speedEnemy;
var eData;

var SC_AI_Drone_Get_eData =
{
    npcGetEnemyData: function (enemyID, veloSwitch)
    {
		//var max_structure = ship.GetFinalCacheValue(enemyID, "structure");
        var current_structure = ship.GetCurrentValue(enemyID, "structure");

		if(generator.ShipExists(enemyID) &&current_structure > 0)
		{
			currentCoordsEnemy = ship.GetCoordinates(enemyID);
			anglerOfEnemy = ship.GetRotation(enemyID);
			radToDegrerOfEnemy = anglerOfEnemy * (180.0 / Math.PI);
			pointXOfEnemy = (1 * Math.cos(radToDegrerOfEnemy * Math.PI / 180)) + currentCoordsEnemy.x;
			pointYOfEnemy = (1 * Math.sin(radToDegrerOfEnemy * Math.PI / 180)) + currentCoordsEnemy.y;
			dirXOfEnemy = pointXOfEnemy - currentCoordsEnemy.x;
			dirYOfEnemy = pointYOfEnemy - currentCoordsEnemy.y;
			forwardOfEnemy = { x: dirXOfEnemy, y: dirYOfEnemy };

			if (veloSwitch == 1)
			{
				veloEnemy = ship.GetVelocity(enemyID);
				speedEnemy = Math.sqrt(veloEnemy.x * veloEnemy.x + veloEnemy.y * veloEnemy.y);
				veloEnemy.x /= speedEnemy;
				veloEnemy.y /= speedEnemy;

				eData = { eCoord: currentCoordsEnemy, eAngle: radToDegrerOfEnemy, eForward: forwardOfEnemy, eVelo: veloEnemy, eSpeed: speedEnemy };
				return eData;
			}
			else
			{
				eData = { eCoord: currentCoordsEnemy, eAngle: radToDegrerOfEnemy, eForward: forwardOfEnemy, eVelo: null, eSpeed: null };
				return eData;
			}
		}
		else{
			return null;
		}

       

       
    }
};

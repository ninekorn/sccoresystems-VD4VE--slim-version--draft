using(console);
using(ship);

var currentCoordsNPC;
var angler;
var radToDegrer;
var npcPointX00;
var npcPointY00;
var dirX00;
var dirY00;
var forwardNPC;
var veloNPC;
var speedNPC;
var nData;

var SC_AI_Drone_Get_nData =
{
    npcGetSelfNPCData: function (currentObjective, veloSwitch) {

		var current_structure = ship.GetCurrentValue(currentObjective.nid, "structure");

		if(generator.ShipExists(currentObjective.nid) &&current_structure > 0)
		{
			currentCoordsNPC = ship.GetCoordinates(currentObjective.nid);
			angler = ship.GetRotation(currentObjective.nid);
			radToDegrer = angler * (180.0 / Math.PI);
			npcPointX00 = (1 * Math.cos(radToDegrer * Math.PI / 180)) + currentCoordsNPC.x;
			npcPointY00 = (1 * Math.sin(radToDegrer * Math.PI / 180)) + currentCoordsNPC.y;
			dirX00 = npcPointX00 - currentCoordsNPC.x;
			dirY00 = npcPointY00 - currentCoordsNPC.y;
			forwardNPC = { x: dirX00, y: dirY00 };

			if (veloSwitch == 1) {
				veloNPC = ship.GetVelocity(currentObjective.nid);
				speedNPC = Math.sqrt(veloNPC.x * veloNPC.x + veloNPC.y * veloNPC.y);
				veloNPC.x /= speedNPC;
				veloNPC.y /= speedNPC;
				nData = { nCoord: currentCoordsNPC, nAngle: radToDegrer, nForward: forwardNPC, nVelo: veloNPC, nSpeed: speedNPC };
				return nData;
			}
			else {
				nData = { nCoord: currentCoordsNPC, nAngle: radToDegrer, nForward: forwardNPC, nVelo: null, nSpeed: null };
				return nData;
			}
		}
		else{
			return null;
		}
    }
};

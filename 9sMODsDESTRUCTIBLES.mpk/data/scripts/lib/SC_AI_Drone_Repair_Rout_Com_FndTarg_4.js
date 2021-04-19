using(console);
using(player);
using(npc);
using(ship);

include(SC_Utilities.js);

var lockdist = 25;

var SC_AI_Drone_Repair_Rout_Com_FndTarg_4 =
{
    npcFindTarget: function (ceData)
    {
		var cDroneHP = ship.GetCurrentValue(ceData.objt.nid, "structure");
		ceData.dLStrc = cDroneHP;
		ceData.ts = -1;

		var arrayOfShips = null;
		arrayOfShips = npc.GetShipsInScope(ceData.objt.nid);
		var shipNDist = [];

		/*for(var i = 0;i < ceData.arrF.length;i++)
		{
			console.PrintError(ceData.arrF[i]);
		}*/

		if (ceData.arrF.length > 0)
		{
			if (arrayOfShips.length > 0)
			{
				for (var i = 0; i < arrayOfShips.length; i++)
				{
					var otherShip = arrayOfShips[i];
					
					if (generator.ShipExists(otherShip) && ship.GetCurrentValue(otherShip, "structure") < ship.GetFinalCacheValue(otherShip, "structure") && npc.GetTag(otherShip, "ownerPlayerShipId") == ceData.objt.pid ||
						generator.ShipExists(otherShip) && ship.GetCurrentValue(otherShip, "structure") < ship.GetFinalCacheValue(otherShip, "structure") && otherShip == ceData.objt.pid)
					{						
						//var playName = ceData.pName.toLowerCase();
						var shipName = game.GetShipOwner(otherShip).toLowerCase();

						if (SC_Utilities.contains(ceData.arrF, shipName))
						{
							var otherShipCoord = ship.GetCoordinates(otherShip);
						
							var distNpcToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.nData.nCoord);
							var distPlayerToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.pData.pCoord);
							if (distPlayerToEnemy <= lockdist) // && distNpcToEnemy <= lockdist
							{
								ship.SetDamageByShip(otherShip, 1, ceData.objt.nid);
								ship.SetDamageByShip(ceData.objt.nid, 1, otherShip);

								var someData = { id: otherShip, dist: distPlayerToEnemy, eToD: 0 };
								shipNDist.push(someData);
							}
						}
					}
				}

				if (shipNDist != null)
				{
					if (shipNDist.length > 0)
					{
						shipNDist.sort(function (a, b)
						{
							var adist = a.dist;
							var bdist = b.dist;
							//var acdam = a.damage;
							//var bcdam = b.damage;

							if (adist < bdist) {
								return -1;
							}
							else if (adist > bdist) {
								return 1;
							}
							else {
								return 0;
							}
						});                   

						ceData.lLSwtc = 1;
						ceData.ts = shipNDist[0].id;
						ceData.eneL =  shipNDist[0].dist;
						//console.PrintError(ceData.eneL);

					}
					else {
						npc.Unstick(ceData.objt.nid);
						npc.Unlock(ceData.objt.nid);
						ceData.lLSwtc = 0;
						ceData.ts = -1;
					}
				}
				else {
           
					npc.Unstick(ceData.objt.nid);
					npc.Unlock(ceData.objt.nid);
					ceData.lLSwtc = 0;
					ceData.ts = -1;   
				}
			}
			else
			{
				npc.Unstick(ceData.objt.nid);
				npc.Unlock(ceData.objt.nid);
				ceData.lLSwtc = 0;
				ceData.ts = -1;
			}
		}
		else 
		{
			npc.Unstick(ceData.objt.nid);
			npc.Unlock(ceData.objt.nid);
			ceData.lLSwtc = 0;
			ceData.ts = -1;
		}
		//console.PrintError( game.GetShipOwner(ceData.ts));
        return ceData;
    }
};

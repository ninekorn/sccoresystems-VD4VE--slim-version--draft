using(storage);
using(storage);

include(SC_Utilities.js);


var arrayOfFriendliesGlobal = [];
var arrayOfFriendlies;

var SC_AI_Drone_Mining_Friendlies_2 =
{
	AIAddFriendlies : function (currentObjective, addFriendSwitch)
	{				
		if(arrayOfFriendliesGlobal.length < currentObjective.droneIndex || arrayOfFriendliesGlobal[0] == null)
		{							
			if(!storage.IsSetGlobal("FriendliesList" + currentObjective.pid))
			{
				var someFriendliesData = [
			
					currentObjective.pName.toLowerCase(),
					"turret",
					"merchant",
					"miner",
					"patrol",
					"repair",
					"mining",
					"combat",
					"scurvy",					
				];
				var dat = {allies:someFriendliesData};
				arrayOfFriendliesGlobal[currentObjective.droneIndex] = dat;
				storage.SetGlobal("FriendliesList" + currentObjective.pid, someFriendliesData); 
			}
			else
			{		 		
				var friendliesLister = storage.GetGlobal("FriendliesList" + currentObjective.pid); 
				var dat = {allies:friendliesLister};
				arrayOfFriendliesGlobal[currentObjective.droneIndex] = dat;
			}
		}
		else
		{
			if(!storage.IsSetGlobal("FriendliesList" + currentObjective.pid))
			{
				var someFriendliesData = [
			
					currentObjective.pName.toLowerCase(),
					"turret",
					"merchant",
					"miner",
					"patrol",
					"repair",
					"mining",
					"combat",
					"scurvy",					
				];
				var dat = {allies:someFriendliesData};
				arrayOfFriendliesGlobal[currentObjective.droneIndex] = dat;
				storage.SetGlobal("FriendliesList" + currentObjective.pid, someFriendliesData); 
			}
			else
			{		 		
				var friendliesLister = storage.GetGlobal("FriendliesList" + currentObjective.pid); 
				var dat = {allies:friendliesLister};
				arrayOfFriendliesGlobal[currentObjective.droneIndex] = dat;
			}
		}
		
		if (addFriendSwitch == 1)
        {
			var friendliesLister = storage.GetGlobal("FriendliesList" + currentObjective.pid); 

			var playerToAddToFriendlies = storage.GetGlobal("FriendlyToAdd" + currentObjective.pid);

			if(!SC_Utilities.contains(friendliesLister,playerToAddToFriendlies))
			{
				friendliesLister.push(playerToAddToFriendlies.toLowerCase());

				var dat = {allies:friendliesLister};
				arrayOfFriendliesGlobal[currentObjective.droneIndex] = dat;

				var glob = storage.GetGlobal("currentCommand" + currentObjective.nid);
				glob.addFriend = 0;
				storage.SetGlobal("currentCommand" + currentObjective.nid, glob);
				storage.SetGlobal("FriendliesList" + currentObjective.pid,  arrayOfFriendliesGlobal[currentObjective.droneIndex].allies); 
			}		
        }
		else if (addFriendSwitch == 3)
        {
            var playerToRemoveFromFriendlies = storage.GetGlobal("FriendlyToAdd" + currentObjective.pid);
            var indexOfToRemovePlayer = playerToRemoveFromFriendlies.index;

			var friendliesLister = storage.GetGlobal("FriendliesList" + currentObjective.pid); 
			friendliesLister.splice(indexOfToRemovePlayer, 1);

			var dat = {allies:friendliesLister};
			arrayOfFriendliesGlobal[currentObjective.droneIndex] = dat;
            //arrayOfFriendliesGlobal[currentObjective.droneIndex].allies.splice(indexOfToRemovePlayer, 1);

            var glob = storage.GetGlobal("currentCommand" + currentObjective.nid);
            glob.addFriend = 0;
            storage.SetGlobal("currentCommand" + currentObjective.nid, glob);
			storage.SetGlobal("FriendliesList" + currentObjective.pid,  arrayOfFriendliesGlobal[currentObjective.droneIndex].allies); 
        }

		return arrayOfFriendliesGlobal[currentObjective.droneIndex].allies;
	}
};


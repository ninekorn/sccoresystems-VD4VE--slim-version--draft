using(items);

var SC_AI_Drone_Mining_InventoryChecks_2 = 
{
	AICheckStateOfInventory: function(npcID) 
	{
		var NPCContainerID = items.GetGameObjectContainerId(npcID);
		var itemsInCargoOfNPC = items.GetItemsAndCargo(NPCContainerID);

		for (var i = 0; i < 1; i++) 
		{

			var totalSizeOfNPCStorage = 0;
			var emptySpaceToRemoveNPC = 0;
			var initialQuantityOfItemsInNpcStorage = 0;

			if (itemsInCargoOfNPC != null) {
				if (itemsInCargoOfNPC.length > 0) {
					if (itemsInCargoOfNPC[0] != null) {
						for (var i = 0; i < itemsInCargoOfNPC.length; i++) {
							initialQuantityOfItemsInNpcStorage += itemsInCargoOfNPC[i].quantity;
						}
					}
					else {
						initialQuantityOfItemsInNpcStorage = 0;
					}
				}
				else {
					initialQuantityOfItemsInNpcStorage = 0;
				}
			}
			else {
				initialQuantityOfItemsInNpcStorage = 0;
			}

			items.AddItem(NPCContainerID, "goods_emptyspace", 100000);
			var itemsInCargoOfNPCAfterAddingEmptySpace = items.GetItemsAndCargo(NPCContainerID);
			if (itemsInCargoOfNPCAfterAddingEmptySpace != null) {
				if (itemsInCargoOfNPCAfterAddingEmptySpace.length > 0) {
					if (itemsInCargoOfNPCAfterAddingEmptySpace[0] != null) {
						for (var i = 0; i < itemsInCargoOfNPCAfterAddingEmptySpace.length; i++) {
							totalSizeOfNPCStorage += itemsInCargoOfNPCAfterAddingEmptySpace[i].quantity;
						}
					}
					else {
						totalSizeOfNPCStorage = 0;
					}
				}
				else {
					totalSizeOfNPCStorage = 0;
				}
			}
			else {
				totalSizeOfNPCStorage = 0;
			}



			var emptySpaceToRemoveNPC = totalSizeOfNPCStorage - initialQuantityOfItemsInNpcStorage;
			items.RemoveCargo(NPCContainerID, "goods_emptyspace", emptySpaceToRemoveNPC);
			if (initialQuantityOfItemsInNpcStorage < totalSizeOfNPCStorage) {

				return 0;
			}
			else
			{
				return 1;
			}
		}
	},


	AIIsPlayerCargoFull: function (playerID) {
		for (var i = 0; i < 1; i++) {
			var totalSizeOfPlayerStorage = 0;
			var initialQuantityOfItemsInPlayerStorage = 0;
			var emptySpaceToRemovePlayer = 0;

			var PlayerContainerID = items.GetGameObjectContainerId(playerID);
			var itemsInCargoOfPlayer = items.GetItemsAndCargo(PlayerContainerID);
			if (itemsInCargoOfPlayer != null) {
				if (itemsInCargoOfPlayer.length > 0) {
					if (itemsInCargoOfPlayer[0] != null) {
						for (var i = 0; i < itemsInCargoOfPlayer.length; i++) {
							initialQuantityOfItemsInPlayerStorage += itemsInCargoOfPlayer[i].quantity;
						}
					}
					else {
						initialQuantityOfItemsInPlayerStorage = 0;
					}
				}
				else {
					initialQuantityOfItemsInPlayerStorage = 0;
				}
			}
			else {
				initialQuantityOfItemsInPlayerStorage = 0;
			}


			items.AddItem(PlayerContainerID, "goods_emptyspace", 100000);
			var itemsInCargoOfPlayerAfterAddingEmptySpace = items.GetItemsAndCargo(PlayerContainerID);

			if (itemsInCargoOfPlayerAfterAddingEmptySpace != null) {
				if (itemsInCargoOfPlayerAfterAddingEmptySpace.length > 0) {
					if (itemsInCargoOfPlayerAfterAddingEmptySpace[0] != null) {
						for (var i = 0; i < itemsInCargoOfPlayerAfterAddingEmptySpace.length; i++) {
							totalSizeOfPlayerStorage += itemsInCargoOfPlayerAfterAddingEmptySpace[i].quantity;
						}
					}
					else {
						totalSizeOfPlayerStorage = 0;
					}
				}
				else {
					totalSizeOfPlayerStorage = 0;
				}
			}
			else {
				totalSizeOfPlayerStorage = 0;
			}


			var emptySpaceToRemovePlayer = totalSizeOfPlayerStorage - initialQuantityOfItemsInPlayerStorage;
			items.RemoveCargo(PlayerContainerID, "goods_emptyspace", emptySpaceToRemovePlayer);

			if (emptySpaceToRemovePlayer > 1)
			{
				var someData = {swtc: 0, space: emptySpaceToRemovePlayer};
				return someData;
			}
			else
			{	
				var someData = {swtc: 1, space: emptySpaceToRemovePlayer};
				return someData;
			}
		}
	}
};
 
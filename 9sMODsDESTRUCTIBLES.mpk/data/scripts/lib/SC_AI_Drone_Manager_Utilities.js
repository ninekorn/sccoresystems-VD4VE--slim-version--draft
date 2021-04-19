using(npc);
using(storage);
using(generator);
using(console);

var SC_AI_Drone_Manager_Utilities =
{

    //inpired from https://stackoverflow.com/questions/18484577/how-to-get-a-random-number-from-a-range-excluding-some-values
    randomInArray: function (player_id)
    {
        if (!storage.IsSetGlobal("data" + player_id)) {
            for (var i = 0; i < 6; i++) //arrayOfFormationPosNId.length // 0 for random and 1 to 5 for the formations
            {
                arrayOfFormationPosNId[i] = { pos: 0, id: -1 };
            }

            currentDroneCounter = { droneCounter: droneCounter, arrayOfFormationPosNId: arrayOfFormationPosNId };
            //currentDroneCounter.droneCounter = droneCounter;
            //currentDroneCounter.arrayOfPos = null;
            //currentDroneCounter.arrayOfPos = arrayOfFormationPosNId;

            //currentDroneCounter.droneCounter = droneCounter;
            //currentDroneCounter.arrayOfPos = null;
            //currentDroneCounter.arrayOfPos = arrayOfFormationPosNId;

            storage.SetGlobal("data" + player_id, currentDroneCounter);



        }
        else {
            var someData = storage.GetGlobal("data" + player_id);
            var arrayOfPos = someData.arrayOfPos;
            var tempPosArray = [];

            for (var i = 1; i < arrayOfPos.length; i++) {
                if (arrayOfPos[i].pos == 0) {
                    tempPosArray.push(i);
                }
            }

            var miner = 0;
            var maxer = tempPosArray.length; //- 1

            var randomNumber = Math.floor(Math.random() * (maxer - miner) + miner); //Math.floor(

            var result = tempPosArray[randomNumber];
            arrayOfPos[result].pos = 1;

            someData.arrayOfPos = arrayOfPos;
            storage.SetGlobal("data" + player_id, someData);

        }



        console.PrintError("player_id: " + player_id);
        
        return result;
    },

    resetGlobalArray: function (player_id)
    {

        var currentDroneCounter;
        /*var someData = storage.GetGlobal("data" + player_id);
        var arrayOfPos = someData.arrayOfPos;
        var tempPosArray = [];

        for (var i = 1; i < arrayOfPos.length; i++) {
            if (arrayOfPos[i].pos == 0) {
                tempPosArray.push(i);
            }
        }

        var miner = 0;
        var maxer = tempPosArray.length; //- 1

        var randomNumber = Math.floor(Math.random() * (maxer - miner) + miner); //Math.floor(

        var result = tempPosArray[randomNumber];
        arrayOfPos[result].pos = 1;

        someData.arrayOfPos = arrayOfPos;
        storage.SetGlobal("data" + player_id, someData);*/



        console.PrintError("player_id: " + player_id);
        var arrayOfFormationPosNId = [];

        if (!storage.IsSetGlobal("data" + player_id))
        {       
            for (var i = 0; i < 6; i++) //arrayOfFormationPosNId.length // 0 for random and 1 to 5 for the formations
            {
                arrayOfFormationPosNId[i] = { pos: 0, id: -1 };
            }

            currentDroneCounter = { droneCounter: droneCounter, arrayOfFormationPosNId: arrayOfFormationPosNId};
            //currentDroneCounter.droneCounter = droneCounter;
            //currentDroneCounter.arrayOfPos = null;
            //currentDroneCounter.arrayOfPos = arrayOfFormationPosNId;
           
            //currentDroneCounter.droneCounter = droneCounter;
            //currentDroneCounter.arrayOfPos = null;
            //currentDroneCounter.arrayOfPos = arrayOfFormationPosNId;

            storage.SetGlobal("data" + player_id, currentDroneCounter);



        }
        else
        {
            currentDroneCounter = storage.GetGlobal("data" + player_id);
            arrayOfFormationPosNId = currentDroneCounter.arrayOfPos;
        }





        //var currentDroneCounter = storage.GetGlobal("data" + player_id);

        var droneCounter = 0;
        //var arrayOfFormationPosNId = currentDroneCounter.arrayOfPos;

        for (var i = 0; i < arrayOfFormationPosNId.length; i++) {
            if (arrayOfFormationPosNId[i].id != -1) {
                var shipExists = generator.ShipExists(arrayOfFormationPosNId[i].id);

                if (shipExists && ship.GetCurrentValue(arrayOfFormationPosNId[i].id, "structure") > 0) {
                    if (player_id == npc.GetTag(arrayOfFormationPosNId[i].id, "ownerPlayerShipId")) {
                        arrayOfFormationPosNId[i].id = arrayOfFormationPosNId[i].id;
                        arrayOfFormationPosNId[i].pos = 1;
                        droneCounter++;
                    }
                    else {
                        arrayOfFormationPosNId[i].pos = 0;
                        arrayOfFormationPosNId[i].id = -1;
                    }
                }
                else {
                    arrayOfFormationPosNId[i].pos = 0;
                    arrayOfFormationPosNId[i].id = -1;
                }
            }
            else {
                arrayOfFormationPosNId[i].pos = 0;
                arrayOfFormationPosNId[i].id = -1;
            }
        }

        currentDroneCounter.droneCounter = droneCounter;
        currentDroneCounter.arrayOfPos = null;
        currentDroneCounter.arrayOfPos = arrayOfFormationPosNId;

        storage.SetGlobal("data" + player_id, currentDroneCounter);


        return currentDroneCounter;
    },

    getIDSOfDronesByClass: function (player_id, droneType)
    {
        var currentDroneCounter = storage.GetGlobal("data" + player_id);
        //var droneCounter = 0;
        var arrayOfFormationPosNId = currentDroneCounter.arrayOfPos;
        var tempArray = [];

        for (var i = 0; i < arrayOfFormationPosNId.length; i++) {
            //console.PrintError(arrayOfFormationPosNId[i].pos + " _ " + arrayOfFormationPosNId[i].id);
            if (arrayOfFormationPosNId[i].id != -1) {
                var test = generator.ShipExists(arrayOfFormationPosNId[i].id);

                if (test)
                {
                    if (droneType == npc.GetTag(arrayOfFormationPosNId[i].id, "droneClass"))
                    {
                        var array = { pos: arrayOfFormationPosNId[i].pos, id: arrayOfFormationPosNId[i].id, formation: arrayOfFormationPosNId[i].formation };
                        tempArray.push(array);
                        //arrayOfPos[i].id = arrayOfPos[i].id;
                        //droneCounter++;
                    }
                }
            }
        }
        return tempArray;
    },


    getIDSOfDrones: function (player_id) {
        var currentDroneCounter = storage.GetGlobal("data" + player_id);
        //var droneCounter = 0;
        var arrayOfFormationPosNId = currentDroneCounter.arrayOfPos;
        var tempArray = [];

        for (var i = 0; i < arrayOfFormationPosNId.length; i++) {
            //console.PrintError(arrayOfFormationPosNId[i].pos + " _ " + arrayOfFormationPosNId[i].id);
            if (arrayOfFormationPosNId[i].id != -1) {
                var test = generator.ShipExists(arrayOfFormationPosNId[i].id);

                if (test)
                {                 
                    var array = { pos: arrayOfFormationPosNId[i].pos, id: arrayOfFormationPosNId[i].id, formation: arrayOfFormationPosNId[i].formation };
                    tempArray.push(array);
                        //arrayOfPos[i].id = arrayOfPos[i].id;
                        //droneCounter++;                 
                }
            }
        }
        return tempArray;
    },

    setInitialGlobalArray: function (weaponDistanceType, chosenPosition)
    {
        var index;

        var drone_combat_energy_cc_removed = null;
        var drone_combat_energy_rc_removed = null;
        var drone_combat_energy_cc_removed_index = null;
        var drone_combat_energy_rc_removed_index = null;
        var drone_combat_energy_cc_added = null;
        var drone_combat_energy_rc_added = null;
        var drone_combat_energy_cc_added_index = null;
        var drone_combat_energy_rc_added_index = null;

        if (weaponDistanceType == "cc")
        {
            if (storage.IsSetGlobal("drone_combat_energy_cc_removed_"+ chosenPosition))
            {
                var someRemovedenergy;
                drone_combat_energy_cc_removed = storage.GetGlobal("drone_combat_energy_cc_removed_"+ chosenPosition, someRemovedenergy);

                if (drone_combat_energy_cc_removed.length > 0)
                {
                    //there are drones that were removed from the game... the drone index is available.
                    index = drone_combat_energy_cc_removed[0].index;
                    drone_combat_energy_cc_removed[0].id = null;
                    drone_combat_energy_cc_removed.shift();

                    storage.SetGlobal("drone_combat_energy_cc_removed_"+chosenPosition, drone_combat_energy_cc_removed);
                }
                else 
				{
                    //there are no drone index that was removed from the game. create a new one.
                    if (storage.IsSetGlobal("drone_combat_energy_cc_added_"+ chosenPosition))
                    {
                        drone_combat_energy_cc_added = storage.GetGlobal("drone_combat_energy_cc_added_"+ chosenPosition);
						//console.PrintError("drone_combat_energy_cc_added.length: " + drone_combat_energy_cc_added.length);
                        index = drone_combat_energy_cc_added.length;
                        var droneData = { id: null, index: index };

                        drone_combat_energy_cc_added.push(droneData);
                        storage.SetGlobal("drone_combat_energy_cc_added_"+ chosenPosition, drone_combat_energy_cc_added);
                    }
                    else {
                        var droneDataMain = [];
                        index = 0;
                        var droneData = { id: null, index: index };
                        droneDataMain.push(droneData);
                        storage.SetGlobal("drone_combat_energy_cc_added_"+ chosenPosition, droneDataMain);
                    }
                }
            }
            else
            {
                if (storage.IsSetGlobal("drone_combat_energy_cc_added_"+ chosenPosition))
                {
                    drone_combat_energy_cc_added = storage.GetGlobal("drone_combat_energy_cc_added_"+ chosenPosition);

					//console.PrintError("drone_combat_energy_cc_added.length: " + drone_combat_energy_cc_added.length);
                    index = drone_combat_energy_cc_added.length;
                    var droneData = { id: null, index: index };

                    drone_combat_energy_cc_added.push(droneData);
                    storage.SetGlobal("drone_combat_energy_cc_added_"+ chosenPosition, drone_combat_energy_cc_added);
                }
                else {
                    var droneDataMain = [];
                    index = 0;
                    var droneData = { id: null, index: index };
                    droneDataMain.push(droneData);
                    storage.SetGlobal("drone_combat_energy_cc_added_"+ chosenPosition, droneDataMain);
                }
            }
        }
        else {
            if (storage.IsSetGlobal("drone_combat_energy_rc_removed_"+ chosenPosition))
            {
                var someRemovedenergy;
                drone_combat_energy_rc_removed = storage.GetGlobal("drone_combat_energy_rc_removed_"+ chosenPosition, someRemovedenergy);

                if (drone_combat_energy_rc_removed.length > 0)
                {
                    //there are drones that were removed from the game... the drone index is available.
                    index = drone_combat_energy_rc_removed[0].index;
                    drone_combat_energy_rc_removed[0].id = null;
                    drone_combat_energy_rc_removed.shift();

                    storage.SetGlobal("drone_combat_energy_rc_removed_"+chosenPosition, drone_combat_energy_rc_removed);
                }
                else 
				{
                    //there are no drone index that was removed from the game. create a new one.
                    if (storage.IsSetGlobal("drone_combat_energy_rc_added_"+ chosenPosition))
                    {
                        drone_combat_energy_rc_added = storage.GetGlobal("drone_combat_energy_rc_added_"+ chosenPosition);
						//console.PrintError("drone_combat_energy_cc_added.length: " + drone_combat_energy_cc_added.length);
                        index = drone_combat_energy_cc_added.length;
                        var droneData = { id: null, index: index };

                        drone_combat_energy_rc_added.push(droneData);
                        storage.SetGlobal("drone_combat_energy_rc_added_"+ chosenPosition, drone_combat_energy_rc_added);
                    }
                    else {
                        var droneDataMain = [];
                        index = 0;
                        var droneData = { id: null, index: index };
                        droneDataMain.push(droneData);
                        storage.SetGlobal("drone_combat_energy_rc_added_"+ chosenPosition, droneDataMain);
                    }
                }
            }
            else
            {
                if (storage.IsSetGlobal("drone_combat_energy_rc_added_"+ chosenPosition))
                {
                    drone_combat_energy_rc_added = storage.GetGlobal("drone_combat_energy_rc_added_"+ chosenPosition);

					//console.PrintError("drone_combat_energy_cc_added.length: " + drone_combat_energy_cc_added.length);
                    index = drone_combat_energy_rc_added.length;
                    var droneData = { id: null, index: index };

                    drone_combat_energy_rc_added.push(droneData);
                    storage.SetGlobal("drone_combat_energy_rc_added_"+ chosenPosition, drone_combat_energy_rc_added);
                }
                else {
                    var droneDataMain = [];
                    index = 0;
                    var droneData = { id: null, index: index };
                    droneDataMain.push(droneData);
                    storage.SetGlobal("drone_combat_energy_rc_added_"+ chosenPosition, droneDataMain);
                }
            }
        }
        return index;
    },

    setFinalGlobalDroneArray: function (droneid, droneIndex, weaponDistanceType, weaponPropulsionType, chosenPosition)
    {
        var index = 0;
        if (weaponDistanceType == "cc")
        {
            if (storage.IsSetGlobal("drone_combat_energy_cc_added_"+ chosenPosition)) {

                var someTest;
                var toReceiveData = storage.GetGlobal("drone_combat_energy_cc_added_"+ chosenPosition);

                var droneData = { id: droneid, index: droneIndex };
                toReceiveData[droneIndex] = droneData;
                storage.SetGlobal("drone_combat_energy_cc_added_"+ chosenPosition, toReceiveData);
            }
            else
            {
                var droneDataMain = [];
                index = 0;
                var droneData = { id: droneid, index: index };
                droneDataMain.push(droneData);
                storage.SetGlobal("drone_combat_energy_cc_added_"+ chosenPosition, droneDataMain);
            }
        }
        else
        {
            if (storage.IsSetGlobal("drone_combat_energy_rc_added_"+ chosenPosition)) {

                var someTest;
                var toReceiveData = storage.GetGlobal("drone_combat_energy_rc_added_"+ chosenPosition);

                var droneData = { id: droneid, index: droneIndex };
                toReceiveData[droneIndex] = droneData;
                storage.SetGlobal("drone_combat_energy_rc_added_"+ chosenPosition, toReceiveData);
            }
            else
            {
                var droneDataMain = [];
                index = 0;
                var droneData = { id: droneid, index: index };
                droneDataMain.push(droneData);
                storage.SetGlobal("drone_combat_energy_rc_added_"+ chosenPosition, droneDataMain);
            }
        }
    },




    setRemovedFromGlobalDroneArray: function (droneIndex, weaponDistanceType, weaponPropulsionType,chosenPosition)
	{
        if (weaponDistanceType == "cc")
        {
            if (storage.IsSetGlobal("drone_combat_energy_cc_removed_"+ chosenPosition))
            {
                var someTest;
                var toReceiveData = storage.GetGlobal("drone_combat_energy_cc_removed_"+ chosenPosition, someTest);
                var droneData = { id: null, index: droneIndex };
                toReceiveData.push(droneData);
                storage.SetGlobal("drone_combat_energy_cc_removed_"+ chosenPosition, toReceiveData);
            }
            else
            {
                var droneData = { id: null, index: droneIndex };
                var toReceiveData = [];
                toReceiveData.push(droneData);
                storage.SetGlobal("drone_combat_energy_cc_removed_"+ chosenPosition, toReceiveData);
            }
		}
        else
        {
           if (storage.IsSetGlobal("drone_combat_energy_rc_removed_"+ chosenPosition))
            {
                var someTest;
                var toReceiveData = storage.GetGlobal("drone_combat_energy_rc_removed_"+ chosenPosition, someTest);
                var droneData = { id: null, index: droneIndex };
                toReceiveData.push(droneData);
                storage.SetGlobal("drone_combat_energy_rc_removed_"+ chosenPosition, toReceiveData);
            }
            else
            {
                var droneData = { id: null, index: droneIndex };
                var toReceiveData = [];
                toReceiveData.push(droneData);
                storage.SetGlobal("drone_combat_energy_rc_removed_"+ chosenPosition, toReceiveData);
            }
		}
    },










	setInitialGlobalArrayRepair: function (weaponDistanceType, chosenPosition)
    {
        var index;

        var drone_combat_energy_cc_removed = null;
        var drone_combat_energy_rc_removed = null;
        var drone_combat_energy_cc_removed_index = null;
        var drone_combat_energy_rc_removed_index = null;
        var drone_combat_energy_cc_added = null;
        var drone_combat_energy_rc_added = null;
        var drone_combat_energy_cc_added_index = null;
        var drone_combat_energy_rc_added_index = null;


        if (storage.IsSetGlobal("drone_repair_removed_"+ chosenPosition))
        {
            var someRemovedenergy;
            drone_combat_energy_cc_removed = storage.GetGlobal("drone_repair_removed_"+ chosenPosition, someRemovedenergy);

            if (drone_combat_energy_cc_removed.length > 0)
            {
                //there are drones that were removed from the game... the drone index is available.
                index = drone_combat_energy_cc_removed[0].index;
                drone_combat_energy_cc_removed[0].id = null;
                drone_combat_energy_cc_removed.shift();

                storage.SetGlobal("drone_repair_removed_"+chosenPosition, drone_combat_energy_cc_removed);
            }
            else 
			{
                //there are no drone index that was removed from the game. create a new one.
                if (storage.IsSetGlobal("drone_repair_added_"+ chosenPosition))
                {
                    drone_combat_energy_cc_added = storage.GetGlobal("drone_repair_added_"+ chosenPosition);
					//console.PrintError("drone_combat_energy_cc_added.length: " + drone_combat_energy_cc_added.length);
                    index = drone_combat_energy_cc_added.length;
                    var droneData = { id: null, index: index };

                    drone_combat_energy_cc_added.push(droneData);
                    storage.SetGlobal("drone_repair_added_"+ chosenPosition, drone_combat_energy_cc_added);
                }
                else {
                    var droneDataMain = [];
                    index = 0;
                    var droneData = { id: null, index: index };
                    droneDataMain.push(droneData);
                    storage.SetGlobal("drone_repair_added_"+ chosenPosition, droneDataMain);
                }
            }
        }
        else
        {
            if (storage.IsSetGlobal("drone_repair_added_"+ chosenPosition))
            {
                drone_combat_energy_cc_added = storage.GetGlobal("drone_repair_added_"+ chosenPosition);

				//console.PrintError("drone_combat_energy_cc_added.length: " + drone_combat_energy_cc_added.length);
                index = drone_combat_energy_cc_added.length;
                var droneData = { id: null, index: index };

                drone_combat_energy_cc_added.push(droneData);
                storage.SetGlobal("drone_repair_added_"+ chosenPosition, drone_combat_energy_cc_added);
            }
            else {
                var droneDataMain = [];
                index = 0;
                var droneData = { id: null, index: index };
                droneDataMain.push(droneData);
                storage.SetGlobal("drone_repair_added_"+ chosenPosition, droneDataMain);
            }
        }    
        return index;
    },

    setFinalGlobalDroneArrayRepair: function (droneid, droneIndex, weaponDistanceType, weaponPropulsionType, chosenPosition)
    {     
        if (storage.IsSetGlobal("drone_repair_added_"+ chosenPosition)) {

            var someTest;
            var toReceiveData = storage.GetGlobal("drone_repair_added_"+ chosenPosition);

            var droneData = { id: droneid, index: droneIndex };
            toReceiveData[droneIndex] = droneData;
            storage.SetGlobal("drone_repair_added_"+ chosenPosition, toReceiveData);
        }
        else
        {
            var droneDataMain = [];
            index = 0;
            var droneData = { id: droneid, index: index };
            droneDataMain.push(droneData);
            storage.SetGlobal("drone_repair_added_"+ chosenPosition, droneDataMain);
        }
    }, 

	setRemovedFromGlobalDroneArrayRepair: function (droneIndex, weaponDistanceType, weaponPropulsionType,chosenPosition)
	{      
        if (storage.IsSetGlobal("drone_repair_removed_"+ chosenPosition))
        {
            var someTest;
            var toReceiveData = storage.GetGlobal("drone_repair_removed_"+ chosenPosition, someTest);
            var droneData = { id: null, index: droneIndex };
            toReceiveData.push(droneData);
            storage.SetGlobal("drone_repair_removed_"+ chosenPosition, toReceiveData);
        }
        else
        {
            var droneData = { id: null, index: droneIndex };
            var toReceiveData = [];
            toReceiveData.push(droneData);
            storage.SetGlobal("drone_repair_removed_"+ chosenPosition, toReceiveData);
        }
    },

















	setInitialGlobalArrayMining: function (weaponDistanceType, chosenPosition)
    {
        var index;

        var drone_combat_energy_cc_removed = null;
        var drone_combat_energy_rc_removed = null;
        var drone_combat_energy_cc_removed_index = null;
        var drone_combat_energy_rc_removed_index = null;
        var drone_combat_energy_cc_added = null;
        var drone_combat_energy_rc_added = null;
        var drone_combat_energy_cc_added_index = null;
        var drone_combat_energy_rc_added_index = null;


        if (storage.IsSetGlobal("drone_mining_removed_"+ chosenPosition))
        {
            var someRemovedenergy;
            drone_combat_energy_cc_removed = storage.GetGlobal("drone_mining_removed_"+ chosenPosition, someRemovedenergy);

            if (drone_combat_energy_cc_removed.length > 0)
            {
                //there are drones that were removed from the game... the drone index is available.
                index = drone_combat_energy_cc_removed[0].index;
                drone_combat_energy_cc_removed[0].id = null;
                drone_combat_energy_cc_removed.shift();

                storage.SetGlobal("drone_mining_removed_"+chosenPosition, drone_combat_energy_cc_removed);
            }
            else 
			{
                //there are no drone index that was removed from the game. create a new one.
                if (storage.IsSetGlobal("drone_mining_added_"+ chosenPosition))
                {
                    drone_combat_energy_cc_added = storage.GetGlobal("drone_mining_added_"+ chosenPosition);
					//console.PrintError("drone_combat_energy_cc_added.length: " + drone_combat_energy_cc_added.length);
                    index = drone_combat_energy_cc_added.length;
                    var droneData = { id: null, index: index };

                    drone_combat_energy_cc_added.push(droneData);
                    storage.SetGlobal("drone_mining_added_"+ chosenPosition, drone_combat_energy_cc_added);
                }
                else {
                    var droneDataMain = [];
                    index = 0;
                    var droneData = { id: null, index: index };
                    droneDataMain.push(droneData);
                    storage.SetGlobal("drone_mining_added_"+ chosenPosition, droneDataMain);
                }
            }
        }
        else
        {
            if (storage.IsSetGlobal("drone_mining_added_"+ chosenPosition))
            {
                drone_combat_energy_cc_added = storage.GetGlobal("drone_mining_added_"+ chosenPosition);

				//console.PrintError("drone_combat_energy_cc_added.length: " + drone_combat_energy_cc_added.length);
                index = drone_combat_energy_cc_added.length;
                var droneData = { id: null, index: index };

                drone_combat_energy_cc_added.push(droneData);
                storage.SetGlobal("drone_mining_added_"+ chosenPosition, drone_combat_energy_cc_added);
            }
            else {
                var droneDataMain = [];
                index = 0;
                var droneData = { id: null, index: index };
                droneDataMain.push(droneData);
                storage.SetGlobal("drone_mining_added_"+ chosenPosition, droneDataMain);
            }
        }    
        return index;
    },

    setFinalGlobalDroneArrayMining: function (droneid, droneIndex, weaponDistanceType, weaponPropulsionType, chosenPosition)
    {     
        if (storage.IsSetGlobal("drone_mining_added_"+ chosenPosition)) {

            var someTest;
            var toReceiveData = storage.GetGlobal("drone_mining_added_"+ chosenPosition);

            var droneData = { id: droneid, index: droneIndex };
            toReceiveData[droneIndex] = droneData;
            storage.SetGlobal("drone_mining_added_"+ chosenPosition, toReceiveData);
        }
        else
        {
            var droneDataMain = [];
            index = 0;
            var droneData = { id: droneid, index: index };
            droneDataMain.push(droneData);
            storage.SetGlobal("drone_mining_added_"+ chosenPosition, droneDataMain);
        }
    }, 

	setRemovedFromGlobalDroneArrayMining: function (droneIndex, weaponDistanceType, weaponPropulsionType,chosenPosition)
	{      
        if (storage.IsSetGlobal("drone_mining_removed_"+ chosenPosition))
        {
            var someTest;
            var toReceiveData = storage.GetGlobal("drone_mining_removed_"+ chosenPosition, someTest);
            var droneData = { id: null, index: droneIndex };
            toReceiveData.push(droneData);
            storage.SetGlobal("drone_mining_removed_"+ chosenPosition, toReceiveData);
        }
        else
        {
            var droneData = { id: null, index: droneIndex };
            var toReceiveData = [];
            toReceiveData.push(droneData);
            storage.SetGlobal("drone_mining_removed_"+ chosenPosition, toReceiveData);
        }
    }






};
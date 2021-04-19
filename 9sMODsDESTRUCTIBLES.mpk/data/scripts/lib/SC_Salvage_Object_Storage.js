using(npc);
using(console);
using(timer);

var systemSpaceObjects;// = [];

var SC_Salvage_Object_Storage =
{
    AddSpaceObjectToStorage: function (system_id, SpaceObjectID, randomYieldToStartOff)
    {
        if (storage.IsSetGlobal("systemid_" + system_id + "_salvage"))
        {
            var toReceiveData = storage.GetGlobal("systemid_" + system_id + "_salvage");
            var objectdata = { sys_id: system_id, id: SpaceObjectID, yield: randomYieldToStartOff, GameTime: timer.GetGameTime() };
            toReceiveData.push(objectdata);

            storage.SetGlobal("systemid_" + system_id + "_salvage", toReceiveData);
            /*var someTest;
            var toReceiveData = storage.GetGlobal("drone_mining_added_" + chosenPosition);

            var droneData = { id: droneid, index: droneIndex };
            toReceiveData[droneIndex] = droneData;
            storage.SetGlobal("drone_mining_added_" + chosenPosition, toReceiveData);*/
        }
        else
        {
            var toSendData = [];
            var objectdata = { sys_id: system_id, id: SpaceObjectID, yield: randomYieldToStartOff, GameTime: timer.GetGameTime()};
            toSendData.push(objectdata);
            storage.SetGlobal("systemid_" + system_id + "_salvage", toSendData);
        }
    },
    GetSpaceObjectStorageLength: function (system_id)
    {
        if (storage.IsSetGlobal("systemid_" + system_id + "_salvage"))
        {
            var toReceiveData = storage.GetGlobal("systemid_" + system_id + "_salvage");
            return toReceiveData.length;
        }
        else
        {
            return 0;
        }
    },


    RemoveFromYield: function (system_id, SpaceObjectID, currentYieldMinus)
    {
        if (storage.IsSetGlobal("systemid_" + system_id + "_salvage"))
        {
            var toReceiveData = storage.GetGlobal("systemid_" + system_id + "_salvage");

            for (var s = 0; s < toReceiveData.length;s++)
            {
                if (toReceiveData[s].id == SpaceObjectID)
                {
                    toReceiveData[s].yield = toReceiveData[s].yield - currentYieldMinus;
                    break;
                }
            }

            storage.SetGlobal("systemid_" + system_id + "_salvage", toReceiveData);
        }
        else
        {
            console.PrintError("the space object doesn't exist in the global array and this message is never supposed to print even with a console debug_reinit. If it prints, tell me. 0");
        }
    },

    SetDepleted: function (system_id, SpaceObjectID)
    {
        if (storage.IsSetGlobal("systemid_" + system_id + "_salvage"))
        {
            var toReceiveData = storage.GetGlobal("systemid_" + system_id + "_salvage");
            var s = 0
            for (s = 0; s < toReceiveData.length; s++)
            {
                if (toReceiveData[s].id == SpaceObjectID)
                {
                    toReceiveData[s] = null;
                    break;
                }
            }

            toReceiveData.splice(s, 1);

            storage.SetGlobal("systemid_" + system_id + "_salvage", toReceiveData);
        }
        else
        {
            console.PrintError("the space object doesn't exist in the global array and this message is never supposed to print even with a console debug_reinit. If it prints, tell me. 1");
        }
    },


    GetSpaceObjectYield: function (system_id, SpaceObjectID)
    {
        if (storage.IsSetGlobal("systemid_" + system_id + "_salvage"))
        {
            var toReceiveData = storage.GetGlobal("systemid_" + system_id + "_salvage");

            for (var s = 0; s < toReceiveData.length; s++)
            {
                if (toReceiveData[s].id == SpaceObjectID)
                {
                    return toReceiveData[s].yield;
                }
            }
        }
        else
        {
            console.PrintError("the space object doesn't exist in the global array and this message is never supposed to print even with a console debug_reinit. If it prints, tell me. 2");
        }
    },
























    /*AddSpaceObjectToStorage: function (system_id, SpaceObjectID)
    {
        if (systemSpaceObjects != null)
        {
            var objectdata = { sys_id: system_id, id: SpaceObjectID };
            systemSpaceObjects.push(objectdata);
        }
        else
        {
            systemSpaceObjects = [];
            var objectdata = { sys_id: system_id, id: SpaceObjectID };
            systemSpaceObjects.push(objectdata);
        }      
    },

    GetSpaceObjectStorageLength: function (system_id, SpaceObjectID)
    {
        if (systemSpaceObjects != null)
        {
            return systemSpaceObjects.length;
        }
        else
        {
            systemSpaceObjects = [];
            return 0;
        }
    },

    ReturnSystemSpaceObjectFromStorage: function (system_id)
    {
        var SpaceObjectArray = [];

        //console.PrintError(systemSpaceObjects.length);

        if (systemSpaceObjects != null)
        {
            if (systemSpaceObjects.length > 0)
            {
                for (var s = 0; s < systemSpaceObjects.length; s++)
                {
                    if (systemSpaceObjects[s].sys_id == system_id)
                    {
                        var objectdata = { sys_id: systemSpaceObjects[s].sys_id, id: systemSpaceObjects[s].id };
                        SpaceObjectArray.push(objectdata);
                    }
                }
            }
        }


        if (SpaceObjectArray.length > 0)
        {
            return SpaceObjectArray;
        }
        else
        {
            return null;
        }
        //var coords = game.GetObjectCoordinates(args.system_id, jag_id);
        //var objectdata = { sys_id: system_id, id: SpaceObjectID };
        //systemSpaceObjects.push(objectdata);
    }*/
};
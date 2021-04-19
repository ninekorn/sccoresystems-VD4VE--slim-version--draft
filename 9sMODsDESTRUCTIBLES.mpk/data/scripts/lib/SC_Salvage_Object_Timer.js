using(npc);
using(console);
using(timer);

var counterFrame = 0;
var counterDevice = 0;

var deviceArray = [];


var SC_Salvage_Object_Timer =
{

    /*InitAddPlayerID: function (player_id)
    {
        if (storage.IsSetGlobal("player_id" + player_id + "_salvage_device_"))
        {

        }
        else
        {
            storage.SetGlobal("player_id" + player_id + "_salvage_device_" + args.device_id, null);
        }
    },*/





    ClearCurrentDevice: function (sys_id, device_id, ship_id, slot_id) //IsWorking
    {
        game.IsShipPlayerControlled(ship_id);

        var playerName = game.GetShipOwner(ship_id);

        if (storage.IsSetGlobal("GlobalIndex_Player_" + playerName))
        {
            var someGlobalIndex = storage.GetGlobal("GlobalIndex_Player_" + playerName);


            if (!storage.IsSetGlobal("systemid_" + sys_id + "_ship_" + ship_id + "_salvage_device_" + device_id))
            {
                storage.SetGlobal("systemid_" + sys_id + "_ship_" + ship_id + "_salvage_device_" + device_id, { swtch: 1, ship_id: ship_id, device_id: device_id, slot_id: slot_id, timer_id: null, reset: 1 });

            }
            else
            {
                var globalSalvageStorage = storage.GetGlobal("systemid_" + sys_id + "_ship_" + ship_id + "_salvage_device_" + device_id);

                if (globalSalvageStorage != null) {

                    if (globalSalvageStorage.timer_id != null)
                    {
                        //timer.AddOrUpdate("CustomOnFrame", null);
                        //timer.AddOrUpdate(0.25, "CustomOnFrame", null, 1); //

                        game.ShipStopSound(ship_id, "mining_process_" + slot_id);
                        visual.DeviceDeactivateEffect(ship_id, device_id, "mining_visual_effect");

                        timer.ClearTimer(globalSalvageStorage.timer_id);

                        var data = { swtch: 1, ship_id: ship_id, device_id: device_id, slot_id: slot_id, timer_id: null, reset: 0 };
                        storage.SetGlobal("systemid_" + sys_id + "_ship_" + ship_id + "_salvage_device_" + device_id, data); 


                        //console.PrintError("cleared device timer: " + globalSalvageStorage.timer_id);
                        return 1;
                    }
                    else
                    {
                        //console.PrintError("null timer0");
                        return 0;
                    }
                }
                else
                {
                    //console.PrintError("null globalSalvageStorage");
                    return 0;
                }



                /*var count = 0;

                for (var d = 0; d < globalSalvageStorage.length; d++)
                {
                    if (globalSalvageStorage[d].device_id == device_id)
                    {
                        timer.ClearTimer(globalSalvageStorage[d].timer_id);
                        console.PrintError("cleared device timer: " + globalSalvageStorage[d].timer_id);
                        count++;
                    }
                }

                if (count != 0) {

                }
                else {
                    console.PrintError("cleared device");
                }*/
            }
            




            /**/



            /*if (deviceArray == null)
            {
                deviceArray = [];
                var ArrayOfDevice = [];
                var dataOfDevice = { isWorking: -1, ship_id: ship_id, sys_id: sys_id, device_id: device_id, timer_id: timer_id };
                ArrayOfDevice.push(dataOfDevice);
                deviceArray[someGlobalIndex] = ArrayOfDevice;
                return ArrayOfDevice;
            }
            else
            {
                if (deviceArray[someGlobalIndex] == null)
                {
                    var ArrayOfDevice = [];
                    var dataOfDevice = { isWorking: -1, ship_id: ship_id, sys_id: sys_id, device_id: device_id, timer_id: timer_id };
                    ArrayOfDevice.push(dataOfDevice);
                    deviceArray[someGlobalIndex] = ArrayOfDevice;
                    return ArrayOfDevice;
                }
                else
                {
                    var ArrayOfDevice = deviceArray[someGlobalIndex];

                    var count = 0;

                    var d = 0;

                    for (d = 0; d < ArrayOfDevice.length; d++)
                    {
                        if (ArrayOfDevice[d].device_id == device_id)
                        {
                            //count++;
                            break;
                        }
                    }
                    return ArrayOfDevice[d];
                }
            }*/
        }
    },

    SetDevice: function (sys_id, device_id, ship_id, timer_id) //IsWorking
    {
        game.IsShipPlayerControlled(ship_id);
        var playerName = game.GetShipOwner(ship_id);

        if (storage.IsSetGlobal("GlobalIndex_Player_" + playerName))
        {
            var someGlobalIndex = storage.GetGlobal("GlobalIndex_Player_" + playerName);

            /*if (deviceArray == null)
            {
                deviceArray = [];
                var ArrayOfDevice = [];
                var dataOfDevice = { isWorking: 1, ship_id: ship_id, sys_id: sys_id, device_id: device_id, timer_id: timer_id };
                ArrayOfDevice.push(dataOfDevice);
                deviceArray[someGlobalIndex] = ArrayOfDevice;
                return ArrayOfDevice[0];
            }
            else
            {
                if (deviceArray[someGlobalIndex] == null)
                {
                    var ArrayOfDevice = [];
                    var dataOfDevice = { isWorking: 1, ship_id: ship_id, sys_id: sys_id, device_id: device_id, timer_id: timer_id };
                    ArrayOfDevice.push(dataOfDevice);
                    deviceArray[someGlobalIndex] = ArrayOfDevice;
                    return ArrayOfDevice[0];
                }
                else
                {
                    var ArrayOfDevice = deviceArray[someGlobalIndex];

                    var count = 0;

                    var d = 0;

                    for (d = 0; d < ArrayOfDevice.length; d++)
                    {
                        if (ArrayOfDevice[d].device_id == device_id)
                        {
                            ArrayOfDevice[d].timer_id = timer_id;
                            break;
                        }
                    }

                    console.PrintError("ArrayOfDevice[d].timer_id: " + ArrayOfDevice[d].timer_id);

                    return ArrayOfDevice[d];
                }
            }*/
        }
    },





    AddDeviceID: function (sys_id, device_id, slot_id, ship_id, timer_id)
    {
        game.IsShipPlayerControlled(ship_id);

        var playerName = game.GetShipOwner(ship_id);

        if (storage.IsSetGlobal("GlobalIndex_Player_" + playerName)) 
        {
            var someGlobalIndex = storage.GetGlobal("GlobalIndex_Player_" + playerName);

            //storage.GetGlobal("systemid_" + sys_id + "_ship_" + ship_id + "_salvage_device_" + device_id); //, { swtch: 1, ship_id: ship_id, device_id: device_id, slot_id: slot_id, timer_id: timer_id}

            var globalSalvageStorage = storage.GetGlobal("systemid_" + sys_id + "_ship_" + ship_id + "_salvage_device_" + device_id);

            if (globalSalvageStorage != null) {

                if (globalSalvageStorage.timer_id != null)
                {
                    timer.ClearTimer(globalSalvageStorage.timer_id);
                    //console.PrintError("cleared device timer: " + globalSalvageStorage.timer_id);
                }
                else {
                    //console.PrintError("null timer1");
                }
            }
            else {
                //console.PrintError("null globalSalvageStorage");
            }

            var data = { swtch: 1, ship_id: ship_id, device_id: device_id, slot_id: slot_id, timer_id: timer_id, reset: 0 };

            storage.SetGlobal("systemid_" + sys_id + "_ship_" + ship_id + "_salvage_device_" + device_id, data ); 
            //storage.SetGlobal("systemid_" + sys_id + "_ship_" + ship_id + "_salvage_device_" + device_id + "_swtch", 1); 


            /*if (!storage.IsSetGlobal("systemid_" + sys_id + "_ship_" + ship_id + "_salvage_device_" + device_id))
            {
                storage.SetGlobal("systemid_" + sys_id + "_ship_" + ship_id + "_salvage_device_" + device_id, { swtch: 1, ship_id: ship_id, device_id: device_id, slot_id: slot_id, timer_id: timer_id }); 
            }
            else
            {
                var globalSalvageStorage = storage.GetGlobal("systemid_" + sys_id + "_ship_" + ship_id + "_salvage_device_" + device_id); 

                var count = 0;
                for (var d = 0; d < globalSalvageStorage.length; d++)
                {
                    if (globalSalvageStorage[d].device_id == device_id)
                    {
                        timer.ClearTimer(globalSalvageStorage[d].timer_id);
                        count++;
                    }
                }

                if (count != 0)
                {

                }
                else
                {

                }

                var deviceData = { swtch: 1, ship_id: ship_id, device_id: device_id, slot_id: slot_id, timer_id: timer_id };
                globalSalvageStorage.push(deviceData);

                storage.SetGlobal("systemid_" + sys_id + "_ship_" + ship_id + "_salvage_device_" + device_id, globalSalvageStorage); 
            }*/

            /*if (deviceArray == null)
            {
                deviceArray = [];
                var ArrayOfDevice = [];
                var dataOfDevice = { isWorking: isWorking, ship_id: ship_id, sys_id: sys_id, device_id: device_id, timer_id: timer_id };
                ArrayOfDevice.push(dataOfDevice);
                deviceArray[someGlobalIndex] = ArrayOfDevice;
            }
            else
            {        
                if (deviceArray[someGlobalIndex] == null)
                {
                    var ArrayOfDevice = [];
                    var dataOfDevice = { isWorking: isWorking, ship_id: ship_id, sys_id: sys_id, device_id: device_id, timer_id: timer_id };
                    ArrayOfDevice.push(dataOfDevice);
                    deviceArray[someGlobalIndex] = ArrayOfDevice;
                }
                else
                {
                    var ArrayOfDevice = deviceArray[someGlobalIndex];
                    var dataOfDevice = { isWorking: isWorking, ship_id: ship_id, sys_id: sys_id, device_id: device_id, timer_id: timer_id };
                    ArrayOfDevice.push(dataOfDevice);
                    deviceArray[someGlobalIndex] = ArrayOfDevice;


                    /*var count = 0;
                    for (var d = 0; d < ArrayOfDevice.length; d++)
                    {
                        if (ArrayOfDevice[d].device_id == device_id)
                        {
                            count++;
                        }
                    }

                    if (count != 0)
                    {

                    }
                    else
                    {
                   
                    }
                }
            }*/
        }

        /*if (storage.IsSetGlobal("GlobalIndex_Player"))
        {
            var someGlobalIndex = storage.GetGlobal("GlobalIndex_Player");
            someGlobalIndex++;
            storage.SetGlobal("GlobalIndex_Player_" + args.name, someGlobalIndex);
            storage.SetGlobal("GlobalIndex_Player", someGlobalIndex);
        }
        else {
            var someGlobalIndex = 0;
            storage.SetGlobal("GlobalIndex_Player_" + args.name, someGlobalIndex);
            storage.SetGlobal("GlobalIndex_Player", someGlobalIndex);
        }*/

        if (deviceArray == null)
        {
            deviceArray = [];
        }
        var objectData = { sys_id: sys_id, device_id: device_id, ship_id: ship_id };
        deviceArray.push(objectData);

        var arrayData = { index: deviceArray.length - 1, length: deviceArray.length };
        return arrayData;
    },



    AddCounterFrame: function ()
    {
        counterFrame++;
        return counterFrame;
    },

    GetCounterFrame: function () {
        return counterFrame;
    },

    ResetCounterFrame: function ()
    {
        counterFrame = 0;
        return counterFrame;
    },

    AddCounterDevice: function () {
         counterDevice++;
         return counterDevice;
    },

    GetCounterDevice: function () {
        return counterDevice;
    },

    ResetCounterDevice: function () {
        counterDevice = 0;
        return counterDevice;
    }

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
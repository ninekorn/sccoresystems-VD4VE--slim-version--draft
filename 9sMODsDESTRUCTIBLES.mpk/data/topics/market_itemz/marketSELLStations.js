using(relations);
using(npc);
using(console);
using(ship);
using(generator);
using(game);
using(items);
using(player);
using(storage);

var sta;
var pin;

var sbvl;
var csipt;
var spcid;

var css;
var iicvl;

function OnCheckRequirements()
{
	var npid = topic.GetCurrentNpcShipId();
    if (npc.GetTag(npid, "class") == "marketSELLStations")
	{
		return true;
	}
	return false;
}

function OnDialogue(args)
{
	var pid = PLAYER_SHIP;
    var na = game.GetShipOwner(pid);

    pin = topic.GetInput();
    sta = topic.GetState();

	var plsp = PLAYER_SHIP; 

    if (pin == NO_INPUT)
    {
		var arr;
        var sid = storage.GetGlobal("marketTerminalChosenSystem" + pid, arr);
        sbvl = game.GetSystemBases(sid);

		var i;
        for (i = 0; i < sbvl.length; i++)
        {
			var bsif = generator.GetBaseByID(sbvl[i]);
            var names = bsif.name;
            topic.AddChoice(6 + i, names);
            topic.AddChoice(3, "Disconnect");
		}
	}
    else if (pin == 3)
    {
        topic.DialogueBreak();
    }

    else if (pin >= 6 && pin < 6 + sbvl.length)
	{
        csipt = pin - 6;

		topic.AddPhrase("What shop section is of interest to you?")

		//var systemBases = sbvl[csipt];
		//css = systemBases;
		//var currentStation = storage.SetGlobal("marketTerminalChosenStation" + player_id, css);
		//spcid = items.GetStationspcid(systemBases);
		//iicvl = items.GetItemsAndCargo(spcid);

		var stbs = sbvl[csipt];
        css = stbs;
        spcid = items.GetShipStationStorageContainerId(plsp, stbs);
		iicvl = items.GetItemsAndCargo(spcid);
		storage.SetGlobal("marketTerminalChosenStation" + pid, css);

		var tlohuivl = [];
		var tloenivl = [];
		var tlofivl = [];
		var tlogeivl = [];
		var tlogrivl = [];
		var tloraivl = [];
		var tlorcivl = [];
		var tloshivl = [];
		var tloboivl = [];
		var tlofdeivl = [];
		var tlofconivl = [];
		var tlocomivl = [];
		var tlofgoivl = [];
		var tloorivl = [];
		var tloweivl = [];

		for (var i = 0; i < iicvl.length; i++)
		{
			if (iicvl[i].xml_id.slice(0, 4) == "hull") {
				tlohuivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 6) == "engine") {
				tloenivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 8) == "fueltank") {
				tlofivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 9) == "generator") {
				tlogeivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 8) == "grappler") {
				tlogrivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 5) == "radar") {
				tloraivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 3) == "rcs") {
				tlorcivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 6) == "shield") {
				tloshivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 7) == "booster") {
				tloboivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 6) == "weapon") {
				tloweivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 6) == "device") {
				tlofdeivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 10) == "consumable") {
				tlofconivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 10) == "components") {
				tlocomivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 5) == "goods") {
				tlofgoivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 3) == "ore") {
				tloorivl.push(iicvl[i]);
			}
		}

		if (tlohuivl.length > 0) {
			topic.AddChoice(50, "Hulls");
		}
		if (tloenivl.length > 0) {
			topic.AddChoice(60, "Engines");
		}
		if (tlofivl.length > 0) {
			topic.AddChoice(70, "Fueltanks");
		}
		if (tlogeivl.length > 0) {
			topic.AddChoice(80, "Generators");
		}
		if (tlogrivl.length > 0) {
			topic.AddChoice(90, "Grapplers");
		}
		if (tloraivl.length > 0) {
			topic.AddChoice(100, "Radars");
		}
		if (tlorcivl.length > 0) {
			topic.AddChoice(110, "Rcs");
		}
		if (tloshivl.length > 0) {
			topic.AddChoice(120, "Shields");
		}
		if (tloboivl.length > 0) {
			topic.AddChoice(130, "Boosters");
		}
		if (tloweivl.length > 0) {
			topic.AddChoice(140, "Weapons");
		}
		if (tlofdeivl.length > 0) {
			topic.AddChoice(150, "Devices");
		}
		if (tlofconivl.length > 0) {
			topic.AddChoice(160, "Consumables");
		}
		if (tlocomivl.length > 0) {
			topic.AddChoice(170, "Components");
		}
		if (tlofgoivl.length > 0) {
			topic.AddChoice(180, "Goods");
		}
		if (tloorivl.length > 0) {
			topic.AddChoice(190, "Ores");
        } 
        topic.AddChoice(3, "Disconnect");
		//topic.AddChoice(5000 + currentSystemInput, "Go Back!");

	}

	////////////////////////////HULLS/////////////////////////////////////////////
    else if (pin == 50) {
        player.StartDialogue(na, "marketSELLHulls");
	}

	////////////////////////////ENGINES/////////////////////////////////////////////
    else if (pin == 60) {
        player.StartDialogue(na, "marketSELLEngines");
	}

	////////////////////////////FUELTANK/////////////////////////////////////////////
    else if (pin == 70) {
        player.StartDialogue(na, "marketSELLFuel");
	}

	////////////////////////////GENERATOR/////////////////////////////////////////////
    else if (pin == 80) {
        player.StartDialogue(na, "marketSELLGenerators");
	}

	////////////////////////////GRAPPLER/////////////////////////////////////////////
    else if (pin == 90) {
        player.StartDialogue(na, "marketSELLGrapplers");
	}

	////////////////////////////RADAR/////////////////////////////////////////////
    else if (pin == 100) {
        player.StartDialogue(na, "marketSELLRadars");
	}

	////////////////////////////RCS/////////////////////////////////////////////
    else if (pin == 110) {
        player.StartDialogue(na, "marketSELLRcs");
	}

	////////////////////////////SHIELD/////////////////////////////////////////////
    else if (pin == 120) {
        player.StartDialogue(na, "marketSELLShields");
	}

	////////////////////////////BOOSTER/////////////////////////////////////////////
    else if (pin == 130) {
        player.StartDialogue(na, "marketSELLBoosters");
	}

	////////////////////////////WEAPONS/////////////////////////////////////////////
    else if (pin == 140) {
        player.StartDialogue(na, "marketSELLWeapons");
	}

	////////////////////////////DEVICE/////////////////////////////////////////////
    else if (pin == 150) {
        player.StartDialogue(na, "marketSELLDevices");
	}

	////////////////////////////CONSUMABLE/////////////////////////////////////////////
    else if (pin == 160) {
        player.StartDialogue(na, "marketSELLConsumables");
	}

	////////////////////////////COMPONENTS/////////////////////////////////////////////
    else if (pin == 170) {
        player.StartDialogue(na, "marketSELLComponents");
	}

	////////////////////////////GOODS/////////////////////////////////////////////
    else if (pin == 180) {
        player.StartDialogue(na, "marketSELLGoods");
	}

	////////////////////////////ORES/////////////////////////////////////////////
    else if (pin == 190) {
        player.StartDialogue(na, "marketSELLOres");
	}
}
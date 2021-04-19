using(npc);
using(console);
using(ship);
using(generator);
using(game);
using(items);
using(player);
using(storage);

var st;
var pin;
var sbvl;
var csi;
var scid;
var css;
var iicvl;

function OnCheckRequirements()
{
	var npc_id = topic.GetCurrentNpcShipId();
	if (npc.GetTag(npc_id, "class") == "marketBUYStations")
	{
		return true;
	}
	return false;
}

function OnDialogue()
{
    var pid = PLAYER_SHIP;
    var na = game.GetShipOwner(pid);

    pin = topic.GetInput();
	st = topic.GetState();

    if (pin == NO_INPUT) {
		var arr;
        var sid = storage.GetGlobal("marketTerminalChosenSystem" + pid, arr);
        sbvl = game.GetSystemBases(sid);

		var i;
		for (i = 0; i < sbvl.length; i++) {
			var bsinf = generator.GetBaseByID(sbvl[i]);
            var nas = bsinf.name;
            topic.AddChoice(6 + i, nas);
		}
	}

    else if (pin >= 6 && pin < 6 + sbvl.length)
	{
        csi = pin - 6;

        topic.AddPhrase("What shop section is of interest to you?");

		var sbss = sbvl[csi];
        css = sbss;
        var csst = storage.SetGlobal("marketTerminalChosenStation" + pid, css);
        scid = items.GetStationShopContainerId(sbss);
		iicvl = items.GetItemsAndCargo(scid);

		var tlofhivl = [];
		var tloeivl = [];
		var tlofivl = [];
		var tlogivl = [];
		var tlogrivl = [];
		var tlorivl = [];
		var tlorcivl = [];
		var tlofsvl = [];
		var tlobivl = [];
		var tlodwivl = [];
		var tlocivl = [];
		var tlofcoivl = [];
		var tlogoivl = [];
		var tlooivl = [];
		var tloweivl = [];

		for (var i = 0; i < iicvl.length; i++)
		{
			if (iicvl[i].xml_id.slice(0, 4) == "hull") {
				tlofhivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 6) == "engine") {
				tloeivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 8) == "fueltank") {
				tlofivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 9) == "generator") {
				tlogivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 8) == "grappler") {
				tlogrivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 5) == "radar") {
				tlorivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 3) == "rcs") {
				tlorcivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 6) == "shield") {
				tlofsvl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 7) == "booster") {
				tlobivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 6) == "weapon") {
				tloweivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 6) == "device") {
				tlodwivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 10) == "consumable") {
				tlocivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 10) == "components") {
				tlofcoivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 5) == "goods") {
				tlogoivl.push(iicvl[i]);
			}

			else if (iicvl[i].xml_id.slice(0, 3) == "ore") {
				tlooivl.push(iicvl[i]);
			}
		}

		if (tlofhivl.length > 0) {
			topic.AddChoice(50, "Hulls");
		}
		if (tloeivl.length > 0) {
			topic.AddChoice(60, "Engines");
		}
		if (tlofivl.length > 0) {
			topic.AddChoice(70, "Fueltanks");
		}
		if (tlogivl.length > 0) {
			topic.AddChoice(80, "Generators");
		}
		if (tlogrivl.length > 0) {
			topic.AddChoice(90, "Grapplers");
		}
		if (tlorivl.length > 0) {
			topic.AddChoice(100, "Radars");
		}
		if (tlorcivl.length > 0) {
			topic.AddChoice(110, "Rcs");
		}
		if (tlofsvl.length > 0) {
			topic.AddChoice(120, "Shields");
		}
		if (tlobivl.length > 0) {
			topic.AddChoice(130, "Boosters");
		}
		if (tloweivl.length > 0) {
			topic.AddChoice(140, "Weapons");
		}
		if (tlodwivl.length > 0) {
			topic.AddChoice(150, "Devices");
		}
		if (tlocivl.length > 0) {
			topic.AddChoice(160, "Consumables");
		}
		if (tlofcoivl.length > 0) {
			topic.AddChoice(170, "Components");
		}
		if (tlogoivl.length > 0) {
			topic.AddChoice(180, "Goods");
		}
		if (tlooivl.length > 0) {
			topic.AddChoice(190, "Ores");
		}
		//topic.AddChoice(5000 + currentSystemInput, "Go Back!");
		//topic.AddChoice(10, "Disconnect");
	}

	////////////////////////////HULLS/////////////////////////////////////////////
    else if (pin == 50) {
        player.StartDialogue(na, "marketBUYHulls");
	}

	////////////////////////////ENGINES/////////////////////////////////////////////
    else if (pin == 60) {
        player.StartDialogue(na, "marketBUYEngines");
	}

	////////////////////////////FUELTANK/////////////////////////////////////////////
    else if (pin == 70) {
        player.StartDialogue(na, "marketBUYFuel");
	}

	////////////////////////////GENERATOR/////////////////////////////////////////////
    else if (pin == 80) {
        player.StartDialogue(na, "marketBUYGenerators");
	}

	////////////////////////////GRAPPLER/////////////////////////////////////////////
    else if (pin == 90) {
        player.StartDialogue(na, "marketBUYGrapplers");
	}

	////////////////////////////RADAR/////////////////////////////////////////////
    else if (pin == 100) {
        player.StartDialogue(na, "marketBUYRadars");
	}

	////////////////////////////RCS/////////////////////////////////////////////
    else if (pin == 110) {
        player.StartDialogue(na, "marketBUYRcs");
	}

	////////////////////////////SHIELD/////////////////////////////////////////////
    else if (pin == 120) {
        player.StartDialogue(na, "marketBUYShields");
	}

	////////////////////////////BOOSTER/////////////////////////////////////////////
    else if (pin == 130) {
        player.StartDialogue(na, "marketBUYBoosters");
	}

	////////////////////////////WEAPONS/////////////////////////////////////////////
    else if (pin == 140) {
        player.StartDialogue(na, "marketBUYWeapons");
	}

	////////////////////////////DEVICE/////////////////////////////////////////////
    else if (pin == 150) {
        player.StartDialogue(na, "marketBUYDevices");
	}

	////////////////////////////CONSUMABLE/////////////////////////////////////////////
    else if (pin == 160) {
        player.StartDialogue(na, "marketBUYConsumables");
	}

	////////////////////////////COMPONENTS/////////////////////////////////////////////
    else if (pin == 170) {
        player.StartDialogue(na, "marketBUYComponents");
	}

	////////////////////////////GOODS/////////////////////////////////////////////
    else if (pin == 180) {
        player.StartDialogue(na, "marketBUYGoods");
	}

	////////////////////////////ORES/////////////////////////////////////////////
    else if (pin == 190) {
        player.StartDialogue(na, "marketBUYOres");
	}
}
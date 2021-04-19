using(relations);
using(npc);
using(console);
using(ship);
using(generator);
using(game);
using(items);
using(player);
using(storage);
using(station);

var sa;
var pin;
var sc;
var tnodwivl = 0;
var lodwivl = [];
var css;
var sbs;

function OnCheckRequirements() {
	var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketBUYDevices") {
		return true;
	}
	return false;
}

function OnDialogue(args) {

    var pid = PLAYER_SHIP;
    var na = game.GetShipOwner(pid);

    pin = topic.GetInput();
	sa = topic.GetState();

    var pshp = PLAYER_SHIP;

    if (pin == NO_INPUT)
	{
		var arr1;
        sbs = storage.GetGlobal("marketTerminalChosenStation" + pid, arr1);
		css = sbs;

        sc = items.GetStationShopContainerId(sbs);

		var iicg = items.GetItemsAndCargo(sc);
		lodwivl = [];
		var i;
        for (i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 6) == "device") {
                lodwivl.push(iicg[i]);
			}
		}

		topic.AddChoice(150000, "Devices");
		topic.AddChoice(10, "Disconnect");
	}

    else if (pin == 10) {
		topic.DialogueBreak();
	}

    else if (pin == 11) {
        player.StartDialogue(na, "marketBUYStations");
	}
	////////////////////////////DEVICE/////////////////////////////////////////////

    else if (pin == 150000) {

        var iicg = items.GetItemsAndCargo(sc);
		lodwivl = [];
		var i;
        for (i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 6) == "device") {
                lodwivl.push(iicg[i]);
			}
		}

		var ii;
		for (ii = 0; ii < lodwivl.length; ii++) {

            var pmd = CalculateBuyPriceOfItemModifier(pid);
			var obj = generator.GetItemByXmlID(lodwivl[ii].xml_id);
            var spc = Math.ceil(obj.price * pmd);

            var cstg = lodwivl[ii].xml_id.replace(/_/g, " ");
            topic.AddChoice(1500000 + ii, cstg.toUpperCase() + " " + "Price: " + "$" + spc + ".");
        }
		tnodwivl =ii;
		topic.AddChoice(11, "Go back to list of stations");
		topic.AddChoice(10, "Disconnect");
	}



	////////////////////////////DEVICES INDIVIDUAL SECTION/////////////////////////////////////////////
    else if (pin >= 1500000 && pin < 1500000 + tnodwivl)
	{			
        var iind = pin - 1500000;
	
		var eid;
		var exmid;
	
        eid = lodwivl[iind];		
        exmid = lodwivl[iind].xml_id;	
			
        var pmd = CalculateBuyPriceOfItemModifier(pid);
        var obj = generator.GetItemByXmlID(exmid);
        var spc = Math.ceil(obj.price * pmd);

        var mon = player.GetMoney(na);

        if (mon < spc)
        {
            var csit = exmid.replace(/_/g, " ");

            topic.AddPhrase("You do not have enough money to buy the " + csit.toUpperCase() + " .Price: " + "$" + spc + " Money: " + "$" + mon);
            topic.AddChoice(150000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
        else {
            var csit = exmid.replace(/_/g, " ");
            topic.AddPhrase("" + csit.toUpperCase() + "" + " Buy price:" + "$" + spc + ".");
            topic.AddChoice(15000000 + iind, "Buy this item?");
            topic.AddChoice(150000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
	}

    else if (pin >= 15000000 && pin < 15000000 +lodwivl.length)
	{
        var iind = pin - 15000000;
	
        var eid;
        var exmid;
		
        eid = lodwivl[iind];		
        exmid = lodwivl[iind].xml_id;	

        var spsid = items.GetShipStationStorageContainerId(pshp,css);

        var obj = generator.AddItemToSpecifiedContainer(spsid, exmid, 1); 	
		
        items.RemoveItemQuantity(sc, eid.item_id, 1); 

        var itr = lodwivl.indexOf(eid);
        lodwivl.splice(itr,1);

        var pmd = CalculateBuyPriceOfItemModifier(pid);
        var obj = generator.GetItemByXmlID(exmid);
        var spc = Math.ceil(obj.price * pmd);

        player.RemoveMoney(na, spc);

        var csit = exmid.replace(/_/g, " ");
        topic.AddPhrase("You have bought " + csit.toUpperCase() + " for " + "$" + spc + ".");
        topic.AddChoice(150000, "Go Back!");
        topic.AddChoice(10, "Disconnect");
    }


}
function CalculateBuyPriceOfItemModifier(args) {
    var mi = 1;
    var ma = 1.25;
    var tmg = -ship.GetFinalCacheValue(args, "trade_margin");
    var sinf = station.GetBaseByID(sbs);

    var fct = relations.GetFactionDispositionToShip(sinf.faction, args);
    var rcoe = 0.1 * GetFactionRelationCoef(fct);
    var coef = ma - tmg - rcoe;
    coef = Clamp(mi, coef, ma);
    return coef;
}

function GetFactionRelationCoef(re) {
    re = Clamp(-150.0, re, 150.0);
    return re / 150.0;
}

function Clamp(mi, cur, ma) {
    if (cur < mi) {
        return mi;
    }
    if (cur > ma) {
        return ma;
    }
    return cur;
}



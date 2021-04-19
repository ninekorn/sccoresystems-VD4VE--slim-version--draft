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
var scid;
var tnohvl = 0;
var lohivl = [];
var css;
var sbs;

function OnCheckRequirements()
{
	var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketBUYHulls") {
		return true;
	}
	return false;
}

function OnDialogue(args)
{
	var pid = PLAYER_SHIP;
	//var shipID = args.ship_id;
    var na = game.GetShipOwner(pid);

    pin = topic.GetInput();
	sa = topic.GetState();
	var pshp = PLAYER_SHIP; 


    if (pin == NO_INPUT)
	{
		var arr1;
        sbs = storage.GetGlobal("marketTerminalChosenStation" + pid, arr1);
		css = sbs;

        scid = items.GetStationShopContainerId(sbs);

		var iicg = items.GetItemsAndCargo(scid);
		lohivl = [];
		var i;
        for (i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 4) == "hull") {
                lohivl.push(iicg[i]);
			}
		}

		topic.AddChoice(50000, "Hulls");
		topic.AddChoice(10, "Disconnect");
	}

    else if (pin == 10) {
		topic.DialogueBreak();
	}


    else if (pin == 11) {
        player.StartDialogue(na, "marketBUYStations");
	}

	////////////////////////////HULLS/////////////////////////////////////////////
    else if (pin == 50000)
	{

        var iicg = items.GetItemsAndCargo(scid);
		lohivl = [];
		var i;
        for (i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 4) == "hull") {
                lohivl.push(iicg[i]);
			}
		}

		var ii;
		for (ii = 0; ii < lohivl.length; ii++)
		{

            var pmd = CalculateBuyPriceOfItemModifier(pid);
			var obj = generator.GetItemByXmlID(lohivl[ii].xml_id);
            var spce = Math.ceil(obj.price * pmd);

            var currentString = lohivl[ii].xml_id.replace(/_/g, " ");
            topic.AddChoice(500000 + ii, currentString.toUpperCase() + " " + "Price: " + "$" + spce + ".");

			//topic.AddChoice(500000 + ii, lohivl[ii].xml_id);
		}

		tnohvl = ii;
		topic.AddChoice(11, "Go back to list of stations");
		topic.AddChoice(10, "Disconnect");
	}

	////////////////////////////HULLS INDIVIDUAL SECTION/////////////////////////////////////////////
    else if (pin >= 500000 && pin < 500000 + tnohvl)
	{			
        var iidx = pin - 500000;

        var eid;
        var exmid;
		
        eid = lohivl[iidx];		
        exmid = lohivl[iidx].xml_id;	
	
        var pmd = CalculateBuyPriceOfItemModifier(pid);
        var obj = generator.GetItemByXmlID(exmid);
        var spce = Math.ceil(obj.price * pmd);     

        var mon = player.GetMoney(na);

        if (mon < spce) {
            var csit = exmid.replace(/_/g, " ");

            topic.AddPhrase("You do not have enough money to buy the " + csit.toUpperCase() + " .Price: " + "$" + spce + " Money: " + "$" + mon);
            topic.AddChoice(50000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
        else {
            var csit = exmid.replace(/_/g, " ");
            topic.AddPhrase("" + csit.toUpperCase() + "" + " Buy price:" + "$" + spce + ".");
            topic.AddChoice(5000000 + iidx, "Buy this item?");
            topic.AddChoice(50000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
	}

    else if (pin >= 5000000 && pin < 5000000 +lohivl.length)
	{
        var iidx = pin - 5000000;
	
        var eid;
        var exmid;
		
        eid = lohivl[iidx];		
        exmid = lohivl[iidx].xml_id;	

        var spsid = items.GetShipStationStorageContainerId(pshp,css);

        var obj = generator.AddItemToSpecifiedContainer(spsid, exmid, 1);		

        items.RemoveItemQuantity(scid, eid.item_id, 1); 

        var pmd = CalculateBuyPriceOfItemModifier(pid);
        var obj = generator.GetItemByXmlID(exmid);
        var spce = Math.ceil(obj.price * pmd);

        player.RemoveMoney(na, spce);

        var csit = exmid.replace(/_/g, " ");
        topic.AddPhrase("You have bought " + csit.toUpperCase() + " for " + "$" + spce + ".");
		topic.AddChoice(50000, "Go Back!");
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
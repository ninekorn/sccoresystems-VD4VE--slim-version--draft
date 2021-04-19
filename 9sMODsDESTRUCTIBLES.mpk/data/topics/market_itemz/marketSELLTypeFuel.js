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

var sta;
var pin;
var spcid;
var tnpffuivl = 0;
var lofuivl = [];
var css;
var sbs;

function OnCheckRequirements() {
    var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketSELLFuel") {
        return true;
    }
    return false;
}

function OnDialogue(args) {
    var psph = PLAYER_SHIP;
    var na = game.GetShipOwner(psph);

    pin = topic.GetInput();
    sta = topic.GetState();


    if (pin == NO_INPUT) {

        var arr1;
        sbs = storage.GetGlobal("marketTerminalChosenStation" + psph, arr1);
        css = sbs;

        spcid = items.GetShipStationStorageContainerId(psph, css);

        var iicg = items.GetItemsAndCargo(spcid);
        lofuivl = [];
        var i;
        for (i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 4) == "fuel") {
                lofuivl.push(iicg[i]);
            }
        }

        topic.AddChoice(130000, "Fuel");
        topic.AddChoice(10, "Disconnect");
    }

    else if (pin == 10) {
        topic.DialogueBreak();
    }

    else if (pin == 11) {
        player.StartDialogue(na, "marketSELLStations");
    }


    ////////////////////////////BOOSTER/////////////////////////////////////////////
    else if (pin == 130000) {

        var iicg = items.GetItemsAndCargo(spcid);
        lofuivl = [];
        var i;
        for (i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 4) == "fuel") {
                lofuivl.push(iicg[i]);
            }
        }

        var ii;
        for (ii = 0; ii < lofuivl.length; ii++) {

            var pmd = CalculateSellPriceOfItemModifier(psph);
            var obj = generator.GetItemByXmlID(lofuivl[ii].xml_id);
            var spce = Math.ceil(obj.price * pmd);

            var cstg = lofuivl[ii].xml_id.replace(/_/g, " ");
            topic.AddChoice(1300000 + ii, cstg.toUpperCase() + " " + "Price: " + spce + "$");
        }

        tnpffuivl = ii;
        topic.AddChoice(11, "Go back to list of stations");
        topic.AddChoice(10, "Disconnect");
    }

    ////////////////////////////BOOSTER INDIVIDUAL SECTION/////////////////////////////////////////////

    else if (pin >= 1300000 && pin < 1300000 + tnpffuivl) {

        var iidx = pin - 1300000;

        var eid;
        var exmid;

        eid = lofuivl[iidx];
        exmid = lofuivl[iidx].xml_id;

        var pmd = CalculateSellPriceOfItemModifier(psph);
        var obj = generator.GetItemByXmlID(exmid);
        var sellingPrice = Math.ceil(obj.price * pmd);

        var cstg = exmid.replace(/_/g, " ");

        topic.AddPhrase("" + cstg.toUpperCase() + "" + " Sell price:" + sellingPrice + "$.");
        topic.AddChoice(13000000 + iidx, "Sell this item?");
        topic.AddChoice(130000, "Go Back!");
        topic.AddChoice(10, "Disconnect");
    }
    else if (pin >= 13000000 && pin < 13000000 + lofuivl.length) {
        var iidx = pin - 13000000;

        var eid;
        var exmid;

        eid = lofuivl[iidx];
        exmid = lofuivl[iidx].xml_id;

        var shpce0 = items.GetStationShopContainerId(css);

        var obj = generator.AddItemToSpecifiedContainer(shpce0, exmid, 1);

        var spstid = items.GetShipStationStorageContainerId(psph, css);
        items.RemoveItemQuantity(spstid, eid.item_id, 1);

        var pmd = CalculateSellPriceOfItemModifier(psph);
        var obj = generator.GetItemByXmlID(exmid);
        var slpce = Math.ceil(obj.price * pmd);


        var na = game.GetShipOwner(psph);
        player.AddMoney(na, slpce);

        var cstg = exmid.replace(/_/g, " ");
        topic.AddPhrase("You have sold " + cstg.toUpperCase() + " for " + slpce + "$.");
        topic.AddChoice(130000, "Go Back!");
        topic.AddChoice(10, "Disconnect");
    }
}



function CalculateSellPriceOfItemModifier(args) {
    var mi = 0.4;
    var ma = 0.9;
    var stin = station.GetBaseByID(sbs);
    var fact = relations.GetFactionDispositionToShip(stin.faction, args);

    var tdmg = -ship.GetFinalCacheValue(args, "trade_margin");

    var relc = 0.2 * GetFactionRelationCoef(fact);
    var coef = mi + tdmg + relc;
    coef = Clamp(mi, coef, ma);
    return coef;
}

function GetFactionRelationCoef(re) {
    re = Clamp(-150.0, re, 150.0);
    return re / 150.0;
}

function Clamp(mi, cu, ma) {
    if (cu < mi) {
        return mi;
    }
    if (cu > ma) {
        return ma;
    }
    return cu;
}
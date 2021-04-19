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
var tnofenivl = 0;
var loeivl = [];
var css;
var sbs;

function OnCheckRequirements() {
    var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketSELLEngines") {
        return true;
    }
    return false;
}

function OnDialogue(args) {
    var pshp = PLAYER_SHIP;
    var na = game.GetShipOwner(pshp);

    pin = topic.GetInput();
    sta = topic.GetState();


    if (pin == NO_INPUT) {

        var arr1;
        sbs = storage.GetGlobal("marketTerminalChosenStation" + pshp, arr1);
        css = sbs;

        spcid = items.GetShipStationStorageContainerId(pshp, css);

        var iicg = items.GetItemsAndCargo(spcid);
        loeivl = [];
        var i;
        for (i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 6) == "engine") {
                loeivl.push(iicg[i]);
            }
        }

        topic.AddChoice(130000, "Engines");
        topic.AddChoice(10, "Disconnect");
    }

    else if (pin == 10) {
        topic.DialogueBreak();
    }

    else if (pin == 11) {
        player.StartDialogue(na, "marketSELLStations");
    }


    ////////////////////////////ENGINES/////////////////////////////////////////////
    else if (pin == 130000) {

        var iicg = items.GetItemsAndCargo(spcid);
        loeivl = [];
        var i;
        for (i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 6) == "engine") {
                loeivl.push(iicg[i]);
            }
        }

        var ii;
        for (ii = 0; ii < loeivl.length; ii++) {

            var pmd = CalculateSellPriceOfItemModifier(pshp);
            var obj = generator.GetItemByXmlID(loeivl[ii].xml_id);
            var spce = Math.ceil(obj.price * pmd);

            var cstrg = loeivl[ii].xml_id.replace(/_/g, " ");
            topic.AddChoice(1300000 + ii, cstrg.toUpperCase() + " " + "Price: " + spce + "$");
        }

        tnofenivl = ii;
        topic.AddChoice(11, "Go back to list of stations");
        topic.AddChoice(10, "Disconnect");
    }

    ////////////////////////////ENGINES INDIVIDUAL SECTION/////////////////////////////////////////////

    else if (pin >= 1300000 && pin < 1300000 + tnofenivl) {

        var iidx = pin - 1300000;

        var eid;
        var exmid;

        eid = loeivl[iidx];
        exmid = loeivl[iidx].xml_id;

        var pmd = CalculateSellPriceOfItemModifier(pshp);
        var obj = generator.GetItemByXmlID(exmid);
        var slpce = Math.ceil(obj.price * pmd);

        var cstrg = exmid.replace(/_/g, " ");

        topic.AddPhrase("" + cstrg.toUpperCase() + "" + " Sell price:" + slpce + "$.");
        topic.AddChoice(13000000 + iidx, "Sell this item?");
        topic.AddChoice(130000, "Go Back!");
        topic.AddChoice(10, "Disconnect");
    }
    else if (pin >= 13000000 && pin < 13000000 + loeivl.length) {
        var iidx = pin - 13000000;

        var eid;
        var exmid;

        eid = loeivl[iidx];
        exmid = loeivl[iidx].xml_id;

        var spcid0 = items.GetStationShopContainerId(css);

        var obj = generator.AddItemToSpecifiedContainer(spcid0, exmid, 1);

        var spsid = items.GetShipStationStorageContainerId(pshp, css);
        items.RemoveItemQuantity(spsid, eid.item_id, 1);

        var pmd = CalculateSellPriceOfItemModifier(pshp);
        var obj = generator.GetItemByXmlID(exmid);
        var slpce = Math.ceil(obj.price * pmd);


        var na = game.GetShipOwner(pshp);
        player.AddMoney(na, slpce);

        var cstrg = exmid.replace(/_/g, " ");
        topic.AddPhrase("You have sold " + cstrg.toUpperCase() + " for " + slpce + "$.");
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
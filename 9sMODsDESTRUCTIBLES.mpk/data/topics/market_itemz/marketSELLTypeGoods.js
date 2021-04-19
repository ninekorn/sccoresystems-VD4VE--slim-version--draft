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
var ccidx;
var scid;
var tnogoivl = 0;
var logoivl = [];
var css;
var cunpt;
var stonb = "0";
var sbs;
var cuidx;
var quattb = 0;
var tpce = 0;
var cuipce = 0;
var cusit = "";
var eid;
var exmid;

function OnCheckRequirements() {
    var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketSELLGoods") {
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
        stonb = "0";

        var arr1;
        sbs = storage.GetGlobal("marketTerminalChosenStation" + pshp, arr1);
        css = sbs;

        scid = items.GetShipStationStorageContainerId(pshp, css);

        var iicg = items.GetItemsAndCargo(scid);
        logoivl = [];
        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 4) == "good") {
                logoivl.push(iicg[i]);
            }
        }

        topic.AddChoice(190000, "Goods");
        topic.AddChoice(10, "Disconnect");
    }

    else if (pin == 10) {
        topic.DialogueBreak();
    }
    else if (pin == 11) {
        player.StartDialogue(na, "marketSELLStations");
    }
    ////////////////////////////GOODS/////////////////////////////////////////////
    else if (pin == 190000) {
        stonb = "0";
        var quat;

        var iicg = items.GetItemsAndCargo(scid);
        logoivl = [];

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 4) == "good") {
                logoivl.push(iicg[i]);
            }
        }

        var arpce = [];
        var gspcev = storage.GetGlobal("globalShopPrice", arpce);

        var i;
        for (i = 0; i < logoivl.length; i++) {
            quat = logoivl[i].quantity;

            var pce;
            for (var j = 0; j < gspcev.length; j++) {
                if (gspcev[j].base_id == sbs && gspcev[j].resourceID == logoivl[i].xml_id) {
                    var priceModifier = CalculateSellPriceOfCargo(pshp);
                    pce = Math.ceil(gspcev[j].price * priceModifier);
                }
            }

            var spce = pce;
            var custg = logoivl[i].xml_id.replace(/_/g, " ");
            topic.AddChoice(1900000 + i, custg.toUpperCase() + " " + "Quantity:" + " " + quat + " Price per unit: " + " $" + spce + ".");
        }

        tnogoivl = i;
        topic.AddChoice(11, "Go back to list of stations");
        topic.AddChoice(10, "Disconnect");
    }

    ////////////////////////////GOODS INDIVIDUAL SECTION/////////////////////////////////////////////
    else if (pin >= 1900000 && pin < 1900000 + tnogoivl) {
        cunpt = 1900000;
        ccidx = pin - 1900000;

        var quat = 0;

        eid = logoivl[ccidx];
        exmid = logoivl[ccidx].xml_id;

        cusit = exmid.replace(/_/g, " ");

        var spsid = items.GetShipStationStorageContainerId(pshp, css);

        var iicg = items.GetItemsAndCargo(spsid);

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id == exmid) {
                quat = iicg[i].quantity;
                break;
            }
        }

        var arpce = [];
        var gspcev = storage.GetGlobal("globalShopPrice", arpce);

        var pce;
        for (var j = 0; j < gspcev.length; j++) {
            if (gspcev[j].base_id == sbs && gspcev[j].resourceID == exmid) {

                var pmd = CalculateSellPriceOfCargo(pshp);
                pce = Math.ceil(gspcev[j].price * pmd);
            }
        }
        cuipce = pce;
        var quatts = parseInt(stonb);

        if (quatts <= quat) {
            topic.AddPhrase("Choose a quantity you wish to sell!");
            topic.AddChoice(1000000000 + ccidx, "Add digit 0 to quantity");
            topic.AddChoice(1100000000 + ccidx, "Add digit 1 to quantity");
            topic.AddChoice(1200000000 + ccidx, "Add digit 2 to quantity");
            topic.AddChoice(1300000000 + ccidx, "Add digit 3 to quantity");
            topic.AddChoice(1400000000 + ccidx, "Add digit 4 to quantity");
            topic.AddChoice(1500000000 + ccidx, "Add digit 5 to quantity");
            topic.AddChoice(1600000000 + ccidx, "Add digit 6 to quantity");
            topic.AddChoice(1700000000 + ccidx, "Add digit 7 to quantity");
            topic.AddChoice(1800000000 + ccidx, "Add digit 8 to quantity");
            topic.AddChoice(1900000000 + ccidx, "Add digit 9 to quantity");

            topic.AddChoice(19000000 + ccidx, "Finished setting quantity! Sell " + quatts + " " + cusit.toUpperCase() + " for " + "$" + spce + ".");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
        else {
            topic.AddPhrase("You don't have that much on you!");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
    }

    else if (pin >= 19000000 && pin < 19000000 + logoivl.length) {
        var quatts = parseInt(stonb);

        //var iidx = pin - 19000000;

        var quat = 0;

        var iicg = items.GetItemsAndCargo(scid);

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id == exmid) {
                quat = iicg[i].quantity;
                break;
            }
        }

        if (quatts > quat) {
            stonb = "";
            ccidx = 0;
            cunpt = 0;
            topic.AddPhrase("You're trying to sell more than you own! Do you know how to count? Start over.");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
        else {
            var scid0 = items.GetStationShopContainerId(css);

            var obj = generator.AddItemToSpecifiedContainer(scid0, exmid, quatts);

            var spsid = items.GetShipStationStorageContainerId(pshp, css);

            items.RemoveCargo(spsid, exmid, quatts);

            var spce = cuipce * quatts;

            var na = game.GetShipOwner(pshp);
            player.AddMoney(na, spce);
            stonb = "";
            ccidx = 0;
            topic.AddPhrase("You have sold " + quatts + " " + cusit.toUpperCase() + " for " + "$" + spce + ".");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
    }


    else if (pin == 1000000000 + ccidx) {
        cuidx = pin - 1000000000;
        stonb += "0";

        quattb = parseInt(stonb);
        tpce = quattb * cuipce;

        topic.AddChoice((cunpt * 10) + cuidx, "Finished setting quantity! Sell " + stonb + " " + cusit.toUpperCase() + " for " + "$" + tpce + ".");
        topic.AddChoice(cunpt + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1100000000 + ccidx) {
        cuidx = pin - 1100000000;
        stonb += "1";

        quattb = parseInt(stonb);
        tpce = quattb * cuipce;
        topic.AddChoice((cunpt * 10) + cuidx, "Finished setting quantity! Sell " + stonb + " " + cusit.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cunpt + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1200000000 + ccidx) {
        cuidx = pin - 1200000000;
        stonb += "2";

        quattb = parseInt(stonb);
        tpce = quattb * cuipce;
        topic.AddChoice((cunpt * 10) + cuidx, "Finished setting quantity! Sell " + stonb + " " + cusit.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cunpt + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1300000000 + ccidx) {
        cuidx = pin - 1300000000;
        stonb += "3";

        quattb = parseInt(stonb);
        tpce = quattb * cuipce;
        topic.AddChoice((cunpt * 10) + cuidx, "Finished setting quantity! Sell " + stonb + " " + cusit.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cunpt + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1400000000 + ccidx) {
        cuidx = pin - 1400000000;
        stonb += "4";

        quattb = parseInt(stonb);
        tpce = quattb * cuipce;
        topic.AddChoice((cunpt * 10) + cuidx, "Finished setting quantity! Sell " + stonb + " " + cusit.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cunpt + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1500000000 + ccidx) {
        cuidx = pin - 1500000000;
        stonb += "5";

        quattb = parseInt(stonb);
        tpce = quattb * cuipce;
        topic.AddChoice((cunpt * 10) + cuidx, "Finished setting quantity! Sell " + stonb + " " + cusit.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cunpt + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1600000000 + ccidx) {
        cuidx = pin - 1600000000;
        stonb += "6";

        quattb = parseInt(stonb);
        tpce = quattb * cuipce;
        topic.AddChoice((cunpt * 10) + cuidx, "Finished setting quantity! Sell " + stonb + " " + cusit.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cunpt + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1700000000 + ccidx) {
        cuidx = pin - 1700000000;
        stonb += "7";

        quattb = parseInt(stonb);
        tpce = quattb * cuipce;
        topic.AddChoice((cunpt * 10) + cuidx, "Finished setting quantity! Sell " + stonb + " " + cusit.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cunpt + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1800000000 + ccidx) {
        cuidx = pin - 1800000000;
        stonb += "8";

        quattb = parseInt(stonb);
        tpce = quattb * cuipce;
        topic.AddChoice((cunpt * 10) + cuidx, "Finished setting quantity! Sell " + stonb + " " + cusit.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cunpt + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1900000000 + ccidx) {
        cuidx = pin - 1900000000;
        stonb += "9";

        quattb = parseInt(stonb);
        tpce = quattb * cuipce;
        topic.AddChoice((cunpt * 10) + cuidx, "Finished setting quantity! Sell " + stonb + " " + cusit.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cunpt + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
}

function CalculateSellPriceOfCargo(args) {
    var mi = 0.95;
    var ma = 0.99;
    var trmg = -ship.GetFinalCacheValue(args, "trade_margin");

    var stif = station.GetBaseByID(sbs);
    var fact = relations.GetFactionDispositionToShip(stif.faction, args);
    var relco = 0.2 * GetFactionRelationCoef(fact);

    var coef = mi + (trmg + relco) * 0.04;
    coef = Clamp(mi, coef, ma);
    //console.PrintError("CARGO SellPriceCoef: " + coef + " based on tradeMargin=" + trade_margin + " relation_coef=" + relation_coef);
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

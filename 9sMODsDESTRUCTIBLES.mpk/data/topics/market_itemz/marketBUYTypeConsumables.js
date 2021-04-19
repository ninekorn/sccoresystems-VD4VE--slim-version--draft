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
var cci;
var sci;
var tnocivl = 0;
var locivl = [];
var css;
var cinp;
var son = "";
var sbs;
var cip = 0;

var eid;
var exmid;

var csi;
var qtb = 0;
var tp = 0;
var cind = -1;
var sp = 0;

function OnCheckRequirements() {
    var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketBUYConsumables") {
        return true;
    }
    return false;
}

function OnDialogue(args)
{
    var pid = PLAYER_SHIP;
    var name = game.GetShipOwner(pid);

    pin = topic.GetInput();
    sa = topic.GetState();

    if (pin == NO_INPUT)
    {
        son = "0";
        cip = 0;

        var arr;
        storage.GetGlobal("marketTerminalChosenSystem" + pid, arr);

        var arr1;
        sbs = storage.GetGlobal("marketTerminalChosenStation" + pid, arr1);
        css = sbs;

        sci = items.GetStationShopContainerId(sbs);

        var iicg = items.GetItemsAndCargo(sci);
        locivl = [];

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 10) == "consumable") {
                locivl.push(iicg[i]);
            }
        }

        topic.AddChoice(190000, "Consumables");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1) {
        topic.DialogueBreak();
    }
    else if (pin == 2) {
        player.StartDialogue(name, "marketBUYStations");
    }

    ////////////////////////////CONSUMABLES/////////////////////////////////////////////
    else if (pin == 190000) {
        son = "0";
        cip = 0;
        var quat;

        var iicg = items.GetItemsAndCargo(sci);
        locivl = [];

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 10) == "consumable") {
                locivl.push(iicg[i]);
            }
        }

        var i;
        for (i = 0; i < locivl.length; i++) {
            quat = locivl[i].quantity;
           
            var priceModifier = CalculateBuyPriceOfItemModifier(pid);
            var obj = generator.GetItemByXmlID(locivl[i].xml_id);
            var price = Math.ceil(obj.price * priceModifier);

            var currentString = locivl[i].xml_id.replace(/_/g, " ");
            topic.AddChoice(1900000 + i, currentString.toUpperCase() + " " + "Quantity:" + " " + quat + " Price: " + price + "$");
        }

        tnocivl = i;
        topic.AddChoice(2, "Go back to list of stations");
        topic.AddChoice(1, "Disconnect");
    }

    ////////////////////////////CONSUMABLES INDIVIDUAL SECTION/////////////////////////////////////////////
    else if (pin >= 1900000 && pin < 1900000 + tnocivl) {
        cinp = 1900000;
        cci = pin - 1900000;

        var quat = 0;

        eid = locivl[cci];
        exmid = locivl[cci].xml_id;

        csi = exmid.replace(/_/g, " ");

        var iicg = items.GetItemsAndCargo(sci);

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id == exmid) {
                quat = iicg[i].quantity;
                break;
            }
        }

        var pmd = CalculateBuyPriceOfItemModifier(pid);
        var obj = generator.GetItemByXmlID(locivl[cci].xml_id);
        var pri = Math.ceil(obj.price * pmd);

        cip = pri;
        qtb = parseInt(son);

        sp = pri * qtb;

        var mon = player.GetMoney(name);

        if (mon < sp) {
            topic.AddPhrase("You do not have enough money to buy the " + csi.toUpperCase() + " . Price: " + "$" + sp + " Money: " + "$" + mon + ".");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(1, "Disconnect");
        }
        else
        {
            topic.AddPhrase("Choose a quantity you wish to buy!");

            topic.AddChoice(1000000000 + cci, "Add digit 0 to quantity");
            topic.AddChoice(1100000000 + cci, "Add digit 1 to quantity");
            topic.AddChoice(1200000000 + cci, "Add digit 2 to quantity");
            topic.AddChoice(1300000000 + cci, "Add digit 3 to quantity");
            topic.AddChoice(1400000000 + cci, "Add digit 4 to quantity");
            topic.AddChoice(1500000000 + cci, "Add digit 5 to quantity");
            topic.AddChoice(1600000000 + cci, "Add digit 6 to quantity");
            topic.AddChoice(1700000000 + cci, "Add digit 7 to quantity");
            topic.AddChoice(1800000000 + cci, "Add digit 8 to quantity");
            topic.AddChoice(1900000000 + cci, "Add digit 9 to quantity");

            topic.AddChoice(19000000 + cci, "Finished setting quantity? Buy " + qtb + " " + csi.toUpperCase() + " for " + "$" + sp + ".");

            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(1, "Disconnect");
        }
    }

    else if (pin >= 19000000 && pin < 19000000 + locivl.length) {
        qtb = parseInt(son);
        sp = cip * qtb;

        var mon = player.GetMoney(name);

        if (qtb == 0) {
            topic.AddPhrase("Please select a quantity.")
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(1, "Disconnect");
        }
        else {
            if (mon < sp) {
                topic.AddPhrase("You do not have enough money to buy the " + csi.toUpperCase() + " . Price: " + "$" + sp + " Money: " + "$" + mon + ".");
                topic.AddChoice(190000, "Go Back!");
                topic.AddChoice(1, "Disconnect");
            }
            else {
                var itemIndex = pin - 19000000;
                var quat = 0;

                //eid = locivl[itemIndex];
                //exmid = locivl[itemIndex].xml_id;

                var iicg = items.GetItemsAndCargo(sci);
                for (var i = 0; i < iicg.length; i++)
                {
                    if (iicg[i].xml_id == exmid)
                    {
                        quat = iicg[i].quantity;
                        break;
                    }
                }

                if (qtb > quat)
                {
                    son = "0";
                    cci = 0;

                    cinp = 0;
                    topic.AddPhrase("Youre trying to buy more than the shop owns! Do you know how to count? Start over.");
                    topic.AddChoice(190000, "Go Back!");
                    topic.AddChoice(1, "Disconnect");
                }
                else
                {
                    var spsid = items.GetShipStationStorageContainerId(pid, css);

                    var iicgop = items.GetItemsAndCargo(spsid);
                    var qbt = 0;
                    for (var i = 0; i < iicgop.length; i++) {
                        if (iicgop[i].xml_id == exmid) {
                            qbt = iicgop[i].quantity;
                        }
                    }

                    var obj = generator.AddItemToSpecifiedContainer(spsid, exmid, qtb);

                    var iicgop = items.GetItemsAndCargo(spsid);
                    var qat = 0;
                    for (var i = 0; i < iicgop.length; i++) {
                        if (iicgop[i].xml_id == exmid) {
                            qat = iicgop[i].quantity;
                        }
                    }

                    var qtwt = 0;

                    if (qat > qbt)
                    {
                        qtwt = qat - qbt;

                        items.RemoveItemQuantity(sci, eid.item_id, qtwt); 

                        sp = cip * qtwt;

                        var na = game.GetShipOwner(pid);
                        player.RemoveMoney(na, sp);
                        son = "0";

                        cci = 0;

                        topic.AddPhrase("You have bought " + qtwt + " " + csi.toUpperCase() + " for " + "$" + sp + ".");
                        topic.AddChoice(190000, "Go Back!");
                        topic.AddChoice(1, "Disconnect");
                    }
                    else
                    {
                        son = "0";
                        topic.AddPhrase("There is no more space in your Cargo. Make some space!");
                        topic.AddChoice(190000, "Go Back!");
                        topic.AddChoice(1, "Disconnect");
                    }
                }
            }
        }
    }
    else if (pin == 1000000000 + cci) {
        cind = pin - 1000000000;
        son += "0";

        qtb = parseInt(son);
        tp = qtb * cip;

        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");
        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1100000000 + cci) {
        cind = pin - 1100000000;
        son += "1";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1200000000 + cci) {
        cind = pin - 1200000000;
        son += "2";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1300000000 + cci) {
        cind = pin - 1300000000;
        son += "3";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1400000000 + cci) {
        cind = pin - 1400000000;
        son += "4";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1500000000 + cci) {
        cind = pin - 1500000000;
        son += "5";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1600000000 + cci) {
        cind = pin - 1600000000;
        son += "6";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1700000000 + cci) {
        cind = pin - 1700000000;
        son += "7";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1800000000 + cci) {
        cind = pin - 1800000000;
        son += "8";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1900000000 + cci) {
        cind = pin - 1900000000;
        son += "9";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
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

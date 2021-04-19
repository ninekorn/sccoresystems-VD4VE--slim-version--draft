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
var scid;
var tnogoivl = 0;
var logivl = [];
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
var cid = -1;

function OnCheckRequirements()
{
    var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketBUYGoods") {
        return true;
    }
    return false;
}

function OnDialogue(args)
{
    var pid = PLAYER_SHIP;
    var na = game.GetShipOwner(pid);

    pin = topic.GetInput();
    sa = topic.GetState();

    if (pin == NO_INPUT)
    {
        son = "0";
        cip = 0;

        var arr1;
        sbs = storage.GetGlobal("marketTerminalChosenStation" + pid, arr1);
        css = sbs;

        scid = items.GetStationShopContainerId(sbs);

        var iicg = items.GetItemsAndCargo(scid);
        logivl = [];
        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 5) == "goods") {
                logivl.push(iicg[i]);
            }
        }

        topic.AddChoice(190000, "Goods");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1) {
        topic.DialogueBreak();
    }
    else if (pin == 2) {
        player.StartDialogue(na, "marketBUYStations");
    }
    ////////////////////////////GOODS/////////////////////////////////////////////
    else if (pin == 190000)
    {
        son = "0";
        cip = 0;
        var quat;

        var iicg = items.GetItemsAndCargo(scid);
        logivl = [];
        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 5) == "goods") {
                logivl.push(iicg[i]);
            }
        }

        var i;
        for (i = 0; i < logivl.length; i++) {
            quat = logivl[i].quantity;

            var arrprc = [];
            var gspv = storage.GetGlobal("globalShopPrice", arrprc);

            var pce;
            for (var j = 0; j < gspv.length; j++) {
                if (gspv[j].base_id == sbs && gspv[j].resourceID == logivl[i].xml_id) {
                    pce = Math.ceil(gspv[j].price);
                }
            }

            var spce = pce;

            var cstg = logivl[i].xml_id.replace(/_/g, " ");
            topic.AddChoice(1900000 + i, cstg.toUpperCase() + " " + "Quantity:" + " " + quat + " Price: " + spce + "$");
        }

        tnogoivl = i;
        topic.AddChoice(2, "Go back to list of stations");
        topic.AddChoice(1, "Disconnect");
    }

    ////////////////////////////GOODS INDIVIDUAL SECTION/////////////////////////////////////////////
    else if (pin >= 1900000 && pin < 1900000 + tnogoivl)
    {
        cinp = 1900000;
        cci = pin - 1900000;

        var quat = 0;

        eid = logivl[cci];
        exmid = logivl[cci].xml_id;

        csi = exmid.replace(/_/g, " ");

        var iicg = items.GetItemsAndCargo(scid);

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id == exmid) {
                quat = iicg[i].quantity;
                break;
            }
        }

        var arrpce = [];
        var gspv = storage.GetGlobal("globalShopPrice", arrpce);

        var pce;
        for (var j = 0; j < gspv.length; j++) {
            if (gspv[j].base_id == sbs && gspv[j].resourceID == exmid) {
                pce = Math.ceil(gspv[j].price);
            }
        }

        cip = pce;
        qtb = parseInt(son);

        spce = pce * qtb;

        var mon = player.GetMoney(na);

        if (mon < spce)
        {
            topic.AddPhrase("You do not have enough money to buy the " + csi.toUpperCase() + " . Price: " + "$" + spce + " Money: " + "$" + mon + ".");
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
            topic.AddChoice(19000000 + cci, "Finished setting quantity? Buy " + qtb + " " + csi.toUpperCase() + " for " + "$" + spce + ".");

            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(1, "Disconnect");
        }
    }

    else if (pin >= 19000000 && pin < 19000000 + logivl.length)
    {
        qtb = parseInt(son);
        spce = cip * qtb;

        var mon = player.GetMoney(na);

        if (qtb == 0)
        {
            topic.AddPhrase("Please select a quantity.")
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(1, "Disconnect");
        }
        else
        {
            if (mon < spce)
            {
                topic.AddPhrase("You do not have enough money to buy the " + csi.toUpperCase() + " . Price: " + "$" + spce + " Money: " + "$" + mon + ".");
                topic.AddChoice(190000, "Go Back!");
                topic.AddChoice(1, "Disconnect");
            }
            else
            {
                var itemIndex = pin - 19000000;
                var quat = 0;

                eid = logivl[itemIndex];
                exmid = logivl[itemIndex].xml_id;

                var iicg = items.GetItemsAndCargo(scid);
                for (var i = 0; i < iicg.length; i++) {
                    if (iicg[i].xml_id == exmid) {
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
                    var quatbt = 0;
                    for (var i = 0; i < iicgop.length; i++) {
                        if (iicgop[i].xml_id == exmid) {
                            quatbt = iicgop[i].quantity;
                        }
                    }

                    var obj = generator.AddItemToSpecifiedContainer(spsid, exmid, qtb);

                    var iicgop = items.GetItemsAndCargo(spsid);
                    var quatat = 0;
                    for (var i = 0; i < iicgop.length; i++) {
                        if (iicgop[i].xml_id == exmid) {
                            quatat = iicgop[i].quantity;
                        }
                    }

                    var quattwt = 0;

                    if (quatat > quatbt) {
                        quattwt = quatat - quatbt;
                        items.RemoveCargo(scid, exmid, quattwt);

                        spce = cip * quattwt;

                        var na = game.GetShipOwner(pid);
                        player.RemoveMoney(na, spce);
                        son = "0";

                        cci = 0;

                        topic.AddPhrase("You have bought " + quattwt + " " + csi.toUpperCase() + " for " + "$" + spce + ".");
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

    else if (pin == 1000000000 + cci)
    {
        cid = pin - 1000000000; // index of item,
        son += "0";

        qtb = parseInt(son);
        tp = qtb * cip;

        topic.AddChoice((cinp * 10) + cid, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");
        topic.AddChoice(cinp + cid, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1100000000 + cci)
    {
        cid = pin - 1100000000;
        son += "1";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cid, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cid, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1200000000 + cci) {
        cid = pin - 1200000000;
        son += "2";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cid, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cid, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1300000000 + cci) {
        cid = pin - 1300000000;
        son += "3";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cid, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cid, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1400000000 + cci) {
        cid = pin - 1400000000;
        son += "4";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cid, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cid, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1500000000 + cci) {
        cid = pin - 1500000000;
        son += "5";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cid, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cid, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1600000000 + cci) {
        cid = pin - 1600000000;
        son += "6";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cid, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cid, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1700000000 + cci) {
        cid = pin - 1700000000;
        son += "7";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cid, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cid, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1800000000 + cci) {
        cid = pin - 1800000000;
        son += "8";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cid, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cid, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1900000000 + cci) {
        cid = pin - 1900000000;
        son += "9";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cid, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cid, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
}

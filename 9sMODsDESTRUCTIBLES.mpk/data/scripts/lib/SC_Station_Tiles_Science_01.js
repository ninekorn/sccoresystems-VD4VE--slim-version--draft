using(console);
using(storage);


var arrayOfXMLScraps =
[
        "scrap_metal_000"
];
//arrayOfXMLScraps.push("scrap_metal_001");
//arrayOfXMLScraps.push("scrap_metal_002");
//arrayOfXMLScraps.push("scrap_metal_003");
//arrayOfXMLScraps.push("scrap_metal_004");
//arrayOfXMLScraps.push("scrap_metal_005");
//arrayOfXMLScraps.push("scrap_metal_006");
//arrayOfXMLScraps.push("scrap_metal_007");
//arrayOfXMLScraps.push("scrap_metal_008");

var initialized = 0;
var stationData ;
var splittedArray;

var partWidthL = 2;
var partWidthR = 1;

var partHeightB = 2;
var partHeightT = 1;




var SC_Station_Tiles_Science_01 =
{
    buildTiles: function (theBase, thebaseid)
    {


        var getSomeIndex = theBase.base_xml.substring(11, theBase.base_xml.length); //outpost_01_0

        var newString = "";
        for (var i = 0; i < theBase.base_xml.length; i++) {
            if (i > theBase.base_xml.length - getSomeIndex.length) {
                break;
            }
            else {
                newString += theBase.base_xml[i];
            }
        }
        //console.PrintError(newString);


        if (!storage.IsSetGlobal("station_tiles" + newString)) {
		//if(initialized == 0 || stationData == null)
		//{
			arrayOfStationTiles = [];
		
            var widthLeft = 46;
            var widthRight = 16;

            var heightTop = 60;
            var heightBottom = 37;


            var arrayOfStationTiles = [];

        for (var x = -widthLeft; x <= widthRight; x++) {
            for (var y = -heightBottom; y <= heightTop; y++) {
                arrayOfStationTiles.push(0);
            }
        }

        var xx = 0;
        var yy = 0;

        for (var x = -widthLeft; x <= widthRight; x++) {
            for (var y = -heightBottom; y <= heightTop; y++) {
                xx = x;
                yy = y;

                if (x < 0) {
                    xx *= -1;
                    xx += (widthRight);
                }

                if (y < 0) {
                    yy *= -1;
                    yy += (heightTop);
                }

                    if (x == 12 && y >= -5 && y <= 1) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }

                    else if (x >= 6 && x < 12 && y == -5) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }

                    else if (x == 6 && y >= -11 && y <= -6) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 7 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 8 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 9 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 10 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 11 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 12 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 12 && y == -10) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == -10) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == -12) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == -13) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == -13) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == -14) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == -15) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }

                    else if (x == 13 && y == -16) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == -17) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == -18) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == -18) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == -19) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == -20) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }

                    else if (x == 14 && y == -21) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 15 && y == -21) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 15 && y == -22) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 15 && y == -23) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 15 && y == -24) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }

                    else if (x == 14 && y == -24) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == -25) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == -26) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == -27) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == -28) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == -28) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == -29) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == -30) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == -31) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == -32) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == -32) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == -33) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == -34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == -34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == -35) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 12 && y == -35) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 11 && y == -35) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 11 && y == -34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 10 && y == -34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 9 && y == -34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 8 && y == -34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 7 && y == -34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 7 && y == -35) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 6 && y == -35) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 5 && y == -35) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 4 && y == -35) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 3 && y == -35) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 3 && y == -36) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 2 && y == -36) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 1 && y == -36) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 1 && y == -35) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 0 && y == -35) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -1 && y == -35) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -2 && y == -35) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -3 && y == -35) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -3 && y == -34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -4 && y == -34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -5 && y == -34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }

                    else if (x == -6 && y == -34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }

                    else if (x == -7 && y == -34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }

                    else if (x == -8 && y == -34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -8 && y == -35) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -9 && y == -35) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -35) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == -34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == -33) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -33) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -32) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -31) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -30) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -29) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -28) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -27) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == -27) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == -26) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == -25) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == -24) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == -23) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == -22) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == -21) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == -20) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == -19) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == -18) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -18) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -17) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -16) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -15) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -14) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -13) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == -13) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == -12) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == -10) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -9 && y == -10) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -8 && y == -10) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -8 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -7 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -6 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }

                    else if (x == -5 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }

                    else if (x == -4 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }

                    else if (x == -3 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }

                    else if (x == -2 && y == -11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }














                    else if (x == -2 && y >= -11 && y <= -6) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }


                    if (x >= -11 && x <= -2 && y == -5) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }

                    else if (x == -11 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == -3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -12 && y == -3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -12 && y == -2) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -12 && y == -1) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -13 && y == -1) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -13 && y == 0) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -13 && y == 1) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -13 && y == 2) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -13 && y == 3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -13 && y == 4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }

                    else if (x >= -18 && x <= -14 && y == 4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -18 && y == 3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -19 && y == 3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -19 && y == 2) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -19 && y == 1) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -19 && y == 0) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -19 && y == -1) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -19 && y == -2) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -18 && y == -2) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -18 && y == -3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -18 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -18 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -19 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -19 && y == -5) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -20 && y == -5) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -20 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -21 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -21 && y == -3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -22 && y == -3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -23 && y == -3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -24 && y == -3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -25 && y == -3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -25 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -26 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -27 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -28 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -28 && y == -5) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= -34 && x <= -29 && y == -5) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -34 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -35 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -36 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -37 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -37 && y == -3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -38 && y == -3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -39 && y == -3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -40 && y == -3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -40 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -41 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -42 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -43 && y == -4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -43 && y == -3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -43 && y == -2) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -42 && y >= -2 && y <= 3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }

                    else if (x == -43 && y >= 3 && y <= 12) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -42 && y >= 12 && y <= 18) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -43 && y == 18) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -43 && y == 19) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -42 && y == 19) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -42 && y == 20) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -41 && y == 20) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -40 && y == 20) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= -40 && x <= -36 && y == 19) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= -36 && x <= -33 && y == 20) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= -33 && x <= -28 && y == 21) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= -28 && x <= -25 && y == 20) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= -25 && x <= -21 && y == 19) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -21 && y == 20) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -20 && y == 20) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -19 && y == 20) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -19 && y == 19) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -18 && y == 19) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -18 && y == 18) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -19 && y >= 13 && y <= 18) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -18 && y == 13) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -18 && y == 12) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= -17 && x <= -13 && y == 12) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -13 && y >= 13 && y <= 17) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -12 && y >= 17 && y <= 19) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y >= 19 && y <= 21) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= -10 && x <= -2 && y == 21) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -2 && y >= 21 && y <= 27) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= -8 && x <= -3 && y == 27) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }




                    else if (x == -8 && y == 26) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -9 && y == 26) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == 26) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == 27) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == 27) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == 28) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == 28) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y >= 29 && y <= 34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y >= 34 && y <= 42) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y >= 42 && y <= 49) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == 49) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -11 && y == 50) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == 50) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == 51) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -9 && y == 51) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -9 && y == 50) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= -8 && x <= -3 && y == 50) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= -3 && x <= 6 && y == 51) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= 6 && x <= 12 && y == 50) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 12 && y == 51) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == 51) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == 50) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == 50) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == 49) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y >= 44 && y <= 49) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y >= 39 && y <= 44) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 15 && y == 39) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 15 && y == 38) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y >= 34 && y <= 38) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y >= 29 && y <= 34) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == 29) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == 28) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 14 && y == 27) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == 27) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 13 && y == 26) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 12 && y == 27) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }

                    else if (x >= 6 && x <= 12 && y == 27) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 6 && y == 26) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 5 && y >= 21 && y <= 26) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= 6 && x <= 12 && y == 21) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 12 && y >= 15 && y <= 20) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 11 && y == 15) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 11 && y == 14) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 10 && y == 14) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 10 && y == 13) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 9 && y == 13) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 9 && y == 12) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 8 && y == 12) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 8 && y == 11) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 8 && y == 13) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 9 && y == 14) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 10 && y == 15) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }



                    else if (x == 9 && y == 15) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 8 && y == 15) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 8 && y == 16) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 7 && y == 16) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 7 && y == 17) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 6 && y == 17) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= -6 && x <= 5 && y == 17) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -6 && y == 16) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -7 && y == 16) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -7 && y == 15) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -8 && y == 15) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -8 && y == 14) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -9 && y == 14) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -9 && y == 13) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y == 13) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -10 && y >= 3 && y <= 13) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -9 && y == 3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -9 && y == 2) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -8 && y == 2) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -8 && y == 1) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -7 && y == 1) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -7 && y == 0) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -6 && y == 0) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == -6 && y == -1) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= -8 && x <= 5 && y == -1) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 5 && y == -1) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 6 && y == -1) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 6 && y == 0) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 7 && y == 0) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 7 && y == 1) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 8 && y == 1) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 8 && y == 2) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 9 && y == 2) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 9 && y == 3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 8 && y == 3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 8 && y == 4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 8 && y == 5) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 9 && y == 4) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 10 && y == 3) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 10 && y == 2) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 11 && y == 2) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 11 && y == 1) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 12 && y == 1) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x == 12 && y == 0) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }


                    else if (x >= -4 && x <= 5 && y == 7) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= -5 && x <= 6 && y == 8) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                    else if (x >= -4 && x <= 5 && y == 9) {
                        arrayOfStationTiles[xx + (widthLeft + widthRight) * yy] = 1;
                    }
                }
            }


            if (theBase == null) {
                //console.PrintError("theBase null: " + scriptCounter);
            }
            else {
                if (theBase.base_xml == null) {
                    //console.PrintError("theBase.base_xml null: " + scriptCounter);
                }
                else {
                    //console.PrintError("string SC_Station_Tiles_Outpost_01: " + theBase.id);
                }
                //console.PrintError("theBase: " + theBase.id + " SC_Station_Tiles_Outpost_01.js " + " otherbaseID: " + thebaseid);
            }




            var getSomeIndex = theBase.base_xml.substring(11, theBase.base_xml.length); //outpost_01_0

            var parsedAngle = parseInt(getSomeIndex);

            var someExtraData = [];

            for (var i = 0; i < arrayOfStationTiles.length; i++) {
                someExtraData[i] = arrayOfStationTiles[i];
            }

            stationData = {
                baseID: theBase.id, sys_id: theBase.sys_id, coord: theBase.coord, xml_id: theBase.base_xml,
                widthL: widthLeft, widthR: widthRight, heightT: heightTop, heightB: heightBottom,
                grid: arrayOfStationTiles, rot: parsedAngle, visualTiles: someExtraData
            };
            storage.SetGlobal("station_tiles" + theBase.id, stationData);
            storage.SetGlobal("station_tiles" + newString, stationData);

            scriptCounter++;
            return stationData;
        }
        else {
            var stationData = storage.GetGlobal("station_tiles" + newString);

            var getSomeIndex = theBase.base_xml.substring(11, theBase.base_xml.length); //outpost_01_0
            var parsedAngle = parseInt(getSomeIndex);

            var someExtraData = [];

            for (var i = 0; i < stationData.grid.length; i++) {
                someExtraData[i] = stationData.grid[i];
            }

            stationData.baseID = theBase.id;
            stationData.sys_id = theBase.sys_id;
            stationData.coord = theBase.coord;
            stationData.xml_id = theBase.base_xml;
            stationData.widthL = widthLeft;
            stationData.widthR = widthRight;
            stationData.heightT = heightTop;
            stationData.heightB = heightBottom;
            stationData.grid = stationData.grid;//arrayOfStationTiles;
            stationData.rot = parsedAngle;
            stationData.visualTiles = someExtraData;

            storage.SetGlobal("station_tiles" + theBase.id, stationData);

            return stationData;
        }
    },

	setSplittedArrayData: function(theBase)
	{
		if(stationData == null)
		{
		   stationData = SC_Station_Tiles_Science_01.buildTiles(theBase);
		}
		var original = {Height:stationData.widthL+stationData.widthR , Width:stationData.heightB+stationData.heightT};
		
		splittedArray = Split(original,partWidthL,partWidthR,partHeightB,partHeightT,stationData.grid,stationData.widthL,stationData.widthR);
	},

	getSplittedArrayData: function(theBase, indexX, indexY)
	{
		if(stationData == null)
		{
		   stationData = SC_Station_Tiles_Science_01.buildTiles(theBase);
		}
		var currentArrayIndex = indexX + (partWidthL+partWidthR+1) * indexY;
		return splittedArray[currentArrayIndex];
	}
};

//https: //gamedev.stackexchange.com/questions/11584/xna-splitting-one-large-texture-into-an-array-of-smaller-textures
function Split( original, partWidthL,partWidthR,partHeightB,partHeightT, stationGrid , widthL,widthR)
{
	var yCount = original.Height / (partHeightB+partHeightT+1) + ( (partHeightB+partHeightT+1) % original.Height == 0 ? 0 : 1);
	var xCount = original.Width / (partWidthL+partWidthR+1) + ((partWidthL+partWidthR+1) % original.Width == 0 ? 0 : 1);

	var newArray = [];
	var dataPerPart = (partWidthL+partWidthR+1) * (partHeightB+partHeightT+1);

	var index = 0;

	for (var y = 0; y <= yCount * (partHeightB + partHeightT + 1); y += (partHeightB+partHeightT+1))
	{
		for (var x = 0; x <= xCount * (partWidthL + partWidthR + 1); x += (partWidthL+partWidthR+1))
		{
		    var partData = [];
			var partPosition = {x:null,y:null};

			for (var py = -partHeightB; py <= (partHeightT); py++)
			{
				for (var px = -partWidthL; px <= (partWidthR); px++)
				{
					var partIndex = px + (partWidthL + partWidthR + 1) * py;
					/*if(px == 0 && py == 0)
					{
						partPosition = partIndex;
					}*/

					
					if (y + py >= original.Height || x + px >= original.Width)
					{
						partData[partIndex] = 0;
					}                     
					else
					{
						//partData[partIndex] = stationGrid[(x + px) + (y + py) * original.Width];
						partData[partIndex] =  stationGrid[(xx+px) + (widthL + widthR + 1) * (yy+py)];
					}					
				}
			}

			newArray[index++] = part;
		}
	}

	return newArray;







    /*yCount = original.Height / partHeight + (partHeight % original.Height == 0 ? 0 : 1);//The number of textures in each horizontal row
    xCount = original.Height / partHeight + (partHeight % original.Height == 0 ? 0 : 1);//The number of textures in each vertical column
    Texture2D[] r = new Texture2D[xCount * yCount];//Number of parts = (area of original) / (area of each part).
    int dataPerPart = partWidth * partHeight;//Number of pixels in each of the split parts

    //Get the pixel data from the original texture:
    Color[] originalData = new Color[original.Width * original.Height];
    original.GetData<Color>(originalData);

    int index = 0;
    for (int y = 0; y < yCount * partHeight; y += partHeight)
        for (int x = 0; x < xCount * partWidth; x += partWidth)
        {
            //The texture at coordinate {x, y} from the top-left of the original texture
            Texture2D part = new Texture2D(original.GraphicsDevice, partWidth, partHeight);
            //The data for part
            Color[] partData = new Color[dataPerPart];

            //Fill the part data with colors from the original texture
            for (int py = 0; py < partHeight; py++)
                for (int px = 0; px < partWidth; px++)
                {
                    int partIndex = px + py * partWidth;
                    //If a part goes outside of the source texture, then fill the overlapping part with Color.Transparent
                    if (y + py >= original.Height || x + px >= original.Width)
                        partData[partIndex] = Color.Transparent;
                    else
                        partData[partIndex] = originalData[(x + px) + (y + py) * original.Width];
                }

            //Fill the part with the extracted data
            part.SetData<Color>(partData);
            //Stick the part in the return array:                    
            r[index++] = part;
        }
    //Return the array of parts.
    return r;*/
}


















//https: //stackoverflow.com/questions/13695317/rotate-a-point-around-another-point
function RotatePoint(pointToRotate, centerPoint, angleInDegrees) {
    var angleInRadians = angleInDegrees * (Math.PI / 180);
    var cosTheta = Math.cos(angleInRadians);
    var sinTheta = Math.sin(angleInRadians);

    var newX = (cosTheta * (pointToRotate.x - centerPoint.x) - sinTheta * (pointToRotate.y - centerPoint.y) + centerPoint.x);
    var newY = (sinTheta * (pointToRotate.x - centerPoint.x) + cosTheta * (pointToRotate.y - centerPoint.y) + centerPoint.y);

    var newPos = { x: newX, y: newY };

    return newPos;
}


function indexOfStuff(someIndex) {
    if (someIndex == "0") {
        return 0;
    }
    if (someIndex == "45") {
        return 1;
    }
    else if (someIndex == "90") {
        return 2;
    }
    else if (someIndex == "135") {
        return 3;
    }
    else if (someIndex == "180") {
        return 4;
    }
    else if (someIndex == "225") {
        return 5;
    }
    else if (someIndex == "270") {
        return 6;
    }
    else if (someIndex == "315") {
        return 7;
    }
}

//var id1 = generator.AddSpecialObject(args.sys_id, args.bases[0].coord.x - 3.5, args.bases[0].coord.y - 23, "station_platform_refuel", 0);
//var id2 = generator.AddSpecialObject(args.sys_id, args.bases[0].coord.x + 3, args.bases[0].coord.y - 23, "station_platform_repair", 0);

//var idOfBase = args.bases[0].id;

//var id3 = generator.AddNPCShipToSystem("drone repair", "ai_repair_high", 1, "xml_repair_low", args.sys_id, args.bases[0].coord.x + 5.15, args.bases[0].coord.y - 21.5, { class: "stationDialog", someTag: "drone_repair", greeting: "terminal", stationID: idOfBase }); //, unique_id: "stationDialog"
//var id6 = generator.AddNPCShipToSystem("drone retriever", "ai_retriever_drone", 1, "xml_drone_retriever", args.sys_id, args.bases[0].coord.x + 13, args.bases[0].coord.y - 10, { class: "stationDialog", someTag: "drone_retriever", greeting: "terminal", stationID: idOfBase }); //, unique_id: "stationDialog"

